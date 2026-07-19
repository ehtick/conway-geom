#pragma once

/*
 * Resident tile pool — the C++ twin of conway's TS memory system
 * (bldrs-ai/conway src/core/mem/: ChunkedPool + SharedAssetPool), for the
 * demand-driven geometry residency design ("Resident memory: two regimes" in
 * design/new/streaming-federated-loader.md).
 *
 * Why not per-asset malloc/free: wasm linear memory never shrinks, so the tab
 * pays the heap's high-water mark forever and evict/refill churn turns
 * fragmentation into a permanent leak. This pool allocates ONE region at init
 * (the byte budget, paid deliberately and exactly once), carves it into
 * fixed-size chunks with a freelist, and serves refcounted assets out of
 * those chunks. The region's high-water mark is the budget by construction;
 * fragmentation reduces to bounded internal waste in each asset's last chunk.
 *
 * Lifetime regimes, for orientation: tessellation *scratch* (phase-bounded,
 * dies at commit) stays on the per-thread ScratchArena — this pool is for
 * *demand-bounded residents*: committed tile payloads that live until the
 * scheduler evicts them. The commit copy is AFTP phase 2's exact-size commit,
 * redirected from the general heap into pool chunks.
 *
 * Sharing: assets are refcounted (the instance ⇄ asset / occurrence ⇄
 * definition relationship — many products, one mapped representation).
 * Storage is keyed on the asset, so releasing one holder never frees a
 * payload another still references; chunks return to the freelist on the
 * last release only. The scheduling/budget policy lives TS-side
 * (DemandGeometryQueue + GeometryTilePool); this type owns bytes and
 * refcounts only, and its accounting mirrors the TS spec so the two sides
 * agree chunk-for-chunk.
 *
 * A tile's chunks are generally NON-CONTIGUOUS. Consumers either walk the
 * per-chunk segments (e.g. one gl.bufferSubData per chunk at GPU upload) or
 * gather-copy out via readAsset(). Payload writers scatter-copy in via
 * commitAsset().
 *
 * Not thread-safe: confine a pool to one thread (the geometry worker that
 * owns residency) or guard it externally.
 */

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace conway {

/// Stable identity of a shareable resident payload. For IFC geometry this is
/// the representation's local ID (mapped items make many products carry the
/// same AssetId); nothing in the pool is geometry-specific.
using AssetId = std::uint64_t;

class TilePool {
 public:
  struct Stats {
    std::size_t totalChunks = 0;
    std::size_t freeChunks = 0;
    std::size_t bytesInUse = 0;   // physical, chunk-rounded
    std::size_t assetCount = 0;
    std::size_t commits = 0;
    std::size_t retains = 0;
    std::size_t releases = 0;
    std::size_t failedCommits = 0;
  };

  /// One region, allocated exactly once. budgetBytes rounds down to whole
  /// chunks and must fit at least one chunk.
  TilePool(std::size_t budgetBytes, std::size_t chunkBytes)
      : chunkBytes_(chunkBytes), totalChunks_(budgetBytes / chunkBytes) {
    assert(chunkBytes_ > 0);
    assert(totalChunks_ >= 1);

    region_.resize(totalChunks_ * chunkBytes_);
    freeList_.reserve(totalChunks_);

    for (std::size_t chunk = totalChunks_; chunk-- > 0;) {
      freeList_.push_back(static_cast<std::uint32_t>(chunk));
    }
  }

  TilePool(const TilePool&) = delete;
  TilePool& operator=(const TilePool&) = delete;

  std::size_t chunkBytes() const { return chunkBytes_; }
  std::size_t totalChunks() const { return totalChunks_; }
  std::size_t freeChunks() const { return freeList_.size(); }
  std::size_t totalBytes() const { return totalChunks_ * chunkBytes_; }

  std::size_t bytesInUse() const {
    return (totalChunks_ - freeList_.size()) * chunkBytes_;
  }

  /// The physical (chunk-rounded) cost of a byte size — what a commit of
  /// `bytes` actually consumes. Budget layers above use this so logical
  /// charges cover physical use (the TS-side invariant).
  std::size_t chunkRound(std::size_t bytes) const {
    return ((bytes + chunkBytes_ - 1) / chunkBytes_) * chunkBytes_;
  }

  bool isResident(AssetId asset) const {
    return assets_.find(asset) != assets_.end();
  }

  std::uint32_t refCountOf(AssetId asset) const {
    auto found = assets_.find(asset);
    return found == assets_.end() ? 0u : found->second.refCount;
  }

  std::size_t byteSizeOf(AssetId asset) const {
    auto found = assets_.find(asset);
    return found == assets_.end() ? 0u : found->second.byteSize;
  }

  /// Take a reference on an already-resident asset. Returns false if the
  /// asset is absent — the caller's cue to extract into scratch and
  /// commitAsset(). (Split from commit because only the caller can produce
  /// the payload; TS's unified retain() maps onto retain-else-commit here.)
  bool retainAsset(AssetId asset) {
    auto found = assets_.find(asset);

    if (found == assets_.end()) {
      return false;
    }

    ++found->second.refCount;
    ++retains_;
    return true;
  }

  /// One contiguous piece of a to-be-committed payload (iovec-style), so a
  /// header + several vectors commit without first being concatenated.
  struct PayloadPart {
    const std::byte* data = nullptr;
    std::size_t byteLength = 0;
  };

  /// Make an asset resident with refCount 1 by scatter-copying `byteSize`
  /// bytes from `payload` into freshly acquired chunks. All-or-nothing:
  /// returns false (and changes nothing) if the free chunks can't cover it —
  /// the scheduler's cue to evict and retry. The asset must not already be
  /// resident (use retainAsset first).
  bool commitAsset(AssetId asset, const std::byte* payload,
                   std::size_t byteSize) {
    const PayloadPart part{payload, byteSize};
    return commitAssetParts(asset, &part, 1);
  }

  /// As commitAsset, but the payload is the concatenation of `partCount`
  /// discontiguous parts — the real tessellation-commit shape (a small
  /// header plus the reified vertex and index vectors), copied straight
  /// into chunks with no intermediate contiguous buffer.
  bool commitAssetParts(AssetId asset, const PayloadPart* parts,
                        std::size_t partCount) {
    assert(assets_.find(asset) == assets_.end());

    std::size_t byteSize = 0;

    for (std::size_t part = 0; part < partCount; ++part) {
      byteSize += parts[part].byteLength;
    }

    const std::size_t needed = (byteSize + chunkBytes_ - 1) / chunkBytes_;

    if (needed > freeList_.size()) {
      ++failedCommits_;
      return false;
    }

    Asset entry;
    entry.byteSize = byteSize;
    entry.refCount = 1;
    entry.chunks.reserve(needed);

    for (std::size_t where = 0; where < needed; ++where) {
      entry.chunks.push_back(freeList_.back());
      freeList_.pop_back();
    }

    // Walk parts and chunks together: fill each chunk from as many part
    // spans as it takes, crossing part boundaries mid-chunk when needed.
    std::size_t chunkIndex = 0;
    std::size_t chunkFill = 0;

    for (std::size_t part = 0; part < partCount; ++part) {
      const std::byte* source = parts[part].data;
      std::size_t remaining = parts[part].byteLength;

      while (remaining > 0) {
        if (chunkFill == chunkBytes_) {
          ++chunkIndex;
          chunkFill = 0;
        }

        const std::size_t space = chunkBytes_ - chunkFill;
        const std::size_t span = remaining < space ? remaining : space;

        std::memcpy(chunkData(entry.chunks[chunkIndex]) + chunkFill, source,
                    span);

        source += span;
        remaining -= span;
        chunkFill += span;
      }
    }

    assets_.emplace(asset, std::move(entry));
    ++commits_;
    return true;
  }

  /// Drop a reference. On the last reference the asset's chunks return to
  /// the freelist and true is returned (the caller's cue to drop any
  /// derived state, e.g. GPU buffers).
  bool releaseAsset(AssetId asset) {
    auto found = assets_.find(asset);
    assert(found != assets_.end());
    ++releases_;

    if (--found->second.refCount > 0) {
      return false;
    }

    for (const std::uint32_t chunk : found->second.chunks) {
      freeList_.push_back(chunk);
    }

    assets_.erase(found);
    return true;
  }

  /// Number of chunk segments an asset occupies (for per-segment consumers).
  std::size_t segmentCountOf(AssetId asset) const {
    auto found = assets_.find(asset);
    return found == assets_.end() ? 0u : found->second.chunks.size();
  }

  /// Pointer + byte length of one of an asset's segments, in payload order.
  /// Valid until the asset's last release. This is the zero-copy consumption
  /// path (e.g. one bufferSubData per segment at GPU upload).
  std::pair<const std::byte*, std::size_t> segmentOf(AssetId asset,
                                                     std::size_t index) const {
    auto found = assets_.find(asset);
    assert(found != assets_.end());
    assert(index < found->second.chunks.size());

    const Asset& entry = found->second;
    const std::size_t offset = index * chunkBytes_;
    const std::size_t span = entry.byteSize - offset < chunkBytes_
                                 ? entry.byteSize - offset
                                 : chunkBytes_;

    return {chunkData(entry.chunks[index]), span};
  }

  /// Gather-copy an asset's payload into contiguous storage (the copying
  /// consumption path). `destination` must hold byteSizeOf(asset).
  void readAsset(AssetId asset, std::byte* destination) const {
    auto found = assets_.find(asset);
    assert(found != assets_.end());

    const Asset& entry = found->second;
    std::size_t copied = 0;

    for (const std::uint32_t chunk : entry.chunks) {
      const std::size_t span = entry.byteSize - copied < chunkBytes_
                                   ? entry.byteSize - copied
                                   : chunkBytes_;

      std::memcpy(destination + copied, chunkData(chunk), span);
      copied += span;
    }
  }

  Stats stats() const {
    Stats result;
    result.totalChunks = totalChunks_;
    result.freeChunks = freeList_.size();
    result.bytesInUse = bytesInUse();
    result.assetCount = assets_.size();
    result.commits = commits_;
    result.retains = retains_;
    result.releases = releases_;
    result.failedCommits = failedCommits_;
    return result;
  }

 private:
  struct Asset {
    std::vector<std::uint32_t> chunks;
    std::size_t byteSize = 0;
    std::uint32_t refCount = 0;
  };

  std::byte* chunkData(std::uint32_t chunk) {
    return region_.data() + static_cast<std::size_t>(chunk) * chunkBytes_;
  }

  const std::byte* chunkData(std::uint32_t chunk) const {
    return region_.data() + static_cast<std::size_t>(chunk) * chunkBytes_;
  }

  std::size_t chunkBytes_ = 0;
  std::size_t totalChunks_ = 0;
  std::vector<std::byte> region_;
  std::vector<std::uint32_t> freeList_;
  std::unordered_map<AssetId, Asset> assets_;

  std::size_t commits_ = 0;
  std::size_t retains_ = 0;
  std::size_t releases_ = 0;
  std::size_t failedCommits_ = 0;
};

}  // namespace conway
