#pragma once

/*
 * Module-level surface over the resident TilePool (structures/tile_pool.h) —
 * the seam the wasm bindings (conway-api.cpp) and, later, the tessellation
 * commit path talk to. Free functions over one module-owned pool instance,
 * mirroring the conway TS policy layer's expectations (bldrs-ai/conway
 * src/core/mem/ + GeometryTilePool):
 *
 *   init once with the byte budget (paid exactly once — this is the region
 *   the wasm heap grows by, deliberately, at open), then per asset:
 *   retain-else-commit, release on evict, segment walk for GPU upload.
 *
 * Geometry tile payload layout (written by the conway-api.cpp glue from a
 * reified Geometry; this header stays payload-agnostic):
 *
 *   u32 vertexByteLength   | 8-byte header, contiguous in the first chunk
 *   u32 indexByteLength    | (init enforces chunkBytes >= 16)
 *   float vertexData[..]     starting at byte offset 8
 *   u32   indexData[..]      immediately after vertexData
 *
 * No emscripten dependencies — natively compilable alongside tile_pool.h.
 */

#include <cstdint>
#include <memory>

#include "tile_pool.h"

namespace conway {

inline constexpr std::size_t kTilePoolMinChunkBytes = 16;

/// The module's pool. One per wasm instance, sized at open. unique_ptr so
/// re-init (e.g. a new model with a different budget) releases the old
/// region deterministically.
inline std::unique_ptr<TilePool>& moduleTilePool() {
  static std::unique_ptr<TilePool> pool;
  return pool;
}

/// (Re)create the module pool. Any previous pool — and every tile in it —
/// is dropped. Returns false on degenerate sizing.
inline bool tilePoolInit(std::size_t budgetBytes, std::size_t chunkBytes) {
  if (chunkBytes < kTilePoolMinChunkBytes || budgetBytes < chunkBytes) {
    return false;
  }

  moduleTilePool() = std::make_unique<TilePool>(budgetBytes, chunkBytes);
  return true;
}

inline bool tilePoolInitialized() { return moduleTilePool() != nullptr; }

/// Commit a raw payload from module memory into freshly acquired chunks.
/// All-or-nothing; false when absent pool, duplicate asset, or no room —
/// the scheduler's cue to evict and retry.
inline bool tilePoolCommitBytes(AssetId asset, const std::byte* payload,
                                std::size_t byteLength) {
  TilePool* pool = moduleTilePool().get();

  if (pool == nullptr || pool->isResident(asset)) {
    return false;
  }

  return pool->commitAsset(asset, payload, byteLength);
}

inline bool tilePoolRetain(AssetId asset) {
  TilePool* pool = moduleTilePool().get();
  return pool != nullptr && pool->retainAsset(asset);
}

/// Drop one reference; true when this was the last one (chunks returned).
inline bool tilePoolRelease(AssetId asset) {
  TilePool* pool = moduleTilePool().get();

  if (pool == nullptr || !pool->isResident(asset)) {
    return false;
  }

  return pool->releaseAsset(asset);
}

inline bool tilePoolIsResident(AssetId asset) {
  TilePool* pool = moduleTilePool().get();
  return pool != nullptr && pool->isResident(asset);
}

inline std::uint32_t tilePoolRefCount(AssetId asset) {
  TilePool* pool = moduleTilePool().get();
  return pool == nullptr ? 0 : pool->refCountOf(asset);
}

inline std::size_t tilePoolByteSize(AssetId asset) {
  TilePool* pool = moduleTilePool().get();
  return pool == nullptr ? 0 : pool->byteSizeOf(asset);
}

inline std::size_t tilePoolSegmentCount(AssetId asset) {
  TilePool* pool = moduleTilePool().get();
  return pool == nullptr ? 0 : pool->segmentCountOf(asset);
}

/// Pointer (as an address) + byte length of one payload segment, for the
/// zero-copy per-segment GPU upload walk. 0 when absent / out of range.
inline std::uintptr_t tilePoolSegmentAddress(AssetId asset,
                                             std::size_t segment) {
  TilePool* pool = moduleTilePool().get();

  if (pool == nullptr || segment >= pool->segmentCountOf(asset)) {
    return 0;
  }

  return reinterpret_cast<std::uintptr_t>(pool->segmentOf(asset, segment).first);
}

inline std::size_t tilePoolSegmentByteLength(AssetId asset,
                                             std::size_t segment) {
  TilePool* pool = moduleTilePool().get();

  if (pool == nullptr || segment >= pool->segmentCountOf(asset)) {
    return 0;
  }

  return pool->segmentOf(asset, segment).second;
}

/// Gather a tile's payload to contiguous module memory (the copying
/// consumption path). Destination must hold tilePoolByteSize(asset).
inline bool tilePoolReadAsset(AssetId asset, std::byte* destination) {
  TilePool* pool = moduleTilePool().get();

  if (pool == nullptr || !pool->isResident(asset)) {
    return false;
  }

  return pool->readAsset(asset, destination);
}

inline TilePool::Stats tilePoolStats() {
  TilePool* pool = moduleTilePool().get();
  return pool == nullptr ? TilePool::Stats{} : pool->stats();
}

}  // namespace conway
