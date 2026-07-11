#pragma once

/*
 * Per-thread bump scratch arena for the Allocation-Free Tessellation Pipeline
 * (AFTP) — see design/new/emsdk-upgrade-scalable-allocator.md (Phase 4) in
 * bldrs-ai/conway.
 *
 * The phase-1 telemetry (structures/alloc_telemetry.h) showed one Arty_Z7 load
 * makes ~218M allocator calls while 99.9% of faces peak under ~1 MiB of
 * transient scratch. This arena is the mechanism that removes that traffic:
 * each ThreadPool worker owns one arena, hands out aligned bump slices for a
 * face's temporaries, and rewinds (reset) after the face — zero malloc/free in
 * steady state. Oversized faces spill individual allocations to the heap
 * (counted), so correctness never depends on the buffer being large enough.
 *
 * This header is the primitive only. Wiring the tessellation temporaries
 * (CDT state, trim polygons, NURBS grids) to draw from it — via the
 * ScratchAllocator adapter below — is a separate, digest-verified step; this
 * type changes no geometry on its own.
 *
 * Not thread-safe by design: never share one arena across threads. Use
 * ThreadScratchArena() to get the calling thread's instance.
 */

#include <cstddef>
#include <cstdint>
#include <memory_resource>
#include <new>
#include <vector>

namespace conway {

// Default per-thread arena capacity (size of the base chunk). Sized from the
// telemetry: ≥99.9% of Arty_Z7 faces and ~98% of the NURBS-dense jet engine
// peak under ~1–2 MiB of live scratch, and a bump allocator (no per-allocation
// free) consumes the *cumulative* not peak size within a face, so we give
// generous headroom. Per-thread × a handful of workers, so a few MiB each is
// cheap.
constexpr std::size_t kDefaultScratchCapacity = 8u * 1024u * 1024u;

// Total backing bytes retained across rewind()/reset() for reuse. The base
// chunk is always kept; grown chunks are kept only while the total stays under
// this cap, then released back to the heap. This keeps steady-state faces
// malloc-free (they fit the base chunk) while a rare oversized face's huge
// transient chunk does not permanently inflate per-thread RSS.
constexpr std::size_t kDefaultRetainedCapacity = 32u * 1024u * 1024u;

/**
 * A single-threaded bump allocator that GROWS in chunks instead of spilling.
 *
 * allocate() returns aligned storage by bumping within the current chunk; when
 * a request doesn't fit, it advances to the next already-grown chunk or, if
 * none is large enough, appends a fresh geometrically-larger chunk (one heap
 * allocation) and bumps within that. This is the key difference from a
 * fixed-buffer arena: a face whose cumulative scratch far exceeds the base
 * chunk costs O(log size) heap allocations for growth, not one per allocation.
 *
 * Individual deallocations are no-ops. reset()/rewind() rewind the bump
 * position for reuse; grown chunks are retained (up to kDefaultRetainedCapacity)
 * so the next face reuses them without touching the heap, and the excess is
 * released so an oversized face doesn't pin memory.
 */
class ScratchArena {
 public:
  explicit ScratchArena(std::size_t capacity = kDefaultScratchCapacity,
                         std::size_t retainedCapacity = kDefaultRetainedCapacity)
      : capacity_(capacity),
        retainedCapacity_(retainedCapacity < capacity ? capacity
                                                       : retainedCapacity) {
    chunks_.push_back(
        Chunk{static_cast<std::uint8_t*>(::operator new(capacity)), capacity});
  }

  ~ScratchArena() {
    for (const Chunk& c : chunks_) {
      ::operator delete(c.ptr);
    }
  }

  ScratchArena(const ScratchArena&) = delete;
  ScratchArena& operator=(const ScratchArena&) = delete;

  /**
   * Return `bytes` of storage aligned to `align` (a power of two), bumping
   * within the current chunk and growing (or advancing to a retained chunk)
   * when it doesn't fit.
   *
   * @param bytes Number of bytes requested.
   * @param align Required alignment (power of two).
   * @return Pointer to usable storage (never null unless the heap is
   *   exhausted, in which case operator new throws).
   */
  void* allocate(std::size_t bytes,
                 std::size_t align = alignof(std::max_align_t)) {
    if (bytes == 0) {
      bytes = 1;
    }
    // Fast path: fits the current chunk.
    void* p = tryBump(chunks_[current_], bytes, align);
    if (p != nullptr) {
      return p;
    }
    // The current chunk is full — its used portion becomes committed scratch.
    committedLower_ += offset_;
    // Advance through any already-grown chunks that can satisfy the request.
    while (current_ + 1 < chunks_.size()) {
      ++current_;
      offset_ = 0;
      p = tryBump(chunks_[current_], bytes, align);
      if (p != nullptr) {
        return p;
      }
      // This retained chunk is too small for the request; leave it unused and
      // keep advancing (its 0 used bytes add nothing to the committed total).
    }
    // No retained chunk fits — grow. Geometric: at least the current total
    // backing, so cumulative capacity doubles and chunk count stays O(log n).
    std::size_t grow = totalBacking();
    std::size_t newSize = bytes + align > grow ? bytes + align : grow;
    chunks_.push_back(
        Chunk{static_cast<std::uint8_t*>(::operator new(newSize)), newSize});
    current_ = chunks_.size() - 1;
    offset_ = 0;
    ++growthCount_;
    growthBytes_ += newSize;
    p = tryBump(chunks_[current_], bytes, align);
    return p;  // fits by construction (newSize >= bytes + align)
  }

  /**
   * A saved position in the arena. Take one with mark(), restore it with
   * rewind() — this makes scopes nestable (an inner scope frees only what it
   * allocated, never the outer scope's still-live scratch), which a plain
   * reset-to-zero cannot do safely when tessellation helpers call each other.
   */
  struct Marker {
    std::size_t chunk;
    std::size_t offset;
    std::size_t committedLower;
  };

  /** Capture the current position for a later rewind(). */
  Marker mark() const { return Marker{current_, offset_, committedLower_}; }

  /**
   * Restore to a previously marked position: rewind the bump position to the
   * marked chunk/offset. Chunks grown after the mark become reusable free
   * space; the excess beyond the retention cap is released to the heap.
   * Lifetime growth stats are left intact (they measure the whole run).
   */
  void rewind(Marker m) {
    current_ = m.chunk;
    offset_ = m.offset;
    committedLower_ = m.committedLower;
    trimToRetentionCap();
  }

  /** Rewind to the base chunk and drop retained growth beyond the cap. */
  void reset() {
    current_ = 0;
    offset_ = 0;
    committedLower_ = 0;
    trimToRetentionCap();
  }

  /** Peak cumulative bump bytes handed out within a face since construction. */
  std::size_t highWater() const { return highWater_; }

  /** Number of heap chunks grown (beyond the base) since construction. */
  std::uint64_t spillCount() const { return growthCount_; }

  /** Total bytes of grown chunks since construction. */
  std::uint64_t spillBytes() const { return growthBytes_; }

  /** Base-chunk capacity in bytes. */
  std::size_t capacity() const { return capacity_; }

  /** Total backing bytes currently held (base + retained grown chunks). */
  std::size_t totalCapacity() const { return totalBacking(); }

 private:
  struct Chunk {
    std::uint8_t* ptr;
    std::size_t size;
  };

  // Try to bump `bytes` (aligned) within `c` starting at offset_. Returns the
  // pointer and advances offset_ on success, or nullptr if it doesn't fit.
  void* tryBump(const Chunk& c, std::size_t bytes, std::size_t align) {
    // Align the absolute address, not the offset: operator new only guarantees
    // max_align_t on the chunk base, so an over-aligned request (align > chunk
    // base alignment) must be satisfied against the real pointer.
    const std::uintptr_t addr = reinterpret_cast<std::uintptr_t>(c.ptr);
    const std::uintptr_t aligned =
        (addr + offset_ + (align - 1)) & ~(align - 1);
    const std::size_t base = static_cast<std::size_t>(aligned - addr);
    if (base <= c.size && bytes <= c.size - base) {
      offset_ = base + bytes;
      const std::size_t live = committedLower_ + offset_;
      if (live > highWater_) {
        highWater_ = live;
      }
      return c.ptr + base;
    }
    return nullptr;
  }

  std::size_t totalBacking() const {
    std::size_t total = 0;
    for (const Chunk& c : chunks_) {
      total += c.size;
    }
    return total;
  }

  // Release trailing grown chunks (never the base, never a chunk at/below the
  // current position) while the retained backing exceeds the cap.
  void trimToRetentionCap() {
    std::size_t total = totalBacking();
    while (chunks_.size() > current_ + 1 && chunks_.size() > 1 &&
           total > retainedCapacity_) {
      total -= chunks_.back().size;
      ::operator delete(chunks_.back().ptr);
      chunks_.pop_back();
    }
  }

  std::size_t capacity_;
  std::size_t retainedCapacity_;
  std::vector<Chunk> chunks_;
  std::size_t current_ = 0;         // index of the chunk we bump within
  std::size_t offset_ = 0;          // bump offset within chunks_[current_]
  std::size_t committedLower_ = 0;  // used bytes in chunks below current_
  std::size_t highWater_ = 0;
  std::uint64_t growthCount_ = 0;
  std::uint64_t growthBytes_ = 0;
};

/**
 * The calling thread's scratch arena (lazily constructed, default capacity).
 * Each thread gets its own instance, so no synchronization is needed.
 *
 * @return Reference to this thread's arena.
 */
inline ScratchArena& ThreadScratchArena() {
  static thread_local ScratchArena arena(kDefaultScratchCapacity);
  return arena;
}

/**
 * RAII guard that checkpoints a scratch arena on construction and rewinds to
 * that checkpoint on destruction — the per-face boundary. Because it restores
 * the marked position (not zero), scopes nest safely: a helper that opens its
 * own scope frees only its own scratch and leaves the caller's intact.
 * Defaults to the calling thread's arena.
 */
class ScratchArenaScope {
 public:
  explicit ScratchArenaScope(ScratchArena& arena = ThreadScratchArena())
      : arena_(arena), mark_(arena.mark()) {}
  ~ScratchArenaScope() { arena_.rewind(mark_); }

  ScratchArenaScope(const ScratchArenaScope&) = delete;
  ScratchArenaScope& operator=(const ScratchArenaScope&) = delete;

  ScratchArena& arena() { return arena_; }

 private:
  ScratchArena& arena_;
  ScratchArena::Marker mark_;
};

/**
 * A std::pmr memory resource backed by a ScratchArena. This lets *type-erased*
 * pmr containers (std::pmr::vector / std::pmr::unordered_map — e.g. the
 * WingedEdgeMesh storage) draw from the arena without becoming a distinct type
 * per allocator: the resource is a runtime pointer, so a pmr WingedEdgeMesh is
 * one type whether it's arena-backed or heap-backed. deallocate is a no-op;
 * the arena reclaims wholesale at reset()/rewind().
 */
class ScratchArenaResource : public std::pmr::memory_resource {
 public:
  explicit ScratchArenaResource(ScratchArena& arena) : arena_(&arena) {}

 private:
  void* do_allocate(std::size_t bytes, std::size_t align) override {
    return arena_->allocate(bytes, align);
  }
  void do_deallocate(void* /*p*/, std::size_t /*bytes*/,
                     std::size_t /*align*/) override {}
  bool do_is_equal(
      const std::pmr::memory_resource& other) const noexcept override {
    return this == &other;
  }

  ScratchArena* arena_;
};

/**
 * The calling thread's scratch arena exposed as a pmr memory resource. Pair
 * with a ScratchArenaScope at the per-face boundary so the arena rewinds after
 * the pmr containers built on it are done. Never outlive a rewind.
 *
 * @return A pmr resource that allocates from this thread's ScratchArena.
 */
inline std::pmr::memory_resource* ThreadScratchResource() {
  static thread_local ScratchArenaResource resource(ThreadScratchArena());
  return &resource;
}

/**
 * A minimal C++ allocator that draws from a ScratchArena, so existing STL
 * containers (e.g. the std::vector temporaries inside CDT/tessellation) can be
 * retargeted to arena storage with only a type change. deallocate() is a
 * no-op; the arena reclaims at reset().
 *
 * By default it binds to the calling thread's arena at allocation time, which
 * is what the tessellation hot path wants (the worker's own arena). A specific
 * arena can be supplied for tests or nested scopes.
 */
template <typename T>
class ScratchAllocator {
 public:
  using value_type = T;

  ScratchAllocator() noexcept : arena_(nullptr) {}
  explicit ScratchAllocator(ScratchArena& arena) noexcept : arena_(&arena) {}

  template <typename U>
  explicit ScratchAllocator(const ScratchAllocator<U>& other) noexcept
      : arena_(other.arena_) {}

  T* allocate(std::size_t n) {
    ScratchArena& a = arena_ != nullptr ? *arena_ : ThreadScratchArena();
    return static_cast<T*>(a.allocate(n * sizeof(T), alignof(T)));
  }

  void deallocate(T* /*p*/, std::size_t /*n*/) noexcept {
    // No-op: arena memory is reclaimed wholesale at ScratchArena::reset().
  }

  template <typename U>
  bool operator==(const ScratchAllocator<U>& other) const noexcept {
    return arena_ == other.arena_;
  }
  template <typename U>
  bool operator!=(const ScratchAllocator<U>& other) const noexcept {
    return arena_ != other.arena_;
  }

  // Exposed so rebinding copy-construction can carry the arena across types.
  ScratchArena* arena_;
};

}  // namespace conway
