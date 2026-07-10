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
#include <new>
#include <vector>

namespace conway {

// Default per-thread arena capacity. Sized from the telemetry: ≥99.9% of
// Arty_Z7 faces and ~98% of the NURBS-dense jet engine peak under ~1–2 MiB of
// live scratch, and a bump allocator (no per-allocation free) consumes the
// *cumulative* not peak size within a face, so we give generous headroom.
// Per-thread × a handful of workers, so a few MiB each is cheap.
constexpr std::size_t kDefaultScratchCapacity = 8u * 1024u * 1024u;

/**
 * A single-threaded bump allocator with a heap-spill slow path.
 *
 * allocate() returns aligned storage from the fixed backing buffer; when a
 * request doesn't fit, it spills to the heap and records the event. reset()
 * rewinds the bump offset for reuse and frees every spilled block. Individual
 * deallocations are no-ops — memory is reclaimed wholesale at reset(), which
 * is the per-face boundary.
 */
class ScratchArena {
 public:
  explicit ScratchArena(std::size_t capacity = kDefaultScratchCapacity)
      : capacity_(capacity),
        buffer_(static_cast<std::uint8_t*>(::operator new(capacity))) {}

  ~ScratchArena() {
    freeSpills();
    ::operator delete(buffer_);
  }

  ScratchArena(const ScratchArena&) = delete;
  ScratchArena& operator=(const ScratchArena&) = delete;

  /**
   * Return `bytes` of storage aligned to `align` (a power of two). Draws from
   * the bump buffer when it fits, otherwise spills to the heap.
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
    // Align the absolute address, not the offset: operator new only guarantees
    // max_align_t on the buffer base, so an over-aligned request (align >
    // buffer alignment) must be satisfied against the real pointer.
    const std::uintptr_t bufAddr = reinterpret_cast<std::uintptr_t>(buffer_);
    const std::uintptr_t aligned = (bufAddr + offset_ + (align - 1)) & ~(align - 1);
    const std::size_t base = static_cast<std::size_t>(aligned - bufAddr);
    if (base <= capacity_ && bytes <= capacity_ - base) {
      offset_ = base + bytes;
      if (offset_ > highWater_) {
        highWater_ = offset_;
      }
      return buffer_ + base;
    }
    // Spill path: oversized (or arena-exhausted) request goes to the heap and
    // lives until reset(). Counted so the wiring step can quantify the tail.
    void* p = ::operator new(bytes);
    spills_.push_back(p);
    ++spillCount_;
    spillBytes_ += bytes;
    return p;
  }

  /** Rewind to the base and release spilled blocks. Call per face. */
  void reset() {
    offset_ = 0;
    freeSpills();
  }

  /** Peak bump-buffer bytes used since construction (excludes spills). */
  std::size_t highWater() const { return highWater_; }

  /** Total heap spills since construction. */
  std::uint64_t spillCount() const { return spillCount_; }

  /** Total bytes that went through the spill path since construction. */
  std::uint64_t spillBytes() const { return spillBytes_; }

  /** Backing-buffer capacity in bytes. */
  std::size_t capacity() const { return capacity_; }

 private:
  void freeSpills() {
    for (void* p : spills_) {
      ::operator delete(p);
    }
    spills_.clear();
  }

  std::size_t capacity_;
  std::uint8_t* buffer_;
  std::size_t offset_ = 0;
  std::size_t highWater_ = 0;
  std::vector<void*> spills_;
  std::uint64_t spillCount_ = 0;
  std::uint64_t spillBytes_ = 0;
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
 * RAII guard that rewinds a scratch arena when it goes out of scope — the
 * per-face boundary. Defaults to the calling thread's arena.
 */
class ScratchArenaScope {
 public:
  explicit ScratchArenaScope(ScratchArena& arena = ThreadScratchArena())
      : arena_(arena) {}
  ~ScratchArenaScope() { arena_.reset(); }

  ScratchArenaScope(const ScratchArenaScope&) = delete;
  ScratchArenaScope& operator=(const ScratchArenaScope&) = delete;

  ScratchArena& arena() { return arena_; }

 private:
  ScratchArena& arena_;
};

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
