#ifdef CONWAY_ALLOC_TELEMETRY

#include "alloc_telemetry.h"

#include <atomic>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <malloc.h>  // malloc_usable_size (dlmalloc/emscripten provide it)

// The real allocator, provided by wasm-ld's --wrap.
extern "C" {
void* __real_malloc(size_t size);
void* __real_calloc(size_t count, size_t size);
void* __real_realloc(void* ptr, size_t size);
void __real_free(void* ptr);
}

namespace {

// ---- per-thread, active only inside an AllocTelemetryScope ----------------

struct ThreadFaceStats {
  bool active = false;
  uint64_t allocCalls = 0;
  uint64_t liveBytes = 0;
  uint64_t peakBytes = 0;
};

thread_local ThreadFaceStats tls;

// Active allocation-attribution site for this thread (see AllocTagScope).
thread_local conway::AllocSite g_currentSite = conway::AllocSite::Other;

// Per-site in-scope allocation counts (process-wide).
std::atomic<uint64_t>
    g_siteCounts[static_cast<int>(conway::AllocSite::Count)] = {};

inline void onAlloc(void* ptr) {
  if (!tls.active || ptr == nullptr) {
    return;
  }
  tls.allocCalls += 1;
  tls.liveBytes += malloc_usable_size(ptr);
  if (tls.liveBytes > tls.peakBytes) {
    tls.peakBytes = tls.liveBytes;
  }
  g_siteCounts[static_cast<int>(g_currentSite)].fetch_add(
      1, std::memory_order_relaxed);
}

inline void onFree(void* ptr) {
  if (!tls.active || ptr == nullptr) {
    return;
  }
  uint64_t size = malloc_usable_size(ptr);
  // Frees of memory allocated before the scope began can underflow the
  // in-scope live counter; clamp — we are sizing the scratch arena, and
  // pre-scope memory would not live in it.
  tls.liveBytes = (size > tls.liveBytes) ? 0 : tls.liveBytes - size;
}

// ---- process-wide aggregates, merged on scope exit -------------------------

// log2 buckets of per-face peak bytes: bucket i covers [2^i, 2^(i+1)).
// 48 buckets reaches 256 TiB, comfortably beyond any wasm32 value.
constexpr int kBuckets = 48;

std::atomic<uint64_t> g_faces{0};
std::atomic<uint64_t> g_totalAllocCalls{0};
std::atomic<uint64_t> g_maxAllocCallsPerFace{0};
std::atomic<uint64_t> g_totalPeakBytes{0};
std::atomic<uint64_t> g_maxPeakBytes{0};
std::atomic<uint64_t> g_peakHistogram[kBuckets] = {};
// Bytes still live when the scope closes — allocations that escape the face
// (mesh growth/commits). These would NOT live in a per-face scratch arena;
// reporting them separately decomposes scratch (arena-sizable) from commit
// (reservation-aware accumulation's job).
std::atomic<uint64_t> g_totalEscapedBytes{0};
std::atomic<uint64_t> g_maxEscapedBytes{0};

inline int bucketFor(uint64_t bytes) {
  int b = 0;
  while (bytes > 1 && b < kBuckets - 1) {
    bytes >>= 1;
    b += 1;
  }
  return b;
}

inline void atomicMax(std::atomic<uint64_t>& target, uint64_t value) {
  uint64_t cur = target.load(std::memory_order_relaxed);
  while (cur < value &&
         !target.compare_exchange_weak(cur, value, std::memory_order_relaxed)) {
  }
}

}  // namespace

// ---- the wrappers -----------------------------------------------------------

extern "C" {

void* __wrap_malloc(size_t size) {
  void* ptr = __real_malloc(size);
  onAlloc(ptr);
  return ptr;
}

void* __wrap_calloc(size_t count, size_t size) {
  void* ptr = __real_calloc(count, size);
  onAlloc(ptr);
  return ptr;
}

void* __wrap_realloc(void* ptr, size_t size) {
  onFree(ptr);
  void* out = __real_realloc(ptr, size);
  onAlloc(out);
  return out;
}

void __wrap_free(void* ptr) {
  onFree(ptr);
  __real_free(ptr);
}

}  // extern "C"

namespace conway {

AllocTelemetryScope::AllocTelemetryScope() {
  if (!tls.active) {
    outermost_ = true;
    tls.active = true;
    tls.allocCalls = 0;
    tls.liveBytes = 0;
    tls.peakBytes = 0;
  }
}

AllocTelemetryScope::~AllocTelemetryScope() {
  if (!outermost_) {
    return;
  }
  tls.active = false;
  g_faces.fetch_add(1, std::memory_order_relaxed);
  g_totalAllocCalls.fetch_add(tls.allocCalls, std::memory_order_relaxed);
  g_totalPeakBytes.fetch_add(tls.peakBytes, std::memory_order_relaxed);
  atomicMax(g_maxAllocCallsPerFace, tls.allocCalls);
  atomicMax(g_maxPeakBytes, tls.peakBytes);
  g_peakHistogram[bucketFor(tls.peakBytes)].fetch_add(
      1, std::memory_order_relaxed);
  g_totalEscapedBytes.fetch_add(tls.liveBytes, std::memory_order_relaxed);
  atomicMax(g_maxEscapedBytes, tls.liveBytes);
}

void DumpAllocTelemetry(const char* label) {
  uint64_t faces = g_faces.load();
  if (faces == 0) {
    fprintf(stderr, "[alloc-telemetry] %s: no scoped faces recorded\n",
            label != nullptr ? label : "");
    return;
  }
  fprintf(stderr,
          "[alloc-telemetry] %s: faces=%" PRIu64 " allocCalls(total=%" PRIu64
          " avg=%.1f max=%" PRIu64 ") peakBytes(avg=%" PRIu64 " max=%" PRIu64
          ")\n",
          label != nullptr ? label : "", faces, g_totalAllocCalls.load(),
          static_cast<double>(g_totalAllocCalls.load()) /
              static_cast<double>(faces),
          g_maxAllocCallsPerFace.load(), g_totalPeakBytes.load() / faces,
          g_maxPeakBytes.load());
  fprintf(stderr,
          "[alloc-telemetry]   escapedBytes(avg=%" PRIu64 " max=%" PRIu64
          ") -- allocations outliving the face (mesh commits)\n",
          g_totalEscapedBytes.load() / faces, g_maxEscapedBytes.load());
  // Per-callsite attribution of the in-scope allocations (AllocTagScope).
  static const char* const kSiteNames[] = {
      "other",         "earcut",       "cdt",         "surface_eval",
      "nurbs_inverse", "tri_bounds",   "tri_bspline", "tri_cylinder",
      "tri_sphere",    "tri_toroidal", "tri_conical", "tri_revolution",
      "tri_extrusion"};
  for (int i = 0; i < static_cast<int>(AllocSite::Count); ++i) {
    uint64_t count = g_siteCounts[i].load();
    if (count == 0) {
      continue;
    }
    fprintf(stderr,
            "[alloc-telemetry]   site %-13s: %12" PRIu64 " (%5.1f%%, %.1f/face)\n",
            kSiteNames[i], count,
            100.0 * static_cast<double>(count) /
                static_cast<double>(g_totalAllocCalls.load()),
            static_cast<double>(count) / static_cast<double>(faces));
  }
  // Cumulative histogram: what fraction of faces fit an arena of 2^(i+1)?
  uint64_t running = 0;
  for (int i = 0; i < kBuckets; ++i) {
    uint64_t count = g_peakHistogram[i].load();
    if (count == 0) {
      continue;
    }
    running += count;
    fprintf(stderr,
            "[alloc-telemetry]   peak<%8" PRIu64 "KiB: %10" PRIu64
            " faces (%6.2f%% cumulative)\n",
            (uint64_t(1) << (i + 1)) / 1024, count,
            100.0 * static_cast<double>(running) / static_cast<double>(faces));
  }
}

AllocTagScope::AllocTagScope(AllocSite site) : previous_(g_currentSite) {
  g_currentSite = site;
}

AllocTagScope::~AllocTagScope() { g_currentSite = previous_; }

void ResetAllocTelemetry() {
  g_faces.store(0);
  g_totalAllocCalls.store(0);
  g_maxAllocCallsPerFace.store(0);
  g_totalPeakBytes.store(0);
  g_maxPeakBytes.store(0);
  g_totalEscapedBytes.store(0);
  g_maxEscapedBytes.store(0);
  for (int i = 0; i < kBuckets; ++i) {
    g_peakHistogram[i].store(0);
  }
  for (int i = 0; i < static_cast<int>(AllocSite::Count); ++i) {
    g_siteCounts[i].store(0);
  }
}

}  // namespace conway

#endif  // CONWAY_ALLOC_TELEMETRY
