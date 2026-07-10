// Standalone sanity test for the AFTP scratch arena primitive
// (structures/scratch_arena.h). Builds with a plain host C++20 compiler — no
// emsdk, no genie — so the invariants can be checked quickly while iterating:
//
//   c++ -std=c++20 -O2 -o /tmp/sa_test scratch_arena_test.cpp && /tmp/sa_test
//
// Exit 0 = all checks passed; non-zero = the first failed assertion.
//
// Guarded by CONWAY_SCRATCH_ARENA_STANDALONE_TEST so this file is an empty
// translation unit in the normal build (the genie ConwayCoreFiles glob pulls in
// conway_geometry/structures/**, and a stray main() would clash with the wasm
// module entry). Build the standalone test with the define set.
#ifdef CONWAY_SCRATCH_ARENA_STANDALONE_TEST

#include "scratch_arena.h"

#include <cstdint>
#include <cstdio>
#include <vector>

namespace {

int g_failures = 0;

void check(bool cond, const char* what) {
  if (!cond) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_failures;
  }
}

// Bump allocations are aligned and non-overlapping; the offset advances.
void testBumpAndAlignment() {
  conway::ScratchArena arena(4096);

  void* a = arena.allocate(1, 1);
  void* b = arena.allocate(8, 8);
  void* c = arena.allocate(64, 64);

  check(a != nullptr && b != nullptr && c != nullptr, "non-null allocations");
  check((reinterpret_cast<std::uintptr_t>(b) & 7u) == 0, "b is 8-aligned");
  check((reinterpret_cast<std::uintptr_t>(c) & 63u) == 0, "c is 64-aligned");
  check(b >= static_cast<std::uint8_t*>(a) + 1, "b after a");
  check(arena.highWater() > 0, "high-water advanced");
  check(arena.spillCount() == 0, "no spills for in-capacity requests");
}

// reset() rewinds so storage is reused (same base pointer after reset).
void testReset() {
  conway::ScratchArena arena(4096);
  void* first = arena.allocate(128, 16);
  arena.reset();
  void* second = arena.allocate(128, 16);
  check(first == second, "reset rewinds to the same base");
}

// Oversized requests spill to the heap, are counted, and freed at reset().
void testSpill() {
  conway::ScratchArena arena(256);
  void* big = arena.allocate(4096, 16);  // exceeds capacity -> spill
  check(big != nullptr, "spill returns usable storage");
  check(arena.spillCount() == 1, "one spill counted");
  check(arena.spillBytes() >= 4096, "spill bytes counted");
  // Exhaust-then-spill: fill the buffer, next small alloc also spills.
  conway::ScratchArena tight(64);
  (void)tight.allocate(64, 1);
  (void)tight.allocate(1, 1);
  check(tight.spillCount() == 1, "spill when arena exhausted");
  arena.reset();
  check(arena.spillCount() == 1, "reset does not zero lifetime spill count");
}

// The STL allocator adapter routes a std::vector through a given arena.
void testAllocatorAdapter() {
  conway::ScratchArena arena(1u << 20);
  using ArenaVec = std::vector<int, conway::ScratchAllocator<int>>;

  ArenaVec v{conway::ScratchAllocator<int>(arena)};
  for (int i = 0; i < 1000; ++i) {
    v.push_back(i);
  }
  long sum = 0;
  for (int x : v) {
    sum += x;
  }
  check(sum == 499500, "vector-through-arena holds correct values");
  check(arena.highWater() > 0, "arena served the vector");

  // Rebinding to a different element type must carry the same arena.
  conway::ScratchAllocator<int> ai(arena);
  conway::ScratchAllocator<double> ad(ai);
  check(ad.arena_ == &arena, "rebound allocator keeps the arena");
}

// The scope guard rewinds its arena on destruction (per-face boundary).
void testScope() {
  conway::ScratchArena arena(4096);
  void* base;
  {
    conway::ScratchArenaScope scope(arena);
    base = scope.arena().allocate(512, 16);
  }
  void* afterScope = arena.allocate(512, 16);
  check(base == afterScope, "scope guard rewound the arena on exit");
}

// Nested scopes: an inner scope must free ONLY its own scratch, never the
// outer scope's still-live allocations (the use-after-reset hazard that a
// reset-to-zero scope would cause when tessellation helpers nest).
void testNesting() {
  conway::ScratchArena arena(4096);
  conway::ScratchArenaScope outer(arena);
  int* outerBuf = static_cast<int*>(arena.allocate(sizeof(int) * 4, alignof(int)));
  outerBuf[0] = 0xABCD;
  void* innerFirst;
  {
    conway::ScratchArenaScope inner(arena);
    innerFirst = arena.allocate(256, 16);
    (void)arena.allocate(256, 16);
  }
  // outer's data survives the inner scope...
  check(outerBuf[0] == 0xABCD, "outer scratch survives inner scope exit");
  // ...and the inner region is reused after the inner scope closes.
  void* reused = arena.allocate(256, 16);
  check(reused == innerFirst, "inner scope freed only its own region");
}

// Marker/rewind restores spill bookkeeping too: spills taken after a mark are
// freed on rewind, but the lifetime spill count is not rolled back.
void testMarkerSpill() {
  conway::ScratchArena arena(64);
  auto m = arena.mark();
  (void)arena.allocate(4096, 16);  // spill after the mark
  check(arena.spillCount() == 1, "spill counted");
  arena.rewind(m);
  check(arena.spillCount() == 1, "rewind keeps lifetime spill count");
  // A fresh spill after rewind reuses the freed slot bookkeeping cleanly.
  (void)arena.allocate(4096, 16);
  check(arena.spillCount() == 2, "post-rewind spill counted");
}

}  // namespace

int main() {
  testBumpAndAlignment();
  testReset();
  testSpill();
  testAllocatorAdapter();
  testScope();
  testNesting();
  testMarkerSpill();

  if (g_failures == 0) {
    std::printf("scratch_arena_test: all checks passed\n");
    return 0;
  }
  std::fprintf(stderr, "scratch_arena_test: %d failure(s)\n", g_failures);
  return 1;
}

#endif  // CONWAY_SCRATCH_ARENA_STANDALONE_TEST
