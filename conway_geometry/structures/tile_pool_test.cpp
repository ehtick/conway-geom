// Standalone sanity test for the resident tile pool primitive
// (structures/tile_pool.h). Builds with a plain host C++20 compiler — no
// emsdk, no genie — so the invariants can be checked quickly while iterating:
//
//   c++ -std=c++20 -O2 -DCONWAY_TILE_POOL_STANDALONE_TEST -o /tmp/tp_test
//       tile_pool_test.cpp  &&  /tmp/tp_test
//
// Exit 0 = all checks passed; non-zero = the first failed assertion.
//
// Guarded like scratch_arena_test.cpp so this file is an empty translation
// unit in the normal build (the genie ConwayCoreFiles glob pulls in
// conway_geometry/structures/**, and a stray main() would clash with the wasm
// module entry).
#ifdef CONWAY_TILE_POOL_STANDALONE_TEST

#include "tile_pool.h"

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

std::vector<std::byte> ramp(std::size_t n, std::uint8_t seed) {
  std::vector<std::byte> bytes(n);
  for (std::size_t i = 0; i < n; ++i) {
    bytes[i] = static_cast<std::byte>((seed + i) & 0xFF);
  }
  return bytes;
}

void testSizingAndRounding() {
  conway::TilePool pool(1050, 100);
  check(pool.totalChunks() == 10, "sizing: whole chunks from byte budget");
  check(pool.totalBytes() == 1000, "sizing: totalBytes");
  check(pool.chunkRound(250) == 300, "chunkRound rounds up");
  check(pool.chunkRound(300) == 300, "chunkRound exact");
  check(pool.bytesInUse() == 0, "empty pool has no use");
}

void testCommitReadRoundTrip() {
  conway::TilePool pool(1000, 100);

  // 250 bytes spans 3 chunks — the payload straddles chunk boundaries.
  const auto payload = ramp(250, 7);
  check(pool.commitAsset(1, payload.data(), payload.size()), "commit fits");
  check(pool.bytesInUse() == 300, "physical use is chunk-rounded");
  check(pool.byteSizeOf(1) == 250, "logical size preserved");
  check(pool.segmentCountOf(1) == 3, "segment count");

  std::vector<std::byte> out(250);
  pool.readAsset(1, out.data());
  check(out == payload, "gather read matches scatter commit");

  // Per-segment view re-assembles to the same payload.
  std::vector<std::byte> assembled;
  for (std::size_t seg = 0; seg < pool.segmentCountOf(1); ++seg) {
    auto [ptr, len] = pool.segmentOf(1, seg);
    assembled.insert(assembled.end(), ptr, ptr + len);
  }
  check(assembled == payload, "segment walk matches payload");
}

void testInterleavedChunksStayCorrect() {
  conway::TilePool pool(1000, 100);

  // Fragment the freelist: commit A and B, free A, commit C — C's chunks
  // interleave with B's. Data integrity must not depend on contiguity.
  const auto a = ramp(300, 1);
  const auto b = ramp(300, 2);
  const auto c = ramp(450, 3);

  check(pool.commitAsset(10, a.data(), a.size()), "commit A");
  check(pool.commitAsset(11, b.data(), b.size()), "commit B");
  check(pool.releaseAsset(10), "free A");
  check(pool.commitAsset(12, c.data(), c.size()), "commit C into A's holes");

  std::vector<std::byte> outB(300), outC(450);
  pool.readAsset(11, outB.data());
  pool.readAsset(12, outC.data());
  check(outB == b, "B intact after interleave");
  check(outC == c, "C correct across non-contiguous chunks");
}

void testRefcountSharing() {
  conway::TilePool pool(1000, 100);

  const auto payload = ramp(100, 9);
  check(!pool.retainAsset(42), "retain of absent asset says commit needed");
  check(pool.commitAsset(42, payload.data(), payload.size()), "commit");
  check(pool.retainAsset(42), "second holder retains");
  check(pool.refCountOf(42) == 2, "refcount 2");
  check(pool.bytesInUse() == 100, "shared asset stored once");

  check(!pool.releaseAsset(42), "first release keeps asset");
  check(pool.isResident(42), "still resident for second holder");
  check(pool.bytesInUse() == 100, "no premature free");

  check(pool.releaseAsset(42), "last release frees");
  check(!pool.isResident(42), "gone");
  check(pool.bytesInUse() == 0, "chunks returned");
}

void testAllOrNothingExhaustion() {
  conway::TilePool pool(300, 100);

  const auto big = ramp(200, 1);
  check(pool.commitAsset(1, big.data(), big.size()), "200 fits");

  const auto tooBig = ramp(150, 2);
  check(!pool.commitAsset(2, tooBig.data(), tooBig.size()),
        "150 needs 2 chunks, 1 free: refused");
  check(pool.freeChunks() == 1, "refusal changed nothing");
  check(pool.stats().failedCommits == 1, "failure counted");

  const auto exact = ramp(100, 3);
  check(pool.commitAsset(3, exact.data(), exact.size()), "exact fit still ok");
  check(pool.freeChunks() == 0, "full");
}

void testChurnStaysBounded() {
  conway::TilePool pool(500, 100);

  // 200 rounds of fill/evict: the region never grows and the high-water
  // mark is the budget by construction.
  for (int round = 0; round < 200; ++round) {
    const auto a = ramp(300, static_cast<std::uint8_t>(round));
    const auto b = ramp(200, static_cast<std::uint8_t>(round + 1));

    check(pool.commitAsset(1, a.data(), a.size()), "churn commit A");
    check(pool.commitAsset(2, b.data(), b.size()), "churn commit B");
    check(pool.bytesInUse() == 500, "churn at budget");

    std::vector<std::byte> out(300);
    pool.readAsset(1, out.data());
    check(out == a, "churn payload intact");

    check(pool.releaseAsset(1), "churn free A");
    check(pool.releaseAsset(2), "churn free B");
    check(pool.bytesInUse() == 0, "churn drained");
  }

  check(pool.stats().commits == 400, "all commits counted");
  check(pool.totalChunks() == 5, "pool never grew");
}

}  // namespace

int main() {
  testSizingAndRounding();
  testCommitReadRoundTrip();
  testInterleavedChunksStayCorrect();
  testRefcountSharing();
  testAllOrNothingExhaustion();
  testChurnStaysBounded();

  if (g_failures == 0) {
    std::printf("tile_pool_test: all checks passed\n");
    return 0;
  }

  std::fprintf(stderr, "tile_pool_test: %d failure(s)\n", g_failures);
  return 1;
}

#endif  // CONWAY_TILE_POOL_STANDALONE_TEST
