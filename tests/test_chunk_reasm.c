/// @file test_chunk_reasm.c
/// @brief Unit tests for the camera/stream chunk reassembler (S17).

#include "chunk_reasm.h"

#include <stdio.h>
#include <string.h>

static int g_pass = 0;
static int g_fail = 0;

#define ASSERT_TRUE(cond, msg)                                                 \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf("  FAIL: %s\n", msg);                                             \
      g_fail++;                                                                \
    } else {                                                                   \
      g_pass++;                                                                \
    }                                                                          \
  } while (0)

static size_t mk_chunk(uint8_t *out, uint32_t seq, uint16_t idx,
                       uint16_t count, const uint8_t *data, uint16_t plen) {
  memcpy(out, &seq, 4);
  memcpy(out + 4, &idx, 2);
  memcpy(out + 6, &count, 2);
  memcpy(out + 8, &plen, 2);
  memcpy(out + CHUNK_HDR_LEN, data, plen);
  return CHUNK_HDR_LEN + plen;
}

int main(void) {
  static uint8_t buf[8192];
  static uint8_t chunk[2048];
  static uint8_t jpeg[4000];
  for (size_t i = 0; i < sizeof(jpeg); i++) {
    jpeg[i] = (uint8_t)(i * 37 + 11);
  }
  chunk_reasm_t r;

  // Single-chunk frame completes immediately, byte-exact.
  chunk_reasm_init(&r, buf, sizeof(buf));
  size_t n = mk_chunk(chunk, 5, 0, 1, jpeg, 300);
  ASSERT_TRUE(chunk_reasm_feed(&r, chunk, n) == 300, "single chunk done");
  ASSERT_TRUE(memcmp(buf, jpeg, 300) == 0, "single chunk bytes");
  ASSERT_TRUE(r.frames_ok == 1 && r.frames_dropped == 0, "single ledger");

  // Three-chunk frame, in order.
  chunk_reasm_init(&r, buf, sizeof(buf));
  n = mk_chunk(chunk, 6, 0, 3, jpeg, 1390);
  ASSERT_TRUE(chunk_reasm_feed(&r, chunk, n) == 0, "3a incomplete");
  n = mk_chunk(chunk, 6, 1, 3, jpeg + 1390, 1390);
  ASSERT_TRUE(chunk_reasm_feed(&r, chunk, n) == 0, "3b incomplete");
  n = mk_chunk(chunk, 6, 2, 3, jpeg + 2780, 1220);
  ASSERT_TRUE(chunk_reasm_feed(&r, chunk, n) == 4000, "3c complete");
  ASSERT_TRUE(memcmp(buf, jpeg, 4000) == 0, "3-chunk bytes exact");

  // Gap in the middle drops the frame, next full frame recovers.
  chunk_reasm_init(&r, buf, sizeof(buf));
  n = mk_chunk(chunk, 7, 0, 3, jpeg, 1000);
  chunk_reasm_feed(&r, chunk, n);
  n = mk_chunk(chunk, 7, 2, 3, jpeg, 1000);   // idx 1 lost
  ASSERT_TRUE(chunk_reasm_feed(&r, chunk, n) == 0, "gap: no frame");
  ASSERT_TRUE(r.chunk_gaps == 1 && r.frames_dropped == 1, "gap counted");
  n = mk_chunk(chunk, 8, 0, 1, jpeg, 50);
  ASSERT_TRUE(chunk_reasm_feed(&r, chunk, n) == 50, "gap: resync on idx 0");

  // idx-0 restart mid-assembly abandons the old frame (counted).
  chunk_reasm_init(&r, buf, sizeof(buf));
  n = mk_chunk(chunk, 9, 0, 2, jpeg, 100);
  chunk_reasm_feed(&r, chunk, n);
  n = mk_chunk(chunk, 10, 0, 1, jpeg, 60);
  ASSERT_TRUE(chunk_reasm_feed(&r, chunk, n) == 60, "restart completes new");
  ASSERT_TRUE(r.frames_dropped == 1, "restart drop counted");

  // Wrong-seq continuation counted as gap.
  chunk_reasm_init(&r, buf, sizeof(buf));
  n = mk_chunk(chunk, 11, 0, 2, jpeg, 100);
  chunk_reasm_feed(&r, chunk, n);
  n = mk_chunk(chunk, 99, 1, 2, jpeg, 100);
  ASSERT_TRUE(chunk_reasm_feed(&r, chunk, n) == 0 && r.chunk_gaps == 1,
              "wrong seq counted");

  // Malformed: short payload, bad plen, idx >= count, count 0.
  chunk_reasm_init(&r, buf, sizeof(buf));
  ASSERT_TRUE(chunk_reasm_feed(&r, chunk, 5) == 0, "short payload");
  n = mk_chunk(chunk, 12, 0, 1, jpeg, 100);
  ASSERT_TRUE(chunk_reasm_feed(&r, chunk, n + 1) == 0, "plen mismatch");
  n = mk_chunk(chunk, 12, 3, 2, jpeg, 100);
  ASSERT_TRUE(chunk_reasm_feed(&r, chunk, n) == 0, "idx >= count");
  n = mk_chunk(chunk, 12, 0, 0, jpeg, 100);
  ASSERT_TRUE(chunk_reasm_feed(&r, chunk, n) == 0, "count 0");
  ASSERT_TRUE(r.hdr_errors == 4, "hdr errors counted");
  ASSERT_TRUE(r.frames_ok == 0, "no frames from garbage");

  // Oversize frame rejected once it exceeds the buffer.
  static uint8_t small[256];
  chunk_reasm_init(&r, small, sizeof(small));
  n = mk_chunk(chunk, 13, 0, 2, jpeg, 200);
  ASSERT_TRUE(chunk_reasm_feed(&r, chunk, n) == 0, "oversize part 1");
  n = mk_chunk(chunk, 13, 1, 2, jpeg, 200);
  ASSERT_TRUE(chunk_reasm_feed(&r, chunk, n) == 0 && r.oversize == 1,
              "oversize rejected + counted");

  printf("chunk_reasm: %d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
