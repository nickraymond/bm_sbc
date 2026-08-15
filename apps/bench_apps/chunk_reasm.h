/// @file chunk_reasm.h
/// @brief Reassembler for the camera/stream chunk format (S17 BUILD-4).
///
/// Chunk payload format (contract: ADIN_SPI_OpenMV
/// firmware/bm_he/src/camera_svc.h -- change in lockstep or not at all):
/// a 10-byte little-endian header
///   frame_seq u32 | chunk_idx u16 | chunk_count u16 | payload_len u16
/// followed by payload_len JPEG bytes. Chunks of one frame are published
/// in order by a single publisher (the camera node) and pub/sub is
/// fire-and-forget, so this reassembler is strictly sequential: any gap
/// drops the frame under assembly and resyncs on the next chunk_idx 0.
/// Every drop is counted -- the counters are the loss ledger (D21).
///
/// Header-only, pure C, no OS calls: unit-tested by tests/test_chunk_reasm.c.

#ifndef CHUNK_REASM_H
#define CHUNK_REASM_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define CHUNK_HDR_LEN 10u

typedef struct {
  uint8_t *buf;    // caller-owned assembly buffer
  size_t cap;
  int open;        // a frame is under assembly
  uint32_t frame_seq;
  uint16_t next_idx;
  uint16_t count;
  size_t filled;
  // ledger
  uint32_t frames_ok;
  uint32_t frames_dropped;  // assemblies abandoned (gap/restart/error)
  uint32_t chunk_gaps;      // chunks that didn't fit the open assembly
  uint32_t hdr_errors;      // malformed headers / length mismatches
  uint32_t oversize;        // frames larger than the buffer
} chunk_reasm_t;

static inline void chunk_reasm_init(chunk_reasm_t *r, uint8_t *buf,
                                    size_t cap) {
  memset(r, 0, sizeof(*r));
  r->buf = buf;
  r->cap = cap;
}

static inline void chunk_reasm_abandon_(chunk_reasm_t *r) {
  if (r->open) {
    r->frames_dropped++;
    r->open = 0;
    r->filled = 0;
  }
}

/// Feed one pub/sub payload. Returns the completed JPEG length (bytes in
/// r->buf) or 0.
static inline size_t chunk_reasm_feed(chunk_reasm_t *r,
                                      const uint8_t *payload, size_t len) {
  if (len < CHUNK_HDR_LEN) {
    r->hdr_errors++;
    return 0;
  }
  uint32_t seq;
  uint16_t idx, count, plen;
  memcpy(&seq, payload, 4);          // little-endian host assumed (Pi/x86)
  memcpy(&idx, payload + 4, 2);
  memcpy(&count, payload + 6, 2);
  memcpy(&plen, payload + 8, 2);
  if (count == 0 || idx >= count || (size_t)plen != len - CHUNK_HDR_LEN) {
    r->hdr_errors++;
    chunk_reasm_abandon_(r);
    return 0;
  }

  if (idx == 0) {
    chunk_reasm_abandon_(r);         // restart: previous tail isn't coming
    if ((size_t)count * plen > r->cap && count > 1) {
      // conservative upper bound; exact check happens as chunks land
    }
    r->open = 1;
    r->frame_seq = seq;
    r->count = count;
    r->next_idx = 0;
    r->filled = 0;
  } else if (!r->open || seq != r->frame_seq || idx != r->next_idx) {
    r->chunk_gaps++;
    chunk_reasm_abandon_(r);
    return 0;
  }

  if (r->filled + plen > r->cap) {
    r->oversize++;
    chunk_reasm_abandon_(r);
    return 0;
  }
  memcpy(r->buf + r->filled, payload + CHUNK_HDR_LEN, plen);
  r->filled += plen;
  r->next_idx++;

  if (r->next_idx == r->count) {
    size_t done = r->filled;
    r->open = 0;
    r->filled = 0;
    r->frames_ok++;
    return done;
  }
  return 0;
}

#endif // CHUNK_REASM_H
