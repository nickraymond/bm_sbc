/// @file app_main.cpp
/// @brief Rate/drop measurement app for the UDP transport (bench Stage 1).
///
/// One binary, two roles selected by environment variable:
///
///   S15_ROLE=rx  (default)  Subscribe to the stream topic; print a
///                           receiver-side throughput ledger once per
///                           second (RX_STAT) — receiver-side counting is
///                           the ground truth for delivered rate.
///   S15_ROLE=tx             Publish payload chunks at a target offered
///                           rate for a fixed duration, counting every
///                           bm_pub() error.  BmENOMEM returns are L2
///                           TX-queue drops surfacing synchronously; the
///                           bm_l2_get_tx_queue_drops() counter is the
///                           same events seen at the queue itself (it
///                           also catches drops from traffic this app
///                           didn't send, e.g. forwarded frames).
///
/// Environment knobs (tx role):
///   S15_MBPS     offered rate in Mbit/s of payload bytes (default 8)
///   S15_SECONDS  publish duration (default 30)
///   S15_PAYLOAD  payload bytes per publish (default 1024, max 1400)
///
/// Output markers (grepped by scripts/demo docs):
///   RX_STAT / TX_STAT       once per second
///   TX_DONE                 final totals
///   TX_VERDICT DROPS_OBSERVED enomem=<n> l2_drops=<n>
///   TX_VERDICT NO_DROPS

#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include "bm_log.h"

extern "C" {
#include "device.h" // node_id()
#include "l2.h"     // bm_l2_get_tx_queue_drops()
}

#include "pubsub.h" // bm_sub, bm_pub, BM_COMMON_PUB_SUB_VERSION

static const char *k_topic = "s15/stream";

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

static bool s_role_tx = false;
static double s_mbps = 8.0;
static unsigned s_seconds = 30;
static size_t s_payload_len = 1024;

static struct timespec s_start;
static bool s_started = false;

static uint64_t s_last_stat_ms = 0;

// rx counters
static uint64_t s_rx_msgs = 0;
static uint64_t s_rx_bytes = 0;
static uint64_t s_rx_msgs_win = 0;
static uint64_t s_rx_bytes_win = 0;

// tx counters
static uint64_t s_tx_ok = 0;
static uint64_t s_tx_enomem = 0;
static uint64_t s_tx_other_err = 0;
static uint64_t s_tx_bytes = 0;
static bool s_tx_done = false;
static uint8_t s_payload[1400];

static double now_sec(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)(ts.tv_sec - s_start.tv_sec) +
         (double)(ts.tv_nsec - s_start.tv_nsec) / 1e9;
}

// ---------------------------------------------------------------------------
// RX role
// ---------------------------------------------------------------------------

static void on_stream_msg(uint64_t node, const char * /*topic*/,
                          uint16_t /*topic_len*/, const uint8_t * /*data*/,
                          uint16_t data_len, uint8_t /*type*/,
                          uint8_t /*version*/) {
  (void)node;
  s_rx_msgs++;
  s_rx_bytes += data_len;
  s_rx_msgs_win++;
  s_rx_bytes_win += data_len;
}

// ---------------------------------------------------------------------------
// App contract
// ---------------------------------------------------------------------------

void setup(void) {
  const char *role = getenv("S15_ROLE");
  s_role_tx = role && strcmp(role, "tx") == 0;

  const char *mbps = getenv("S15_MBPS");
  if (mbps) {
    s_mbps = atof(mbps);
  }
  const char *secs = getenv("S15_SECONDS");
  if (secs) {
    s_seconds = (unsigned)atoi(secs);
  }
  const char *plen = getenv("S15_PAYLOAD");
  if (plen) {
    long v = atol(plen);
    if (v >= 16 && v <= (long)sizeof(s_payload)) {
      s_payload_len = (size_t)v;
    }
  }

  clock_gettime(CLOCK_MONOTONIC, &s_start);
  s_started = true;

  if (s_role_tx) {
    memset(s_payload, 0xA5, sizeof(s_payload));
    bm_log_info("[%016" PRIx64 "] stream_bench TX: %.2f Mbit/s offered, "
                "%u s, %zu B payloads, topic=%s",
                node_id(), s_mbps, s_seconds, s_payload_len, k_topic);
  } else {
    bm_sub(k_topic, on_stream_msg);
    bm_log_info("[%016" PRIx64 "] stream_bench RX: subscribed to %s",
                node_id(), k_topic);
  }
}

void loop(void) {
  if (!s_started) {
    return;
  }
  double t = now_sec();

  // Give the stack a settling window before pushing traffic (neighbor
  // discovery + link-up happen on the 100 ms renegotiation timer).
  const double warmup_sec = 3.0;

  if (s_role_tx && !s_tx_done && t > warmup_sec) {
    // Quota pacing: total payload bytes owed by now at the offered rate.
    double active = t - warmup_sec;
    if (active > (double)s_seconds) {
      active = (double)s_seconds;
    }
    uint64_t quota = (uint64_t)(active * s_mbps * 125000.0);
    while (s_tx_bytes < quota) {
      BmErr err = bm_pub(k_topic, s_payload, (uint16_t)s_payload_len, 0,
                         BM_COMMON_PUB_SUB_VERSION);
      s_tx_bytes += s_payload_len; // offered bytes, delivered or not
      if (err == BmOK) {
        s_tx_ok++;
      } else if (err == BmENOMEM) {
        s_tx_enomem++;
      } else {
        s_tx_other_err++;
      }
    }
    if (t - warmup_sec >= (double)s_seconds) {
      s_tx_done = true;
      // The publish loop above BLOCKS in bm_pub when the L2 TX queue is
      // full (10 ms enqueue timeout >> per-frame service time), so an
      // offered rate above the wire limit shows up as wall-clock stretch
      // (achieved < offered), not as drops.  wall_s makes that visible.
      double wall = now_sec() - warmup_sec;
      double offered_mbps = (double)s_tx_bytes * 8.0 / 1e6 / s_seconds;
      double achieved_mbps = (double)s_tx_bytes * 8.0 / 1e6 / wall;
      uint32_t l2_drops = bm_l2_get_tx_queue_drops();
      printf("TX_DONE offered_mbps=%.2f achieved_mbps=%.2f wall_s=%.1f "
             "payload_bytes=%" PRIu64 " pub_ok=%" PRIu64
             " pub_enomem=%" PRIu64 " pub_other=%" PRIu64
             " l2_drops=%" PRIu32 "\n",
             offered_mbps, achieved_mbps, wall, s_tx_bytes, s_tx_ok,
             s_tx_enomem, s_tx_other_err, l2_drops);
      if (s_tx_enomem > 0 || l2_drops > 0) {
        printf("TX_VERDICT DROPS_OBSERVED enomem=%" PRIu64
               " l2_drops=%" PRIu32 "\n",
               s_tx_enomem, l2_drops);
      } else {
        printf("TX_VERDICT NO_DROPS\n");
      }
    }
  }

  // Once-per-second stat lines.
  uint64_t ms = (uint64_t)(t * 1000.0);
  if (ms - s_last_stat_ms >= 1000) {
    s_last_stat_ms = ms;
    if (s_role_tx) {
      if (!s_tx_done) {
        printf("TX_STAT t=%.0f ok=%" PRIu64 " enomem=%" PRIu64
               " l2_drops=%" PRIu32 "\n",
               t, s_tx_ok, s_tx_enomem, bm_l2_get_tx_queue_drops());
      }
    } else {
      double win_mbps = (double)s_rx_bytes_win * 8.0 / 1e6;
      // tx_drops on an rx-role node = L2 FORWARD-path drops (the L2
      // thread enqueueing into its own full queue) — the transit ledger
      // on a pass-through node like the S16 Light Pi.
      printf("RX_STAT t=%.0f mbps=%.2f msgs=%" PRIu64 " total_mb=%.2f "
             "total_msgs=%" PRIu64 " rx_drops=%" PRIu32 " tx_drops=%" PRIu32
             "\n",
             t, win_mbps, s_rx_msgs_win, (double)s_rx_bytes / 1e6, s_rx_msgs,
             bm_l2_get_rx_queue_drops(), bm_l2_get_tx_queue_drops());
      s_rx_bytes_win = 0;
      s_rx_msgs_win = 0;
    }
  }
}
