/// @file test_udp_port_util.c
/// @brief Unit tests for udp endpoint parsing and the TX rate shaper.

#include "udp_port_util.h"

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

#define ASSERT_EQ_U64(a, b, msg)                                               \
  do {                                                                         \
    unsigned long long va = (unsigned long long)(a);                           \
    unsigned long long vb = (unsigned long long)(b);                           \
    if (va != vb) {                                                            \
      printf("  FAIL: %s (got %llu, expected %llu)\n", msg, va, vb);           \
      g_fail++;                                                                \
    } else {                                                                   \
      g_pass++;                                                                \
    }                                                                          \
  } while (0)

// ---- endpoint parsing -----------------------------------------------------

static void test_parse_ok(void) {
  char ip[UDP_ENDPOINT_IP_LEN];
  uint16_t port = 0;
  ASSERT_TRUE(udp_endpoint_parse("10.42.0.2:22000", ip, sizeof(ip), &port),
              "valid endpoint parses");
  ASSERT_TRUE(strcmp(ip, "10.42.0.2") == 0, "ip extracted");
  ASSERT_EQ_U64(port, 22000, "port extracted");

  ASSERT_TRUE(udp_endpoint_parse("0.0.0.0:1", ip, sizeof(ip), &port),
              "wildcard ip + min port");
  ASSERT_EQ_U64(port, 1, "min port value");

  ASSERT_TRUE(udp_endpoint_parse("127.0.0.1:65535", ip, sizeof(ip), &port),
              "max port");
  ASSERT_EQ_U64(port, 65535, "max port value");
}

static void test_parse_bad(void) {
  char ip[UDP_ENDPOINT_IP_LEN];
  uint16_t port = 0;
  ASSERT_TRUE(!udp_endpoint_parse("10.42.0.2", ip, sizeof(ip), &port),
              "missing port rejected");
  ASSERT_TRUE(!udp_endpoint_parse(":22000", ip, sizeof(ip), &port),
              "missing ip rejected");
  ASSERT_TRUE(!udp_endpoint_parse("10.42.0.2:", ip, sizeof(ip), &port),
              "empty port rejected");
  ASSERT_TRUE(!udp_endpoint_parse("10.42.0.2:0", ip, sizeof(ip), &port),
              "port 0 rejected");
  ASSERT_TRUE(!udp_endpoint_parse("10.42.0.2:65536", ip, sizeof(ip), &port),
              "port 65536 rejected");
  ASSERT_TRUE(!udp_endpoint_parse("10.42.0.2:22a", ip, sizeof(ip), &port),
              "non-numeric port rejected");
  ASSERT_TRUE(!udp_endpoint_parse("10.42.0.999:22000", ip, sizeof(ip), &port),
              "bad ipv4 rejected");
  ASSERT_TRUE(!udp_endpoint_parse("nereus000:22000", ip, sizeof(ip), &port),
              "hostname rejected (static ip bench)");
  ASSERT_TRUE(!udp_endpoint_parse(NULL, ip, sizeof(ip), &port),
              "NULL input rejected");
  char tiny[4];
  ASSERT_TRUE(!udp_endpoint_parse("10.42.0.2:22000", tiny, sizeof(tiny), &port),
              "too-small ip buffer rejected");
}

// ---- rate shaper ----------------------------------------------------------

#define MS (1000ULL * 1000)
#define SEC (1000ULL * MS)

static void test_shaper_disabled(void) {
  UdpRateShaper s;
  udp_rate_shaper_init(&s, 0, 0);
  ASSERT_EQ_U64(udp_rate_shaper_delay_ns(&s, 100000, 1 * SEC), 0,
                "rate 0 = never delays");
}

static void test_shaper_10mbps_steady(void) {
  // 10 Mbps = 1,250,000 B/s.  A 1515 B datagram costs 1,212,000 ns.
  UdpRateShaper s;
  udp_rate_shaper_init(&s, 10, 0);
  uint64_t t0 = 100 * SEC;

  // First send after long idle: full burst credit, no delay.
  ASSERT_EQ_U64(udp_rate_shaper_delay_ns(&s, 1515, t0), 0, "first send free");

  // Immediately sending the burst window's worth (10 ms @ 10 Mbps =
  // 12,500 B ≈ 8.25 datagrams) stays free; beyond it, delays appear.
  uint64_t wait = 0;
  int sent = 1;
  while (wait == 0 && sent < 100) {
    wait = udp_rate_shaper_delay_ns(&s, 1515, t0);
    sent++;
  }
  ASSERT_TRUE(sent >= 8 && sent <= 10, "burst window ~8 datagrams @10ms");
  ASSERT_TRUE(wait > 0, "post-burst send delayed");

  // Steady state: every further immediate send adds one datagram-cost of
  // delay (1,212,000 ns per 1515 B).
  uint64_t w1 = udp_rate_shaper_delay_ns(&s, 1515, t0);
  uint64_t w2 = udp_rate_shaper_delay_ns(&s, 1515, t0);
  ASSERT_EQ_U64(w2 - w1, 1212000, "steady-state cost per 1515 B datagram");
}

static void test_shaper_credit_capped(void) {
  UdpRateShaper s;
  udp_rate_shaper_init(&s, 10, 0);
  // Exhaust credit at t0.
  uint64_t t0 = 10 * SEC;
  for (int i = 0; i < 50; i++) {
    (void)udp_rate_shaper_delay_ns(&s, 1515, t0);
  }
  // After a long idle gap, credit is capped at burst_ns (10 ms), not the
  // whole gap: a full re-burst is allowed but no more.
  uint64_t t1 = t0 + 60 * SEC;
  int free_sends = 0;
  while (udp_rate_shaper_delay_ns(&s, 1515, t1) == 0 && free_sends < 100) {
    free_sends++;
  }
  ASSERT_TRUE(free_sends >= 8 && free_sends <= 10,
              "idle credit capped at burst window");
}

static void test_shaper_2mbps_stream(void) {
  // The BENCHSPEC video-stream case: 2 Mbps offered through a 10 Mbps
  // shaper must never delay when paced at the offered rate.
  UdpRateShaper s;
  udp_rate_shaper_init(&s, 10, 0);
  // 2 Mbps = 250,000 B/s; 1412 B chunks every 5,648,000 ns.
  uint64_t t = 5 * SEC;
  bool any_wait = false;
  for (int i = 0; i < 1000; i++) {
    if (udp_rate_shaper_delay_ns(&s, 1412, t) != 0) {
      any_wait = true;
    }
    t += 5648000;
  }
  ASSERT_TRUE(!any_wait, "2 Mbps stream never shaped by 10 Mbps limiter");
}

int main(void) {
  printf("udp_port_util tests:\n");
  test_parse_ok();
  test_parse_bad();
  test_shaper_disabled();
  test_shaper_10mbps_steady();
  test_shaper_credit_capped();
  test_shaper_2mbps_stream();
  printf("%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
