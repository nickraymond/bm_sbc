#include "udp_port_util.h"

#include <arpa/inet.h> // inet_pton
#include <stdlib.h>    // strtoul
#include <string.h>

#define SHAPER_DEFAULT_BURST_NS (10ULL * 1000 * 1000) // 10 ms
#define NS_PER_SEC (1000000000ULL)

bool udp_endpoint_parse(const char *s, char *ip_out, size_t ip_sz,
                        uint16_t *port_out) {
  if (!s || !ip_out || !port_out || ip_sz == 0) {
    return false;
  }
  const char *colon = strrchr(s, ':');
  if (!colon || colon == s || colon[1] == '\0') {
    return false;
  }
  size_t ip_len = (size_t)(colon - s);
  if (ip_len >= ip_sz) {
    return false;
  }

  // Port: decimal, 1–65535, digits only.
  char *end = NULL;
  unsigned long port = strtoul(colon + 1, &end, 10);
  if (!end || *end != '\0' || port < 1 || port > 65535) {
    return false;
  }

  memcpy(ip_out, s, ip_len);
  ip_out[ip_len] = '\0';

  // Validate the address (IPv4 for the bench).
  struct in_addr dummy;
  if (inet_pton(AF_INET, ip_out, &dummy) != 1) {
    return false;
  }

  *port_out = (uint16_t)port;
  return true;
}

void udp_rate_shaper_init(UdpRateShaper *s, uint32_t rate_mbps,
                          uint64_t burst_ns) {
  memset(s, 0, sizeof(*s));
  // 1 Mbit/s = 125000 bytes/s.
  s->rate_bps = (uint64_t)rate_mbps * 125000ULL;
  s->burst_ns = burst_ns ? burst_ns : SHAPER_DEFAULT_BURST_NS;
}

uint64_t udp_rate_shaper_delay_ns(UdpRateShaper *s, size_t bytes,
                                  uint64_t now_ns) {
  if (s->rate_bps == 0) {
    return 0; // shaping disabled
  }
  // Virtual send time may lag `now` by at most burst_ns of earned credit.
  uint64_t floor_ns = (now_ns > s->burst_ns) ? now_ns - s->burst_ns : 0;
  uint64_t start = s->next_send_ns > floor_ns ? s->next_send_ns : floor_ns;
  uint64_t wait = start > now_ns ? start - now_ns : 0;
  uint64_t cost_ns = ((uint64_t)bytes * NS_PER_SEC) / s->rate_bps;
  s->next_send_ns = start + cost_ns;
  return wait;
}
