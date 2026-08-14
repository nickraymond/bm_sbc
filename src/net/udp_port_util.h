#pragma once

/// @file udp_port_util.h
/// @brief Pure helpers for the UDP port device: endpoint parsing + rate
///        shaping.  No sockets, no clocks — fully unit-testable.

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// Max characters in a peer IP string (IPv4 dotted quad; room for IPv6
/// later without an ABI change).
#define UDP_ENDPOINT_IP_LEN 46

/// Parse "ip:port" (IPv4 dotted quad, decimal port 1–65535).
/// Validates the address with inet_pton.
/// @return true on success; *ip_out NUL-terminated, *port_out host order.
bool udp_endpoint_parse(const char *s, char *ip_out, size_t ip_sz,
                        uint16_t *port_out);

/// Token-bucket rate shaper, virtual-clock formulation (integer-exact).
///
/// next_send_ns is the virtual clock: the earliest monotonic time the next
/// byte may be sent.  A send "costs" bytes/rate seconds of virtual time;
/// idle periods accumulate at most burst_ns of credit.
typedef struct {
  uint64_t rate_bps;     ///< bytes per second; 0 = shaping disabled
  uint64_t burst_ns;     ///< max accumulated credit (burst window)
  uint64_t next_send_ns; ///< virtual clock (monotonic ns domain)
} UdpRateShaper;

/// Initialize the shaper.  rate_mbps in megabits/s; 0 disables shaping.
/// burst_ns of 0 selects the default (10 ms).
void udp_rate_shaper_init(UdpRateShaper *s, uint32_t rate_mbps,
                          uint64_t burst_ns);

/// Account for sending @p bytes at monotonic time @p now_ns.
/// @return nanoseconds the caller must wait BEFORE sending (0 = send now).
///         State is updated assuming the caller honors the wait.
uint64_t udp_rate_shaper_delay_ns(UdpRateShaper *s, size_t bytes,
                                  uint64_t now_ns);

#ifdef __cplusplus
}
#endif
