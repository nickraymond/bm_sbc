#pragma once

/// @file udp_port_device.h
/// @brief Per-peer port network device over UDP/IP (real-Ethernet bench hop).
///
/// Derived member-for-member from virtual_port_device (see its header for
/// the full design rationale).  Only the transport differs:
///
///   virtual_port_device            udp_port_device
///   ---------------------------    ------------------------------
///   AF_UNIX SOCK_DGRAM             AF_INET SOCK_DGRAM
///   sockaddr_un, socket path       sockaddr_in, ip:port
///   peer = node-id-derived path    peer = configured ip:port
///
/// Everything else is deliberately identical and load-bearing:
///
///  - Wire format: [1-byte egress port (1–15)][L2 frame ≤ 1514 B] — the
///    VIRTUAL_PORT_DGRAM_* macros are reused verbatim so the two devices
///    can never diverge on framing or MTU (1514 is the network-wide max
///    frame size; oversize RX datagrams are dropped with a logged length
///    + ingress port).
///  - num_ports() returns the constant 15 (NOT the configured peer count):
///    gateway_device_get() computes its serial port from this value while
///    gateway_uart_rx_cb hardcodes GATEWAY_UART_PORT — a non-constant
///    count silently breaks the serial link's auto-link-up.
///  - Link-up fires from retry_negotiation() on the L2 renegotiation
///    timer, never from enable() (races L2 thread startup).  UDP has no
///    on-disk socket to probe, so "peer reachable" is the configured-peer
///    check: a configured slot renegotiates successfully on the first
///    timer tick.
///  - One bound receive socket + per-peer unbound send sockets; RX thread
///    with a 1 s SO_RCVTIMEO.
///  - Module-level singleton; NetworkDevice.callbacks points at the
///    singleton's callbacks struct (gateway composition contract).
///
/// Additions over virtual_port_device:
///
///  - Token-bucket TX rate shaper (10BASE-T1L emulation), default 10 Mbps,
///    configurable, 0 = off.  Shaping delays the L2 TX thread; sustained
///    overload therefore overflows L2's 32-deep event queue upstream —
///    that loss is counted by bm_l2_get_tx_queue_drops() (bench bm_core
///    observability patch), not here.
///  - Local TX/RX frame/byte/drop counters via udp_port_device_get_stats().

#include "network_device.h"
#include "udp_port_util.h"
#include "virtual_port_device.h" // wire-format macros + peer-count constants

/// Configured peer endpoint (slot index i ↔ port number i+1).
typedef struct {
  char ip[UDP_ENDPOINT_IP_LEN]; ///< IPv4 dotted quad
  uint16_t port;                ///< UDP port, host order
} UdpPeerCfg;

/// Static configuration for udp_port_device_get().
/// All fields are copied; the struct need not outlive the call.
typedef struct {
  uint64_t own_node_id; ///< For logs only (addressing is ip:port).

  char listen_ip[UDP_ENDPOINT_IP_LEN]; ///< Bind address ("0.0.0.0" typical)
  uint16_t listen_port;                ///< Bind port (bench default 22000)

  /// TX rate limit in Mbit/s; 0 disables shaping.  Bench default 10.
  uint32_t rate_mbps;

  /// Peers in port-slot order (peers[0] → port 1, …).  One slot beyond the
  /// cap so a 16th entry triggers the same truncation warning as VPD.
  UdpPeerCfg peers[VIRTUAL_PORT_CFG_MAX_PEERS];
  uint8_t num_peers;
} UdpPortCfg;

/// Device-local counters (monotonic since process start).
typedef struct {
  uint64_t tx_frames;
  uint64_t tx_bytes;         ///< L2 frame bytes (excl. the 1-byte header)
  uint64_t rx_frames;
  uint64_t rx_bytes;         ///< L2 frame bytes (excl. the 1-byte header)
  uint64_t tx_shaper_wait_ns; ///< Cumulative shaper sleep time
  uint32_t tx_drop_oversize; ///< send() rejected frames > 1514 B
  uint32_t rx_drop_oversize; ///< datagrams > 1515 B (logged, REV-14 backstop)
  uint32_t rx_drop_short;    ///< datagrams below min frame size
  uint32_t rx_drop_bad_port; ///< ingress-port byte outside 1–15
} UdpPortStats;

/// Build and return a NetworkDevice backed by UDP/IP.
/// Same contract as virtual_port_device_get(): the device is not yet
/// enabled; bm_l2_init() invokes trait->enable().
NetworkDevice udp_port_device_get(const UdpPortCfg *cfg);

/// Snapshot the device counters into *out (thread-safe).
void udp_port_device_get_stats(UdpPortStats *out);
