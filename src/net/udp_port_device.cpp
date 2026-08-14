#include "udp_port_device.h"
#include "bm_config.h" // bm_debug()
#include "bm_log.h"    // bm_log_warn()
#include <arpa/inet.h> // inet_pton, htons
#include <errno.h>
#include <netinet/in.h> // sockaddr_in
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h> // struct timeval (SO_RCVTIMEO)
#include <time.h>     // clock_gettime, nanosleep
#include <unistd.h>   // close

// UDP transport: AF_INET SOCK_DGRAM — see design notes in udp_port_device.h
// and the full port/wire rationale in virtual_port_device.h.

// -------------------------------------------------------------------------
// Peer table data structure
// -------------------------------------------------------------------------

/// One slot in the peer table.  Slots are indexed 0–14; port numbers are
/// slot_index + 1 (i.e. port 1 == peers[0], port 15 == peers[14]).
typedef struct {
  /// Peer's resolved destination address (valid when active).
  struct sockaddr_in dst;

  /// Unbound SOCK_DGRAM fd used to sendto() the peer.
  /// -1 when the socket has not been opened yet (or has been closed).
  int send_fd;

  /// True when this slot contains a valid, configured peer.
  bool active;
} UdpPeerEntry;

/// All mutable state for one UdpPortDevice instance.
/// Module-level singleton for the same reason as VirtualPortState: the
/// trait functions receive only a void *self (or nothing for num_ports).
typedef struct {
  // ----- peer table -----
  UdpPeerEntry peers[VIRTUAL_PORT_MAX_PEERS];

  // ----- own receive socket -----
  int recv_fd;
  struct sockaddr_in listen_addr;

  // ----- RX thread -----
  pthread_t rx_thread;
  bool rx_running;

  // ----- lock (protects peers[], recv_fd, rx_running, enabled, shaper,
  //             stats, callbacks) -----
  pthread_mutex_t lock;

  // ----- device identity -----
  uint64_t own_node_id;

  // ----- TX rate shaper -----
  UdpRateShaper shaper;

  // ----- counters -----
  UdpPortStats stats;

  // ----- device state -----
  bool enabled;

  // ----- bm_core callbacks (populated by bm_l2_init) -----
  NetworkDeviceCallbacks callbacks;
} UdpPortState;

static UdpPortState g_udp_state;

static uint64_t monotonic_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

// -------------------------------------------------------------------------
// num_ports()
// -------------------------------------------------------------------------

/// Returns the fixed maximum number of ports (one per peer slot).
/// MUST stay the constant 15 — gateway_device_get() derives its serial
/// port from this while gateway_uart_rx_cb hardcodes GATEWAY_UART_PORT;
/// returning the real peer count silently kills the serial link.
static uint8_t udp_num_ports(void) {
  return (uint8_t)VIRTUAL_PORT_MAX_PEERS;
}

// -------------------------------------------------------------------------
// RX thread
// -------------------------------------------------------------------------

/// Background thread: recvfrom() loop with 1-second SO_RCVTIMEO timeout.
/// Reads datagrams, validates length + ingress-port byte, and dispatches
/// the frame payload to callbacks.receive().
static void *udp_rx_thread(void *arg) {
  UdpPortState *s = (UdpPortState *)arg;
  uint8_t buf[VIRTUAL_PORT_MAX_DGRAM_LEN + 1]; // +1 detects oversize
  while (1) {
    pthread_mutex_lock(&s->lock);
    bool running = s->rx_running;
    int fd = s->recv_fd;
    pthread_mutex_unlock(&s->lock);
    if (!running || fd < 0) {
      break;
    }
#ifdef __linux__
    // MSG_TRUNC: report the true datagram length even when it exceeds the
    // buffer, so the oversize log line carries the real size.
    ssize_t n = recvfrom(fd, buf, sizeof(buf), MSG_TRUNC, NULL, NULL);
#else
    ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, NULL, NULL);
#endif
    if (n < 0) {
      // EAGAIN/EWOULDBLOCK = SO_RCVTIMEO fired — check rx_running and loop.
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      break; // EBADF or other fatal error — exit thread
    }
    uint8_t port_num = VIRTUAL_PORT_DGRAM_PORT(buf);
    if ((size_t)n > VIRTUAL_PORT_MAX_DGRAM_LEN) {
      // REV-14 backstop: 1514 is the network-wide max frame size.
      bm_log_warn("udp_rx: oversize datagram dropped (len=%zd, "
                  "ingress port=%u, max=%d)",
                  n, (unsigned)port_num, VIRTUAL_PORT_MAX_DGRAM_LEN);
      pthread_mutex_lock(&s->lock);
      s->stats.rx_drop_oversize++;
      pthread_mutex_unlock(&s->lock);
      continue;
    }
    if ((size_t)n < VIRTUAL_PORT_MIN_DGRAM_LEN) {
      pthread_mutex_lock(&s->lock);
      s->stats.rx_drop_short++;
      pthread_mutex_unlock(&s->lock);
      continue;
    }
    if (port_num < 1 || port_num > VIRTUAL_PORT_MAX_PEERS) {
      pthread_mutex_lock(&s->lock);
      s->stats.rx_drop_bad_port++;
      pthread_mutex_unlock(&s->lock);
      continue;
    }
    uint8_t *frame = VIRTUAL_PORT_DGRAM_FRAME_PTR(buf);
    size_t frame_len = VIRTUAL_PORT_FRAME_LEN((size_t)n);
    // Snapshot callback pointer under lock; invoke outside lock.
    pthread_mutex_lock(&s->lock);
    void (*rcv)(uint8_t, uint8_t *, size_t) = s->callbacks.receive;
    s->stats.rx_frames++;
    s->stats.rx_bytes += frame_len;
    pthread_mutex_unlock(&s->lock);
    if (rcv) {
      rcv(port_num, frame, frame_len);
    }
  }
  return NULL;
}

// -------------------------------------------------------------------------
// enable() / disable()
// -------------------------------------------------------------------------

/// Bind the receive socket, open send sockets for each peer, and start the
/// RX thread.
static BmErr udp_enable(void *self) {
  UdpPortState *s = (UdpPortState *)self;
  pthread_mutex_lock(&s->lock);
  if (s->enabled) {
    pthread_mutex_unlock(&s->lock);
    return BmOK;
  }

  // Create the receive socket.
  int rfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (rfd < 0) {
    pthread_mutex_unlock(&s->lock);
    bm_debug("udp_enable: socket() failed errno=%d\n", errno);
    return BmEIO;
  }
  // 1-second receive timeout so the RX thread periodically wakes and checks
  // rx_running instead of blocking forever on recvfrom().
  struct timeval tv = {1, 0};
  setsockopt(rfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  int one = 1;
  setsockopt(rfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  if (bind(rfd, (struct sockaddr *)&s->listen_addr,
           sizeof(s->listen_addr)) < 0) {
    close(rfd);
    pthread_mutex_unlock(&s->lock);
    bm_debug("udp_enable: bind(port %u) failed errno=%d\n",
             (unsigned)ntohs(s->listen_addr.sin_port), errno);
    return BmEIO;
  }
  s->recv_fd = rfd;
  s->rx_running = true;

  // Open unbound send sockets for configured peers (non-fatal on failure;
  // retry_negotiation() re-opens them).
  for (int i = 0; i < VIRTUAL_PORT_MAX_PEERS; i++) {
    if (!s->peers[i].active || s->peers[i].send_fd >= 0) {
      continue;
    }
    int sfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sfd >= 0) {
      s->peers[i].send_fd = sfd;
    }
  }

  // Start the RX thread.
  if (pthread_create(&s->rx_thread, NULL, udp_rx_thread, s) != 0) {
    s->rx_running = false;
    close(rfd);
    s->recv_fd = -1;
    pthread_mutex_unlock(&s->lock);
    bm_debug("udp_enable: pthread_create() failed\n");
    return BmEIO;
  }
  s->enabled = true;
  pthread_mutex_unlock(&s->lock);

  // Do NOT call link_change here.  The L2 thread starts its renegotiation
  // timers concurrently with this call, so firing link_change now would race
  // with bm_l2_start_renegotiate_check (between ll_item_add and
  // bm_timer_start).  udp_retry_negotiation() reports each configured peer
  // once the 100 ms renegotiation timer fires — by which point the L2
  // thread is stable.  (Same design as virtual_port_device; independently
  // hit live in this project's INTERIM 2a bring-up.)
  return BmOK;
}

/// Stop the RX thread, close all sockets, and fire link_change(idx, false)
/// for every previously-active peer.
static BmErr udp_disable(void *self) {
  UdpPortState *s = (UdpPortState *)self;
  pthread_mutex_lock(&s->lock);
  if (!s->enabled) {
    pthread_mutex_unlock(&s->lock);
    return BmOK;
  }
  s->enabled = false;
  s->rx_running = false;
  int rfd = s->recv_fd;
  s->recv_fd = -1;
  void (*lc)(uint8_t, bool) = s->callbacks.link_change;
  pthread_mutex_unlock(&s->lock);

  // Close recv_fd; the 1-second SO_RCVTIMEO guarantees the RX thread exits
  // within ≤1 second even if close() doesn't interrupt recvfrom().
  if (rfd >= 0) {
    close(rfd);
  }
  pthread_join(s->rx_thread, NULL);

  // Close all peer send sockets.
  pthread_mutex_lock(&s->lock);
  for (int i = 0; i < VIRTUAL_PORT_MAX_PEERS; i++) {
    if (s->peers[i].send_fd >= 0) {
      close(s->peers[i].send_fd);
      s->peers[i].send_fd = -1;
    }
  }
  pthread_mutex_unlock(&s->lock);

  // Notify L2 that all ports are down.
  if (lc) {
    for (int i = 0; i < VIRTUAL_PORT_MAX_PEERS; i++) {
      if (s->peers[i].active) {
        lc((uint8_t)i, false);
      }
    }
  }
  return BmOK;
}

// -------------------------------------------------------------------------
// enable_port() / disable_port()
// -------------------------------------------------------------------------

/// Open the send socket for one peer (port 1–15) and notify L2 it is up.
static BmErr udp_enable_port(void *self, uint8_t port_num) {
  UdpPortState *s = (UdpPortState *)self;
  if (port_num < 1 || port_num > VIRTUAL_PORT_MAX_PEERS) {
    return BmEINVAL;
  }
  int idx = port_num - 1;
  pthread_mutex_lock(&s->lock);
  if (!s->peers[idx].active) {
    pthread_mutex_unlock(&s->lock);
    return BmEINVAL;
  }
  if (s->peers[idx].send_fd < 0) {
    int sfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sfd >= 0) {
      s->peers[idx].send_fd = sfd;
    }
  }
  void (*lc)(uint8_t, bool) = s->callbacks.link_change;
  pthread_mutex_unlock(&s->lock);
  if (lc) {
    lc((uint8_t)idx, true);
  }
  return BmOK;
}

/// Close the send socket for one peer and notify L2 it is down.
static BmErr udp_disable_port(void *self, uint8_t port_num) {
  UdpPortState *s = (UdpPortState *)self;
  if (port_num < 1 || port_num > VIRTUAL_PORT_MAX_PEERS) {
    return BmEINVAL;
  }
  int idx = port_num - 1;
  pthread_mutex_lock(&s->lock);
  if (!s->peers[idx].active) {
    pthread_mutex_unlock(&s->lock);
    return BmEINVAL;
  }
  if (s->peers[idx].send_fd >= 0) {
    close(s->peers[idx].send_fd);
    s->peers[idx].send_fd = -1;
  }
  void (*lc)(uint8_t, bool) = s->callbacks.link_change;
  pthread_mutex_unlock(&s->lock);
  if (lc) {
    lc((uint8_t)idx, false);
  }
  return BmOK;
}

// -------------------------------------------------------------------------
// send()
// -------------------------------------------------------------------------

/// Apply the TX rate shaper for one datagram of @p wire_len bytes.
/// Sleeps outside the lock; deliberately delays the calling (L2 TX) thread —
/// see the header note on where sustained-overload drops surface.
static void udp_shape(UdpPortState *s, size_t wire_len) {
  pthread_mutex_lock(&s->lock);
  uint64_t wait_ns =
      udp_rate_shaper_delay_ns(&s->shaper, wire_len, monotonic_ns());
  s->stats.tx_shaper_wait_ns += wait_ns;
  pthread_mutex_unlock(&s->lock);
  if (wait_ns == 0) {
    return;
  }
  struct timespec req = {(time_t)(wait_ns / 1000000000ULL),
                         (long)(wait_ns % 1000000000ULL)};
  while (nanosleep(&req, &req) != 0 && errno == EINTR) {
  }
}

/// Send a raw L2 frame on one port (1–15) or flood all active peers (port 0).
/// Wire format: [1-byte egress-port-num | frame bytes].
static BmErr udp_send(void *self, uint8_t *data, size_t length, uint8_t port) {
  UdpPortState *s = (UdpPortState *)self;
  if (!data || length == 0) {
    return BmEINVAL;
  }
  if (length > VIRTUAL_PORT_MAX_FRAME_LEN) {
    // 1514 is the network-wide max frame size (enforced at senders; this
    // is the local assert half of the REV-14 decision).
    bm_log_warn("udp_send: oversize frame rejected (len=%zu, port=%u)",
                length, (unsigned)port);
    pthread_mutex_lock(&s->lock);
    s->stats.tx_drop_oversize++;
    pthread_mutex_unlock(&s->lock);
    return BmEINVAL;
  }
  if (port > VIRTUAL_PORT_MAX_PEERS) {
    return BmEINVAL;
  }

  uint8_t dgram[VIRTUAL_PORT_MAX_DGRAM_LEN];
  BmErr err = BmOK;

  if (port == 0) {
    // Flood: deliver to every active peer, tagging each datagram with the
    // sender's egress port number so the receiver knows the ingress port.
    for (int i = 0; i < VIRTUAL_PORT_MAX_PEERS; i++) {
      pthread_mutex_lock(&s->lock);
      bool active = s->peers[i].active;
      int sfd = s->peers[i].send_fd;
      struct sockaddr_in dst = s->peers[i].dst;
      pthread_mutex_unlock(&s->lock);
      if (!active || sfd < 0) {
        continue;
      }
      size_t dlen = VIRTUAL_PORT_DGRAM_LEN(length);
      udp_shape(s, dlen);
      dgram[VIRTUAL_PORT_DGRAM_PORT_OFF] = (uint8_t)(i + 1);
      memcpy(VIRTUAL_PORT_DGRAM_FRAME_PTR(dgram), data, length);
      if (sendto(sfd, dgram, dlen, 0, (struct sockaddr *)&dst, sizeof(dst)) <
          0) {
        bm_debug("udp_send: flood peer %d failed errno=%d\n", i + 1, errno);
        err = BmEIO;
      } else {
        pthread_mutex_lock(&s->lock);
        s->stats.tx_frames++;
        s->stats.tx_bytes += length;
        pthread_mutex_unlock(&s->lock);
      }
    }
  } else {
    // Unicast to one peer.
    int idx = port - 1;
    pthread_mutex_lock(&s->lock);
    bool active = s->peers[idx].active;
    int sfd = s->peers[idx].send_fd;
    struct sockaddr_in dst = s->peers[idx].dst;
    pthread_mutex_unlock(&s->lock);
    if (!active || sfd < 0) {
      return BmEINVAL;
    }
    size_t dlen = VIRTUAL_PORT_DGRAM_LEN(length);
    udp_shape(s, dlen);
    dgram[VIRTUAL_PORT_DGRAM_PORT_OFF] = port;
    memcpy(VIRTUAL_PORT_DGRAM_FRAME_PTR(dgram), data, length);
    if (sendto(sfd, dgram, dlen, 0, (struct sockaddr *)&dst, sizeof(dst)) <
        0) {
      bm_debug("udp_send: unicast port %d failed errno=%d\n", port, errno);
      err = BmEIO;
    } else {
      pthread_mutex_lock(&s->lock);
      s->stats.tx_frames++;
      s->stats.tx_bytes += length;
      pthread_mutex_unlock(&s->lock);
    }
  }
  return err;
}

// -------------------------------------------------------------------------
// retry_negotiation()
// -------------------------------------------------------------------------

/// Report a configured peer as linked (and re-open its send socket if
/// needed).  UDP is connectionless — there is no equivalent of VPD's
/// on-disk socket-path probe — so "reachable" is the configured-peer
/// check: any configured slot renegotiates successfully on the first
/// 100 ms L2 timer tick.  This is where link_change(idx, true) fires
/// (never from enable(); see the comment there).
/// @param port_num  1-based port number.
/// @param renegotiated  set to true if a send socket is now open.
static BmErr udp_retry_negotiation(void *self, uint8_t port_num,
                                   bool *renegotiated) {
  UdpPortState *s = (UdpPortState *)self;
  if (renegotiated) {
    *renegotiated = false;
  }
  if (port_num < 1 || port_num > VIRTUAL_PORT_MAX_PEERS) {
    return BmEINVAL;
  }
  int idx = port_num - 1;
  pthread_mutex_lock(&s->lock);
  UdpPeerEntry *p = &s->peers[idx];
  if (!p->active) {
    pthread_mutex_unlock(&s->lock);
    return BmOK; // no peer configured — not an error
  }
  bool connected = false;
  if (p->send_fd < 0) {
    int sfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sfd >= 0) {
      p->send_fd = sfd;
      connected = true;
    }
  } else {
    connected = true;
  }
  if (connected && renegotiated) {
    *renegotiated = true;
  }
  void (*lc)(uint8_t, bool) = s->callbacks.link_change;
  pthread_mutex_unlock(&s->lock);
  // Fire link_change(idx, true) so l2 stops the renegotiation timer and
  // marks the port enabled in enabled_ports_mask.
  if (connected && lc) {
    lc((uint8_t)idx, true);
  }
  return BmOK;
}

// -------------------------------------------------------------------------
// port_stats() / handle_interrupt() stubs
// -------------------------------------------------------------------------

static BmErr udp_port_stats(void *self, uint8_t port_index, void *stats) {
  (void)self;
  (void)port_index;
  (void)stats;
  return BmOK;
}

static BmErr udp_handle_interrupt(void *self) {
  (void)self;
  return BmOK;
}

// -------------------------------------------------------------------------
// NetworkDeviceTrait + udp_port_device_get()
// -------------------------------------------------------------------------

static const NetworkDeviceTrait s_udp_trait = {
    udp_send,        udp_enable,       udp_disable,
    udp_enable_port, udp_disable_port, udp_retry_negotiation,
    udp_num_ports,   udp_port_stats,   udp_handle_interrupt,
};

NetworkDevice udp_port_device_get(const UdpPortCfg *cfg) {
  // 15-peer cap (same policy + warning as VPD).
  uint8_t num_peers = cfg->num_peers;
  if (num_peers > VIRTUAL_PORT_MAX_PEERS) {
    bm_log_warn("udp: peer count %u exceeds cap %d", (unsigned)num_peers,
                VIRTUAL_PORT_MAX_PEERS);
    num_peers = VIRTUAL_PORT_MAX_PEERS;
  }

  memset(&g_udp_state, 0, sizeof(g_udp_state));
  pthread_mutex_init(&g_udp_state.lock, NULL);

  // Set sentinel -1 for all fds (0 is valid for stdin).
  g_udp_state.recv_fd = -1;
  for (int i = 0; i < VIRTUAL_PORT_MAX_PEERS; i++) {
    g_udp_state.peers[i].send_fd = -1;
  }

  g_udp_state.own_node_id = cfg->own_node_id;
  udp_rate_shaper_init(&g_udp_state.shaper, cfg->rate_mbps, 0);

  // Bind address.
  g_udp_state.listen_addr.sin_family = AF_INET;
  g_udp_state.listen_addr.sin_port = htons(cfg->listen_port);
  if (inet_pton(AF_INET, cfg->listen_ip, &g_udp_state.listen_addr.sin_addr) !=
      1) {
    bm_log_error("udp: invalid listen ip '%s' — falling back to 0.0.0.0",
                 cfg->listen_ip);
    g_udp_state.listen_addr.sin_addr.s_addr = INADDR_ANY;
  }

  // Populate peer table (peers[i] ↔ port i+1).
  for (int i = 0; i < (int)num_peers; i++) {
    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons(cfg->peers[i].port);
    if (inet_pton(AF_INET, cfg->peers[i].ip, &dst.sin_addr) != 1) {
      bm_log_error("udp: invalid peer %d address '%s' — slot left inactive",
                   i + 1, cfg->peers[i].ip);
      continue;
    }
    g_udp_state.peers[i].dst = dst;
    g_udp_state.peers[i].active = true;
    g_udp_state.peers[i].send_fd = -1;
  }

  // Point dev.callbacks at the singleton's callbacks struct so bm_l2_init's
  // writes are visible to the trait functions (gateway composition contract).
  NetworkDevice dev;
  dev.self = &g_udp_state;
  dev.trait = &s_udp_trait;
  dev.callbacks = &g_udp_state.callbacks;
  return dev;
}

void udp_port_device_get_stats(UdpPortStats *out) {
  if (!out) {
    return;
  }
  pthread_mutex_lock(&g_udp_state.lock);
  *out = g_udp_state.stats;
  pthread_mutex_unlock(&g_udp_state.lock);
}
