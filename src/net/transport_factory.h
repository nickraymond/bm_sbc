#pragma once

/// @file transport_factory.h
/// @brief Network-device factory: builds the node's NetworkDevice from config.
///
/// Extracted from runtime.cpp so that transport selection is data
/// (`transport =` key / `--transport` flag) instead of hardwired
/// construction.  The same binary runs every transport; no #ifdef.
///
/// Composition rules (unchanged from the original runtime.cpp logic):
///   - The base device is selected by TransportKind.
///   - A non-empty uart_path composes the gateway wrapper (uart_l2 serial
///     link on the top port) over the base device.
///   - `serial` is the uart-only degenerate case: base device with zero
///     peers + gateway wrapper (uart_path required).
///
/// Constraint carried from gateway_device_get(): every base device must
/// expose its callbacks struct via `dev.callbacks = &<singleton>.callbacks`
/// (the gateway copies the device by value and shares that pointer).

#include "network_device.h"
#include "transport_kind.h"
#include "udp_port_device.h"
#include "virtual_port_device.h"

typedef struct {
  /// Which base device to build.
  TransportKind kind;

  /// Base config for virtual/serial transports (node id, socket dir, peers).
  /// Also carries own_node_id for every other transport.
  VirtualPortCfg vpc;

  /// Base config for the udp transport (listen endpoint, peers, rate limit).
  UdpPortCfg udp;

  /// Serial device path ("" = no gateway wrapper).  Required for `serial`.
  const char *uart_path;

  /// Serial baud rate (ignored for CDC devices; termios still applied).
  int baud_rate;
} TransportFactoryCfg;

/// Build the NetworkDevice described by @p cfg.
///
/// @param cfg  Factory configuration (borrowed; copied where needed).
/// @param out  Receives the constructed device on success.
///
/// @return 0 on success, non-zero on failure (details via bm_log_error).
int transport_factory_create(const TransportFactoryCfg *cfg,
                             NetworkDevice *out);
