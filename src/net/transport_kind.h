#pragma once

/// @file transport_kind.h
/// @brief Transport selection enum + string parsing (pure, no dependencies).
///
/// The `transport` config key (TOML) / `--transport` CLI flag selects which
/// NetworkDevice backs the node.  Kept in its own translation unit so the
/// parser is unit-testable without linking the full stack.

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef enum {
  TransportVirtual = 0, ///< Unix SOCK_DGRAM virtual ports (default; CI bench)
  TransportUdp,         ///< UDP/IP port device (real-Ethernet bench hop)
  TransportSerial,      ///< uart_l2 gateway only (no local-IPC ports)
  TransportAdin,        ///< ADIN hardware driver (reserved: hardware-day)
} TransportKind;

/// Parse a transport name ("virtual" | "udp" | "serial" | "adin").
/// @return true on success and sets *out; false on unknown/NULL input.
bool transport_kind_parse(const char *s, TransportKind *out);

/// Canonical name for a TransportKind (never NULL).
const char *transport_kind_name(TransportKind kind);

#ifdef __cplusplus
}
#endif
