#include "transport_kind.h"
#include <string.h>

bool transport_kind_parse(const char *s, TransportKind *out) {
  if (!s || !out) {
    return false;
  }
  if (strcmp(s, "virtual") == 0) {
    *out = TransportVirtual;
    return true;
  }
  if (strcmp(s, "udp") == 0) {
    *out = TransportUdp;
    return true;
  }
  if (strcmp(s, "serial") == 0) {
    *out = TransportSerial;
    return true;
  }
  if (strcmp(s, "adin") == 0) {
    *out = TransportAdin;
    return true;
  }
  return false;
}

const char *transport_kind_name(TransportKind kind) {
  switch (kind) {
  case TransportVirtual:
    return "virtual";
  case TransportUdp:
    return "udp";
  case TransportSerial:
    return "serial";
  case TransportAdin:
    return "adin";
  }
  return "unknown";
}
