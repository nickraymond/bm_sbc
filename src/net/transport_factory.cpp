#include "transport_factory.h"
#include "bm_log.h"
#include "gateway_device.h"
#include "uart_l2_transport.h"
#include "virtual_port_device.h"

#include <string.h>

int transport_factory_create(const TransportFactoryCfg *cfg,
                             NetworkDevice *out) {
  if (!cfg || !out) {
    return 1;
  }

  bool gateway_mode = cfg->uart_path && cfg->uart_path[0] != '\0';

  switch (cfg->kind) {
  case TransportVirtual: {
    NetworkDevice base = virtual_port_device_get(&cfg->vpc);
    if (gateway_mode) {
      int uart_err = uart_l2_transport_init(cfg->uart_path, cfg->baud_rate,
                                            gateway_uart_rx_cb, nullptr);
      if (uart_err != 0) {
        bm_log_error("UART transport init failed");
        return 1;
      }
      *out = gateway_device_get(&base);
    } else {
      *out = base;
    }
    return 0;
  }

  case TransportSerial: {
    // uart-only node: base device carries zero peers; the gateway wrapper
    // provides the single (serial) link.
    if (!gateway_mode) {
      bm_log_error("transport=serial requires uart-device");
      return 1;
    }
    VirtualPortCfg vpc = cfg->vpc;
    if (vpc.num_peers != 0) {
      bm_log_warn("transport=serial ignores %u configured peers",
                  (unsigned)vpc.num_peers);
      vpc.num_peers = 0;
    }
    NetworkDevice base = virtual_port_device_get(&vpc);
    int uart_err = uart_l2_transport_init(cfg->uart_path, cfg->baud_rate,
                                          gateway_uart_rx_cb, nullptr);
    if (uart_err != 0) {
      bm_log_error("UART transport init failed");
      return 1;
    }
    *out = gateway_device_get(&base);
    return 0;
  }

  case TransportUdp: {
    NetworkDevice base = udp_port_device_get(&cfg->udp);
    if (gateway_mode) {
      int uart_err = uart_l2_transport_init(cfg->uart_path, cfg->baud_rate,
                                            gateway_uart_rx_cb, nullptr);
      if (uart_err != 0) {
        bm_log_error("UART transport init failed");
        return 1;
      }
      *out = gateway_device_get(&base);
    } else {
      *out = base;
    }
    return 0;
  }

  case TransportAdin: {
    bm_log_error("transport=adin is reserved for ADIN hardware bring-up");
    return 1;
  }
  }

  bm_log_error("unknown transport kind %d", (int)cfg->kind);
  return 1;
}
