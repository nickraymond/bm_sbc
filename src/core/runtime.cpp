#include "runtime.h"
#include "bm_config.h"
#include "bm_log.h"
#include "pcap_file_sink.h"
#include "platform_linux.h"
#include "timer_callback_handler.h"
#include "transport_factory.h"
#include "virtual_port_device.h"
// topology.h, bm_service.h, l2.h, pubsub.h have extern "C" guards.
// device.h, bm_ip.h, bcmp.h, middleware.h do not — wrap them explicitly.
#include "bm_service.h"
#include "l2.h"
#include "pubsub.h"
#include "topology.h"
extern "C" {
#include "bcmp.h"
#include "bm_ip.h"
#include "config_cbor_map_service.h"
#include "configuration.h"
#include "device.h"
#include "middleware.h"
#include "pcap.h"
#include "sys_info_service.h"
}
#include "git_sha.h"
#include "tomlc17.h"
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static const char *k_usage =
    "Usage: bm_sbc --init <toml> [options...]\n"
    "       bm_sbc --node-id <hex64> [options...]\n"
    "\n"
    "  --init       <path>    TOML init file (provides all settings below).\n"
    "  --node-id    <hex64>   This node's 64-bit Bristlemouth node ID.\n"
    "  --transport  <name>    Transport: virtual (default), udp, serial, adin.\n"
    "  --cfg-dir    <path>    Directory for config partition files.\n"
    "  --peer       <hex64>   A peer node ID; repeat up to 15 times.\n"
    "                         (16 peers triggers a truncation warning)\n"
    "  --socket-dir <path>    Unix socket directory (default: /tmp).\n"
    "  --uart       <device>  Serial device path for UART gateway mode.\n"
    "  --baud       <rate>    Baud rate for UART (default: 115200).\n"
    "  --pcap       <path>    Write captured L2 frames to a pcap file.\n"
    "\n"
    "  --log-dir    <path>    Log file directory (default: /var/log/bm_sbc).\n"
    "  --log-level  <level>   Minimum log level: "
    "trace/debug/info/warn/error/fatal\n"
    "                         (default: info).\n"
    "  --log-stdout           Also write logs to stdout.\n"
    "CLI flags override values from the init file.\n";

// App name passed in from main() (which is compiled per-app target and
// therefore has access to BM_SBC_APP_NAME).  Stored here so that
// bm_app_name can reference it at any time after init.
const char *bm_sbc_app_name_runtime = "bm_sbc";

/// Parse a hex64 string (with optional "0x"/"0X" prefix) into a uint64_t.
/// Returns true on success and sets *out.
static bool parse_hex64(const char *s, uint64_t *out) {
  if (!s || !*s) {
    return false;
  }
  char *end = NULL;
  *out = strtoull(s, &end, 16);
  return end && *end == '\0';
}

/// Parse a log level name string to BmSbcLogLevel.  Returns -1 on failure.
static int parse_log_level(const char *s) {
  if (strcmp(s, "trace") == 0)
    return BM_LOG_TRACE;
  if (strcmp(s, "debug") == 0)
    return BM_LOG_DEBUG;
  if (strcmp(s, "info") == 0)
    return BM_LOG_INFO;
  if (strcmp(s, "warn") == 0)
    return BM_LOG_WARN;
  if (strcmp(s, "error") == 0)
    return BM_LOG_ERROR;
  if (strcmp(s, "fatal") == 0)
    return BM_LOG_FATAL;
  return -1;
}

/// Load settings from a TOML init file.  Values are written into the
/// provided output parameters only when present in the file — callers
/// should pre-fill defaults before calling.
///
/// Uses the tomlc17 API: toml_parse_file_ex() / toml_get() / toml_free().
/// Strings returned by toml_get() point into the parsed document memory and
/// must NOT be free()'d individually — toml_free() releases everything.
static int load_init_file(const char *path, VirtualPortCfg *vpc,
                          bool *node_id_set, char *cfg_dir, size_t cfg_dir_sz,
                          char *transport_str, size_t transport_str_sz,
                          char *uart_path, size_t uart_path_sz, int *baud_rate,
                          char *pcap_path, size_t pcap_path_sz, char *log_dir,
                          size_t log_dir_sz, int *log_level, bool *log_stdout) {
  toml_result_t res = toml_parse_file_ex(path);
  if (!res.ok) {
    fprintf(stderr, "bm_sbc: TOML parse error in %s: %s\n", path, res.errmsg);
    return 1;
  }
  toml_datum_t root = res.toptab;

  // node-id (string, hex)
  toml_datum_t d = toml_get(root, "node-id");
  if (d.type == TOML_STRING) {
    if (!parse_hex64(d.u.s, &vpc->own_node_id)) {
      fprintf(stderr, "bm_sbc: invalid node-id in %s: %s\n", path, d.u.s);
      toml_free(res);
      return 1;
    }
    *node_id_set = true;
  }

  // cfg-dir (string)
  d = toml_get(root, "cfg-dir");
  if (d.type == TOML_STRING) {
    strncpy(cfg_dir, d.u.s, cfg_dir_sz - 1);
    cfg_dir[cfg_dir_sz - 1] = '\0';
  }

  // socket-dir (string)
  d = toml_get(root, "socket-dir");
  if (d.type == TOML_STRING) {
    strncpy(vpc->socket_dir, d.u.s, sizeof(vpc->socket_dir) - 1);
  }

  // peers (array of strings)
  toml_datum_t peers_arr = toml_get(root, "peers");
  if (peers_arr.type == TOML_ARRAY) {
    for (int i = 0; i < peers_arr.u.arr.size; i++) {
      if (vpc->num_peers >= VIRTUAL_PORT_CFG_MAX_PEERS) {
        fprintf(stderr, "bm_sbc: too many peers in %s (max %d)\n", path,
                VIRTUAL_PORT_CFG_MAX_PEERS);
        break;
      }
      toml_datum_t elem = peers_arr.u.arr.elem[i];
      if (elem.type == TOML_STRING) {
        uint64_t pid;
        if (parse_hex64(elem.u.s, &pid)) {
          vpc->peer_ids[vpc->num_peers++] = pid;
        } else {
          fprintf(stderr, "bm_sbc: invalid peer in %s: %s\n", path, elem.u.s);
        }
      }
    }
  }

  // transport (string): virtual | udp | serial | adin
  d = toml_get(root, "transport");
  if (d.type == TOML_STRING) {
    strncpy(transport_str, d.u.s, transport_str_sz - 1);
    transport_str[transport_str_sz - 1] = '\0';
  }

  // uart-device (string)
  d = toml_get(root, "uart-device");
  if (d.type == TOML_STRING) {
    strncpy(uart_path, d.u.s, uart_path_sz - 1);
    uart_path[uart_path_sz - 1] = '\0';
  }

  // uart-baud (int)
  d = toml_get(root, "uart-baud");
  if (d.type == TOML_INT64) {
    *baud_rate = (int)d.u.int64;
  }

  // pcap (string)
  d = toml_get(root, "pcap");
  if (d.type == TOML_STRING) {
    strncpy(pcap_path, d.u.s, pcap_path_sz - 1);
    pcap_path[pcap_path_sz - 1] = '\0';
  }

  // log-dir (string)
  d = toml_get(root, "log-dir");
  if (d.type == TOML_STRING) {
    strncpy(log_dir, d.u.s, log_dir_sz - 1);
    log_dir[log_dir_sz - 1] = '\0';
  }

  // log-level (string)
  d = toml_get(root, "log-level");
  if (d.type == TOML_STRING) {
    int lvl = parse_log_level(d.u.s);
    if (lvl < 0) {
      fprintf(stderr, "bm_sbc: invalid log-level in %s: %s\n", path, d.u.s);
      toml_free(res);
      return 1;
    }
    *log_level = lvl;
  }

  // log-stdout (bool)
  d = toml_get(root, "log-stdout");
  if (d.type == TOML_BOOLEAN) {
    *log_stdout = d.u.boolean;
  }

  toml_free(res);
  return 0;
}

// Read Serial and Model from /proc/cpuinfo and build a device name.
// Returns "rpi_<serial>" if Model contains "Raspberry Pi", otherwise
// just "<serial>". Falls back to BM_SBC_DEVICE_NAME if unavailable.
static const char *read_device_name() {
  static char buf[32];
  FILE *f = fopen("/proc/cpuinfo", "r");
  if (!f)
    return BM_SBC_DEVICE_NAME;

  char serial_str[32] = {0};
  bool is_rpi = false;
  char line[128];
  while (fgets(line, sizeof(line), f)) {
    unsigned long long serial = 0;
    if (sscanf(line, "Serial : %llx", &serial) == 1) {
      snprintf(serial_str, sizeof(serial_str), "%llx", serial);
    } else if (strstr(line, "Model") && strstr(line, "Raspberry Pi")) {
      is_rpi = true;
    }
  }
  fclose(f);

  if (serial_str[0] == '\0')
    return BM_SBC_DEVICE_NAME;
  snprintf(buf, sizeof(buf), "%s%s", is_rpi ? "rpi_" : "", serial_str);
  return buf;
}

int bm_sbc_runtime_init(int argc, char **argv, const char *app_name) {
  bm_sbc_app_name_runtime = app_name;
  // Make stdout line-buffered so every bm_debug/printf call ending in '\n'
  // flushes immediately, even when output is redirected to a file.  Without
  // this, ping-reply lines printed by the BCMP thread can be lost when the
  // process is killed before the 8 KiB libc buffer fills up.
  setvbuf(stdout, NULL, _IOLBF, 0);
  // --- CLI parsing -------------------------------------------------------
  VirtualPortCfg vpc;
  memset(&vpc, 0, sizeof(vpc));
  strncpy(vpc.socket_dir, VIRTUAL_PORT_DEFAULT_SOCKET_DIR,
          sizeof(vpc.socket_dir) - 1);
  bool node_id_set = false;
  char cfg_dir[512] = {0};
  char transport_str[16] = {0}; // empty = default (virtual)
  char uart_path[128] = {0};
  char pcap_path[256] = {0};
  int baud_rate = 115200;
  char init_path[512] = {0};
  char log_dir[256] = {0};
  int log_level = -1; // -1 = not set
  bool log_stdout_flag = false;

  // Seed log vars from environment variables; CLI flags and TOML will override.
  {
    const char *env = getenv("BM_SBC_LOG_DIR");
    if (env)
      strncpy(log_dir, env, sizeof(log_dir) - 1);
    env = getenv("BM_SBC_LOG_LEVEL");
    if (env)
      log_level = parse_log_level(env); // -1 if unrecognised
    env = getenv("BM_SBC_LOG_STDOUT");
    if (env && strcmp(env, "1") == 0)
      log_stdout_flag = true;
  }

  static const struct option long_opts[] = {
      {"init", required_argument, NULL, 'i'},
      {"node-id", required_argument, NULL, 'n'},
      {"transport", required_argument, NULL, 't'},
      {"cfg-dir", required_argument, NULL, 'c'},
      {"peer", required_argument, NULL, 'p'},
      {"socket-dir", required_argument, NULL, 's'},
      {"uart", required_argument, NULL, 'u'},
      {"baud", required_argument, NULL, 'b'},
      {"pcap", required_argument, NULL, 'w'},
      {"log-dir", required_argument, NULL, 'd'},
      {"log-level", required_argument, NULL, 'l'},
      {"log-stdout", no_argument, NULL, 'o'},
      {NULL, 0, NULL, 0},
  };

  int opt;
  while ((opt = getopt_long(argc, argv, "", long_opts, NULL)) != -1) {
    switch (opt) {
    case 'i': {
      strncpy(init_path, optarg, sizeof(init_path) - 1);
      break;
    }
    case 'n': {
      if (!parse_hex64(optarg, &vpc.own_node_id)) {
        fprintf(stderr, "bm_sbc: invalid --node-id value: %s\n", optarg);
        fprintf(stderr, "%s", k_usage);
        return 1;
      }
      node_id_set = true;
      break;
    }
    case 't': {
      strncpy(transport_str, optarg, sizeof(transport_str) - 1);
      break;
    }
    case 'c': {
      strncpy(cfg_dir, optarg, sizeof(cfg_dir) - 1);
      break;
    }
    case 'p': {
      if (vpc.num_peers >= VIRTUAL_PORT_CFG_MAX_PEERS) {
        fprintf(stderr, "bm_sbc: too many --peer flags (max %d); ignoring %s\n",
                VIRTUAL_PORT_CFG_MAX_PEERS, optarg);
        break;
      }
      uint64_t pid;
      if (!parse_hex64(optarg, &pid)) {
        fprintf(stderr, "bm_sbc: invalid --peer value: %s\n", optarg);
        fprintf(stderr, "%s", k_usage);
        return 1;
      }
      vpc.peer_ids[vpc.num_peers++] = pid;
      break;
    }
    case 's': {
      strncpy(vpc.socket_dir, optarg, sizeof(vpc.socket_dir) - 1);
      break;
    }
    case 'u': {
      strncpy(uart_path, optarg, sizeof(uart_path) - 1);
      break;
    }
    case 'b': {
      char *end = NULL;
      baud_rate = (int)strtol(optarg, &end, 10);
      if (!end || *end != '\0' || baud_rate <= 0) {
        fprintf(stderr, "bm_sbc: invalid --baud value: %s\n", optarg);
        fprintf(stderr, "%s", k_usage);
        return 1;
      }
      break;
    }
    case 'w': {
      strncpy(pcap_path, optarg, sizeof(pcap_path) - 1);
      break;
    }
    case 'd': {
      strncpy(log_dir, optarg, sizeof(log_dir) - 1);
      break;
    }
    case 'l': {
      log_level = parse_log_level(optarg);
      if (log_level < 0) {
        fprintf(stderr, "bm_sbc: invalid --log-level: %s\n", optarg);
        fprintf(stderr, "%s", k_usage);
        return 1;
      }
      break;
    }
    case 'o': {
      log_stdout_flag = true;
      break;
    }
    default: {
      fprintf(stderr, "bm_sbc: unrecognised option\n");
      fprintf(stderr, "%s", k_usage);
      return 1;
    }
    }
  }

  // --- Load TOML init file (if given), then let CLI flags override ------
  if (init_path[0] != '\0') {
    // Save CLI-provided values so we can re-apply them after the TOML load.
    bool cli_node_id_set = node_id_set;
    uint64_t cli_node_id = vpc.own_node_id;
    char cli_cfg_dir[512];
    strncpy(cli_cfg_dir, cfg_dir, sizeof(cli_cfg_dir));
    char cli_transport_str[sizeof(transport_str)];
    strncpy(cli_transport_str, transport_str, sizeof(cli_transport_str));
    char cli_socket_dir[sizeof(vpc.socket_dir)];
    strncpy(cli_socket_dir, vpc.socket_dir, sizeof(cli_socket_dir));
    uint8_t cli_num_peers = vpc.num_peers;
    uint64_t cli_peer_ids[VIRTUAL_PORT_CFG_MAX_PEERS];
    memcpy(cli_peer_ids, vpc.peer_ids, sizeof(cli_peer_ids));
    char cli_uart_path[128];
    strncpy(cli_uart_path, uart_path, sizeof(cli_uart_path));
    int cli_baud_rate = baud_rate;
    char cli_pcap_path[256];
    strncpy(cli_pcap_path, pcap_path, sizeof(cli_pcap_path));
    char cli_log_dir[256];
    strncpy(cli_log_dir, log_dir, sizeof(cli_log_dir));
    int cli_log_level = log_level;
    bool cli_log_stdout_flag = log_stdout_flag;

    // Reset to defaults before loading from file.
    memset(&vpc, 0, sizeof(vpc));
    strncpy(vpc.socket_dir, VIRTUAL_PORT_DEFAULT_SOCKET_DIR,
            sizeof(vpc.socket_dir) - 1);
    node_id_set = false;
    memset(cfg_dir, 0, sizeof(cfg_dir));
    memset(transport_str, 0, sizeof(transport_str));
    memset(uart_path, 0, sizeof(uart_path));
    memset(pcap_path, 0, sizeof(pcap_path));
    baud_rate = 115200;
    memset(log_dir, 0, sizeof(log_dir));
    log_level = -1;
    log_stdout_flag = false;

    int rc = load_init_file(init_path, &vpc, &node_id_set, cfg_dir,
                            sizeof(cfg_dir), transport_str,
                            sizeof(transport_str), uart_path,
                            sizeof(uart_path), &baud_rate, pcap_path,
                            sizeof(pcap_path), log_dir, sizeof(log_dir),
                            &log_level, &log_stdout_flag);
    if (rc != 0) {
      return rc;
    }

    // Re-apply CLI overrides (non-default values win).
    if (cli_node_id_set) {
      vpc.own_node_id = cli_node_id;
      node_id_set = true;
    }
    if (cli_cfg_dir[0] != '\0') {
      strncpy(cfg_dir, cli_cfg_dir, sizeof(cfg_dir) - 1);
    }
    if (cli_transport_str[0] != '\0') {
      strncpy(transport_str, cli_transport_str, sizeof(transport_str) - 1);
    }
    if (strcmp(cli_socket_dir, VIRTUAL_PORT_DEFAULT_SOCKET_DIR) != 0) {
      strncpy(vpc.socket_dir, cli_socket_dir, sizeof(vpc.socket_dir) - 1);
    }
    if (cli_num_peers > 0) {
      vpc.num_peers = cli_num_peers;
      memcpy(vpc.peer_ids, cli_peer_ids, sizeof(cli_peer_ids));
    }
    if (cli_uart_path[0] != '\0') {
      strncpy(uart_path, cli_uart_path, sizeof(uart_path) - 1);
    }
    if (cli_baud_rate != 115200) {
      baud_rate = cli_baud_rate;
    }
    if (cli_pcap_path[0] != '\0') {
      strncpy(pcap_path, cli_pcap_path, sizeof(pcap_path) - 1);
    }
    if (cli_log_dir[0] != '\0') {
      strncpy(log_dir, cli_log_dir, sizeof(log_dir) - 1);
    }
    if (cli_log_level >= 0) {
      log_level = cli_log_level;
    }
    if (cli_log_stdout_flag) {
      log_stdout_flag = true;
    }
  }

  if (!node_id_set) {
    fprintf(stderr, "bm_sbc: --node-id is required (via --init or CLI)\n");
    fprintf(stderr, "%s", k_usage);
    return 1;
  }

  // --- Transport selection ------------------------------------------------
  TransportKind transport_kind = TransportVirtual; // default when unset
  if (transport_str[0] != '\0' &&
      !transport_kind_parse(transport_str, &transport_kind)) {
    fprintf(stderr, "bm_sbc: invalid transport value: %s\n", transport_str);
    fprintf(stderr, "%s", k_usage);
    return 1;
  }

  // --- Config partition persistence --------------------------------------
  if (cfg_dir[0] != '\0') {
    platform_linux_set_cfg_dir(cfg_dir);
    bm_debug("bm_sbc: cfg-dir=%s\n", cfg_dir);
  }
  // Load persisted partition contents into the in-memory config buffer
  config_init();

  // --- Logging init -------------------------------------------------------
  // Default: also log to stdout when it is a TTY (interactive development).
  bool also_stdout = log_stdout_flag || isatty(STDOUT_FILENO);

  bm_log_init(app_name, vpc.own_node_id, log_dir[0] ? log_dir : NULL,
              also_stdout);

  if (log_level >= 0) {
    bm_log_set_level((BmSbcLogLevel)log_level);
  }

  // --- First structured log line ------------------------------------------
  bool gateway_mode = (uart_path[0] != '\0');
  bm_log_info("node_id=0x%016" PRIx64 " transport=%s peers=%u socket_dir=%s%s%s",
              vpc.own_node_id, transport_kind_name(transport_kind),
              (unsigned)vpc.num_peers, vpc.socket_dir,
              gateway_mode ? " uart=" : "", gateway_mode ? uart_path : "");

  // --- device_init --------------------------------------------------------
  // Build the version string in the same format as bm_protocol embedded
  // apps: "app_name@version_tag" (e.g. "multinode@v0.1.0-3-g472aefb3").
  static char version_str_buf[128];
  snprintf(version_str_buf, sizeof(version_str_buf), "%s@%s", app_name,
           BM_SBC_VERSION_TAG);

  DeviceCfg dev_cfg;
  memset(&dev_cfg, 0, sizeof(dev_cfg));
  dev_cfg.node_id = vpc.own_node_id;
  dev_cfg.git_sha = BM_SBC_GIT_SHA;
  dev_cfg.device_name = read_device_name();
  dev_cfg.version_string = version_str_buf;
  dev_cfg.vendor_id = BM_SBC_VENDOR_ID;
  dev_cfg.product_id = BM_SBC_PRODUCT_ID;
  dev_cfg.hw_ver = BM_SBC_HW_VER;
  dev_cfg.ver_major = BM_SBC_VERSION_MAJOR;
  dev_cfg.ver_minor = BM_SBC_VERSION_MINOR;
  dev_cfg.ver_patch = BM_SBC_VERSION_PATCH;
  device_init(dev_cfg);

  // --- NetworkDevice construction (transport factory) --------------------
  TransportFactoryCfg tf_cfg;
  memset(&tf_cfg, 0, sizeof(tf_cfg));
  tf_cfg.kind = transport_kind;
  tf_cfg.vpc = vpc;
  tf_cfg.uart_path = uart_path;
  tf_cfg.baud_rate = baud_rate;

  NetworkDevice net_dev;
  if (transport_factory_create(&tf_cfg, &net_dev) != 0) {
    return 1;
  }

  // --- Bristlemouth startup sequence ------------------------------------
  BmErr err = BmOK;
  bm_err_check(err, bm_l2_init(net_dev));

  if (pcap_path[0] != '\0') {
    if (pcap_file_sink_open(pcap_path) != 0) {
      bm_log_error("failed to open pcap file: %s", pcap_path);
      return 1;
    }
    bm_l2_register_pcap_callback(pcap_write_packet);
    bm_log_info("pcap capture -> %s", pcap_path);
  }

  bm_err_check(err, timer_callback_handler_init());
  bm_err_check(err, bm_ip_init());
  // Restore client_update_reboot_info from the DFU marker file (if present)
  // so bm_dfu_init() inside bcmp_init() sees DFU_REBOOT_MAGIC on post-swap boot.
  platform_linux_dfu_restore_state();
  bm_err_check(err, bcmp_init(net_dev));
  uint8_t total_ports = net_dev.trait->num_ports();
  bm_err_check(err, topology_init(total_ports));
  bm_err_check(err, bm_service_init());
  bm_err_check(err, bm_pubsub_init());
  bm_err_check(err, bm_middleware_init());

  // Register built-in services so this node responds to service requests.
  sys_info_service_init();
  config_cbor_map_service_init();

  if (err != BmOK) {
    bm_log_error("startup sequence failed err=%d", (int)err);
    return (int)err;
  }
  bm_log_info("stack initialized");
  return 0;
}

void bm_sbc_runtime_set_pre_exec_cb(void (*cb)(void)) {
    platform_linux_set_pre_exec_cb(cb);
}

void bm_sbc_runtime_set_argv(int argc, char **argv){
  platform_linux_set_argv(argc, argv);
}
