/// @file app_main.cpp
/// @brief S17 BUILD-4 bench application services (light + telemetry).
///
/// One binary, role selected by environment variable:
///
///   S17_ROLE=light      Light node (nereus000). Registers the
///                       "light/control" bm_service: level, strobe,
///                       state query. Backend = light HAL: the Pi's
///                       onboard ACT LED via sysfs (visual check) plus
///                       a state file artifact; falls back to
///                       state-file-only when the LED isn't writable.
///   S17_ROLE=telemetry  Telemetry node (nereus001). Subscribes to
///                       camera/stream, reassembles chunked JPEGs
///                       (chunk_reasm.h) and feeds them to the frozen
///                       S3 stream server's ingest (127.0.0.1:8081,
///                       "frame" JSON header line + JPEG bytes) --
///                       browser demo at http://nereus001:8080/stream.
///                       Also: operator CLI on stdin (type `help`),
///                       periodic aggregated uplink via the SHIPPED
///                       spotter_tx_data() (REV-8), and the shipped
///                       gateway_ipc listener for the python client.
///
/// Wire contracts replicated from ADIN_SPI_OpenMV
/// firmware/bm_he/src/camera_svc.h (camera structs; change in lockstep
/// or not at all). The light service structs are defined HERE (this
/// file is both ends' source of truth; the operator CLI in this app is
/// the only requester).
///
/// Environment knobs:
///   S17_ROLE           light | telemetry (required)
///   S17_LED_PATH       LED sysfs dir (default /sys/class/leds/ACT)
///   S17_STATE_PATH     light state artifact (default /tmp/s17_light_state)
///   S17_INGEST         stream-server ingest (default 127.0.0.1:8081)
///   S17_UPLINK_SECS    aggregated-uplink period, 0 = off (default 30)
///   BM_SBC_GATEWAY_IPC gateway_ipc socket path (shared with the python
///                      client; use /tmp/... on the bench)
///
/// Output markers (grepped by demo docs): LIGHT_STAT / TEL_STAT once
/// per second; LIGHT_CMD on every accepted command.

#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <mutex>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

#include "bm_log.h"

extern "C" {
#include "device.h"        // node_id()
#include "messages/time.h" // bcmp_time_set_time (BCMP re-tx crosses Light)
#include "spotter.h"       // spotter_tx_data -- THE shipped uplink (REV-8)
}

#include "bm_service.h"
#include "bm_service_request.h"
#include "chunk_reasm.h"
#include "gateway_ipc.h"
#include "power_info_service.h"
#include "pubsub.h"

// ---------------------------------------------------------------------------
// Shared contracts
// ---------------------------------------------------------------------------

// camera/stream data plane (camera_svc.h): topic + chunk format
// (chunk_reasm.h holds the header layout).
static const char *k_camera_topic = "camera/stream";

// camera/control service. REPLICATED from ADIN_SPI_OpenMV
// firmware/bm_he/src/camera_svc.h -- change in lockstep or not at all.
#define CAMERA_SERVICE "camera/control"
#define CAMERA_REQ_MAGIC 0x314D4143u // 'CAM1' little-endian
#define CAMERA_CMD_CAPTURE 1u
#define CAMERA_CMD_STREAM 2u
#define CAMERA_CMD_STATUS 3u
#define CAMERA_CMD_STOP 4u

struct __attribute__((packed)) camera_req_t {
  uint32_t magic;
  uint8_t cmd;
  uint8_t quality;      // 0 = bridge default (50)
  uint16_t fps_x10;     // 0 = bridge default (10.0 fps)
  uint32_t rate_bps;    // 0 = fps-paced only
  uint16_t secs;        // 0 = bridge default (60)
  uint16_t payload_max; // 0 = 1400 (REV-28)
};                      // 16 B
static_assert(sizeof(camera_req_t) == 16, "camera_req_t ABI (camera_svc.h)");

struct __attribute__((packed)) camera_rep_t {
  uint32_t magic;
  uint8_t ok;
  uint8_t mode_active;
  uint16_t rsvd;
  uint32_t cmds;
  uint32_t pub_ok;
  uint32_t pub_errs;
  uint32_t pub_bytes;
};                      // 24 B
static_assert(sizeof(camera_rep_t) == 24, "camera_rep_t ABI (camera_svc.h)");

// Fixed bench node ids (pi/bm_bench, Nick 2026-08-14; never reused).
static const uint64_t k_camera_node = 0xbe9c000000000003ull;

// light/control service. Packed LE structs, same style + rationale as
// the camera service (cbor helper is config-only; flash-poor HE end).
#define LIGHT_SERVICE "light/control"
#define LIGHT_REQ_MAGIC 0x3154494Cu // 'LIT1' little-endian

#define LIGHT_CMD_LEVEL 1u  // set steady level 0..100
#define LIGHT_CMD_STROBE 2u // on_ms/off_ms for count cycles
#define LIGHT_CMD_QUERY 3u  // state only

struct __attribute__((packed)) light_req_t {
  uint32_t magic;
  uint8_t cmd;
  uint8_t level;   // LIGHT_CMD_LEVEL: 0..100
  uint16_t on_ms;  // LIGHT_CMD_STROBE
  uint16_t off_ms;
  uint16_t count;
};                 // 12 B
static_assert(sizeof(light_req_t) == 12, "light_req_t ABI");

struct __attribute__((packed)) light_rep_t {
  uint32_t magic;
  uint8_t ok;
  uint8_t level;    // current commanded level
  uint8_t strobing; // 1 while a strobe pattern runs
  uint8_t rsvd;
  uint32_t cmds;    // accepted commands since boot
  uint32_t uptime_s;
};                  // 16 B
static_assert(sizeof(light_rep_t) == 16, "light_rep_t ABI");

// ---------------------------------------------------------------------------
// Common state
// ---------------------------------------------------------------------------

static bool s_role_light = false;
static struct timespec s_start;
static uint64_t s_last_stat_ms = 0;

static double now_sec(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)(ts.tv_sec - s_start.tv_sec) +
         (double)(ts.tv_nsec - s_start.tv_nsec) / 1e9;
}

static uint64_t now_ms(void) { return (uint64_t)(now_sec() * 1000.0); }

// ---------------------------------------------------------------------------
// Light role: HAL (sysfs LED + state-file artifact)
// ---------------------------------------------------------------------------

static std::string s_led_dir = "/sys/class/leds/ACT";
static std::string s_state_path = "/tmp/s17_light_state";
static bool s_led_ok = false;

static bool file_write(const std::string &path, const char *text) {
  FILE *f = fopen(path.c_str(), "w");
  if (!f) {
    return false;
  }
  bool ok = fputs(text, f) >= 0;
  fclose(f);
  return ok;
}

static void light_hal_init(void) {
  // Claim the LED: trigger -> none. Restore is a documented README step
  // (echo mmc0 | sudo tee <led>/trigger) -- boring and visible.
  s_led_ok = file_write(s_led_dir + "/trigger", "none");
  if (s_led_ok) {
    bm_log_info("light: LED claimed at %s (trigger=none)", s_led_dir.c_str());
  } else {
    bm_log_warn("light: LED at %s not writable (%s) -- state-file-only "
                "mode. Fix: sudo chmod a+w %s/{trigger,brightness}",
                s_led_dir.c_str(), strerror(errno), s_led_dir.c_str());
  }
}

// Physical LED is binary: any level > 0 lights it. The commanded level
// is recorded in the state file (a real light HAL gets PWM on hardware
// day behind this same call).
static void light_hal_apply(uint8_t level, bool strobing) {
  if (s_led_ok) {
    file_write(s_led_dir + "/brightness", level > 0 ? "1" : "0");
  }
  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"level\": %u, \"strobing\": %s, \"uptime_s\": %.0f}\n", level,
           strobing ? "true" : "false", now_sec());
  file_write(s_state_path, buf);
}

// Service handler (middleware thread) -> these atomics -> loop() applies.
static std::atomic<uint32_t> s_light_cmds{0};
static std::atomic<uint8_t> s_light_level{0};
static std::atomic<bool> s_strobe_armed{false};
static std::atomic<uint16_t> s_strobe_on_ms{0}, s_strobe_off_ms{0},
    s_strobe_count{0};

// loop()-owned strobe machine state
static bool s_strobing = false;
static bool s_strobe_lit = false;
static uint16_t s_strobe_left = 0;
static uint64_t s_strobe_next_ms = 0;

static bool light_service_cb(size_t service_strlen, const char *service,
                             size_t req_data_len, uint8_t *req_data,
                             size_t *buffer_len, uint8_t *reply_data) {
  (void)service_strlen;
  (void)service;
  light_req_t req;
  if (req_data_len != sizeof(req)) {
    return false;
  }
  memcpy(&req, req_data, sizeof(req));
  if (req.magic != LIGHT_REQ_MAGIC) {
    return false;
  }

  bool ok = true;
  switch (req.cmd) {
  case LIGHT_CMD_LEVEL:
    s_light_level = req.level > 100 ? 100 : req.level;
    s_strobe_armed = false; // a level command cancels a pending strobe
    s_light_cmds++;
    bm_log_info("LIGHT_CMD level=%u", (unsigned)s_light_level.load());
    break;
  case LIGHT_CMD_STROBE:
    if (req.on_ms == 0 || req.count == 0) {
      ok = false;
      break;
    }
    s_strobe_on_ms = req.on_ms;
    s_strobe_off_ms = req.off_ms ? req.off_ms : req.on_ms;
    s_strobe_count = req.count;
    s_strobe_armed = true;
    s_light_cmds++;
    bm_log_info("LIGHT_CMD strobe on=%u off=%u count=%u", req.on_ms,
                req.off_ms, req.count);
    break;
  case LIGHT_CMD_QUERY:
    break;
  default:
    ok = false;
    break;
  }

  light_rep_t rep;
  memset(&rep, 0, sizeof(rep));
  rep.magic = LIGHT_REQ_MAGIC;
  rep.ok = ok ? 1 : 0;
  rep.level = s_light_level.load();
  rep.strobing = (s_strobing || s_strobe_armed.load()) ? 1 : 0;
  rep.cmds = s_light_cmds.load();
  rep.uptime_s = (uint32_t)now_sec();
  if (*buffer_len < sizeof(rep)) {
    return false;
  }
  memcpy(reply_data, &rep, sizeof(rep));
  *buffer_len = sizeof(rep);
  return true;
}

static void light_loop(void) {
  uint64_t ms = now_ms();

  // Arm a freshly commanded strobe.
  if (s_strobe_armed.exchange(false)) {
    s_strobing = true;
    s_strobe_lit = false;
    s_strobe_left = s_strobe_count.load();
    s_strobe_next_ms = ms; // fire immediately
  }

  if (s_strobing && ms >= s_strobe_next_ms) {
    if (!s_strobe_lit) {
      s_strobe_lit = true;
      light_hal_apply(100, true);
      s_strobe_next_ms = ms + s_strobe_on_ms.load();
    } else {
      s_strobe_lit = false;
      s_strobe_left--;
      light_hal_apply(s_strobe_left ? 0 : s_light_level.load(),
                      s_strobe_left != 0);
      if (s_strobe_left == 0) {
        s_strobing = false;
        bm_log_info("light: strobe done, level back to %u",
                    (unsigned)s_light_level.load());
      } else {
        s_strobe_next_ms = ms + s_strobe_off_ms.load();
      }
    }
  } else if (!s_strobing) {
    // Steady state: apply the commanded level on change only.
    static int s_applied = -1;
    int lvl = s_light_level.load();
    if (lvl != s_applied) {
      s_applied = lvl;
      light_hal_apply((uint8_t)lvl, false);
    }
  }

  if (ms - s_last_stat_ms >= 1000) {
    s_last_stat_ms = ms;
    printf("LIGHT_STAT t=%.0f level=%u strobing=%d cmds=%" PRIu32 " led=%s\n",
           now_sec(), (unsigned)s_light_level.load(), s_strobing ? 1 : 0,
           s_light_cmds.load(), s_led_ok ? "sysfs" : "file-only");
    fflush(stdout);
  }
}

// ---------------------------------------------------------------------------
// Telemetry role: subscribe -> reassemble -> frozen stream server
// ---------------------------------------------------------------------------

#define TEL_JPEG_CAP (128 * 1024) // 64-chunk BMV6 bound was 87 KB (S6)
#define TEL_QUEUE_CAP 256         // chunk payload backlog (drops counted)

static std::string s_ingest_host = "127.0.0.1";
static int s_ingest_port = 8081;

static std::mutex s_q_lock;
static std::deque<std::vector<uint8_t>> s_chunk_q;
static uint64_t s_q_drops = 0;

static chunk_reasm_t s_reasm;
static uint8_t s_jpeg_buf[TEL_JPEG_CAP];

static int s_ingest_fd = -1;
static uint64_t s_ingest_retry_ms = 0;
static uint64_t s_ingest_frames = 0;
static uint64_t s_ingest_fails = 0;
static uint64_t s_frames_win = 0;
static uint64_t s_bytes_win = 0;

// pubsub RX thread: copy the payload and get out -- reassembly and the
// (blocking) ingest socket live in loop() on the app thread.
static void on_camera_msg(uint64_t node, const char * /*topic*/,
                          uint16_t /*topic_len*/, const uint8_t *data,
                          uint16_t data_len, uint8_t /*type*/,
                          uint8_t /*version*/) {
  (void)node;
  std::lock_guard<std::mutex> g(s_q_lock);
  if (s_chunk_q.size() >= TEL_QUEUE_CAP) {
    s_q_drops++;
    return;
  }
  s_chunk_q.emplace_back(data, data + data_len);
}

static void ingest_close(void) {
  if (s_ingest_fd >= 0) {
    close(s_ingest_fd);
    s_ingest_fd = -1;
  }
}

static bool ingest_connect(void) {
  if (s_ingest_fd >= 0) {
    return true;
  }
  uint64_t ms = now_ms();
  if (ms < s_ingest_retry_ms) {
    return false;
  }
  s_ingest_retry_ms = ms + 1000; // 1 s backoff
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return false;
  }
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons((uint16_t)s_ingest_port);
  if (inet_pton(AF_INET, s_ingest_host.c_str(), &addr.sin_addr) != 1 ||
      connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
    close(fd);
    return false;
  }
  s_ingest_fd = fd;
  bm_log_info("telemetry: ingest connected (%s:%d)", s_ingest_host.c_str(),
              s_ingest_port);
  return true;
}

static bool send_all(int fd, const void *data, size_t n) {
  const uint8_t *p = (const uint8_t *)data;
  while (n) {
    ssize_t w = send(fd, p, n, MSG_NOSIGNAL);
    if (w <= 0) {
      return false;
    }
    p += w;
    n -= (size_t)w;
  }
  return true;
}

// Frozen S3 ingest framing (pi/stream/usb_frame_source.py StreamParser):
// one JSON line {"status":"frame","frame":{"seq":N,"size_bytes":M}}\n
// then exactly M JPEG bytes.
static void ingest_frame(uint32_t seq, const uint8_t *jpeg, size_t n) {
  if (!ingest_connect()) {
    s_ingest_fails++;
    return;
  }
  char hdr[96];
  int h = snprintf(hdr, sizeof(hdr),
                   "{\"status\": \"frame\", \"frame\": {\"seq\": %" PRIu32
                   ", \"size_bytes\": %zu}}\n",
                   seq, n);
  if (!send_all(s_ingest_fd, hdr, (size_t)h) ||
      !send_all(s_ingest_fd, jpeg, n)) {
    bm_log_warn("telemetry: ingest write failed (%s) -- reconnecting",
                strerror(errno));
    ingest_close();
    s_ingest_fails++;
    return;
  }
  s_ingest_frames++;
}

static void telemetry_loop(void) {
  for (;;) {
    std::vector<uint8_t> chunk;
    {
      std::lock_guard<std::mutex> g(s_q_lock);
      if (s_chunk_q.empty()) {
        break;
      }
      chunk = std::move(s_chunk_q.front());
      s_chunk_q.pop_front();
    }
    size_t done = chunk_reasm_feed(&s_reasm, chunk.data(), chunk.size());
    if (done) {
      uint32_t seq = s_reasm.frame_seq;
      s_frames_win++;
      s_bytes_win += done;
      ingest_frame(seq, s_jpeg_buf, done);
    }
  }

  uint64_t ms = now_ms();
  if (ms - s_last_stat_ms >= 1000) {
    double dt = (double)(ms - s_last_stat_ms) / 1000.0;
    s_last_stat_ms = ms;
    printf("TEL_STAT t=%.0f fps=%.1f kBps=%.1f frames_ok=%" PRIu32
           " dropped=%" PRIu32 " gaps=%" PRIu32 " hdr_errs=%" PRIu32
           " q_drops=%" PRIu64 " ingest_ok=%" PRIu64 " ingest_fail=%" PRIu64
           "\n",
           now_sec(), (double)s_frames_win / dt,
           (double)s_bytes_win / dt / 1000.0, s_reasm.frames_ok,
           s_reasm.frames_dropped, s_reasm.chunk_gaps, s_reasm.hdr_errors,
           s_q_drops, s_ingest_frames, s_ingest_fails);
    fflush(stdout);
    s_frames_win = 0;
    s_bytes_win = 0;
  }
}

// ---------------------------------------------------------------------------
// Telemetry role: operator CLI + uplink (C2)
// ---------------------------------------------------------------------------

static uint64_t s_uplink_secs = 30; // 0 disables the periodic uplink
static uint64_t s_uplink_next_ms = 0;
static uint64_t s_uplink_sent = 0;
static bool s_ipc_up = false;

// Service replies (any thread) -> printed markers the demo greps.
static bool camera_reply_cb(bool ack, uint32_t msg_id, size_t /*slen*/,
                            const char * /*service*/, size_t reply_len,
                            uint8_t *reply_data) {
  if (!ack) {
    printf("CAM_REPLY id=%" PRIu32 " TIMEOUT\n", msg_id);
    fflush(stdout);
    return true;
  }
  camera_rep_t rep;
  if (reply_len != sizeof(rep)) {
    printf("CAM_REPLY id=%" PRIu32 " BAD_LEN %zu\n", msg_id, reply_len);
    fflush(stdout);
    return false;
  }
  memcpy(&rep, reply_data, sizeof(rep));
  printf("CAM_REPLY id=%" PRIu32 " ok=%u mode=%u cmds=%" PRIu32
         " pub_ok=%" PRIu32 " pub_errs=%" PRIu32 " pub_bytes=%" PRIu32 "\n",
         msg_id, rep.ok, rep.mode_active, rep.cmds, rep.pub_ok, rep.pub_errs,
         rep.pub_bytes);
  fflush(stdout);
  return true;
}

static bool light_reply_cb(bool ack, uint32_t msg_id, size_t /*slen*/,
                           const char * /*service*/, size_t reply_len,
                           uint8_t *reply_data) {
  if (!ack) {
    printf("LIGHT_REPLY id=%" PRIu32 " TIMEOUT\n", msg_id);
    fflush(stdout);
    return true;
  }
  light_rep_t rep;
  if (reply_len != sizeof(rep)) {
    printf("LIGHT_REPLY id=%" PRIu32 " BAD_LEN %zu\n", msg_id, reply_len);
    fflush(stdout);
    return false;
  }
  memcpy(&rep, reply_data, sizeof(rep));
  printf("LIGHT_REPLY id=%" PRIu32 " ok=%u level=%u strobing=%u cmds=%" PRIu32
         " uptime=%" PRIu32 "s\n",
         msg_id, rep.ok, rep.level, rep.strobing, rep.cmds, rep.uptime_s);
  fflush(stdout);
  return true;
}

static BmErr power_reply_cb(const PowerInfoReplyData *d) {
  printf("POWER_REPLY total_on=%" PRIu32 "s remaining_on=%" PRIu32
         "s upcoming_off=%" PRIu32 "s\n",
         d->total_on_s, d->remaining_on_s, d->upcoming_off_s);
  fflush(stdout);
  return BmOK;
}

static void send_camera_req(uint8_t cmd, uint8_t q, uint16_t fps_x10,
                            uint32_t rate_bps, uint16_t secs) {
  camera_req_t req;
  memset(&req, 0, sizeof(req));
  req.magic = CAMERA_REQ_MAGIC;
  req.cmd = cmd;
  req.quality = q;
  req.fps_x10 = fps_x10;
  req.rate_bps = rate_bps;
  req.secs = secs;
  if (!bm_service_request(strlen(CAMERA_SERVICE), CAMERA_SERVICE, sizeof(req),
                          (const uint8_t *)&req, camera_reply_cb, 6)) {
    printf("CAM_REPLY REQUEST_FAILED\n");
    fflush(stdout);
  }
}

static void send_light_req(uint8_t cmd, uint8_t level, uint16_t on_ms,
                           uint16_t off_ms, uint16_t count) {
  light_req_t req;
  memset(&req, 0, sizeof(req));
  req.magic = LIGHT_REQ_MAGIC;
  req.cmd = cmd;
  req.level = level;
  req.on_ms = on_ms;
  req.off_ms = off_ms;
  req.count = count;
  if (!bm_service_request(strlen(LIGHT_SERVICE), LIGHT_SERVICE, sizeof(req),
                          (const uint8_t *)&req, light_reply_cb, 6)) {
    printf("LIGHT_REPLY REQUEST_FAILED\n");
    fflush(stdout);
  }
}

static void time_sync_camera(void) {
  struct timespec rt;
  clock_gettime(CLOCK_REALTIME, &rt);
  uint64_t utc_us =
      (uint64_t)rt.tv_sec * 1000000ull + (uint64_t)rt.tv_nsec / 1000ull;
  BmErr err = bcmp_time_set_time(k_camera_node, utc_us);
  printf("TIME_SYNC target=%016" PRIx64 " utc_us=%" PRIu64 " err=%d "
         "(camera inherits this node's clock; response logs at debug)\n",
         k_camera_node, utc_us, (int)err);
  fflush(stdout);
}

static void cli_help(void) {
  printf("commands:\n"
         "  capture [q]                 trigger one frame (JPEG q, 0=default)\n"
         "  stream <mbps> <fps> <secs>  start the camera stream\n"
         "  stop                        stop the camera stream\n"
         "  cam-status                  camera service counters\n"
         "  light <level>               set light level 0..100\n"
         "  strobe <on_ms> <off_ms> <n> strobe the light\n"
         "  light-status                light state query\n"
         "  power                       power service query (2-hop, AE3 sim)\n"
         "  time-sync                   push this host's UTC to the camera\n"
         "  status                      local telemetry ledger\n");
  fflush(stdout);
}

static void cli_handle(char *line) {
  char *cmd = strtok(line, " \t");
  if (!cmd) {
    return;
  }
  if (strcmp(cmd, "capture") == 0) {
    const char *q = strtok(nullptr, " \t");
    send_camera_req(CAMERA_CMD_CAPTURE, q ? (uint8_t)atoi(q) : 0, 0, 0, 0);
  } else if (strcmp(cmd, "stream") == 0) {
    const char *mbps = strtok(nullptr, " \t");
    const char *fps = strtok(nullptr, " \t");
    const char *secs = strtok(nullptr, " \t");
    send_camera_req(CAMERA_CMD_STREAM, 0,
                    fps ? (uint16_t)(atof(fps) * 10) : 0,
                    mbps ? (uint32_t)(atof(mbps) * 1e6) : 0,
                    secs ? (uint16_t)atoi(secs) : 0);
  } else if (strcmp(cmd, "stop") == 0) {
    send_camera_req(CAMERA_CMD_STOP, 0, 0, 0, 0);
  } else if (strcmp(cmd, "cam-status") == 0) {
    send_camera_req(CAMERA_CMD_STATUS, 0, 0, 0, 0);
  } else if (strcmp(cmd, "light") == 0) {
    const char *lvl = strtok(nullptr, " \t");
    send_light_req(LIGHT_CMD_LEVEL, lvl ? (uint8_t)atoi(lvl) : 0, 0, 0, 0);
  } else if (strcmp(cmd, "strobe") == 0) {
    const char *on = strtok(nullptr, " \t");
    const char *off = strtok(nullptr, " \t");
    const char *n = strtok(nullptr, " \t");
    send_light_req(LIGHT_CMD_STROBE, 0, on ? (uint16_t)atoi(on) : 200,
                   off ? (uint16_t)atoi(off) : 200,
                   n ? (uint16_t)atoi(n) : 5);
  } else if (strcmp(cmd, "light-status") == 0) {
    send_light_req(LIGHT_CMD_QUERY, 0, 0, 0, 0);
  } else if (strcmp(cmd, "power") == 0) {
    if (power_info_service_request(power_reply_cb, 6) != BmOK) {
      printf("POWER_REPLY REQUEST_FAILED\n");
      fflush(stdout);
    }
  } else if (strcmp(cmd, "time-sync") == 0) {
    time_sync_camera();
  } else if (strcmp(cmd, "status") == 0) {
    printf("TEL_LEDGER frames_ok=%" PRIu32 " dropped=%" PRIu32
           " gaps=%" PRIu32 " hdr_errs=%" PRIu32 " q_drops=%" PRIu64
           " ingest_ok=%" PRIu64 " ingest_fail=%" PRIu64 " uplinks=%" PRIu64
           " ipc=%s\n",
           s_reasm.frames_ok, s_reasm.frames_dropped, s_reasm.chunk_gaps,
           s_reasm.hdr_errors, s_q_drops, s_ingest_frames, s_ingest_fails,
           s_uplink_sent, s_ipc_up ? "up" : "down");
    fflush(stdout);
  } else {
    cli_help();
  }
}

static void cli_poll(void) {
  static char buf[256];
  static size_t fill = 0;
  struct pollfd pfd = {0, POLLIN, 0};
  while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
    char c;
    ssize_t r = read(0, &c, 1);
    if (r <= 0) {
      return; // EOF (piped stdin): stop polling this pass
    }
    if (c == '\n') {
      buf[fill] = '\0';
      fill = 0;
      cli_handle(buf);
    } else if (fill < sizeof(buf) - 1) {
      buf[fill++] = c;
    }
  }
}

// Periodic uplink: aggregate the ledger and hand it to the SHIPPED
// uplink primitive, spotter_tx_data() -> "spotter/transmit-data" on
// pub/sub (in production the mote subscribes and does satellite TX; on
// the bench the publish itself -- pcap + this log line -- is the
// artifact, stated honestly). Option A per the approved plan.
static void uplink_tick(void) {
  if (s_uplink_secs == 0) {
    return;
  }
  uint64_t ms = now_ms();
  if (s_uplink_next_ms == 0) {
    s_uplink_next_ms = ms + s_uplink_secs * 1000;
    return;
  }
  if (ms < s_uplink_next_ms) {
    return;
  }
  s_uplink_next_ms = ms + s_uplink_secs * 1000;
  char report[256];
  int n = snprintf(report, sizeof(report),
                   "{\"src\": \"%016" PRIx64 "\", \"t\": %.0f, "
                   "\"frames_ok\": %" PRIu32 ", \"frames_dropped\": %" PRIu32
                   ", \"chunk_gaps\": %" PRIu32 ", \"ingest_ok\": %" PRIu64
                   ", \"uplink_seq\": %" PRIu64 "}",
                   node_id(), now_sec(), s_reasm.frames_ok,
                   s_reasm.frames_dropped, s_reasm.chunk_gaps,
                   s_ingest_frames, s_uplink_sent);
  BmErr err = spotter_tx_data(report, (uint16_t)n, BmNetworkTypeCellularOnly);
  if (err == BmOK) {
    s_uplink_sent++;
    printf("UPLINK_TX %d B seq=%" PRIu64 " %s\n", n, s_uplink_sent, report);
  } else {
    printf("UPLINK_TX FAILED err=%d\n", (int)err);
  }
  fflush(stdout);
}

// ---------------------------------------------------------------------------
// App contract
// ---------------------------------------------------------------------------

void setup(void) {
  clock_gettime(CLOCK_MONOTONIC, &s_start);

  const char *role = getenv("S17_ROLE");
  if (!role || (strcmp(role, "light") != 0 && strcmp(role, "telemetry") != 0)) {
    bm_log_error("bench_apps: set S17_ROLE=light or S17_ROLE=telemetry");
    exit(2);
  }
  s_role_light = strcmp(role, "light") == 0;

  if (s_role_light) {
    const char *led = getenv("S17_LED_PATH");
    if (led) {
      s_led_dir = led;
    }
    const char *st = getenv("S17_STATE_PATH");
    if (st) {
      s_state_path = st;
    }
    light_hal_init();
    light_hal_apply(0, false);
    if (!bm_service_register(strlen(LIGHT_SERVICE), LIGHT_SERVICE,
                             light_service_cb)) {
      bm_log_error("light: bm_service_register failed");
      exit(1);
    }
    bm_log_info("light: node %016" PRIx64 " serving %s", node_id(),
                LIGHT_SERVICE);
  } else {
    const char *ingest = getenv("S17_INGEST");
    if (ingest) {
      const char *colon = strrchr(ingest, ':');
      if (colon) {
        s_ingest_host.assign(ingest, colon - ingest);
        s_ingest_port = atoi(colon + 1);
      }
    }
    chunk_reasm_init(&s_reasm, s_jpeg_buf, sizeof(s_jpeg_buf));
    if (bm_sub(k_camera_topic, on_camera_msg) != BmOK) {
      bm_log_error("telemetry: bm_sub(%s) failed", k_camera_topic);
      exit(1);
    }
    const char *up = getenv("S17_UPLINK_SECS");
    if (up) {
      s_uplink_secs = (uint64_t)atoi(up);
    }
    // The shipped external-process door (REV-8). Socket path override:
    // BM_SBC_GATEWAY_IPC (both here and in the python client) -- the
    // bench uses /tmp to avoid /run permissions. Failure is non-fatal:
    // the door is an extra, the BM side keeps running.
    s_ipc_up = gateway_ipc_init(node_id()) == 0;
    if (!s_ipc_up) {
      bm_log_warn("telemetry: gateway_ipc_init failed -- python-client "
                  "demo unavailable (set BM_SBC_GATEWAY_IPC to a writable "
                  "path)");
    }
    bm_log_info("telemetry: node %016" PRIx64 " subscribed to %s, "
                "ingest -> %s:%d, uplink every %" PRIu64 "s, ipc=%s",
                node_id(), k_camera_topic, s_ingest_host.c_str(),
                s_ingest_port, s_uplink_secs, s_ipc_up ? "up" : "down");
    cli_help();
  }
}

void loop(void) {
  if (s_role_light) {
    light_loop();
  } else {
    telemetry_loop();
    if (s_ipc_up) {
      gateway_ipc_poll();
    }
    cli_poll();
    uplink_tick();
  }
}
