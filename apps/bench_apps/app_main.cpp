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
///   S18_CTL_SOCK       bench control socket (default /run/bm/bench.sock)
///
/// Output markers (grepped by demo docs): LIGHT_STAT / TEL_STAT once
/// per second; LIGHT_CMD on every accepted command.

#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <cinttypes>
#include <cstddef> // offsetof (camera_rep_t ABI lock)
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <fcntl.h> // O_NONBLOCK on the control socket
#include <mutex>
#include <poll.h>
#include <string>
#include <strings.h>  // strcasecmp (res/pf CLI args)
#include <sys/stat.h>    // chmod (control socket), mkdir (capture dir)
#include <sys/statvfs.h> // free-space floor before a still is written
#include <sys/socket.h>
#include <sys/un.h> // AF_UNIX control socket
#include <unistd.h>
#include <vector>

#include "bm_log.h"

extern "C" {
#include "device.h"        // node_id()
#include "messages/time.h" // bcmp_time_set_time (BCMP re-tx crosses Light)
#include "spotter.h"       // spotter_tx_data -- THE shipped uplink (REV-8)
}

#include "bench_ctl.h" // control-socket wire format (parse + render)
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

// S18 capture geometry. The AE3 sensor letterboxes to 16:10 (QVGA is
// 320x200) and rejects QQVGA/SVGA/WXGA; nothing above HD is offered.
// Out-of-range values are REFUSED by the service (ok = 0), not clamped.
#define CAMERA_RES_DEFAULT 0u
#define CAMERA_RES_QVGA 1u // 320x200
#define CAMERA_RES_VGA 2u  // 640x400
#define CAMERA_RES_HD 3u   // 1280x800
#define CAMERA_PF_DEFAULT 0u
#define CAMERA_PF_COLOR 1u
#define CAMERA_PF_MONO 2u

// Wire code -> operator-facing name. "?" for anything out of range: a
// refused geometry must read as refused everywhere it is printed, never as
// a plausible default.
static const char *k_res_name[] = {"default", "qvga", "vga", "hd"};
static const char *k_pf_name[] = {"default", "color", "mono"};
static const char *res_name_of(uint8_t v) {
  return v < (sizeof(k_res_name) / sizeof(k_res_name[0])) ? k_res_name[v] : "?";
}
static const char *pf_name_of(uint8_t v) {
  return v < (sizeof(k_pf_name) / sizeof(k_pf_name[0])) ? k_pf_name[v] : "?";
}

struct __attribute__((packed)) camera_req_t {
  uint32_t magic;
  uint8_t cmd;
  uint8_t quality;      // 0 = bridge default (50)
  uint16_t fps_x10;     // 0 = bridge default (10.0 fps)
  uint32_t rate_bps;    // 0 = fps-paced only
  uint16_t secs;        // 0 = bridge default (60)
  uint16_t payload_max; // 0 = 1400 (REV-28)
  uint8_t resolution;   // CAMERA_RES_*, 0 = bridge default (S18)
  uint8_t pixformat;    // CAMERA_PF_*,  0 = bridge default (S18)
};                      // 18 B
static_assert(sizeof(camera_req_t) == 18, "camera_req_t ABI (camera_svc.h)");

struct __attribute__((packed)) camera_rep_t {
  uint32_t magic;
  uint8_t ok;
  uint8_t mode_active;
  uint8_t res_active; // last COMMANDED, not confirmed by the sensor
  uint8_t pf_active;
  uint32_t cmds;
  uint32_t pub_ok;
  uint32_t pub_errs;
  uint32_t pub_bytes;
};                      // 24 B (S18 reused the old rsvd u16)
static_assert(sizeof(camera_rep_t) == 24, "camera_rep_t ABI (camera_svc.h)");
static_assert(offsetof(camera_rep_t, cmds) == 8, "camera_rep_t cmds @ 8");

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

// --- what was commanded, and what came back (S18 bite B) -------------------
//
// Service replies arrive on middleware threads; the control socket and the
// still-save are served from loop(). One mutex covers the handful of scalars
// both touch. Nothing here changes the BM path -- it only remembers what
// already went past, so the web tool can ask "what did I command, and what
// did the chain answer?" without scraping the journal.
static std::mutex s_ctl_lock;
static struct {
  bool params_seen;
  char last_cmd[BENCH_CTL_VERB_MAX];
  double last_cmd_t;
  int quality;
  uint8_t res, pf;
  double fps, mbps;
  int secs;

  bool cam_seen;
  double cam_t;
  const char *cam_state; // ok | timeout | bad_len
  camera_rep_t cam;

  bool light_seen;
  double light_t;
  const char *light_state;
  light_rep_t light;
} s_ctl;

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
// Last completed TEL_STAT window, so the control socket reports the same
// fps/kBps the journal prints instead of inventing a second measurement.
static double s_fps_win = 0.0, s_kbps_win = 0.0;

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

// ---------------------------------------------------------------------------
// Telemetry role: still-save with JSON sidecars (S18 bite B)
// ---------------------------------------------------------------------------
//
// An accepted `capture` arms a one-shot save; the next completed frame is
// written to S18_CAPTURE_DIR with a sidecar recording every parameter and
// the measured stats AT CAPTURE TIME. Stream frames are never saved.
//
// Arming happens inside send_camera_req, so it covers the FIFO CLI and the
// control socket identically -- a still taken by hand is saved exactly like
// one taken from the web tool.
//
// Files land via a .tmp + rename(), and the JPEG lands BEFORE the sidecar:
// the sidecar is the COMMIT RECORD, so a sidecar can never point at a JPEG
// that is missing or half-written. Bite C's gallery enumerates sidecars.
//
// Nothing here deletes anything, ever. Below a free-space floor the save is
// refused and counted -- bench captures are evidence, and an instrument that
// silently destroys its own measurements is worse than one that stops.
//
// All of this runs on the app thread (telemetry_loop, cli_poll and ctl_poll
// are all called from loop()), so the state below needs no lock.

#define SAVE_WINDOW_MS 8000u  // how long a capture waits for its frame
#define SAVE_MIN_FREE_MB 200u // refuse below this; never delete

static std::string s_save_dir;            // empty = saving disabled
static const char *s_save_state = "idle"; // idle|armed|saved|timeout|error
static bool s_save_armed = false;
static uint64_t s_save_deadline_ms = 0;
static const char *s_save_source = "cli";
static uint8_t s_save_q = 0, s_save_res = 0, s_save_pf = 0;
static uint32_t s_save_arm_dropped = 0, s_save_arm_gaps = 0;
static char s_save_last[BENCH_CTL_NAME_MAX] = "";
static uint32_t s_save_last_bytes = 0;
static uint64_t s_saves = 0, s_save_errors = 0;

// Context for the NEXT camera command, set by whichever front end is about
// to issue it and consumed by send_camera_req. The FIFO CLI never sets it,
// so it sits at these defaults for hand-typed commands.
static const char *s_cmd_source = "cli";
static bool s_cmd_save = true;

static void save_dir_init(void) {
  const char *env = getenv("S18_CAPTURE_DIR");
  std::string dir;
  if (env && env[0]) {
    dir = env;
  } else {
    const char *home = getenv("HOME");
    dir = std::string((home && home[0]) ? home : "/home/pi") + "/bench_captures";
  }
  if (mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
    bm_log_warn("save: mkdir(%s) failed (%s) -- stills will NOT be saved",
                dir.c_str(), strerror(errno));
    return;
  }
  s_save_dir = dir;
  bm_log_info("save: stills -> %s (the .json sidecar is the commit record)",
              s_save_dir.c_str());
}

static uint64_t save_free_mb(void) {
  struct statvfs vfs;
  if (s_save_dir.empty() || statvfs(s_save_dir.c_str(), &vfs) != 0) {
    return 0;
  }
  return (uint64_t)vfs.f_bavail * (uint64_t)vfs.f_frsize / (1024ull * 1024ull);
}

static void save_arm(uint8_t q, uint8_t res, uint8_t pf) {
  if (s_save_dir.empty()) {
    return; // saving disabled at startup; already logged there
  }
  if (!s_cmd_save) {
    bm_log_info("save: capture requested with save=false -- not saving");
    return;
  }
  s_save_armed = true;
  s_save_state = "armed";
  s_save_deadline_ms = now_ms() + SAVE_WINDOW_MS;
  s_save_source = s_cmd_source;
  s_save_q = q; // 0 means "bridge default" -- recorded as commanded, not guessed
  s_save_res = res;
  s_save_pf = pf;
  // Ledger at arm time: the sidecar reports what moved DURING this capture,
  // which is the only way to answer "did this still lose anything?".
  s_save_arm_dropped = s_reasm.frames_dropped;
  s_save_arm_gaps = s_reasm.chunk_gaps;
}

static bool save_write_file(const std::string &path, const void *data,
                            size_t n) {
  std::string tmp = path + ".tmp";
  FILE *f = fopen(tmp.c_str(), "wb");
  if (!f) {
    return false;
  }
  bool ok = (n == 0) || (fwrite(data, 1, n, f) == n);
  if (fclose(f) != 0) {
    ok = false;
  }
  if (ok && rename(tmp.c_str(), path.c_str()) != 0) {
    ok = false;
  }
  if (!ok) {
    unlink(tmp.c_str());
  }
  return ok;
}

static void save_fail(const char *why, const char *detail) {
  s_save_errors++;
  s_save_state = "error";
  bm_log_warn("save: %s (%s)", why, detail);
  printf("CAP_SAVE ERROR %s (%s)\n", why, detail);
  fflush(stdout);
}

static void save_frame(uint32_t seq, const uint8_t *jpeg, size_t n,
                       uint16_t chunks) {
  if (!s_save_armed) {
    return;
  }
  s_save_armed = false;

  uint64_t free_mb = save_free_mb();
  if (free_mb < SAVE_MIN_FREE_MB) {
    char d[96];
    snprintf(d, sizeof(d), "%llu MB free, floor is %u MB",
             (unsigned long long)free_mb, (unsigned)SAVE_MIN_FREE_MB);
    save_fail("refusing to save, disk nearly full", d);
    return;
  }

  char stamp[BENCH_CTL_STAMP_MAX];
  char jpg_name[BENCH_CTL_NAME_MAX];
  char side_name[BENCH_CTL_NAME_MAX];
  time_t when = time(NULL);
  if (!bench_ctl_stamp(when, stamp, sizeof(stamp)) ||
      !bench_ctl_capture_name(stamp, seq, "jpg", jpg_name, sizeof(jpg_name)) ||
      !bench_ctl_capture_name(stamp, seq, "json", side_name,
                              sizeof(side_name))) {
    save_fail("could not build a capture filename", stamp);
    return;
  }

  std::string jpg_path = s_save_dir + "/" + jpg_name;
  if (!save_write_file(jpg_path, jpeg, n)) {
    save_fail("JPEG write failed", strerror(errno));
    return;
  }

  bench_ctl_sidecar_t sc;
  memset(&sc, 0, sizeof(sc));
  sc.file = jpg_name;
  sc.utc = stamp;
  sc.t = now_sec();
  sc.frame_seq = seq;
  sc.bytes = (uint32_t)n;
  sc.chunks = chunks;
  sc.source = s_save_source;
  sc.quality = s_save_q;
  sc.res = res_name_of(s_save_res);
  sc.pf = pf_name_of(s_save_pf);
  {
    std::lock_guard<std::mutex> g(s_ctl_lock);
    sc.reply_seen = (s_ctl.cam_seen && strcmp(s_ctl.cam_state, "ok") == 0) ? 1 : 0;
    sc.reply_ok = s_ctl.cam.ok;
    sc.reply_res = res_name_of(s_ctl.cam.res_active);
    sc.reply_pf = pf_name_of(s_ctl.cam.pf_active);
    sc.pub_ok = s_ctl.cam.pub_ok;
    sc.pub_errs = s_ctl.cam.pub_errs;
    sc.pub_bytes = s_ctl.cam.pub_bytes;
  }
  sc.frames_ok = s_reasm.frames_ok;
  sc.frames_dropped = s_reasm.frames_dropped;
  sc.chunk_gaps = s_reasm.chunk_gaps;
  sc.hdr_errors = s_reasm.hdr_errors;
  sc.dropped_delta = s_reasm.frames_dropped - s_save_arm_dropped;
  sc.gaps_delta = s_reasm.chunk_gaps - s_save_arm_gaps;
  sc.node = node_id();
  sc.camera_node = k_camera_node;

  static char side[BENCH_CTL_REPLY_MAX];
  int sn = bench_ctl_render_sidecar(&sc, side, sizeof(side));
  if (sn <= 0 || !save_write_file(s_save_dir + "/" + side_name, side,
                                  (size_t)sn)) {
    // The JPEG stays where it is: an orphan is invisible to a
    // sidecar-driven gallery, and deleting a capture to tidy up would
    // destroy the very thing this tool exists to collect.
    save_fail("sidecar write failed (JPEG kept, orphaned)", side_name);
    return;
  }

  s_saves++;
  s_save_state = "saved";
  s_save_last_bytes = (uint32_t)n;
  snprintf(s_save_last, sizeof(s_save_last), "%s", jpg_name);
  printf("CAP_SAVED file=%s bytes=%zu seq=%" PRIu32 " chunks=%u res=%s pf=%s "
         "q=%u src=%s gaps_delta=%" PRIu32 "\n",
         jpg_name, n, seq, (unsigned)chunks, sc.res, sc.pf, (unsigned)sc.quality,
         sc.source, sc.gaps_delta);
  fflush(stdout);
}

// A capture whose frame never arrives must say so. Silence would read as a
// still that saved fine.
static void save_tick(void) {
  if (s_save_armed && now_ms() > s_save_deadline_ms) {
    s_save_armed = false;
    s_save_state = "timeout";
    printf("CAP_SAVE TIMEOUT no frame within %u ms of the capture command\n",
           (unsigned)SAVE_WINDOW_MS);
    fflush(stdout);
  }
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
      // After the ingest: the browser stream is the frozen S3 path and gets
      // the frame first; a still is a copy, not a detour.
      save_frame(seq, s_jpeg_buf, done, s_reasm.count);
    }
  }
  save_tick();

  uint64_t ms = now_ms();
  if (ms - s_last_stat_ms >= 1000) {
    double dt = (double)(ms - s_last_stat_ms) / 1000.0;
    s_last_stat_ms = ms;
    s_fps_win = (double)s_frames_win / dt;
    s_kbps_win = (double)s_bytes_win / dt / 1000.0;
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
// Remember a reply for the control socket. `state` is a literal, so the
// pointer outlives every reader.
static void ctl_note_cam(const camera_rep_t *rep, const char *state) {
  std::lock_guard<std::mutex> g(s_ctl_lock);
  s_ctl.cam_seen = true;
  s_ctl.cam_t = now_sec();
  s_ctl.cam_state = state;
  if (rep) {
    s_ctl.cam = *rep;
  }
}

static bool camera_reply_cb(bool ack, uint32_t msg_id, size_t /*slen*/,
                            const char * /*service*/, size_t reply_len,
                            uint8_t *reply_data) {
  if (!ack) {
    printf("CAM_REPLY id=%" PRIu32 " TIMEOUT\n", msg_id);
    fflush(stdout);
    ctl_note_cam(nullptr, "timeout");
    return true;
  }
  camera_rep_t rep;
  if (reply_len != sizeof(rep)) {
    printf("CAM_REPLY id=%" PRIu32 " BAD_LEN %zu\n", msg_id, reply_len);
    fflush(stdout);
    ctl_note_cam(nullptr, "bad_len");
    return false;
  }
  memcpy(&rep, reply_data, sizeof(rep));
  ctl_note_cam(&rep, "ok");
  printf("CAM_REPLY id=%" PRIu32 " ok=%u mode=%u res=%s pf=%s cmds=%" PRIu32
         " pub_ok=%" PRIu32 " pub_errs=%" PRIu32 " pub_bytes=%" PRIu32 "\n",
         msg_id, rep.ok, rep.mode_active, res_name_of(rep.res_active),
         pf_name_of(rep.pf_active), rep.cmds, rep.pub_ok, rep.pub_errs,
         rep.pub_bytes);
  if (!rep.ok) {
    printf("CAM_REPLY REFUSED — check res/pf spelling "
           "(res = qvga|vga|hd, pf = color|mono)\n");
  }
  fflush(stdout);
  return true;
}

static void ctl_note_light(const light_rep_t *rep, const char *state) {
  std::lock_guard<std::mutex> g(s_ctl_lock);
  s_ctl.light_seen = true;
  s_ctl.light_t = now_sec();
  s_ctl.light_state = state;
  if (rep) {
    s_ctl.light = *rep;
  }
}

static bool light_reply_cb(bool ack, uint32_t msg_id, size_t /*slen*/,
                           const char * /*service*/, size_t reply_len,
                           uint8_t *reply_data) {
  if (!ack) {
    printf("LIGHT_REPLY id=%" PRIu32 " TIMEOUT\n", msg_id);
    fflush(stdout);
    ctl_note_light(nullptr, "timeout");
    return true;
  }
  light_rep_t rep;
  if (reply_len != sizeof(rep)) {
    printf("LIGHT_REPLY id=%" PRIu32 " BAD_LEN %zu\n", msg_id, reply_len);
    fflush(stdout);
    ctl_note_light(nullptr, "bad_len");
    return false;
  }
  memcpy(&rep, reply_data, sizeof(rep));
  ctl_note_light(&rep, "ok");
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

// Accepts "qvga"/"vga"/"hd" and "color"/"mono" (also "grey"/"gray"); an
// empty arg means 0 = bridge default. Anything else is passed through as
// an out-of-range value so the SERVICE refuses it and says so, rather
// than this CLI quietly picking something the operator didn't ask for.
static uint8_t parse_res(const char *s) {
  if (!s) return CAMERA_RES_DEFAULT;
  if (!strcasecmp(s, "qvga")) return CAMERA_RES_QVGA;
  if (!strcasecmp(s, "vga")) return CAMERA_RES_VGA;
  if (!strcasecmp(s, "hd")) return CAMERA_RES_HD;
  return 0xFF;
}
static uint8_t parse_pf(const char *s) {
  if (!s) return CAMERA_PF_DEFAULT;
  if (!strcasecmp(s, "color") || !strcasecmp(s, "colour"))
    return CAMERA_PF_COLOR;
  if (!strcasecmp(s, "mono") || !strcasecmp(s, "grey") ||
      !strcasecmp(s, "gray"))
    return CAMERA_PF_MONO;
  return 0xFF;
}

// Record what was just commanded. Called from send_camera_req, so it covers
// the FIFO CLI and the control socket identically -- the web tool's
// "commanded" half can never drift from what the operator typed by hand.
static void ctl_note_cmd(uint8_t cmd, uint8_t q, uint16_t fps_x10,
                         uint32_t rate_bps, uint16_t secs, uint8_t res,
                         uint8_t pf) {
  const char *name = (cmd == CAMERA_CMD_CAPTURE)  ? "capture"
                     : (cmd == CAMERA_CMD_STREAM) ? "stream"
                     : (cmd == CAMERA_CMD_STOP)   ? "stop"
                                                  : "cam-status";
  std::lock_guard<std::mutex> g(s_ctl_lock);
  s_ctl.params_seen = true;
  snprintf(s_ctl.last_cmd, sizeof(s_ctl.last_cmd), "%s", name);
  s_ctl.last_cmd_t = now_sec();
  // A status query carries no parameters; leave the last real ones standing.
  if (cmd == CAMERA_CMD_CAPTURE || cmd == CAMERA_CMD_STREAM) {
    s_ctl.quality = q;
    s_ctl.res = res;
    s_ctl.pf = pf;
    s_ctl.fps = (double)fps_x10 / 10.0;
    s_ctl.mbps = (double)rate_bps / 1e6;
    s_ctl.secs = secs;
  }
}

static void send_camera_req(uint8_t cmd, uint8_t q, uint16_t fps_x10,
                            uint32_t rate_bps, uint16_t secs, uint8_t res,
                            uint8_t pf) {
  ctl_note_cmd(cmd, q, fps_x10, rate_bps, secs, res, pf);
  if (cmd == CAMERA_CMD_CAPTURE) {
    save_arm(q, res, pf);
  }
  s_cmd_source = "cli"; // the context is per-command, never sticky
  s_cmd_save = true;
  camera_req_t req;
  memset(&req, 0, sizeof(req));
  req.magic = CAMERA_REQ_MAGIC;
  req.cmd = cmd;
  req.quality = q;
  req.fps_x10 = fps_x10;
  req.rate_bps = rate_bps;
  req.secs = secs;
  req.resolution = res;
  req.pixformat = pf;
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
         "  capture [q] [res] [pf]      trigger one frame\n"
         "  stream <mbps> <fps> <secs> [q] [res] [pf]\n"
         "                              start the camera stream\n"
         "      res = qvga|vga|hd (320x200 / 640x400 / 1280x800, 16:10)\n"
         "      pf  = color|mono          omit either for the default\n"
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
    const char *res = strtok(nullptr, " \t");
    const char *pf = strtok(nullptr, " \t");
    send_camera_req(CAMERA_CMD_CAPTURE, q ? (uint8_t)atoi(q) : 0, 0, 0, 0,
                    parse_res(res), parse_pf(pf));
  } else if (strcmp(cmd, "stream") == 0) {
    const char *mbps = strtok(nullptr, " \t");
    const char *fps = strtok(nullptr, " \t");
    const char *secs = strtok(nullptr, " \t");
    const char *q = strtok(nullptr, " \t");
    const char *res = strtok(nullptr, " \t");
    const char *pf = strtok(nullptr, " \t");
    send_camera_req(CAMERA_CMD_STREAM, q ? (uint8_t)atoi(q) : 0,
                    fps ? (uint16_t)(atof(fps) * 10) : 0,
                    mbps ? (uint32_t)(atof(mbps) * 1e6) : 0,
                    secs ? (uint16_t)atoi(secs) : 0, parse_res(res),
                    parse_pf(pf));
  } else if (strcmp(cmd, "stop") == 0) {
    send_camera_req(CAMERA_CMD_STOP, 0, 0, 0, 0, 0, 0);
  } else if (strcmp(cmd, "cam-status") == 0) {
    send_camera_req(CAMERA_CMD_STATUS, 0, 0, 0, 0, 0, 0);
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
// Telemetry role: bench control socket (S18 bite B)
// ---------------------------------------------------------------------------
//
// AF_UNIX SOCK_DGRAM, non-blocking, drained from loop() -- the same shape as
// the shipped gateway_ipc listener (src/net/gateway_ipc.cpp), for the same
// reasons: one datagram is one complete message, so there is no framing code,
// no partial-read buffer and no connection table to leak. It is node-local by
// construction: no port exists on any interface.
//
// Default path /run/bm/bench.sock lives in the unit's RuntimeDirectory, so it
// cannot outlive the process that reads it (S18 bite D's rule, and the reason
// bm-cmd.sh refuses to write a FIFO nobody is reading).
//
// Every request gets exactly one reply, including the malformed ones. A
// command that went nowhere must never look like a command that worked.

static int s_ctl_fd = -1;
static std::string s_ctl_path = "/run/bm/bench.sock";
static uint64_t s_ctl_reqs = 0, s_ctl_refused = 0;

static void ctl_init(void) {
  const char *env = getenv("S18_CTL_SOCK");
  if (env && env[0]) {
    s_ctl_path = env;
  }
  int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
  if (fd < 0) {
    bm_log_warn("ctl: socket() failed (%s) -- web bench tool unavailable",
                strerror(errno));
    return;
  }
  int fl = fcntl(fd, F_GETFL, 0);
  if (fl < 0 || fcntl(fd, F_SETFL, fl | O_NONBLOCK) < 0) {
    bm_log_warn("ctl: O_NONBLOCK failed (%s)", strerror(errno));
    close(fd);
    return;
  }
  int fdfl = fcntl(fd, F_GETFD, 0);
  if (fdfl >= 0) {
    fcntl(fd, F_SETFD, fdfl | FD_CLOEXEC);
  }
  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  if (s_ctl_path.size() >= sizeof(addr.sun_path)) {
    bm_log_warn("ctl: path too long (%zu) -- socket not created",
                s_ctl_path.size());
    close(fd);
    return;
  }
  strncpy(addr.sun_path, s_ctl_path.c_str(), sizeof(addr.sun_path) - 1);
  unlink(s_ctl_path.c_str()); // a stale socket from a crashed run
  if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    // Non-fatal on purpose: run the app by hand outside systemd and /run/bm
    // does not exist. The BM path must not depend on the bench tool.
    bm_log_warn("ctl: bind(%s) failed (%s) -- web bench tool unavailable "
                "(is the unit's RuntimeDirectory there?)",
                s_ctl_path.c_str(), strerror(errno));
    close(fd);
    return;
  }
  // Owner+group only: /run/bm is the unit's pi-owned RuntimeDirectory, and
  // the bench web server runs as the same user. Not world-writable -- unlike
  // gateway_ipc, this socket commands hardware.
  if (chmod(s_ctl_path.c_str(), 0660) < 0) {
    bm_log_warn("ctl: chmod(%s) failed (%s)", s_ctl_path.c_str(),
                strerror(errno));
  }
  s_ctl_fd = fd;
  bm_log_info("ctl: listening on %s (JSON in / JSON out)", s_ctl_path.c_str());
}

static void ctl_reply(const struct sockaddr_un *to, socklen_t tolen,
                      const char *buf, size_t len) {
  if (tolen <= (socklen_t)sizeof(sa_family_t)) {
    // An unbound client cannot be replied to. Say so loudly once per
    // occurrence: silence here would look exactly like a working command.
    bm_log_warn("ctl: client is not bound to an address -- reply dropped "
                "(the client must bind its own socket path)");
    return;
  }
  if (sendto(s_ctl_fd, buf, len, MSG_DONTWAIT, (const struct sockaddr *)to,
             tolen) < 0) {
    bm_log_warn("ctl: sendto failed (%s)", strerror(errno));
  }
}

// Dispatch one request. Verbs map 1:1 onto the FIFO CLI's handlers -- the
// same send_camera_req / send_light_req calls, the same parse_res / parse_pf
// pass-through -- so the socket and the operator CLI cannot drift apart.
static void ctl_dispatch(const char *msg, size_t len,
                         const struct sockaddr_un *from, socklen_t fromlen) {
  static char out[BENCH_CTL_REPLY_MAX];
  bench_ctl_req_t r;
  s_ctl_reqs++;

  if (!bench_ctl_parse_req(msg, len, &r)) {
    s_ctl_refused++;
    int n = bench_ctl_render_err(&r, r.err, out, sizeof(out));
    if (n > 0) {
      ctl_reply(from, fromlen, out, (size_t)n);
    }
    bm_log_warn("ctl: refused a request (%s)", r.err);
    return;
  }

  const uint8_t q = (r.quality >= 0) ? (uint8_t)r.quality : 0;
  const uint8_t res = r.res[0] ? parse_res(r.res) : CAMERA_RES_DEFAULT;
  const uint8_t pf = r.pf[0] ? parse_pf(r.pf) : CAMERA_PF_DEFAULT;

  // Context for the command this request is about to issue. Default is to
  // save: a still you forgot to flag is a still you lost, and this tool
  // exists to collect them. "save": false opts out explicitly.
  s_cmd_source = "socket";
  s_cmd_save = (r.save != 0);

  if (strcmp(r.verb, "capture") == 0) {
    send_camera_req(CAMERA_CMD_CAPTURE, q, 0, 0, 0, res, pf);
  } else if (strcmp(r.verb, "stream") == 0) {
    send_camera_req(CAMERA_CMD_STREAM, q,
                    (r.fps >= 0) ? (uint16_t)(r.fps * 10) : 0,
                    (r.mbps >= 0) ? (uint32_t)(r.mbps * 1e6) : 0,
                    (r.secs >= 0) ? (uint16_t)r.secs : 0, res, pf);
  } else if (strcmp(r.verb, "stop") == 0) {
    send_camera_req(CAMERA_CMD_STOP, 0, 0, 0, 0, 0, 0);
  } else if (strcmp(r.verb, "cam-status") == 0) {
    send_camera_req(CAMERA_CMD_STATUS, 0, 0, 0, 0, 0, 0);
  } else if (strcmp(r.verb, "light") == 0) {
    send_light_req(LIGHT_CMD_LEVEL, (r.level >= 0) ? (uint8_t)r.level : 0, 0, 0,
                   0);
  } else if (strcmp(r.verb, "strobe") == 0) {
    // Same defaults as the FIFO CLI's `strobe` with arguments omitted.
    send_light_req(LIGHT_CMD_STROBE, 0,
                   (r.on_ms >= 0) ? (uint16_t)r.on_ms : 200,
                   (r.off_ms >= 0) ? (uint16_t)r.off_ms : 200,
                   (r.count >= 0) ? (uint16_t)r.count : 5);
  } else if (strcmp(r.verb, "light-status") == 0) {
    send_light_req(LIGHT_CMD_QUERY, 0, 0, 0, 0);
  } else if (strcmp(r.verb, "status") == 0) {
    bench_ctl_status_t st;
    bench_ctl_status_init(&st);
    st.t = now_sec();
    st.node = node_id();
    {
      std::lock_guard<std::mutex> g(s_ctl_lock);
      st.params_seen = s_ctl.params_seen ? 1 : 0;
      st.last_cmd = s_ctl.last_cmd;
      st.last_cmd_t = s_ctl.last_cmd_t;
      st.quality = s_ctl.quality;
      st.res = res_name_of(s_ctl.res);
      st.pf = pf_name_of(s_ctl.pf);
      st.fps = s_ctl.fps;
      st.mbps = s_ctl.mbps;
      st.secs = s_ctl.secs;
      st.cam_seen = s_ctl.cam_seen ? 1 : 0;
      st.cam_t = s_ctl.cam_t;
      st.cam_state = s_ctl.cam_state;
      st.cam_ok = s_ctl.cam.ok;
      st.cam_mode = s_ctl.cam.mode_active;
      st.cam_res = res_name_of(s_ctl.cam.res_active);
      st.cam_pf = pf_name_of(s_ctl.cam.pf_active);
      st.cam_cmds = s_ctl.cam.cmds;
      st.pub_ok = s_ctl.cam.pub_ok;
      st.pub_errs = s_ctl.cam.pub_errs;
      st.pub_bytes = s_ctl.cam.pub_bytes;
      st.light_seen = s_ctl.light_seen ? 1 : 0;
      st.light_t = s_ctl.light_t;
      st.light_state = s_ctl.light_state;
      st.light_ok = s_ctl.light.ok;
      st.light_level = s_ctl.light.level;
      st.light_strobing = s_ctl.light.strobing;
      st.light_cmds = s_ctl.light.cmds;
      st.light_uptime_s = s_ctl.light.uptime_s;
    }
    // The receiver ledger: the honest half of commanded-vs-actual (D21).
    st.frames_ok = s_reasm.frames_ok;
    st.frames_dropped = s_reasm.frames_dropped;
    st.chunk_gaps = s_reasm.chunk_gaps;
    st.hdr_errors = s_reasm.hdr_errors;
    st.oversize = s_reasm.oversize;
    {
      std::lock_guard<std::mutex> g(s_q_lock);
      st.q_drops = s_q_drops;
    }
    st.ingest_ok = s_ingest_frames;
    st.ingest_fail = s_ingest_fails;
    st.uplinks = s_uplink_sent;
    st.fps_win = s_fps_win;
    st.kbps_win = s_kbps_win;
    st.ipc_up = s_ipc_up ? 1 : 0;
    st.save_state = s_save_state;
    st.save_file = s_save_last;
    st.save_bytes = s_save_last_bytes;
    st.saves = s_saves;
    st.save_errors = s_save_errors;
    st.disk_free_mb = save_free_mb();
    st.save_dir = s_save_dir.empty() ? "(disabled)" : s_save_dir.c_str();
    int n = bench_ctl_render_status(&st, &r, out, sizeof(out));
    if (n <= 0) {
      s_ctl_refused++;
      n = bench_ctl_render_err(&r, "status did not fit the reply buffer", out,
                               sizeof(out));
    }
    if (n > 0) {
      ctl_reply(from, fromlen, out, (size_t)n);
    }
    return;
  } else {
    s_ctl_refused++;
    char why[96];
    snprintf(why, sizeof(why), "unknown cmd '%s'", r.verb);
    int n = bench_ctl_render_err(&r, why, out, sizeof(out));
    if (n > 0) {
      ctl_reply(from, fromlen, out, (size_t)n);
    }
    return;
  }

  int n = bench_ctl_render_ack(&r, out, sizeof(out));
  if (n > 0) {
    ctl_reply(from, fromlen, out, (size_t)n);
  }
}

static void ctl_poll(void) {
  if (s_ctl_fd < 0) {
    return;
  }
  for (;;) {
    static char buf[BENCH_CTL_MSG_MAX];
    struct sockaddr_un from;
    socklen_t fromlen = sizeof(from);
    memset(&from, 0, sizeof(from));
    // MSG_TRUNC makes recvfrom report the datagram's REAL size even when it
    // did not fit, so an oversize message is refused out loud instead of
    // being parsed as whatever survived the truncation.
    ssize_t n = recvfrom(s_ctl_fd, buf, sizeof(buf), MSG_TRUNC,
                         (struct sockaddr *)&from, &fromlen);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      if (errno == EINTR) {
        continue;
      }
      bm_log_warn("ctl: recvfrom failed (%s)", strerror(errno));
      return;
    }
    if (n == 0) {
      continue;
    }
    ctl_dispatch(buf, (size_t)n, &from, fromlen);
  }
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
    save_dir_init(); // still-save target (S18 bite B); failure is non-fatal
    ctl_init();      // bench control socket (S18 bite B); ditto
    bm_log_info("telemetry: node %016" PRIx64 " subscribed to %s, "
                "ingest -> %s:%d, uplink every %" PRIu64 "s, ipc=%s, ctl=%s",
                node_id(), k_camera_topic, s_ingest_host.c_str(),
                s_ingest_port, s_uplink_secs, s_ipc_up ? "up" : "down",
                s_ctl_fd >= 0 ? s_ctl_path.c_str() : "off");
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
    ctl_poll();
    uplink_tick();
  }
}
