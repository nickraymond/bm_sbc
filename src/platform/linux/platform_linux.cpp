#include "platform_linux.h"
#include "bm_config.h"
#include "critical_op.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <libgen.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

// bm_configs_generic.h, bm_rtc.h, bm_dfu_generic.h have no extern "C" guards.
extern "C" {
#include "bm_configs_generic.h"
#include "bm_dfu_generic.h"
#include "bm_rtc.h"
#include "dfu.h"
}

int platform_linux_init(void) { return 0; }

// ---------------------------------------------------------------------------
// Config partition — file-backed persistence
// ---------------------------------------------------------------------------

static char s_cfg_dir[512] = {0};
static char s_cfg_paths[BM_CFG_PARTITION_COUNT][512];
static bool s_cfg_dir_set = false;

static const char *k_partition_filenames[BM_CFG_PARTITION_COUNT] = {
    "config.user.bin",
    "config.sys.bin",
    "config.hw.bin",
};

void platform_linux_set_cfg_dir(const char *dir) {
  if (!dir) {
    return;
  }
  // The cfg dir must exist before bm_sbc starts.
  // Fail loudly rather than silently disabling persistence on a typo.
  struct stat st;
  if (stat(dir, &st) != 0 || !S_ISDIR(st.st_mode)) {
    bm_log_error("platform_linux_set_cfg_dir: %s is not an existing directory "
                 "(%s) — config persistence disabled!",
                 dir, strerror(errno));
    return;
  }
  snprintf(s_cfg_dir, sizeof(s_cfg_dir), "%s", dir);
  for (int i = 0; i < BM_CFG_PARTITION_COUNT; i++) {
    snprintf(s_cfg_paths[i], sizeof(s_cfg_paths[i]), "%s/%s", dir,
             k_partition_filenames[i]);
  }
  s_cfg_dir_set = true;
}

bool bm_config_read(BmConfigPartition partition, uint32_t offset,
                    uint8_t *buffer, size_t length, uint32_t timeout_ms) {
  (void)timeout_ms;
  if (!buffer || !length) {
    return true;
  }
  // No cfg-dir configured — return zeros (fresh partition).
  if (!s_cfg_dir_set || partition >= BM_CFG_PARTITION_COUNT) {
    memset(buffer, 0, length);
    return true;
  }
  FILE *f = fopen(s_cfg_paths[partition], "rb");
  if (!f) {
    // File doesn't exist yet — return zeros so config_init() creates a fresh
    // partition, which will be written back via bm_config_write on first save.
    memset(buffer, 0, length);
    return true;
  }
  if (fseek(f, (long)offset, SEEK_SET) != 0) {
    fclose(f);
    memset(buffer, 0, length);
    return true;
  }
  size_t n = fread(buffer, 1, length, f);
  fclose(f);
  // Zero-fill any remainder (file may be shorter than requested length).
  if (n < length) {
    memset(buffer + n, 0, length - n);
  }
  return true;
}

// Atomic write: stage the new contents in a sibling tempfile,
// fsync, then rename over the target.
// On any failure the target is left untouched.
bool bm_config_write(BmConfigPartition partition, uint32_t offset,
                     uint8_t *buffer, size_t length, uint32_t timeout_ms) {
  (void)timeout_ms;
  if (partition >= BM_CFG_PARTITION_COUNT) {
    return false;
  }
  if (!buffer || !length) {
    return false;
  }
  if (!s_cfg_dir_set) {
    bm_log_error(
        "bm_config_write: no cfg-dir set; unable to persist partition %u",
        (unsigned)partition);
    return false;
  }

  const char *target = s_cfg_paths[partition];

  // Stage final contents in a tempfile we'll rename into place.
  size_t final_size = (size_t)offset + length;
  uint8_t *staging = (uint8_t *)calloc(1, final_size);
  if (!staging) {
    return false;
  }
  FILE *existing = fopen(target, "rb");
  if (existing) {
    // Read up to final_size bytes of existing content so we don't clobber
    // unmodified regions outside [offset, offset+length).
    fread(staging, 1, final_size, existing);
    // If existing file is longer than final_size, fold its tail in too.
    if (fseek(existing, 0, SEEK_END) == 0) {
      long actual = ftell(existing);
      if (actual > (long)final_size) {
        size_t extra = (size_t)actual - final_size;
        uint8_t *grown = (uint8_t *)realloc(staging, (size_t)actual);
        if (!grown) {
          free(staging);
          fclose(existing);
          return false;
        }
        staging = grown;
        if (fseek(existing, (long)final_size, SEEK_SET) == 0) {
          fread(staging + final_size, 1, extra, existing);
        }
        final_size = (size_t)actual;
      }
    }
    fclose(existing);
  }
  memcpy(staging + offset, buffer, length);

  char tmp_path[512 + 16];
  snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", target);
  int fd = open(tmp_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    free(staging);
    return false;
  }
  ssize_t w = write(fd, staging, final_size);
  free(staging);
  if (w != (ssize_t)final_size) {
    close(fd);
    unlink(tmp_path);
    return false;
  }
  if (fsync(fd) != 0) {
    close(fd);
    unlink(tmp_path);
    return false;
  }
  if (close(fd) != 0) {
    unlink(tmp_path);
    return false;
  }
  if (rename(tmp_path, target) != 0) {
    unlink(tmp_path);
    return false;
  }
  // Best-effort directory fsync so the rename is durable across power loss.
  int dfd = open(s_cfg_dir, O_RDONLY | O_DIRECTORY);
  if (dfd >= 0) {
    fsync(dfd);
    close(dfd);
  }
  return true;
}

void bm_config_reset(void) {}

// ---------------------------------------------------------------------------
// RTC — backed by CLOCK_REALTIME
// ---------------------------------------------------------------------------

BmErr bm_rtc_set(const RtcTimeAndDate *time_and_date) {
  struct tm t = {};
  t.tm_year = time_and_date->year - 1900;
  t.tm_mon = time_and_date->month - 1;
  t.tm_mday = time_and_date->day;
  t.tm_hour = time_and_date->hour;
  t.tm_min = time_and_date->minute;
  t.tm_sec = time_and_date->second;

  struct timespec ts;
  ts.tv_sec = timegm(&t);
  ts.tv_nsec = (long)time_and_date->ms * 1000000L;

  if (clock_settime(CLOCK_REALTIME, &ts) != 0) {
    if (errno == EPERM || errno == EACCES) {
      bm_log_error("bm_rtc_set: insufficient privileges to set system clock");
      return BmEPERM;
    }
    return BmEIO;
  }
  return BmOK;
}

BmErr bm_rtc_get(RtcTimeAndDate *time_and_date) {
  struct timespec ts;
  if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
    return BmEIO;
  }
  struct tm t;
  gmtime_r(&ts.tv_sec, &t);
  time_and_date->year = (uint16_t)(t.tm_year + 1900);
  time_and_date->month = (uint8_t)(t.tm_mon + 1);
  time_and_date->day = (uint8_t)t.tm_mday;
  time_and_date->hour = (uint8_t)t.tm_hour;
  time_and_date->minute = (uint8_t)t.tm_min;
  time_and_date->second = (uint8_t)t.tm_sec;
  time_and_date->ms = (uint16_t)(ts.tv_nsec / 1000000UL);
  return BmOK;
}

uint64_t bm_rtc_get_micro_seconds(RtcTimeAndDate *time_and_date) {
  struct timespec ts;
  if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
    return 0;
  }
  if (time_and_date) {
    struct tm t;
    gmtime_r(&ts.tv_sec, &t);
    time_and_date->year = (uint16_t)(t.tm_year + 1900);
    time_and_date->month = (uint8_t)(t.tm_mon + 1);
    time_and_date->day = (uint8_t)t.tm_mday;
    time_and_date->hour = (uint8_t)t.tm_hour;
    time_and_date->minute = (uint8_t)t.tm_min;
    time_and_date->second = (uint8_t)t.tm_sec;
    time_and_date->ms = (uint16_t)(ts.tv_nsec / 1000000UL);
  }
  return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000UL);
}

// ---------------------------------------------------------------------------
// DFU — file-backed binary swap with execv() restart
// ---------------------------------------------------------------------------

static char   s_install_path[PATH_MAX] = {0};
static char   s_staging_path[PATH_MAX] = {0};
static char   s_backup_path[PATH_MAX]  = {0};
static char   s_marker_path[PATH_MAX]  = {0};
static int    s_dfu_fd                 = -1;
static char **s_saved_argv             = NULL;
static void (*s_pre_exec_cb)(void)     = NULL;
// Sentinel used as the flash_area opaque handle (address passed to callers).
static int    s_flash_area_tag         = 0;

// Derive all DFU-related file paths from /proc/self/exe.
static bool derive_dfu_paths(void) {
  ssize_t n = readlink("/proc/self/exe", s_install_path,
                       sizeof(s_install_path) - 1);
  if (n <= 0) {
    bm_log_error("platform_linux_set_argv: readlink(/proc/self/exe) failed: %s",
                 strerror(errno));
    return false;
  }
  s_install_path[n] = '\0';

  snprintf(s_staging_path, sizeof(s_staging_path), "%s.staging",
           s_install_path);
  snprintf(s_backup_path,  sizeof(s_backup_path),  "%s.bak",
           s_install_path);

  // dfu_pending.bin lives in the same directory as the binary.
  char dir_buf[PATH_MAX];
  strncpy(dir_buf, s_install_path, sizeof(dir_buf) - 1);
  dir_buf[sizeof(dir_buf) - 1] = '\0';
  char *dir = dirname(dir_buf); // may modify dir_buf in-place (POSIX)
  snprintf(s_marker_path, sizeof(s_marker_path), "%s/dfu_pending.bin", dir);
  return true;
}

void platform_linux_set_argv(int argc, char **argv) {
  (void)argc;
  s_saved_argv = argv;
  derive_dfu_paths();
}

void platform_linux_dfu_restore_state(void) {
  FILE *f = fopen(s_marker_path, "rb");
  if (!f) {
    return; // no pending DFU — clean boot
  }
  ReboootClientUpdateInfo info;
  memset(&info, 0, sizeof(info));
  size_t n = fread(&info, 1, sizeof(info), f);
  fclose(f);
  if (n != sizeof(info) || info.magic != DFU_REBOOT_MAGIC) {
    bm_log_warn("dfu_restore_state: marker file corrupt or truncated — ignoring");
    memset(&client_update_reboot_info, 0, sizeof(client_update_reboot_info));
    return;
  }
  client_update_reboot_info = info;
  bm_log_info("dfu_restore_state: pending DFU resumed (host=0x%016" PRIx64
              " sha=0x%08" PRIx32 ")",
              info.host_node_id, info.gitSHA);
}

// ---------------------------------------------------------------------------
// Staging-file validation helpers
// ---------------------------------------------------------------------------

// Check ELF magic and machine type (EM_AARCH64 = 0x00B7).
static bool validate_staging_elf(void) {
  // ELF header layout (little-endian):
  //   bytes  0-3   e_ident magic  (\x7f E L F)
  //   bytes 16-17  e_type
  //   bytes 18-19  e_machine
  uint8_t hdr[20];
  FILE *f = fopen(s_staging_path, "rb");
  if (!f) {
    bm_log_error("dfu validate_elf: cannot open staging file: %s",
                 strerror(errno));
    return false;
  }
  size_t n = fread(hdr, 1, sizeof(hdr), f);
  fclose(f);
  if (n < sizeof(hdr)) {
    bm_log_error("dfu validate_elf: staging file too small (%zu bytes)", n);
    return false;
  }
  static const uint8_t k_elf_magic[4] = {0x7f, 'E', 'L', 'F'};
  if (memcmp(hdr, k_elf_magic, 4) != 0) {
    bm_log_error("dfu validate_elf: bad ELF magic");
    return false;
  }
  // e_machine at bytes 18-19, little-endian.  EM_AARCH64 = 0x00B7.
  uint16_t e_machine = (uint16_t)(hdr[18] | ((uint16_t)hdr[19] << 8));
  if (e_machine != 0x00B7u) {
    bm_log_error("dfu validate_elf: wrong architecture "
                 "(e_machine=0x%04x, expected 0x00B7)",
                 (unsigned)e_machine);
    return false;
  }
  return true;
}

// Search the staging binary for the app-identity marker string baked into
// .rodata at compile time by main.cpp: "BM_SBC_IMAGE:<app_name>".
static bool validate_staging_marker(void) {
  char expected[64];
  snprintf(expected, sizeof(expected), "BM_SBC_IMAGE:%s",
           bm_sbc_app_name_runtime);
  size_t expected_len = strlen(expected);

  int fd = open(s_staging_path, O_RDONLY);
  if (fd < 0) {
    bm_log_error("dfu validate_marker: cannot open staging file: %s",
                 strerror(errno));
    return false;
  }
  struct stat st;
  if (fstat(fd, &st) != 0 || st.st_size <= 0) {
    bm_log_error("dfu validate_marker: fstat failed");
    close(fd);
    return false;
  }
  void *map = mmap(NULL, (size_t)st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
  close(fd);
  if (map == MAP_FAILED) {
    bm_log_error("dfu validate_marker: mmap failed: %s", strerror(errno));
    return false;
  }
  bool found =
      (memmem(map, (size_t)st.st_size, expected, expected_len) != NULL);
  munmap(map, (size_t)st.st_size);
  if (!found) {
    bm_log_error("dfu validate_marker: marker \"%s\" not found in staging binary",
                 expected);
  }
  return found;
}

void platform_linux_set_pre_exec_cb(void (*cb)(void)) {
  s_pre_exec_cb = cb;
}

// Close all fds > 2 before execv() to avoid leaking UART/socket fds.
static void close_fds_above_stderr(void) {
  long max_fds = sysconf(_SC_OPEN_MAX);
  if (max_fds < 0) {
    max_fds = 1024;
  }
  for (long fd = 3; fd < max_fds; ++fd) {
    close((int)fd);
  }
}

// ---------------------------------------------------------------------------
// Flash-area (staging-file) operations
// ---------------------------------------------------------------------------

BmErr bm_dfu_client_flash_area_open(const void **flash_area) {
  if (!flash_area) {
    return BmEINVAL;
  }
  if (s_staging_path[0] == '\0') {
    bm_log_error("bm_dfu_client_flash_area_open: paths not initialised");
    return BmEPERM;
  }
  if (s_dfu_fd >= 0) {
    close(s_dfu_fd);
  }
  s_dfu_fd = open(s_staging_path, O_RDWR | O_CREAT | O_TRUNC, 0644);
  if (s_dfu_fd < 0) {
    bm_log_error("bm_dfu_client_flash_area_open: open(%s) failed: %s",
                 s_staging_path, strerror(errno));
    return BmEIO;
  }
  *flash_area = &s_flash_area_tag;

  sbc_critical_op(true);
  return BmOK;
}

BmErr bm_dfu_client_flash_area_close(const void *flash_area) {
  (void)flash_area;
  if (s_dfu_fd >= 0) {
    close(s_dfu_fd);
    s_dfu_fd = -1;
  }
  return BmOK;
}

BmErr bm_dfu_client_flash_area_write(const void *flash_area, uint32_t off,
                                     const void *src, uint32_t len) {
  (void)flash_area;
  if (s_dfu_fd < 0) {
    return BmEIO;
  }
  ssize_t w = pwrite(s_dfu_fd, src, (size_t)len, (off_t)off);
  if (w != (ssize_t)len) {
    bm_log_error("bm_dfu_client_flash_area_write: pwrite failed: %s",
                 strerror(errno));
    return BmEIO;
  }
  return BmOK;
}

BmErr bm_dfu_client_flash_area_erase(const void *flash_area, uint32_t off,
                                     uint32_t len) {
  (void)flash_area;
  if (s_dfu_fd < 0) {
    return BmEIO;
  }
  // Zero-fill the region (matches erase-to-zero semantics).
  static const uint8_t k_zeros[256] = {0};
  uint32_t remaining = len;
  off_t cur = (off_t)off;
  while (remaining > 0) {
    uint32_t chunk =
        remaining < (uint32_t)sizeof(k_zeros) ? remaining
                                              : (uint32_t)sizeof(k_zeros);
    ssize_t w = pwrite(s_dfu_fd, k_zeros, (size_t)chunk, cur);
    if (w != (ssize_t)chunk) {
      bm_log_error("bm_dfu_client_flash_area_erase: pwrite failed: %s",
                    strerror(errno));
      return BmEIO;
    }
    cur += (off_t)chunk;
    remaining -= chunk;
  }
  return BmOK;
}

uint32_t bm_dfu_client_flash_area_get_size(const void *flash_area) {
  (void)flash_area;
  return 256u * 1024u * 1024u; // 256 MiB — ample for any Pi binary
}

// ---------------------------------------------------------------------------
// DFU control functions
// ---------------------------------------------------------------------------

BmErr bm_dfu_client_set_pending_and_reset(void) {
  if (s_staging_path[0] == '\0' || s_saved_argv == NULL) {
    bm_log_error("bm_dfu_client_set_pending_and_reset: not initialised");
    return BmEPERM;
  }

  // 1. Validate the staging binary before touching the running binary.
  if (!validate_staging_elf() || !validate_staging_marker()) {
    bm_log_error("dfu set_pending: staging binary failed validation — aborting");
    return BmEINVAL;
  }

  // 2. Flush and close the staging fd.
  if (s_dfu_fd >= 0) {
    fsync(s_dfu_fd);
    close(s_dfu_fd);
    s_dfu_fd = -1;
  }

  // 3. Write the DFU marker file (noinit-RAM substitute) before the swap so
  //    the new process image sees DFU_REBOOT_MAGIC after execv().
  FILE *mf = fopen(s_marker_path, "wb");
  if (!mf) {
    bm_log_error("dfu set_pending: cannot write marker %s: %s",
                 s_marker_path, strerror(errno));
    return BmEIO;
  }
  size_t written = fwrite(&client_update_reboot_info, 1,
                          sizeof(client_update_reboot_info), mf);
  fflush(mf);
  fsync(fileno(mf));
  fclose(mf);
  if (written != sizeof(client_update_reboot_info)) {
    bm_log_error("dfu set_pending: short write to marker file");
    unlink(s_marker_path);
    return BmEIO;
  }

  // 4. Hard-link the current binary to .bak for rollback.
  //    Remove any stale .bak first so link() is idempotent.
  unlink(s_backup_path);
  if (link(s_install_path, s_backup_path) != 0) {
    bm_log_warn("dfu set_pending: failed to create backup link %s: %s",
                s_backup_path, strerror(errno));
    // Non-fatal: rollback is unavailable but the swap can still proceed.
  }

  // 5. Atomically replace the install binary with the staging binary.
  if (rename(s_staging_path, s_install_path) != 0) {
    bm_log_error("dfu set_pending: rename(%s -> %s) failed: %s",
                 s_staging_path, s_install_path, strerror(errno));
    unlink(s_marker_path);
    return BmEIO;
  }
  chmod(s_install_path, 0755);

  // 6. Replace this process image with the new binary (transparent to systemd).
  bm_log_info("dfu set_pending: binary swapped, restarting via execv");
  if (s_pre_exec_cb) { s_pre_exec_cb(); }
  bm_log_shutdown();
  close_fds_above_stderr();
  execv(s_install_path, s_saved_argv);

  // execv() returned — something went badly wrong.  Roll back.
  bm_log_error("dfu set_pending: execv failed: %s — rolling back",
               strerror(errno));
  rename(s_backup_path, s_install_path);
  chmod(s_install_path, 0755);
  unlink(s_marker_path);
  return BmEIO;
}

BmErr bm_dfu_client_set_confirmed(void) {
  unlink(s_marker_path);
  unlink(s_backup_path);

  sbc_critical_op(false);
  return BmOK;
}

BmErr bm_dfu_client_fail_update_and_reset(void) {
  if (s_saved_argv == NULL) {
    bm_log_error("bm_dfu_client_fail_update_and_reset: not initialised");
    return BmEPERM;
  }

  // Roll back to the .bak binary if present.
  struct stat st;
  if (stat(s_backup_path, &st) == 0) {
    if (rename(s_backup_path, s_install_path) == 0) {
      chmod(s_install_path, 0755);
      bm_log_info("dfu fail_update: rolled back to previous binary");
    } else {
      bm_log_error("dfu fail_update: rollback rename failed: %s",
                   strerror(errno));
    }
  }

  unlink(s_marker_path);
  bm_log_info("dfu fail_update: restarting via execv");
  if (s_pre_exec_cb) { s_pre_exec_cb(); }
  bm_log_shutdown();
  close_fds_above_stderr();
  execv(s_install_path, s_saved_argv);

  bm_log_error("dfu fail_update: execv failed: %s", strerror(errno));
  return BmEIO;
}

BmErr bm_dfu_host_get_chunk(uint32_t offset, uint8_t *buffer, size_t len,
                            uint32_t timeouts) {
  (void)offset;
  (void)buffer;
  (void)len;
  (void)timeouts;
  return BmEPERM;
}

void bm_dfu_core_lpm_peripheral_active(void) {}
void bm_dfu_core_lpm_peripheral_inactive(void) {}
