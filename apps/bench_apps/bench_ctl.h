/// @file bench_ctl.h
/// @brief Wire format for the bench control socket (S18 bite B).
///
/// The telemetry role carries a loopback-only control socket
/// (AF_UNIX SOCK_DGRAM, /run/bm/bench.sock) so the S18 web bench tool can
/// drive the camera and read the receiver ledger without a terminal. One
/// JSON object per datagram in, one JSON object per datagram out.
///
/// EVERYTHING PARSEABLE OR PRINTABLE LIVES HERE, and nothing here touches
/// the OS: no sockets, no files, no clock. app_main.cpp fills a POD struct
/// and does the I/O; this header turns bytes into a request and structs
/// into JSON. That split is what makes the wire format unit-testable on a
/// laptop (tests/test_bench_ctl.c) instead of only on a live chain.
///
/// Header-only, pure C (the app is C++, the test is C -- both compile it).
///
/// DELIBERATE NON-VALIDATION: `res` and `pf` are copied through verbatim.
/// D31 made out-of-range geometry the SERVICE's refusal to make (reply
/// ok=0, "CAM_REPLY REFUSED"), precisely so a typo can never be quietly
/// substituted with something the operator didn't ask for. Validating
/// spelling here would move that decision to the wrong end of the chain.

#ifndef BENCH_CTL_H
#define BENCH_CTL_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define BENCH_CTL_SCHEMA 1
#define BENCH_CTL_MSG_MAX 2048u   // request datagram cap
#define BENCH_CTL_REPLY_MAX 4096u // reply datagram cap (status is the big one)
#define BENCH_CTL_VERB_MAX 16u    // "light-status" is the longest verb
#define BENCH_CTL_TOK_MAX 8u      // "colour" is the longest res/pf token
#define BENCH_CTL_NAME_MAX 64u    // capture file basename
#define BENCH_CTL_STAMP_MAX 20u   // "20260815T193012Z" + NUL

// ---------------------------------------------------------------------------
// Flat-object JSON scanner
// ---------------------------------------------------------------------------
//
// Bounded, single pass, no allocation, input need not be NUL-terminated.
// Nested objects/arrays are skipped WHOLE rather than descended, so a key
// inside a nested value can never be mistaken for a top-level one. That is
// the only structural subtlety here; everything else is a byte loop.

static inline int bench_ctl_is_ws_(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

/// Locate the value slice for `key` in a flat JSON object. Returns 1 and
/// sets *val/*vlen (whitespace-trimmed) on a hit, 0 otherwise.
static inline int bench_ctl_find(const char *msg, size_t len, const char *key,
                                 const char **val, size_t *vlen) {
  size_t klen = strlen(key);
  size_t i = 0;
  while (i < len && bench_ctl_is_ws_(msg[i])) {
    i++;
  }
  if (i >= len || msg[i] != '{') {
    return 0;
  }
  i++;
  while (i < len) {
    while (i < len && (bench_ctl_is_ws_(msg[i]) || msg[i] == ',')) {
      i++;
    }
    if (i >= len || msg[i] == '}') {
      return 0; // end of object: key absent
    }
    if (msg[i] != '"') {
      return 0; // malformed where a key must be
    }
    size_t kstart = ++i;
    while (i < len && msg[i] != '"') {
      i += (msg[i] == '\\') ? 2 : 1;
    }
    if (i >= len) {
      return 0;
    }
    size_t kn = i - kstart;
    i++; // past the closing quote
    while (i < len && bench_ctl_is_ws_(msg[i])) {
      i++;
    }
    if (i >= len || msg[i] != ':') {
      return 0;
    }
    i++;
    while (i < len && bench_ctl_is_ws_(msg[i])) {
      i++;
    }
    // Value slice: run to the ',' or '}' that closes it AT THIS DEPTH.
    size_t vstart = i;
    int depth = 0;
    int instr = 0;
    while (i < len) {
      char c = msg[i];
      if (instr) {
        if (c == '\\') {
          i++;
        } else if (c == '"') {
          instr = 0;
        }
      } else if (c == '"') {
        instr = 1;
      } else if (c == '{' || c == '[') {
        depth++;
      } else if (c == '}' || c == ']') {
        if (depth == 0) {
          break;
        }
        depth--;
      } else if (c == ',' && depth == 0) {
        break;
      }
      i++;
    }
    size_t vend = i;
    while (vend > vstart && bench_ctl_is_ws_(msg[vend - 1])) {
      vend--;
    }
    if (kn == klen && memcmp(msg + kstart, key, klen) == 0) {
      *val = msg + vstart;
      *vlen = vend - vstart;
      return vend > vstart; // an empty value is not a value
    }
  }
  return 0;
}

/// Copy a string value into `out`. Accepts a quoted string (minimal escape
/// handling) or a bare token, so a hand-typed test datagram works too.
/// Returns 1 on success, 0 if absent or too long for `cap`.
static inline int bench_ctl_get_str(const char *msg, size_t len,
                                    const char *key, char *out, size_t cap) {
  const char *v;
  size_t n;
  if (!bench_ctl_find(msg, len, key, &v, &n)) {
    return 0;
  }
  size_t o = 0;
  if (v[0] == '"') {
    for (size_t i = 1; i < n; i++) {
      char c = v[i];
      if (c == '"') {
        break;
      }
      if (c == '\\' && i + 1 < n) {
        char e = v[++i];
        c = (e == 'n') ? '\n' : (e == 't') ? '\t' : e;
      }
      if (o + 1 >= cap) {
        return 0; // too long: refuse rather than silently truncate
      }
      out[o++] = c;
    }
  } else {
    if (n + 1 > cap) {
      return 0;
    }
    memcpy(out, v, n);
    o = n;
  }
  out[o] = '\0';
  return 1;
}

/// Copy a numeric value. Returns 1 on success, 0 if absent or unparseable.
static inline int bench_ctl_get_num(const char *msg, size_t len,
                                    const char *key, double *out) {
  const char *v;
  size_t n;
  if (!bench_ctl_find(msg, len, key, &v, &n) || n >= 32) {
    return 0;
  }
  char buf[32];
  memcpy(buf, v, n);
  buf[n] = '\0';
  char *end = NULL;
  double d = strtod(buf, &end);
  if (end == buf) {
    return 0;
  }
  while (end && *end && bench_ctl_is_ws_(*end)) {
    end++;
  }
  if (end && *end) {
    return 0; // trailing junk: "50abc" is not a number
  }
  *out = d;
  return 1;
}

/// Copy a boolean value (true/false, or 1/0). Returns 1 on success.
static inline int bench_ctl_get_bool(const char *msg, size_t len,
                                     const char *key, int *out) {
  const char *v;
  size_t n;
  if (!bench_ctl_find(msg, len, key, &v, &n)) {
    return 0;
  }
  if (n == 4 && memcmp(v, "true", 4) == 0) {
    *out = 1;
    return 1;
  }
  if (n == 5 && memcmp(v, "false", 5) == 0) {
    *out = 0;
    return 1;
  }
  if (n == 1 && (v[0] == '0' || v[0] == '1')) {
    *out = v[0] - '0';
    return 1;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Request
// ---------------------------------------------------------------------------

/// A parsed command. Absent numeric fields stay at their "not given"
/// sentinel (-1) so the app can pass 0 = "bridge default" through to the
/// camera service exactly as the FIFO CLI does.
typedef struct {
  char verb[BENCH_CTL_VERB_MAX];
  double id;   // echoed back so a late reply can't be mistaken for this one
  int have_id; // 0 when the client sent none
  int quality; // -1 absent, else 0..100
  char res[BENCH_CTL_TOK_MAX];
  char pf[BENCH_CTL_TOK_MAX];
  double mbps; // <0 absent
  double fps;  // <0 absent
  int secs;    // -1 absent
  int level;   // -1 absent
  int on_ms, off_ms, count; // -1 absent
  int save;                 // -1 absent (= save), else 0/1
  char err[64];             // reason, when the parse fails
} bench_ctl_req_t;

static inline void bench_ctl_req_init(bench_ctl_req_t *r) {
  memset(r, 0, sizeof(*r));
  r->quality = -1;
  r->mbps = -1.0;
  r->fps = -1.0;
  r->secs = -1;
  r->level = -1;
  r->on_ms = -1;
  r->off_ms = -1;
  r->count = -1;
  r->save = -1;
}

static inline int bench_ctl_fail_(bench_ctl_req_t *r, const char *why) {
  snprintf(r->err, sizeof(r->err), "%s", why);
  return 0;
}

/// Read one optional integer field, range-checked. Returns 0 on a bad value.
static inline int bench_ctl_int_field_(const char *msg, size_t len,
                                       const char *key, double lo, double hi,
                                       int *out, bench_ctl_req_t *r) {
  const char *v;
  size_t n;
  double d;
  if (!bench_ctl_find(msg, len, key, &v, &n)) {
    return 1; // absent is fine
  }
  if (!bench_ctl_get_num(msg, len, key, &d)) {
    snprintf(r->err, sizeof(r->err), "%s is not a number", key);
    return 0;
  }
  if (d < lo || d > hi) {
    snprintf(r->err, sizeof(r->err), "%s out of range (%g..%g)", key, lo, hi);
    return 0;
  }
  *out = (int)d;
  return 1;
}

/// Parse a request datagram. Returns 1 on success; on failure returns 0 with
/// r->err set (and r->id filled where it could be read, so the refusal can
/// still be addressed to the right request).
///
/// A refusal is never silent: app_main replies with the err string. A
/// command that went nowhere must not look like a command that worked --
/// the same rule bm-cmd.sh follows for the FIFO.
static inline int bench_ctl_parse_req(const char *msg, size_t len,
                                      bench_ctl_req_t *r) {
  bench_ctl_req_init(r);
  if (len == 0 || len > BENCH_CTL_MSG_MAX) {
    return bench_ctl_fail_(r, "empty or oversize datagram");
  }
  size_t last = len;
  while (last > 0 && bench_ctl_is_ws_(msg[last - 1])) {
    last--;
  }
  if (last == 0 || msg[last - 1] != '}') {
    return bench_ctl_fail_(r, "not a JSON object");
  }
  double d;
  if (bench_ctl_get_num(msg, len, "id", &d)) {
    r->id = d;
    r->have_id = 1;
  }
  if (bench_ctl_get_num(msg, len, "v", &d) && (int)d != BENCH_CTL_SCHEMA) {
    return bench_ctl_fail_(r, "unsupported schema v (want 1)");
  }
  if (!bench_ctl_get_str(msg, len, "cmd", r->verb, sizeof(r->verb))) {
    return bench_ctl_fail_(r, "missing or oversize cmd");
  }
  for (const char *p = r->verb; *p; p++) {
    int ok = (*p >= 'a' && *p <= 'z') || (*p >= '0' && *p <= '9') || *p == '-';
    if (!ok) {
      return bench_ctl_fail_(r, "cmd has non [a-z0-9-] characters");
    }
  }
  if (r->verb[0] == '\0') {
    return bench_ctl_fail_(r, "empty cmd");
  }
  // res/pf pass through unvalidated ON PURPOSE (see the file header), but an
  // over-long token cannot be a real one, and truncating it would hand the
  // service something the operator never typed.
  const char *v;
  size_t n;
  if (bench_ctl_find(msg, len, "res", &v, &n) &&
      !bench_ctl_get_str(msg, len, "res", r->res, sizeof(r->res))) {
    return bench_ctl_fail_(r, "res too long to be a geometry token");
  }
  if (bench_ctl_find(msg, len, "pf", &v, &n) &&
      !bench_ctl_get_str(msg, len, "pf", r->pf, sizeof(r->pf))) {
    return bench_ctl_fail_(r, "pf too long to be a pixel-format token");
  }
  if (!bench_ctl_int_field_(msg, len, "q", 0, 100, &r->quality, r) ||
      !bench_ctl_int_field_(msg, len, "secs", 0, 65535, &r->secs, r) ||
      !bench_ctl_int_field_(msg, len, "level", 0, 100, &r->level, r) ||
      !bench_ctl_int_field_(msg, len, "on_ms", 0, 65535, &r->on_ms, r) ||
      !bench_ctl_int_field_(msg, len, "off_ms", 0, 65535, &r->off_ms, r) ||
      !bench_ctl_int_field_(msg, len, "count", 0, 65535, &r->count, r)) {
    return 0;
  }
  if (bench_ctl_find(msg, len, "mbps", &v, &n)) {
    if (!bench_ctl_get_num(msg, len, "mbps", &d) || d < 0.0 || d > 100.0) {
      return bench_ctl_fail_(r, "mbps out of range (0..100)");
    }
    r->mbps = d;
  }
  if (bench_ctl_find(msg, len, "fps", &v, &n)) {
    if (!bench_ctl_get_num(msg, len, "fps", &d) || d < 0.0 || d > 1000.0) {
      return bench_ctl_fail_(r, "fps out of range (0..1000)");
    }
    r->fps = d;
  }
  int b;
  if (bench_ctl_find(msg, len, "save", &v, &n)) {
    if (!bench_ctl_get_bool(msg, len, "save", &b)) {
      return bench_ctl_fail_(r, "save is not a boolean");
    }
    r->save = b;
  }
  return 1;
}

// ---------------------------------------------------------------------------
// Replies
// ---------------------------------------------------------------------------

/// `id` is echoed as a number when the client sent one, else omitted --
/// rendered here so every reply builder agrees on the shape.
static inline int bench_ctl_id_(char *out, size_t cap, const bench_ctl_req_t *r) {
  if (r && r->have_id) {
    return snprintf(out, cap, "\"id\": %.0f, ", r->id);
  }
  return snprintf(out, cap, "%s", "");
}

/// Ack for a command that was accepted and dispatched. BM service replies
/// are asynchronous, so this says "submitted", not "done" -- the camera's
/// answer shows up in the next status.
static inline int bench_ctl_render_ack(const bench_ctl_req_t *r, char *out,
                                       size_t cap) {
  char id[32];
  bench_ctl_id_(id, sizeof(id), r);
  int n = snprintf(out, cap,
                   "{\"v\": 1, %s\"ok\": true, \"accepted\": \"%s\", "
                   "\"note\": \"async; poll status for the reply\"}\n",
                   id, r->verb);
  return (n > 0 && (size_t)n < cap) ? n : 0;
}

/// Refusal. `why` is the parse error or an unknown-verb message.
static inline int bench_ctl_render_err(const bench_ctl_req_t *r,
                                       const char *why, char *out,
                                       size_t cap) {
  char id[32];
  bench_ctl_id_(id, sizeof(id), r);
  int n = snprintf(out, cap, "{\"v\": 1, %s\"ok\": false, \"err\": \"%s\"}\n",
                   id, why ? why : "refused");
  return (n > 0 && (size_t)n < cap) ? n : 0;
}

/// The status snapshot: the three things bite B owes the web tool --
/// current params, last replies, live receiver ledger -- plus the save
/// state. Strings are caller-owned and must outlive the call.
typedef struct {
  double t;
  uint64_t node;
  // last commanded parameters (what the operator asked for)
  int params_seen;
  const char *last_cmd;
  double last_cmd_t;
  int quality;
  const char *res;
  const char *pf;
  double fps;
  double mbps;
  int secs;
  // last camera reply (what the camera answered)
  int cam_seen;
  double cam_t;
  const char *cam_state; // ok | timeout | bad_len
  unsigned cam_ok, cam_mode;
  const char *cam_res;
  const char *cam_pf;
  uint32_t cam_cmds, pub_ok, pub_errs, pub_bytes;
  // last light reply
  int light_seen;
  double light_t;
  const char *light_state;
  unsigned light_ok, light_level, light_strobing;
  uint32_t light_cmds, light_uptime_s;
  // receiver ledger (the honest half of commanded-vs-actual)
  uint32_t frames_ok, frames_dropped, chunk_gaps, hdr_errors, oversize;
  uint64_t q_drops, ingest_ok, ingest_fail, uplinks;
  double fps_win, kbps_win;
  int ipc_up;
  // still-save
  const char *save_state; // idle | armed | saved | timeout | error
  const char *save_file;
  uint32_t save_bytes;
  uint64_t saves, save_errors;
  uint64_t disk_free_mb;
  const char *save_dir;
} bench_ctl_status_t;

static inline void bench_ctl_status_init(bench_ctl_status_t *s) {
  memset(s, 0, sizeof(*s));
  s->last_cmd = "";
  s->res = "";
  s->pf = "";
  s->cam_state = "none";
  s->cam_res = "";
  s->cam_pf = "";
  s->light_state = "none";
  s->save_state = "idle";
  s->save_file = "";
  s->save_dir = "";
}

/// Render the status object. Returns bytes written, or 0 if it would not
/// fit -- the caller then sends an err reply rather than half an object,
/// because a truncated JSON body is worse than an honest refusal.
static inline int bench_ctl_render_status(const bench_ctl_status_t *s,
                                          const bench_ctl_req_t *r, char *out,
                                          size_t cap) {
  char id[32];
  bench_ctl_id_(id, sizeof(id), r);
  int n = snprintf(
      out, cap,
      "{\"v\": 1, %s\"ok\": true, \"role\": \"telemetry\", \"t\": %.1f, "
      "\"node\": \"%016llx\", "
      "\"params\": {\"seen\": %s, \"last_cmd\": \"%s\", \"last_cmd_t\": %.1f, "
      "\"q\": %d, \"res\": \"%s\", \"pf\": \"%s\", \"fps\": %.1f, "
      "\"mbps\": %.2f, \"secs\": %d}, "
      "\"cam_reply\": {\"seen\": %s, \"t\": %.1f, \"state\": \"%s\", "
      "\"ok\": %u, \"mode\": %u, \"res\": \"%s\", \"pf\": \"%s\", "
      "\"cmds\": %lu, \"pub_ok\": %lu, \"pub_errs\": %lu, "
      "\"pub_bytes\": %lu}, "
      "\"light_reply\": {\"seen\": %s, \"t\": %.1f, \"state\": \"%s\", "
      "\"ok\": %u, \"level\": %u, \"strobing\": %u, \"cmds\": %lu, "
      "\"uptime_s\": %lu}, "
      "\"ledger\": {\"frames_ok\": %lu, \"dropped\": %lu, \"gaps\": %lu, "
      "\"hdr_errs\": %lu, \"oversize\": %lu, \"q_drops\": %llu, "
      "\"ingest_ok\": %llu, \"ingest_fail\": %llu, \"uplinks\": %llu, "
      "\"fps\": %.1f, \"kBps\": %.1f, \"ipc\": \"%s\"}, "
      "\"save\": {\"state\": \"%s\", \"last_file\": \"%s\", "
      "\"last_bytes\": %lu, \"saved\": %llu, \"errors\": %llu, "
      "\"free_mb\": %llu, \"dir\": \"%s\"}}\n",
      id, s->t, (unsigned long long)s->node, s->params_seen ? "true" : "false",
      s->last_cmd, s->last_cmd_t, s->quality, s->res, s->pf, s->fps, s->mbps,
      s->secs, s->cam_seen ? "true" : "false", s->cam_t, s->cam_state,
      s->cam_ok, s->cam_mode, s->cam_res, s->cam_pf,
      (unsigned long)s->cam_cmds, (unsigned long)s->pub_ok,
      (unsigned long)s->pub_errs, (unsigned long)s->pub_bytes,
      s->light_seen ? "true" : "false", s->light_t, s->light_state,
      s->light_ok, s->light_level, s->light_strobing,
      (unsigned long)s->light_cmds, (unsigned long)s->light_uptime_s,
      (unsigned long)s->frames_ok, (unsigned long)s->frames_dropped,
      (unsigned long)s->chunk_gaps, (unsigned long)s->hdr_errors,
      (unsigned long)s->oversize, (unsigned long long)s->q_drops,
      (unsigned long long)s->ingest_ok, (unsigned long long)s->ingest_fail,
      (unsigned long long)s->uplinks, s->fps_win, s->kbps_win,
      s->ipc_up ? "up" : "down", s->save_state, s->save_file,
      (unsigned long)s->save_bytes, (unsigned long long)s->saves,
      (unsigned long long)s->save_errors, (unsigned long long)s->disk_free_mb,
      s->save_dir);
  return (n > 0 && (size_t)n < cap) ? n : 0;
}

// ---------------------------------------------------------------------------
// Still-save: names and sidecars
// ---------------------------------------------------------------------------

/// UTC stamp for capture filenames: "20260815T193012Z". Returns 1 on success.
static inline int bench_ctl_stamp(time_t when, char *out, size_t cap) {
  struct tm tm;
  if (cap < BENCH_CTL_STAMP_MAX || gmtime_r(&when, &tm) == NULL) {
    return 0;
  }
  return strftime(out, cap, "%Y%m%dT%H%M%SZ", &tm) > 0;
}

/// Build "cap_<stamp>_seq<NNNNNN>.<ext>". Built from a fixed format plus
/// digits, so no operator-supplied text reaches a path -- traversal is
/// impossible by construction, not by filtering.
static inline int bench_ctl_capture_name(const char *stamp, uint32_t seq,
                                         const char *ext, char *out,
                                         size_t cap) {
  int n = snprintf(out, cap, "cap_%s_seq%06lu.%s", stamp, (unsigned long)seq,
                   ext);
  return (n > 0 && (size_t)n < cap) ? n : 0;
}

/// Everything known about one saved still, at the moment it was saved.
typedef struct {
  const char *file;
  const char *utc;
  double t;
  uint32_t frame_seq, bytes;
  uint16_t chunks;
  const char *source; // socket | cli
  // commanded
  int quality;
  const char *res;
  const char *pf;
  // the camera's reply, if it arrived before the frame did
  int reply_seen;
  unsigned reply_ok;
  const char *reply_res;
  const char *reply_pf;
  uint32_t pub_ok, pub_errs, pub_bytes;
  // ledger: absolute at save, and what moved since the capture was armed
  uint32_t frames_ok, frames_dropped, chunk_gaps, hdr_errors;
  uint32_t dropped_delta, gaps_delta;
  uint64_t node, camera_node;
} bench_ctl_sidecar_t;

/// Render the sidecar. Returns bytes written, or 0 if it would not fit.
static inline int bench_ctl_render_sidecar(const bench_ctl_sidecar_t *s,
                                           char *out, size_t cap) {
  int n = snprintf(
      out, cap,
      "{\n  \"schema\": 1,\n  \"file\": \"%s\",\n  \"utc\": \"%s\",\n"
      "  \"t\": %.1f,\n  \"source\": \"%s\",\n"
      "  \"req\": {\"q\": %d, \"res\": \"%s\", \"pf\": \"%s\"},\n"
      "  \"reply\": {\"seen\": %s, \"ok\": %u, \"res\": \"%s\", "
      "\"pf\": \"%s\", \"pub_ok\": %lu, \"pub_errs\": %lu, "
      "\"pub_bytes\": %lu},\n"
      "  \"frame\": {\"seq\": %lu, \"size_bytes\": %lu, \"chunks\": %u},\n"
      "  \"ledger\": {\"frames_ok\": %lu, \"dropped\": %lu, \"gaps\": %lu, "
      "\"hdr_errs\": %lu, \"dropped_delta\": %lu, \"gaps_delta\": %lu},\n"
      "  \"node\": {\"telemetry\": \"%016llx\", \"camera\": \"%016llx\"}\n}\n",
      s->file, s->utc, s->t, s->source, s->quality, s->res, s->pf,
      s->reply_seen ? "true" : "false", s->reply_ok, s->reply_res, s->reply_pf,
      (unsigned long)s->pub_ok, (unsigned long)s->pub_errs,
      (unsigned long)s->pub_bytes, (unsigned long)s->frame_seq,
      (unsigned long)s->bytes, (unsigned)s->chunks,
      (unsigned long)s->frames_ok, (unsigned long)s->frames_dropped,
      (unsigned long)s->chunk_gaps, (unsigned long)s->hdr_errors,
      (unsigned long)s->dropped_delta, (unsigned long)s->gaps_delta,
      (unsigned long long)s->node, (unsigned long long)s->camera_node);
  return (n > 0 && (size_t)n < cap) ? n : 0;
}

#endif // BENCH_CTL_H
