/// @file test_bench_ctl.c
/// @brief Unit tests for the bench control-socket wire format (S18 bite B).
///
/// The point of splitting bench_ctl.h out of app_main.cpp is that the whole
/// parse/render surface can be exercised here, on a laptop, with no socket,
/// no BM stack and no live chain. Anything that only fails on the bench is
/// a bug this file should have caught.

#include "bench_ctl.h"

#include <stdio.h>
#include <string.h>

static int g_pass = 0;
static int g_fail = 0;

#define ASSERT_TRUE(cond, msg)                                                 \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf("  FAIL: %s\n", msg);                                             \
      g_fail++;                                                                \
    } else {                                                                   \
      g_pass++;                                                                \
    }                                                                          \
  } while (0)

static int parse(const char *json, bench_ctl_req_t *r) {
  return bench_ctl_parse_req(json, strlen(json), r);
}

int main(void) {
  bench_ctl_req_t r;
  char out[BENCH_CTL_REPLY_MAX];

  // --- the happy paths, one per verb the web tool sends -------------------
  ASSERT_TRUE(parse("{\"cmd\": \"status\"}", &r), "bare status parses");
  ASSERT_TRUE(strcmp(r.verb, "status") == 0, "verb = status");
  ASSERT_TRUE(!r.have_id && r.quality == -1 && r.secs == -1,
              "absent fields keep their sentinels");
  ASSERT_TRUE(r.save == -1, "save absent = -1 (app default: save)");

  ASSERT_TRUE(
      parse("{\"v\":1,\"id\":17,\"cmd\":\"capture\",\"q\":50,\"res\":\"hd\","
            "\"pf\":\"color\"}",
            &r),
      "full capture parses");
  ASSERT_TRUE(r.have_id && r.id == 17.0, "id captured");
  ASSERT_TRUE(r.quality == 50, "q = 50");
  ASSERT_TRUE(strcmp(r.res, "hd") == 0 && strcmp(r.pf, "color") == 0,
              "res/pf copied verbatim");

  ASSERT_TRUE(parse("{\"cmd\":\"stream\",\"mbps\":2.0,\"fps\":15,\"secs\":600,"
                    "\"q\":50,\"res\":\"vga\",\"pf\":\"mono\"}",
                    &r),
              "full stream parses");
  ASSERT_TRUE(r.mbps == 2.0 && r.fps == 15.0 && r.secs == 600,
              "stream numerics");

  ASSERT_TRUE(parse("{\"cmd\":\"strobe\",\"on_ms\":200,\"off_ms\":250,"
                    "\"count\":10}",
                    &r),
              "strobe parses");
  ASSERT_TRUE(r.on_ms == 200 && r.off_ms == 250 && r.count == 10,
              "strobe numerics");

  ASSERT_TRUE(parse("{\"cmd\":\"light\",\"level\":100}", &r), "light parses");
  ASSERT_TRUE(r.level == 100, "level = 100");

  // Whitespace, key order and unknown extra keys are all fine.
  ASSERT_TRUE(parse("  {\n \"cmd\" : \"capture\" ,\n \"zzz\": 9,\n"
                    " \"q\" : 90 \n}  ",
                    &r),
              "whitespace + unknown keys tolerated");
  ASSERT_TRUE(r.quality == 90 && strcmp(r.verb, "capture") == 0,
              "values survive the noise");

  // --- refusals: every one of these must name a reason -------------------
  ASSERT_TRUE(!parse("{\"q\": 50}", &r), "missing cmd refused");
  ASSERT_TRUE(strstr(r.err, "cmd") != NULL, "missing cmd names cmd");
  ASSERT_TRUE(!parse("", &r), "empty datagram refused");
  ASSERT_TRUE(!parse("capture 50 hd color", &r), "CLI text is not JSON");
  ASSERT_TRUE(!parse("{\"cmd\": \"status\"", &r), "unterminated object refused");
  ASSERT_TRUE(!parse("{\"v\":2,\"cmd\":\"status\"}", &r),
              "schema v=2 refused");
  ASSERT_TRUE(strstr(r.err, "schema") != NULL, "v refusal names the schema");
  ASSERT_TRUE(!parse("{\"cmd\":\"cap ture\"}", &r), "space in verb refused");
  ASSERT_TRUE(!parse("{\"cmd\":\"../../etc/passwd\"}", &r),
              "path-ish verb refused");
  ASSERT_TRUE(!parse("{\"cmd\":\"capture\",\"q\":101}", &r), "q=101 refused");
  ASSERT_TRUE(strstr(r.err, "range") != NULL, "q refusal says range");
  ASSERT_TRUE(!parse("{\"cmd\":\"capture\",\"q\":-1}", &r), "q=-1 refused");
  ASSERT_TRUE(!parse("{\"cmd\":\"capture\",\"q\":\"50abc\"}", &r),
              "q with trailing junk refused");
  ASSERT_TRUE(!parse("{\"cmd\":\"stream\",\"mbps\":1000}", &r),
              "mbps=1000 refused");
  ASSERT_TRUE(!parse("{\"cmd\":\"capture\",\"save\":\"maybe\"}", &r),
              "non-boolean save refused");
  ASSERT_TRUE(!parse("{\"cmd\":\"capture\",\"res\":\"reallylongtoken\"}", &r),
              "over-long res refused, not truncated");
  ASSERT_TRUE(strstr(r.err, "res") != NULL, "res refusal names res");

  // An id on a REFUSED request is still echoed, so the client can match the
  // refusal to the request that caused it.
  ASSERT_TRUE(!parse("{\"id\":42,\"cmd\":\"capture\",\"q\":999}", &r),
              "bad q refused");
  ASSERT_TRUE(r.have_id && r.id == 42.0, "id survives a refusal");

  // Oversize datagram is refused on length alone (no scan of 100 KB).
  {
    static char big[BENCH_CTL_MSG_MAX + 64];
    memset(big, 'x', sizeof(big));
    ASSERT_TRUE(!bench_ctl_parse_req(big, sizeof(big), &r), "oversize refused");
  }

  // --- the nested-value trap ---------------------------------------------
  // A key inside a nested object must NOT be mistaken for a top-level one;
  // this is the only structurally subtle part of the scanner.
  ASSERT_TRUE(parse("{\"cmd\":\"capture\",\"opts\":{\"q\":11},\"q\":50}", &r),
              "nested object skipped");
  ASSERT_TRUE(r.quality == 50, "top-level q wins over a nested q");
  ASSERT_TRUE(parse("{\"cmd\":\"capture\",\"list\":[1,2,3],\"q\":22}", &r),
              "array value skipped");
  ASSERT_TRUE(r.quality == 22, "q after an array still found");
  // A brace or comma INSIDE a string value must not end the value either.
  ASSERT_TRUE(parse("{\"cmd\":\"capture\",\"note\":\"a,b}c\",\"q\":33}", &r),
              "string value with punctuation survives");
  ASSERT_TRUE(r.quality == 33, "q after a punctuated string still found");

  // --- boolean forms ------------------------------------------------------
  ASSERT_TRUE(parse("{\"cmd\":\"capture\",\"save\":false}", &r), "save:false");
  ASSERT_TRUE(r.save == 0, "save = 0");
  ASSERT_TRUE(parse("{\"cmd\":\"capture\",\"save\":true}", &r), "save:true");
  ASSERT_TRUE(r.save == 1, "save = 1");
  ASSERT_TRUE(parse("{\"cmd\":\"capture\",\"save\":1}", &r), "save:1");
  ASSERT_TRUE(r.save == 1, "save = 1 (numeric)");

  // --- replies ------------------------------------------------------------
  parse("{\"id\":7,\"cmd\":\"capture\"}", &r);
  ASSERT_TRUE(bench_ctl_render_ack(&r, out, sizeof(out)) > 0, "ack renders");
  ASSERT_TRUE(strstr(out, "\"id\": 7") != NULL, "ack echoes the id");
  ASSERT_TRUE(strstr(out, "\"ok\": true") != NULL, "ack is ok");
  ASSERT_TRUE(strstr(out, "\"accepted\": \"capture\"") != NULL,
              "ack names the verb");
  ASSERT_TRUE(out[strlen(out) - 1] == '\n', "ack ends with a newline");

  parse("{\"cmd\":\"capture\"}", &r);
  ASSERT_TRUE(bench_ctl_render_ack(&r, out, sizeof(out)) > 0, "ack w/o id");
  ASSERT_TRUE(strstr(out, "\"id\"") == NULL, "no id key when none was sent");

  ASSERT_TRUE(bench_ctl_render_err(&r, "unknown cmd 'captur'", out,
                                   sizeof(out)) > 0,
              "err renders");
  ASSERT_TRUE(strstr(out, "\"ok\": false") != NULL, "err is not ok");
  ASSERT_TRUE(strstr(out, "captur") != NULL, "err carries the reason");
  // A refusal must fit in a tiny buffer or report failure -- never truncate.
  ASSERT_TRUE(bench_ctl_render_err(&r, "some reason", out, 8) == 0,
              "err refuses to truncate");

  // --- status -------------------------------------------------------------
  {
    bench_ctl_status_t s;
    bench_ctl_status_init(&s);
    s.t = 123.4;
    s.node = 0xbe9c000000000001ull;
    s.params_seen = 1;
    s.last_cmd = "capture";
    s.last_cmd_t = 120.1;
    s.quality = 50;
    s.res = "hd";
    s.pf = "color";
    s.fps = 15.0;
    s.mbps = 2.0;
    s.secs = 600;
    s.cam_seen = 1;
    s.cam_t = 120.4;
    s.cam_state = "ok";
    s.cam_ok = 1;
    s.cam_res = "hd";
    s.cam_pf = "color";
    s.pub_ok = 34;
    s.pub_bytes = 42884;
    s.frames_ok = 9092;
    s.ingest_ok = 9092;
    s.fps_win = 15.1;
    s.kbps_win = 28.4;
    s.ipc_up = 1;
    s.save_state = "saved";
    s.save_file = "cap_20260815T193012Z_seq000123.jpg";
    s.save_bytes = 42574;
    s.saves = 7;
    s.disk_free_mb = 21000;
    s.save_dir = "/home/pi/bench_captures";

    parse("{\"id\":24,\"cmd\":\"status\"}", &r);
    int n = bench_ctl_render_status(&s, &r, out, sizeof(out));
    ASSERT_TRUE(n > 0, "status renders");
    ASSERT_TRUE((size_t)n == strlen(out), "status length matches");
    ASSERT_TRUE(strstr(out, "\"id\": 24") != NULL, "status echoes the id");
    ASSERT_TRUE(strstr(out, "\"node\": \"be9c000000000001\"") != NULL,
                "node id renders as 16 hex digits");
    // The three things the tracker says this reply owes the web tool.
    ASSERT_TRUE(strstr(out, "\"params\":") != NULL, "params block present");
    ASSERT_TRUE(strstr(out, "\"cam_reply\":") != NULL, "cam_reply present");
    ASSERT_TRUE(strstr(out, "\"light_reply\":") != NULL, "light_reply present");
    ASSERT_TRUE(strstr(out, "\"ledger\":") != NULL, "ledger present");
    ASSERT_TRUE(strstr(out, "\"save\":") != NULL, "save block present");
    // Commanded vs actual: both numbers must be in the same reply, or the
    // pill in bite C has nothing to compare.
    ASSERT_TRUE(strstr(out, "\"fps\": 15.0") != NULL, "commanded fps present");
    ASSERT_TRUE(strstr(out, "\"fps\": 15.1") != NULL, "measured fps present");
    ASSERT_TRUE(strstr(out, "\"pub_bytes\": 42884") != NULL, "pub_bytes exact");
    ASSERT_TRUE(strstr(out, "\"free_mb\": 21000") != NULL, "free_mb present");
    ASSERT_TRUE(strstr(out, "\"ipc\": \"up\"") != NULL, "ipc state present");

    // Same rule as err: a status that does not fit reports 0 rather than
    // emitting half an object.
    ASSERT_TRUE(bench_ctl_render_status(&s, &r, out, 200) == 0,
                "status refuses to truncate");

    // A freshly booted node has seen nothing, and must still render.
    bench_ctl_status_init(&s);
    ASSERT_TRUE(bench_ctl_render_status(&s, NULL, out, sizeof(out)) > 0,
                "empty status renders");
    ASSERT_TRUE(strstr(out, "\"seen\": false") != NULL, "unseen reads false");
    ASSERT_TRUE(strstr(out, "\"id\"") == NULL, "no id without a request");
  }

  // --- capture names ------------------------------------------------------
  {
    char stamp[BENCH_CTL_STAMP_MAX];
    char name[BENCH_CTL_NAME_MAX];
    // 2026-08-15T19:30:12Z
    ASSERT_TRUE(bench_ctl_stamp((time_t)1786822212, stamp, sizeof(stamp)),
                "stamp renders");
    ASSERT_TRUE(strcmp(stamp, "20260815T193012Z") == 0, "stamp is UTC, exact");
    ASSERT_TRUE(bench_ctl_stamp((time_t)0, stamp, 4) == 0,
                "stamp refuses a short buffer");

    ASSERT_TRUE(bench_ctl_stamp((time_t)1786822212, stamp, sizeof(stamp)),
                "stamp again");
    ASSERT_TRUE(bench_ctl_capture_name(stamp, 123, "jpg", name, sizeof(name)),
                "name renders");
    ASSERT_TRUE(strcmp(name, "cap_20260815T193012Z_seq000123.jpg") == 0,
                "name is exact");
    ASSERT_TRUE(strchr(name, '/') == NULL, "name can never contain a slash");
    ASSERT_TRUE(bench_ctl_capture_name(stamp, 123, "json", name, sizeof(name)),
                "sidecar name renders");
    ASSERT_TRUE(strcmp(name, "cap_20260815T193012Z_seq000123.json") == 0,
                "sidecar shares the base name");
    ASSERT_TRUE(bench_ctl_capture_name(stamp, 4294967295u, "jpg", name,
                                       sizeof(name)),
                "u32 seq max fits");
    ASSERT_TRUE(bench_ctl_capture_name(stamp, 1, "jpg", name, 8) == 0,
                "name refuses a short buffer");
  }

  // --- sidecar ------------------------------------------------------------
  {
    bench_ctl_sidecar_t sc;
    memset(&sc, 0, sizeof(sc));
    sc.file = "cap_20260815T193012Z_seq000123.jpg";
    sc.utc = "20260815T193012Z";
    sc.t = 120.9;
    sc.frame_seq = 123;
    sc.bytes = 42574;
    sc.chunks = 31;
    sc.source = "socket";
    sc.quality = 50;
    sc.res = "hd";
    sc.pf = "color";
    sc.reply_seen = 1;
    sc.reply_ok = 1;
    sc.reply_res = "hd";
    sc.reply_pf = "color";
    sc.pub_ok = 34;
    sc.pub_bytes = 42884;
    sc.frames_ok = 9092;
    sc.node = 0xbe9c000000000001ull;
    sc.camera_node = 0xbe9c000000000003ull;

    int n = bench_ctl_render_sidecar(&sc, out, sizeof(out));
    ASSERT_TRUE(n > 0, "sidecar renders");
    ASSERT_TRUE(strstr(out, "\"size_bytes\": 42574") != NULL,
                "sidecar carries the byte count");
    ASSERT_TRUE(strstr(out, "\"chunks\": 31") != NULL, "chunk count present");
    ASSERT_TRUE(strstr(out, "\"source\": \"socket\"") != NULL,
                "source recorded");
    ASSERT_TRUE(strstr(out, "\"gaps_delta\": 0") != NULL,
                "gaps_delta present (did THIS still lose anything)");
    ASSERT_TRUE(strstr(out, "\"camera\": \"be9c000000000003\"") != NULL,
                "camera node id recorded");
    ASSERT_TRUE(strstr(out, "\"q\": 50") != NULL, "commanded q recorded");
    ASSERT_TRUE(bench_ctl_render_sidecar(&sc, out, 100) == 0,
                "sidecar refuses to truncate");

    // A capture whose reply never arrived still gets a sidecar, marked.
    sc.reply_seen = 0;
    ASSERT_TRUE(bench_ctl_render_sidecar(&sc, out, sizeof(out)) > 0,
                "sidecar without a reply renders");
    ASSERT_TRUE(strstr(out, "\"seen\": false") != NULL,
                "missing reply is marked, not faked");
  }

  printf("bench_ctl: %d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
