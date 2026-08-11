#include "bm_log.h"
#include "bm_os.h"
#include "bm_service_request.h"
#include "device.h"
#include "util.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define CRITICAL_SERVICE_PATH_CAP 64
#define TIMER_TIMEOUT_MS 2000
#define SERVICE_REQUEST_TIMEOUT_S 1
#define TIMER_MAX_WAIT_MS 5

struct CriticalServiceCtx {
  BmSemaphore mut;
  BmTimer timer;
  bool critical_status;
};

static struct CriticalServiceCtx ctx;

static bool critical_service_request_cb(bool ack, uint32_t msg_id,
                                        size_t service_strlen,
                                        const char *service, size_t reply_len,
                                        uint8_t *reply_data) {
  bm_log_info("%s: received reply on service %.*s", __func__,
              (int)service_strlen, service);

  bm_timer_stop(ctx.timer, TIMER_MAX_WAIT_MS);

  return true;
}

static void send_request(bool critical) {
  char service[CRITICAL_SERVICE_PATH_CAP] = {0};

  int n = snprintf(service, sizeof(service), "borealis/%016" PRIx64 "/critical",
                   node_id());
  if (n < 0 || (size_t)n >= sizeof(service)) {
    bm_log_error("%s: failed to format critical operation service path",
                 __func__);
    return;
  }

  if (!bm_service_request(n, service, 0, NULL, critical_service_request_cb,
                          SERVICE_REQUEST_TIMEOUT_S)) {
    bm_log_error("%s: bm_service_request failed", __func__);
  }
}

static void critical_timer_cb(BmTimer timer) {
  (void)timer;
  bm_semaphore_take(ctx.mut, BM_MAX_DELAY_UINT32);
  send_request(ctx.critical_status);
  bm_semaphore_give(ctx.mut);
}

void sbc_critical_op(bool critical) {
  if (!ctx.mut) {
    ctx.mut = bm_mutex_create();
    if (!ctx.mut) {
      bm_log_error("%s: could not create mutex...", __func__);
      return;
    }
  }

  if (!ctx.timer) {
    ctx.timer = bm_timer_create("critical", TIMER_TIMEOUT_MS, true, NULL,
                                critical_timer_cb);
    if (!ctx.timer) {
      bm_log_error("%s: could not create timer...", __func__);
      return;
    }
  }

  bm_semaphore_take(ctx.mut, BM_MAX_DELAY_UINT32);
  bm_timer_stop(ctx.timer, TIMER_MAX_WAIT_MS);
  ctx.critical_status = critical;
  send_request(ctx.critical_status);
  bm_timer_start(ctx.timer, TIMER_MAX_WAIT_MS);
  bm_semaphore_give(ctx.mut);
}
