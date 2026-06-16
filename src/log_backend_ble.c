#include <zephyr/kernel.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/logging/log_msg.h>
#include <zephyr/logging/log_ctrl.h>
#include <string.h>

#include "ble_log_service.h"

/* IMPORTANT: Do NOT use LOG_INF/LOG_ERR inside a log backend */

typedef struct {
    uint8_t *buf;
    size_t cap;
    size_t len;
} line_accum_t;

static line_accum_t *g_active_accum;

static int output_accum_func(uint8_t *data, size_t length, void *ctx)
{
    ARG_UNUSED(ctx);

    line_accum_t *acc = g_active_accum;
    if (!acc || !acc->buf || acc->cap == 0U) {
        return 0;
    }

    size_t room = (acc->cap > acc->len) ? (acc->cap - acc->len) : 0U;
    if (room == 0U) {
        return 0;
    }

    size_t n = (length < room) ? length : room;
    memcpy(acc->buf + acc->len, data, n);
    acc->len += n;
    return (int)length;
}

static uint8_t fmt_buf[384];
LOG_OUTPUT_DEFINE(ble_log_output, output_accum_func, fmt_buf, sizeof(fmt_buf));

static void backend_process(const struct log_backend *const backend,
                            union log_msg_generic *msg)
{
    ARG_UNUSED(backend);

    struct log_msg *m = (struct log_msg *)&msg->log;

    line_accum_t acc = {
        .buf = fmt_buf,
        .cap = sizeof(fmt_buf) - 2U,
        .len = 0U,
    };

    g_active_accum = &acc;
    log_output_msg_process(&ble_log_output, m, 0U);
    g_active_accum = NULL;

    if (acc.len + 2U <= sizeof(fmt_buf)) {
        fmt_buf[acc.len++] = '\r';
        fmt_buf[acc.len++] = '\n';
    }

    (void)ble_log_send_as(fmt_buf, acc.len);
}

static void backend_dropped(const struct log_backend *const backend, uint32_t cnt)
{
    ARG_UNUSED(backend);

    char s[64];
    int n = snprintk(s, sizeof(s), "[DROPPED=%u]\r\n", (unsigned)cnt);
    if (n > 0) {
        (void)ble_log_send_as((const uint8_t *)s, (size_t)n);
    }
}

static void backend_panic(const struct log_backend *const backend)
{
    ARG_UNUSED(backend);
}

static const struct log_backend_api backend_api = {
    .process = backend_process,
    .dropped = backend_dropped,
    .panic   = backend_panic,
};

LOG_BACKEND_DEFINE(ble_backend, backend_api, true);
