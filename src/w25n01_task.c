#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "ble_log_service.h"
#include "w25n01_task.h"

LOG_MODULE_REGISTER(w25n01_mem, LOG_LEVEL_INF);

/* GPIO */
#define GPIO0_NODE DT_NODELABEL(gpio0)
#define GPIO1_NODE DT_NODELABEL(gpio1)

#define CS_PIN     17   /* P0.17 */
#define WP_PIN     12   /* P0.12 */
#define HOLD_PIN   8    /* P1.08 */

/* SPI: keep using spi2 so i2c1 remains available */
#define SPI_NODE DT_NODELABEL(spi2)

/* NAND geometry for Winbond W25N01 (1 Gbit = 128 MiB) */
#define NAND_PAGE_SIZE            2048U
#define PAGES_PER_BLOCK           64U
#define NAND_TOTAL_BLOCKS         1024U

/* Keep a tiny reserved area at the front, then use the rest of the chip for the fill test. */
#define LOG_BLOCK_START           4U
#define LOG_BLOCK_COUNT           (NAND_TOTAL_BLOCKS - LOG_BLOCK_START)
#define LOG_FIRST_PAGE            (LOG_BLOCK_START * PAGES_PER_BLOCK)
#define LOG_TOTAL_PAGES           (LOG_BLOCK_COUNT * PAGES_PER_BLOCK)
#define LOG_LAST_PAGE_EXCL        (LOG_FIRST_PAGE + LOG_TOTAL_PAGES)
#define LOG_CAPACITY_BYTES        (LOG_BLOCK_COUNT * PAGES_PER_BLOCK * NAND_PAGE_SIZE)

/* Queue and text-line limits */
#define LOG_LINE_MAX              320
#define LOG_MSGQ_DEPTH            96
#define MEM_STATUS_PERIOD_MS      10000

/* Wear detection is driven by the PPG task */
static volatile bool g_worn;

/* Per-block state for the usable log region */
enum {
    BLOCK_STATE_UNKNOWN = 0,
    BLOCK_STATE_ERASED  = 1,
    BLOCK_STATE_BAD     = 2,
};

/* Storage state */
static bool     g_storage_ready;
static bool     g_storage_full;
static uint8_t  g_block_state[LOG_BLOCK_COUNT];
static uint32_t g_next_page = LOG_FIRST_PAGE;
static uint32_t g_written_pages;
static uint32_t g_bytes_logged_total;
static uint32_t g_lines_logged_total;
static uint32_t g_bad_block_count;
static uint32_t g_queue_drop_count;
static uint32_t g_erase_fail_count;
static uint32_t g_program_fail_count;

/* Fill-test timing */
static int64_t  g_fill_start_ms = -1;
static int64_t  g_fill_full_ms  = -1;
static int64_t  g_last_status_ms;

/* Active page staging buffer */
static uint8_t  g_page_buf[NAND_PAGE_SIZE];
static size_t   g_page_used;
static int64_t  g_last_append_ms;

/* BLE replay state */
static bool g_prev_ble_ready;
static bool g_dumped_this_connection;

/* Binary record format stored in NAND:
 *   [0] 0xA5 sync
 *   [1] record type
 *   [2] payload length N
 *   [3..3+N-1] payload
 * Records are intentionally kept inside a single NAND page. Replay decodes the
 * binary record back into text for the GUI/app.
 */
#define BIN_REC_SYNC        0xA5U
#define BIN_REC_MAX         256U
#define BIN_PAYLOAD_MAX     (BIN_REC_MAX - 3U)

#define BIN_TYPE_TEMP       0x01U
#define BIN_TYPE_IMU        0x02U
#define BIN_TYPE_PPG        0x03U
#define BIN_TYPE_EDA        0x04U
#define BIN_TYPE_PROC       0x05U
#define BIN_TYPE_EVENT      0x06U

typedef struct {
    uint16_t len;
    uint8_t  bytes[BIN_REC_MAX];
} bin_record_t;

/* Queue from sensor/algo producers into NAND writer */
K_MSGQ_DEFINE(g_log_msgq, sizeof(bin_record_t), LOG_MSGQ_DEPTH, 4);

static struct spi_config spi_cfg = {
    .frequency = 1000000,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .slave = 0,
};

static const struct device *gpio0;
static const struct device *gpio1;
static const struct device *spi_dev;

static inline void cs_low(void)  { gpio_pin_set(gpio0, CS_PIN, 0); }
static inline void cs_high(void) { gpio_pin_set(gpio0, CS_PIN, 1); }

static int spi_tx(const uint8_t *tx, size_t len)
{
    struct spi_buf b = { .buf = (void *)tx, .len = len };
    struct spi_buf_set s = { .buffers = &b, .count = 1 };
    return spi_write(spi_dev, &spi_cfg, &s);
}

static int spi_rx(uint8_t *rx, size_t len)
{
    struct spi_buf b = { .buf = rx, .len = len };
    struct spi_buf_set s = { .buffers = &b, .count = 1 };
    return spi_read(spi_dev, &spi_cfg, &s);
}

/* NAND commands */
#define CMD_RESET        0xFF
#define CMD_WREN         0x06
#define CMD_GET_FEATURE  0x0F
#define CMD_SET_FEATURE  0x1F
#define CMD_BLOCK_ERASE  0xD8
#define CMD_PROG_LOAD    0x02
#define CMD_PROG_EXEC    0x10
#define CMD_PAGE_READ    0x13
#define CMD_READ_CACHE   0x03

#define REG_STATUS       0xC0
#define REG_PROTECTION   0xA0

#define SR_OIP           BIT(0)
#define SR_EFAIL         BIT(2)
#define SR_PFAIL         BIT(3)

static uint8_t get_status(void)
{
    uint8_t tx[3] = { CMD_GET_FEATURE, REG_STATUS, 0x00 };
    uint8_t rx[3] = { 0 };

    cs_low();
    spi_tx(tx, sizeof(tx));
    spi_rx(rx, sizeof(rx));
    cs_high();

    return rx[2];
}

static bool wait_ready(const char *tag, int timeout_ms)
{
    int elapsed = 0;

    while (1) {
        uint8_t sr = get_status();
        if ((sr & SR_OIP) == 0U) {
            if (sr & (SR_EFAIL | SR_PFAIL)) {
                LOG_ERR("%s: READY but fail bits set (STATUS=0x%02X)", tag, sr);
                return false;
            }
            return true;
        }

        k_msleep(5);
        elapsed += 5;
        if (elapsed >= timeout_ms) {
            LOG_ERR("%s: TIMEOUT (STATUS=0x%02X)", tag, sr);
            return false;
        }
    }
}

static void nand_reset(void)
{
    uint8_t cmd = CMD_RESET;
    cs_low();
    spi_tx(&cmd, 1);
    cs_high();
    k_msleep(5);
}

static void nand_wren(void)
{
    uint8_t cmd = CMD_WREN;
    cs_low();
    spi_tx(&cmd, 1);
    cs_high();
}

static void set_protection_off(void)
{
    uint8_t tx[3] = { CMD_SET_FEATURE, REG_PROTECTION, 0x00 };
    cs_low();
    spi_tx(tx, sizeof(tx));
    cs_high();
    k_msleep(2);
}

static bool nand_block_erase(uint32_t page_addr)
{
    uint8_t tx[4] = {
        CMD_BLOCK_ERASE,
        (uint8_t)((page_addr >> 16) & 0xFF),
        (uint8_t)((page_addr >> 8) & 0xFF),
        (uint8_t)(page_addr & 0xFF),
    };

    nand_wren();
    cs_low();
    spi_tx(tx, sizeof(tx));
    cs_high();

    return wait_ready("ERASE", 3000);
}

static bool nand_program_page(uint32_t page_addr, const uint8_t *data, size_t len)
{
    if (len > NAND_PAGE_SIZE) {
        return false;
    }

    uint8_t hdr[3] = { CMD_PROG_LOAD, 0x00, 0x00 };

    nand_wren();
    cs_low();
    spi_tx(hdr, sizeof(hdr));
    spi_tx(data, len);
    cs_high();

    uint8_t exec[4] = {
        CMD_PROG_EXEC,
        (uint8_t)((page_addr >> 16) & 0xFF),
        (uint8_t)((page_addr >> 8) & 0xFF),
        (uint8_t)(page_addr & 0xFF),
    };

    cs_low();
    spi_tx(exec, sizeof(exec));
    cs_high();

    return wait_ready("PROGRAM", 3000);
}

static bool nand_page_read_to_cache(uint32_t page_addr)
{
    uint8_t tx[4] = {
        CMD_PAGE_READ,
        (uint8_t)((page_addr >> 16) & 0xFF),
        (uint8_t)((page_addr >> 8) & 0xFF),
        (uint8_t)(page_addr & 0xFF),
    };

    cs_low();
    spi_tx(tx, sizeof(tx));
    cs_high();

    return wait_ready("PAGE_READ", 3000);
}

static int nand_read_cache(uint16_t col, uint8_t *out, size_t len)
{
    uint8_t tx[4] = {
        CMD_READ_CACHE,
        (uint8_t)((col >> 8) & 0xFF),
        (uint8_t)(col & 0xFF),
        0x00
    };

    cs_low();
    spi_tx(tx, sizeof(tx));
    int ret = spi_rx(out, len);
    cs_high();
    return ret;
}

static uint32_t storage_used_bytes(void)
{
    return (g_written_pages * NAND_PAGE_SIZE) + (uint32_t)g_page_used;
}

static int64_t storage_elapsed_ms(void)
{
    if (g_fill_start_ms < 0) {
        return 0;
    }

    int64_t end_ms = (g_fill_full_ms >= 0) ? g_fill_full_ms : k_uptime_get();
    if (end_ms < g_fill_start_ms) {
        return 0;
    }

    return end_ms - g_fill_start_ms;
}

static void send_text_line_live(const char *line)
{
    if (!ble_log_is_ready() || !line || !line[0]) {
        return;
    }

    char out[LOG_LINE_MAX + 8];
    int n = snprintk(out, sizeof(out), "%s\r\n", line);
    if (n > 0) {
        (void)ble_log_send_as((const uint8_t *)out, (size_t)n);
    }
}

static void send_mem_status(const char *tag)
{
    char line[LOG_LINE_MAX];
    uint32_t used = storage_used_bytes();
    int64_t elapsed_ms = storage_elapsed_ms();
    uint32_t avg_bps = 0U;

    if (elapsed_ms > 0) {
        avg_bps = (uint32_t)(((uint64_t)used * 1000ULL) / (uint64_t)elapsed_ms);
    }

    int n = snprintk(line, sizeof(line),
        "MEM_STATUS: tag=%s used=%u cap=%u pages=%u lines=%u bad_blocks=%u qdrop=%u efail=%u pfail=%u elapsed_ms=%lld avg_Bps=%u worn=%d full=%d",
        tag ? tag : "NA",
        (unsigned)used,
        (unsigned)LOG_CAPACITY_BYTES,
        (unsigned)g_written_pages,
        (unsigned)g_lines_logged_total,
        (unsigned)g_bad_block_count,
        (unsigned)g_queue_drop_count,
        (unsigned)g_erase_fail_count,
        (unsigned)g_program_fail_count,
        elapsed_ms,
        (unsigned)avg_bps,
        (int)g_worn,
        (int)g_storage_full);

    if (n > 0) {
        send_text_line_live(line);
    }
}

static void init_page_buf(void)
{
    memset(g_page_buf, 0xFF, sizeof(g_page_buf));
    g_page_used = 0;
    g_last_append_ms = 0;
}

static void mark_block_bad(uint32_t page_addr, const char *reason, bool erase_fail)
{
    uint32_t block = page_addr / PAGES_PER_BLOCK;
    if (block < LOG_BLOCK_START || block >= NAND_TOTAL_BLOCKS) {
        return;
    }

    uint32_t block_idx = block - LOG_BLOCK_START;
    if (g_block_state[block_idx] != BLOCK_STATE_BAD) {
        g_block_state[block_idx] = BLOCK_STATE_BAD;
        g_bad_block_count++;
    }

    if (erase_fail) {
        g_erase_fail_count++;
    } else {
        g_program_fail_count++;
    }

    char line[LOG_LINE_MAX];
    snprintk(line, sizeof(line),
             "MEM_EVENT: bad_block=%u reason=%s t=%lld",
             (unsigned)block,
             reason ? reason : "UNKNOWN",
             k_uptime_get());
    send_text_line_live(line);

    g_next_page = (block + 1U) * PAGES_PER_BLOCK;
}

static bool ensure_page_available(uint32_t page)
{
    while (page < LOG_LAST_PAGE_EXCL) {
        uint32_t block = page / PAGES_PER_BLOCK;
        uint32_t block_idx = block - LOG_BLOCK_START;

        if (g_block_state[block_idx] == BLOCK_STATE_BAD) {
            page = (block + 1U) * PAGES_PER_BLOCK;
            g_next_page = page;
            continue;
        }

        if (g_block_state[block_idx] != BLOCK_STATE_ERASED) {
            LOG_INF("Erasing log block %u for storage", (unsigned)block);
            if (!nand_block_erase(block * PAGES_PER_BLOCK)) {
                LOG_ERR("Block erase failed for block %u", (unsigned)block);
                mark_block_bad(block * PAGES_PER_BLOCK, "ERASE", true);
                page = g_next_page;
                continue;
            }
            g_block_state[block_idx] = BLOCK_STATE_ERASED;
        }

        g_next_page = page;
        return true;
    }

    if (!g_storage_full) {
        g_storage_full = true;
        g_fill_full_ms = k_uptime_get();
        LOG_ERR("NAND log region full");
        send_mem_status("FULL");
        send_text_line_live("MEM_EVENT: storage_full=1");
    }
    return false;
}

static bool flush_current_page(void)
{
    if (!g_storage_ready) {
        return false;
    }

    if (g_page_used == 0U) {
        return true;
    }

    memset(&g_page_buf[g_page_used], 0xFF, NAND_PAGE_SIZE - g_page_used);

    while (1) {
        uint32_t page_addr = g_next_page;

        if (!ensure_page_available(page_addr)) {
            return false;
        }

        page_addr = g_next_page;
        if (nand_program_page(page_addr, g_page_buf, NAND_PAGE_SIZE)) {
            g_next_page = page_addr + 1U;
            g_written_pages++;
            init_page_buf();
            return true;
        }

        LOG_ERR("Program failed at page %u", (unsigned)page_addr);
        mark_block_bad(page_addr, "PROGRAM", false);
    }
}


static bool queue_record(uint8_t type, const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len > BIN_PAYLOAD_MAX) {
        g_queue_drop_count++;
        return false;
    }

    bin_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.len = (uint16_t)(3U + payload_len);
    rec.bytes[0] = BIN_REC_SYNC;
    rec.bytes[1] = type;
    rec.bytes[2] = payload_len;

    if (payload_len > 0U && payload) {
        memcpy(&rec.bytes[3], payload, payload_len);
    }

    if (k_msgq_put(&g_log_msgq, &rec, K_NO_WAIT) != 0) {
        g_queue_drop_count++;
        return false;
    }

    return true;
}

static bool queue_text_record(uint8_t type, const char *text)
{
    if (!text || !text[0]) {
        return false;
    }

    size_t len = strlen(text);
    if (len > BIN_PAYLOAD_MAX) {
        len = BIN_PAYLOAD_MAX;
    }

    return queue_record(type, (const uint8_t *)text, (uint8_t)len);
}

static uint8_t *put_bytes(uint8_t *p, const void *src, size_t n)
{
    memcpy(p, src, n);
    return p + n;
}

static bool pull_bytes(const uint8_t *payload, size_t payload_len,
                       size_t *off, void *dst, size_t n)
{
    if ((*off + n) > payload_len) {
        return false;
    }
    memcpy(dst, payload + *off, n);
    *off += n;
    return true;
}

static const char *rec_type_name(uint8_t type, bool stored)
{
    switch (type) {
    case BIN_TYPE_TEMP:  return stored ? "STORED_RAW_TEMP" : "RAW_TEMP";
    case BIN_TYPE_IMU:   return stored ? "STORED_RAW_IMU"  : "RAW_IMU";
    case BIN_TYPE_PPG:   return stored ? "STORED_RAW_PPG"  : "RAW_PPG";
    case BIN_TYPE_EDA:   return stored ? "STORED_RAW_EDA"  : "RAW_EDA";
    case BIN_TYPE_PROC:  return stored ? "STORED_PROC_V0"  : "PROC_V0";
    case BIN_TYPE_EVENT: return stored ? "STORED_MEM_EVENT": "MEM_EVENT";
    default:             return stored ? "STORED_UNKNOWN"  : "UNKNOWN";
    }
}

static void send_record_decoded(const bin_record_t *rec, bool stored)
{
    if (!ble_log_is_ready() || !rec || rec->len < 3U || rec->bytes[0] != BIN_REC_SYNC) {
        return;
    }

    uint8_t type = rec->bytes[1];
    uint8_t payload_len = rec->bytes[2];
    if ((3U + payload_len) > rec->len) {
        return;
    }

    const uint8_t *payload = &rec->bytes[3];
    const char *name = rec_type_name(type, stored);
    char line[LOG_LINE_MAX];
    size_t off = 0U;

    if (type == BIN_TYPE_TEMP) {
        int64_t t_ms;
        int32_t temp_c_x100;
        if (!pull_bytes(payload, payload_len, &off, &t_ms, sizeof(t_ms)) ||
            !pull_bytes(payload, payload_len, &off, &temp_c_x100, sizeof(temp_c_x100))) {
            return;
        }
        snprintk(line, sizeof(line), "%s: t=%lld temp_c_x100=%ld",
                 name, t_ms, (long)temp_c_x100);
        send_text_line_live(line);
        return;
    }

    if (type == BIN_TYPE_IMU) {
        int64_t t_ms;
        int32_t ax_mg, ay_mg, az_mg, gx_mdps, gy_mdps, gz_mdps;
        if (!pull_bytes(payload, payload_len, &off, &t_ms, sizeof(t_ms)) ||
            !pull_bytes(payload, payload_len, &off, &ax_mg, sizeof(ax_mg)) ||
            !pull_bytes(payload, payload_len, &off, &ay_mg, sizeof(ay_mg)) ||
            !pull_bytes(payload, payload_len, &off, &az_mg, sizeof(az_mg)) ||
            !pull_bytes(payload, payload_len, &off, &gx_mdps, sizeof(gx_mdps)) ||
            !pull_bytes(payload, payload_len, &off, &gy_mdps, sizeof(gy_mdps)) ||
            !pull_bytes(payload, payload_len, &off, &gz_mdps, sizeof(gz_mdps))) {
            return;
        }
        snprintk(line, sizeof(line),
                 "%s: t=%lld ax_mg=%ld ay_mg=%ld az_mg=%ld gx_mdps=%ld gy_mdps=%ld gz_mdps=%ld",
                 name, t_ms,
                 (long)ax_mg, (long)ay_mg, (long)az_mg,
                 (long)gx_mdps, (long)gy_mdps, (long)gz_mdps);
        send_text_line_live(line);
        return;
    }

    if (type == BIN_TYPE_PPG) {
        int64_t t_ms;
        uint8_t mask;
        uint32_t green = 0U, ir = 0U, red = 0U;

        if (!pull_bytes(payload, payload_len, &off, &t_ms, sizeof(t_ms)) ||
            !pull_bytes(payload, payload_len, &off, &mask, sizeof(mask))) {
            return;
        }

        if (mask & W25N01_PPG_CH_GREEN) {
            if (!pull_bytes(payload, payload_len, &off, &green, sizeof(green))) return;
        }
        if (mask & W25N01_PPG_CH_IR) {
            if (!pull_bytes(payload, payload_len, &off, &ir, sizeof(ir))) return;
        }
        if (mask & W25N01_PPG_CH_RED) {
            if (!pull_bytes(payload, payload_len, &off, &red, sizeof(red))) return;
        }

        int n = snprintk(line, sizeof(line), "%s: t=%lld mask=0x%02X", name, t_ms, mask);
        if ((mask & W25N01_PPG_CH_GREEN) && n > 0 && n < (int)sizeof(line)) {
            n += snprintk(line + n, sizeof(line) - (size_t)n, " green=%u", green);
        }
        if ((mask & W25N01_PPG_CH_IR) && n > 0 && n < (int)sizeof(line)) {
            n += snprintk(line + n, sizeof(line) - (size_t)n, " ir=%u", ir);
        }
        if ((mask & W25N01_PPG_CH_RED) && n > 0 && n < (int)sizeof(line)) {
            n += snprintk(line + n, sizeof(line) - (size_t)n, " red=%u", red);
        }
        send_text_line_live(line);
        return;
    }

    if (type == BIN_TYPE_EDA) {
        int64_t t_ms;
        int16_t raw;
        int32_t mv;
        if (!pull_bytes(payload, payload_len, &off, &t_ms, sizeof(t_ms)) ||
            !pull_bytes(payload, payload_len, &off, &raw, sizeof(raw)) ||
            !pull_bytes(payload, payload_len, &off, &mv, sizeof(mv))) {
            return;
        }
        snprintk(line, sizeof(line), "%s: t=%lld raw=%d mv=%ld",
                 name, t_ms, raw, (long)mv);
        send_text_line_live(line);
        return;
    }

    if (type == BIN_TYPE_PROC || type == BIN_TYPE_EVENT) {
        char text[BIN_PAYLOAD_MAX + 1U];
        size_t n = MIN((size_t)payload_len, sizeof(text) - 1U);
        memcpy(text, payload, n);
        text[n] = '\0';
        snprintk(line, sizeof(line), "%s: %s", name, text);
        send_text_line_live(line);
        return;
    }

    snprintk(line, sizeof(line), "%s: type=0x%02X len=%u", name, type, payload_len);
    send_text_line_live(line);
}

static bool append_record_to_page(const bin_record_t *rec)
{
    if (!rec || rec->len == 0U || rec->len > BIN_REC_MAX) {
        return false;
    }

    if (g_fill_start_ms < 0) {
        g_fill_start_ms = k_uptime_get();
        g_last_status_ms = g_fill_start_ms;
    }

    if ((g_page_used + rec->len) > NAND_PAGE_SIZE) {
        if (!flush_current_page()) {
            return false;
        }
    }

    memcpy(&g_page_buf[g_page_used], rec->bytes, rec->len);
    g_page_used += rec->len;

    g_bytes_logged_total += (uint32_t)rec->len;
    g_lines_logged_total++;
    g_last_append_ms = k_uptime_get();
    return true;
}

static void replay_stored_logs_to_ble(void)
{
    if (!ble_log_is_ready() || !g_storage_ready) {
        return;
    }

    /* Flush the current partial page so the app gets a coherent snapshot */
    (void)flush_current_page();

    send_mem_status("REPLAY");
    send_text_line_live("MEM_STATUS: starting stored binary log replay");

    uint8_t page[NAND_PAGE_SIZE];

    for (uint32_t i = 0; i < g_written_pages; i++) {
        uint32_t page_addr = LOG_FIRST_PAGE + i;

        if (!nand_page_read_to_cache(page_addr)) {
            LOG_ERR("Replay page-read failed @%u", (unsigned)page_addr);
            break;
        }
        if (nand_read_cache(0, page, sizeof(page)) != 0) {
            LOG_ERR("Replay cache-read failed @%u", (unsigned)page_addr);
            break;
        }

        size_t off = 0U;
        while ((off + 3U) <= NAND_PAGE_SIZE) {
            if (page[off] == 0xFFU) {
                break;
            }

            if (page[off] != BIN_REC_SYNC) {
                LOG_WRN("Replay stopped at page=%u off=%u invalid sync=0x%02X",
                        (unsigned)page_addr, (unsigned)off, page[off]);
                break;
            }

            uint8_t payload_len = page[off + 2U];
            size_t rec_len = 3U + (size_t)payload_len;
            if (rec_len > BIN_REC_MAX || (off + rec_len) > NAND_PAGE_SIZE) {
                LOG_WRN("Replay stopped at page=%u off=%u invalid len=%u",
                        (unsigned)page_addr, (unsigned)off, payload_len);
                break;
            }

            bin_record_t rec;
            memset(&rec, 0, sizeof(rec));
            rec.len = (uint16_t)rec_len;
            memcpy(rec.bytes, &page[off], rec_len);
            send_record_decoded(&rec, true);

            off += rec_len;
            k_msleep(2);
        }
    }

    send_text_line_live("MEM_STATUS: stored binary log replay complete");
}

/* ======== public APIs used by tasks / main ======== */

void w25n01_set_worn(bool worn)
{
    if (g_worn == worn) {
        return;
    }

    g_worn = worn;

    char line[LOG_LINE_MAX];
    snprintk(line, sizeof(line), "worn=%d t=%lld",
             (int)worn, k_uptime_get());
    (void)queue_text_record(BIN_TYPE_EVENT, line);

    char live[LOG_LINE_MAX];
    snprintk(live, sizeof(live), "MEM_EVENT: %s", line);
    send_text_line_live(live);
}

void w25n01_log_mem_snapshot(void)
{
    send_mem_status("MANUAL");
}

void w25n01_log_raw_temp_centi(int32_t temp_c_x100, int64_t t_ms)
{
    if (!g_worn || g_storage_full) {
        return;
    }

    uint8_t payload[sizeof(t_ms) + sizeof(temp_c_x100)];
    uint8_t *p = payload;
    p = put_bytes(p, &t_ms, sizeof(t_ms));
    p = put_bytes(p, &temp_c_x100, sizeof(temp_c_x100));
    (void)queue_record(BIN_TYPE_TEMP, payload, (uint8_t)(p - payload));
}

void w25n01_log_raw_imu(int32_t ax_mg, int32_t ay_mg, int32_t az_mg,
                        int32_t gx_mdps, int32_t gy_mdps, int32_t gz_mdps,
                        int64_t t_ms)
{
    if (!g_worn || g_storage_full) {
        return;
    }

    uint8_t payload[sizeof(t_ms) + (6U * sizeof(int32_t))];
    uint8_t *p = payload;
    p = put_bytes(p, &t_ms, sizeof(t_ms));
    p = put_bytes(p, &ax_mg, sizeof(ax_mg));
    p = put_bytes(p, &ay_mg, sizeof(ay_mg));
    p = put_bytes(p, &az_mg, sizeof(az_mg));
    p = put_bytes(p, &gx_mdps, sizeof(gx_mdps));
    p = put_bytes(p, &gy_mdps, sizeof(gy_mdps));
    p = put_bytes(p, &gz_mdps, sizeof(gz_mdps));
    (void)queue_record(BIN_TYPE_IMU, payload, (uint8_t)(p - payload));
}

void w25n01_log_raw_ppg_mask(uint8_t channel_mask,
                             uint32_t red, uint32_t ir, uint32_t green,
                             int64_t t_ms)
{
    if (!g_worn || g_storage_full) {
        return;
    }

    channel_mask &= W25N01_PPG_CH_ALL;
    if (channel_mask == 0U) {
        return;
    }

    uint8_t payload[sizeof(t_ms) + sizeof(channel_mask) + (3U * sizeof(uint32_t))];
    uint8_t *p = payload;
    p = put_bytes(p, &t_ms, sizeof(t_ms));
    p = put_bytes(p, &channel_mask, sizeof(channel_mask));

    /* Store only active channels. Order is GREEN, IR, RED for app decoding. */
    if (channel_mask & W25N01_PPG_CH_GREEN) {
        p = put_bytes(p, &green, sizeof(green));
    }
    if (channel_mask & W25N01_PPG_CH_IR) {
        p = put_bytes(p, &ir, sizeof(ir));
    }
    if (channel_mask & W25N01_PPG_CH_RED) {
        p = put_bytes(p, &red, sizeof(red));
    }

    (void)queue_record(BIN_TYPE_PPG, payload, (uint8_t)(p - payload));
}

void w25n01_log_raw_ppg(uint32_t red, uint32_t ir, uint32_t green, int64_t t_ms)
{
    w25n01_log_raw_ppg_mask(W25N01_PPG_CH_ALL, red, ir, green, t_ms);
}

void w25n01_log_raw_eda(int16_t raw, int32_t mv, int64_t t_ms)
{
    if (!g_worn || g_storage_full) {
        return;
    }

    uint8_t payload[sizeof(t_ms) + sizeof(raw) + sizeof(mv)];
    uint8_t *p = payload;
    p = put_bytes(p, &t_ms, sizeof(t_ms));
    p = put_bytes(p, &raw, sizeof(raw));
    p = put_bytes(p, &mv, sizeof(mv));
    (void)queue_record(BIN_TYPE_EDA, payload, (uint8_t)(p - payload));
}

void w25n01_log_processed_line(const char *line)
{
    if (!g_worn || g_storage_full || !line || !line[0]) {
        return;
    }

    (void)queue_text_record(BIN_TYPE_PROC, line);
}

/* ======== worker thread ======== */
static void w25n01_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    gpio0 = DEVICE_DT_GET(GPIO0_NODE);
    gpio1 = DEVICE_DT_GET(GPIO1_NODE);
    spi_dev = DEVICE_DT_GET(SPI_NODE);

    if (!device_is_ready(gpio0) || !device_is_ready(gpio1) || !device_is_ready(spi_dev)) {
        LOG_ERR("Devices not ready");
        return;
    }

    gpio_pin_configure(gpio0, CS_PIN, GPIO_OUTPUT_HIGH);
    gpio_pin_configure(gpio0, WP_PIN, GPIO_OUTPUT_HIGH);
    gpio_pin_configure(gpio1, HOLD_PIN, GPIO_OUTPUT_HIGH);
    k_msleep(10);

    nand_reset();
    set_protection_off();

    memset(g_block_state, 0, sizeof(g_block_state));
    init_page_buf();
    g_next_page = LOG_FIRST_PAGE;
    g_written_pages = 0U;
    g_bytes_logged_total = 0U;
    g_lines_logged_total = 0U;
    g_bad_block_count = 0U;
    g_queue_drop_count = 0U;
    g_erase_fail_count = 0U;
    g_program_fail_count = 0U;
    g_storage_full = false;
    g_storage_ready = false;
    g_worn = false;
    g_prev_ble_ready = false;
    g_dumped_this_connection = false;
    g_fill_start_ms = -1;
    g_fill_full_ms = -1;
    g_last_status_ms = 0;

    if (!ensure_page_available(g_next_page)) {
        LOG_ERR("No usable blocks available for logging");
        return;
    }

    g_storage_ready = true;

    LOG_INF("W25N01 binary storage ready: blocks=%u..%u (~%u bytes reserved, full-capacity test region)",
        (unsigned)LOG_BLOCK_START,
        (unsigned)(LOG_BLOCK_START + LOG_BLOCK_COUNT - 1U),
        (unsigned)LOG_CAPACITY_BYTES);

    while (1) {
        bin_record_t rec;

        /* Drain one queued binary record if available */
        if (k_msgq_get(&g_log_msgq, &rec, K_MSEC(100)) == 0) {
            if (g_storage_ready && !g_storage_full) {
                (void)append_record_to_page(&rec);

                if (ble_log_is_ready()) {
                    send_record_decoded(&rec, false);
                }
            }
        }

        if (g_storage_ready && g_page_used > 0U && g_last_append_ms > 0) {
            if ((k_uptime_get() - g_last_append_ms) >= 1000) {
                (void)flush_current_page();
            }
        }

        if (g_storage_ready && g_fill_start_ms >= 0 && !g_storage_full) {
            int64_t now = k_uptime_get();
            if ((now - g_last_status_ms) >= MEM_STATUS_PERIOD_MS) {
                send_mem_status("PERIODIC");
                g_last_status_ms = now;
            }
        }

        bool ble_ready = ble_log_is_ready();

        if (ble_ready && !g_prev_ble_ready) {
            g_dumped_this_connection = false;
        }

        if (ble_ready && !g_dumped_this_connection) {
            replay_stored_logs_to_ble();
            g_dumped_this_connection = true;
        }

        if (!ble_ready) {
            g_dumped_this_connection = false;
        }

        g_prev_ble_ready = ble_ready;
    }
}

/* thread objects */
#define W25N01_STACK_SIZE 4096
#define W25N01_PRIORITY   5

K_THREAD_STACK_DEFINE(w25n01_stack, W25N01_STACK_SIZE);
static struct k_thread w25n01_tcb;
static bool started;

void w25n01_task_start(void)
{
    if (started) {
        return;
    }
    started = true;

    k_thread_create(&w25n01_tcb, w25n01_stack, K_THREAD_STACK_SIZEOF(w25n01_stack),
                    w25n01_thread, NULL, NULL, NULL,
                    W25N01_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&w25n01_tcb, "w25n01_task");
}
