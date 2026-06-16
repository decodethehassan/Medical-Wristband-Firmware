#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "algo_v0.h"
#include "max30101_task.h"
#include "w25n01_task.h"
#include "as6221_task.h"
#include "ppg_validation.h"

LOG_MODULE_REGISTER(max30101_demo, LOG_LEVEL_INF);

#define MAX30101_I2C_ADDR 0x57

/* Registers */
#define REG_INTR_STATUS_1     0x00
#define REG_INTR_STATUS_2     0x01
#define REG_INTR_ENABLE_1     0x02
#define REG_INTR_ENABLE_2     0x03
#define REG_FIFO_WR_PTR       0x04
#define REG_FIFO_OVF_CNT      0x05
#define REG_FIFO_RD_PTR       0x06
#define REG_FIFO_DATA         0x07
#define REG_FIFO_CONFIG       0x08
#define REG_MODE_CONFIG       0x09
#define REG_SPO2_CONFIG       0x0A
#define REG_LED1_PA           0x0C
#define REG_LED2_PA           0x0D
#define REG_LED3_PA           0x0E
#define REG_MULTI_LED_CTRL1   0x11
#define REG_MULTI_LED_CTRL2   0x12
#define REG_REV_ID            0xFE
#define REG_PART_ID           0xFF

/*
 * MAX30101 does not support native 64 Hz sampling.
 * Keep sensor at 100 Hz and downsample in firmware to an exact effective 64 Hz
 * for both algo feed and raw-memory logging.
 */
#define PPG_SENSOR_FS_HZ          100U
#define PPG_SENSOR_FRAME_MS       10
#define PPG_TARGET_FS_HZ          64U
#define PPG_LOOP_SLEEP_MS         10
#define PPG_LOG_EVERY_N           64U
#define PPG_FIFO_DBG_EVERY_N      50U

/* -------- Skin / wear detection tuning -------- */
/*
 * Worn detection is used only to enable/disable NAND storage.
 * HR/HRV peak detection remains unchanged.
 *
 * Earlier thresholds were too strict for the current GREEN-first MAX30101 setup:
 * a light/normal finger contact could produce valid HR/SQI, but storage still
 * stayed at worn=0 until extra pressure was applied.  The thresholds below are
 * intentionally contact-friendly while still keeping hysteresis between ON/OFF.
 */
#define WEAR_IR_ON_THRESH           8000U
#define WEAR_IR_OFF_THRESH          3000U
#define WEAR_IR_HARD_OFF_THRESH     1500U

#define WEAR_AC_ON_THRESH            60.0f
#define WEAR_AC_OFF_THRESH           20.0f

#define WEAR_TEMP_ON_C              29.0f
#define WEAR_TEMP_OFF_C             27.0f
#define WEAR_TEMP_STALE_MS          5000

#define WEAR_ON_HOLD_MS             1000
#define WEAR_OFF_HOLD_MS            5000
#define WEAR_DBG_EVERY_N           100U

#define IR_DC_ALPHA                  0.01f
#define IR_AC_ENV_ALPHA              0.05f

static const struct device *i2c_dev;

/* Default client-requested mode: GREEN only. Advanced/app mode can add IR and/or RED. */
static volatile uint8_t g_requested_ppg_channels = MAX30101_PPG_CH_DEFAULT;
static uint8_t g_applied_ppg_channels = 0U;

#define PPG_LED_CURRENT_DEFAULT 0x28U

static int wr(uint8_t reg, uint8_t val)
{
    return i2c_reg_write_byte(i2c_dev, MAX30101_I2C_ADDR, reg, val);
}

static int rd(uint8_t reg, uint8_t *val)
{
    return i2c_reg_read_byte(i2c_dev, MAX30101_I2C_ADDR, reg, val);
}


static uint8_t sanitize_channel_mask(uint8_t mask)
{
#if PPG_VALIDATION_MODE && PPG_VAL_GREEN_ONLY
    ARG_UNUSED(mask);
    return MAX30101_PPG_CH_GREEN;
#else
    mask &= MAX30101_PPG_CH_ALL;

    /* GREEN is the default/base channel requested by the client.
     * Advanced mode adds IR and/or RED, but GREEN remains enabled.
     */
    if ((mask & MAX30101_PPG_CH_GREEN) == 0U) {
        mask |= MAX30101_PPG_CH_GREEN;
    }

    return mask;
#endif
}

const char *max30101_channel_mask_name(uint8_t mask)
{
    mask = sanitize_channel_mask(mask);

    if (mask == MAX30101_PPG_CH_GREEN) {
        return "GREEN";
    }
    if (mask == (MAX30101_PPG_CH_GREEN | MAX30101_PPG_CH_IR)) {
        return "GREEN+IR";
    }
    if (mask == (MAX30101_PPG_CH_GREEN | MAX30101_PPG_CH_RED)) {
        return "GREEN+RED";
    }
    if (mask == MAX30101_PPG_CH_ALL) {
        return "GREEN+IR+RED";
    }
    return "GREEN+CUSTOM";
}

int max30101_set_channel_mask(uint8_t mask)
{
    g_requested_ppg_channels = sanitize_channel_mask(mask);
    LOG_INF("PPG mode requested: mask=0x%02X (%s)",
            g_requested_ppg_channels,
            max30101_channel_mask_name(g_requested_ppg_channels));
    return 0;
}

uint8_t max30101_get_channel_mask(void)
{
    return sanitize_channel_mask(g_requested_ppg_channels);
}

static uint8_t parse_channel_mask_from_cmd(const char *cmd)
{
    uint8_t mask = 0U;

    if (!cmd || !cmd[0]) {
        return MAX30101_PPG_CH_DEFAULT;
    }

    if (strstr(cmd, "DEFAULT") || strstr(cmd, "GREEN_ONLY")) {
        return MAX30101_PPG_CH_GREEN;
    }

    if (strstr(cmd, "GREEN")) {
        mask |= MAX30101_PPG_CH_GREEN;
    }
    if (strstr(cmd, "IR")) {
        mask |= MAX30101_PPG_CH_IR;
    }
    if (strstr(cmd, "RED")) {
        mask |= MAX30101_PPG_CH_RED;
    }

    return sanitize_channel_mask(mask);
}

void max30101_handle_ble_command(const char *cmd)
{
    if (!cmd || !cmd[0]) {
        return;
    }

    if (strstr(cmd, "PPG_MODE") || strstr(cmd, "PPG_CH")) {
        uint8_t mask = parse_channel_mask_from_cmd(cmd);
        (void)max30101_set_channel_mask(mask);
        return;
    }

    LOG_WRN("Unknown BLE command for MAX30101: %s", cmd);
}

static void max30101_apply_channel_mask(uint8_t mask)
{
    mask = sanitize_channel_mask(mask);

    uint8_t red_pa   = (mask & MAX30101_PPG_CH_RED)   ? PPG_LED_CURRENT_DEFAULT : 0x00U;
    uint8_t ir_pa    = (mask & MAX30101_PPG_CH_IR)    ? PPG_LED_CURRENT_DEFAULT : 0x00U;
    uint8_t green_pa = (mask & MAX30101_PPG_CH_GREEN) ? PPG_LED_CURRENT_DEFAULT : 0x00U;

    wr(REG_LED1_PA, red_pa);     /* LED1 = RED */
    wr(REG_LED2_PA, ir_pa);      /* LED2 = IR */
    wr(REG_LED3_PA, green_pa);   /* LED3 = GREEN */

    /* Keep the three FIFO slots stable for firmware compatibility.
     * Inactive channels have LED current 0 and are ignored by logging/app display.
     */
    wr(REG_MULTI_LED_CTRL1, 0x21); /* slot1 RED, slot2 IR */
    wr(REG_MULTI_LED_CTRL2, 0x03); /* slot3 GREEN */

    g_applied_ppg_channels = mask;

    LOG_INF("PPG mode applied: mask=0x%02X (%s) | LED_PA red=0x%02X ir=0x%02X green=0x%02X",
            mask, max30101_channel_mask_name(mask), red_pa, ir_pa, green_pa);
}

static uint32_t parse_sample18(const uint8_t b[3])
{
    uint32_t v = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    return v & 0x3FFFFu;
}

static void dump_regs(void)
{
    uint8_t v;

    rd(REG_MODE_CONFIG, &v);      LOG_INF("MODE_CONFIG      (0x09) = 0x%02X", v);
    rd(REG_FIFO_CONFIG, &v);      LOG_INF("FIFO_CONFIG      (0x08) = 0x%02X", v);
    rd(REG_SPO2_CONFIG, &v);      LOG_INF("SPO2_CONFIG      (0x0A) = 0x%02X", v);
    rd(REG_MULTI_LED_CTRL1, &v);  LOG_INF("MULTI_LED_CTRL1  (0x11) = 0x%02X", v);
    rd(REG_MULTI_LED_CTRL2, &v);  LOG_INF("MULTI_LED_CTRL2  (0x12) = 0x%02X", v);
    rd(REG_LED1_PA, &v);          LOG_INF("LED1_PA (RED)    (0x0C) = 0x%02X", v);
    rd(REG_LED2_PA, &v);          LOG_INF("LED2_PA (IR)     (0x0D) = 0x%02X", v);
    rd(REG_LED3_PA, &v);          LOG_INF("LED3_PA (GREEN)  (0x0E) = 0x%02X", v);
}

static bool max30101_reset_wait(void)
{
    int err = wr(REG_MODE_CONFIG, 0x40);
    if (err) {
        LOG_ERR("Reset write failed err=%d", err);
        return false;
    }

    for (int i = 0; i < 50; i++) {
        uint8_t mc = 0;
        if (rd(REG_MODE_CONFIG, &mc) == 0) {
            if ((mc & 0x40) == 0) {
                return true;
            }
        }
        k_msleep(10);
    }

    LOG_ERR("Reset did not clear!");
    return false;
}

static void max30101_manual_init(void)
{
    uint8_t part = 0;
    uint8_t rev = 0;

    if (rd(REG_PART_ID, &part) == 0 && rd(REG_REV_ID, &rev) == 0) {
        LOG_INF("PART_ID (0xFF)=0x%02X | REV_ID (0xFE)=0x%02X", part, rev);
    } else {
        LOG_ERR("Failed to read PART/REV ID");
    }

    if (!max30101_reset_wait()) {
        return;
    }

    wr(REG_INTR_ENABLE_1, 0x00);
    wr(REG_INTR_ENABLE_2, 0x00);

    wr(REG_FIFO_CONFIG, 0x1F);
    wr(REG_MODE_CONFIG, 0x07);

    /*
     * Keep sensor at 100 Hz, 18-bit pulse width.
     * Effective outgoing PPG rate is reduced to 64 Hz in firmware.
     */
    wr(REG_SPO2_CONFIG, 0x27);

    max30101_apply_channel_mask(max30101_get_channel_mask());

    wr(REG_FIFO_WR_PTR,  0x00);
    wr(REG_FIFO_OVF_CNT, 0x00);
    wr(REG_FIFO_RD_PTR,  0x00);

    uint8_t tmp;
    rd(REG_INTR_STATUS_1, &tmp);
    rd(REG_INTR_STATUS_2, &tmp);

    LOG_INF("Manual Multi-LED configuration applied with active mode: %s.", max30101_channel_mask_name(g_applied_ppg_channels));
    dump_regs();
}

static int wear_on_score(uint32_t ir, float ir_ac_env, bool temp_valid, float temp_c)
{
    int score = 0;

    if (ir >= WEAR_IR_ON_THRESH) {
        score++;
    }
    if (ir_ac_env >= WEAR_AC_ON_THRESH) {
        score++;
    }
    if (temp_valid && temp_c >= WEAR_TEMP_ON_C) {
        score++;
    }

    return score;
}

static int wear_off_score(uint32_t ir, float ir_ac_env, bool temp_valid, float temp_c)
{
    int score = 0;

    if (ir <= WEAR_IR_OFF_THRESH) {
        score++;
    }
    if (ir_ac_env <= WEAR_AC_OFF_THRESH) {
        score++;
    }
    if (temp_valid && temp_c <= WEAR_TEMP_OFF_C) {
        score++;
    }

    return score;
}

static void log_wear_transition(bool worn,
                                uint32_t ir,
                                float ir_dc,
                                float ir_ac_env,
                                bool temp_valid,
                                float temp_c,
                                int64_t temp_age_ms,
                                int on_score,
                                int off_score,
                                int64_t t_ms)
{
    LOG_INF("WEAR_EVENT: state=%s t=%lld ir=%u ir_dc=%.0f ir_ac=%.1f temp=%s%.2f temp_age=%lld on_score=%d off_score=%d",
            worn ? "WORN" : "NOT_WORN",
            t_ms,
            ir,
            (double)ir_dc,
            (double)ir_ac_env,
            temp_valid ? "" : "NA/",
            (double)(temp_valid ? temp_c : -1000.0f),
            temp_valid ? temp_age_ms : -1,
            on_score,
            off_score);
}

static void advance_64hz_time(int64_t *t_ms, uint32_t *frac_us)
{
    /*
     * 1000 / 64 = 15.625 ms
     * Use integer ms + fractional accumulator:
     * +15 ms, remainder +625 us each sample.
     */
    *t_ms += 15;
    *frac_us += 625U;
    if (*frac_us >= 1000U) {
        *t_ms += 1;
        *frac_us -= 1000U;
    }
}

static void max30101_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    LOG_INF("=== MAX30101 REGISTER-LEVEL FIFO READ (NO ZEPHYR DRIVER) ===");
    LOG_INF("PPG sensor=100 Hz, effective output/logging=64 Hz");
    LOG_INF("Wear detect: selected active PPG channel + AC envelope + AS6221 temperature with hysteresis");

    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C0 NOT READY");
        return;
    }

    max30101_manual_init();

    uint32_t tick = 0U;
    uint32_t raw_frame_count = 0U;
    uint32_t pub_frame_count = 0U;

    int64_t next_raw_frame_t_ms = 0;
    int64_t next_pub_t_ms = 0;
    uint32_t next_pub_t_frac_us = 0U;

    /*
     * Rational downsampler from 100 Hz -> 64 Hz.
     * Accumulator-based keep decision:
     *   acc += 64; keep if acc >= 100; then acc -= 100.
     */
    uint32_t decim_acc = 0U;

    bool worn = false;
    int64_t on_candidate_since_ms = -1;
    int64_t off_candidate_since_ms = -1;

    bool metrics_init = false;
    float ir_dc_ema = 0.0f;
    float ir_ac_env = 0.0f;

    while (1) {
        uint8_t requested_mask = max30101_get_channel_mask();
        if (requested_mask != g_applied_ppg_channels) {
            max30101_apply_channel_mask(requested_mask);
        }

        uint8_t wrp = 0;
        uint8_t rdp = 0;
        uint8_t ovf = 0;

        if (rd(REG_FIFO_WR_PTR, &wrp) != 0 ||
            rd(REG_FIFO_RD_PTR, &rdp) != 0 ||
            rd(REG_FIFO_OVF_CNT, &ovf) != 0) {
            LOG_ERR("Failed to read FIFO pointers");
            k_msleep(100);
            continue;
        }

        uint8_t available_slots = (wrp - rdp) & 0x1F;
        uint8_t available_frames = available_slots / 3U;

        if ((++tick % PPG_FIFO_DBG_EVERY_N) == 0U) {
#if !(PPG_VALIDATION_MODE && PPG_VAL_DISABLE_DEBUG_LOGS)
            uint8_t s1 = 0;
            uint8_t s2 = 0;
            uint8_t mc = 0;
            rd(REG_INTR_STATUS_1, &s1);
            rd(REG_INTR_STATUS_2, &s2);
            rd(REG_MODE_CONFIG, &mc);

            LOG_INF("FIFO DBG | WR=%u RD=%u OVF=%u slots=%u frames=%u | INT1=0x%02X INT2=0x%02X | MODE=0x%02X",
                wrp, rdp, ovf, available_slots, available_frames, s1, s2, mc);
#endif
        }

        if (available_frames == 0U) {
            k_msleep(PPG_LOOP_SLEEP_MS);
            continue;
        }

        int64_t now_ms = k_uptime_get();
        if (next_raw_frame_t_ms == 0) {
            next_raw_frame_t_ms = now_ms - ((int64_t)(available_frames - 1U) * PPG_SENSOR_FRAME_MS);
            next_pub_t_ms = next_raw_frame_t_ms;
            next_pub_t_frac_us = 0U;
        }

        for (uint8_t i = 0; i < available_frames; i++) {
            uint8_t raw[9];
            int err = i2c_burst_read(i2c_dev, MAX30101_I2C_ADDR, REG_FIFO_DATA, raw, sizeof(raw));
            if (err) {
                LOG_ERR("FIFO read err=%d", err);
                break;
            }

            uint32_t red   = parse_sample18(&raw[0]);
            uint32_t ir    = parse_sample18(&raw[3]);
            uint32_t green = parse_sample18(&raw[6]);

            uint8_t active_mask = g_applied_ppg_channels;
            uint32_t ppg_signal = (active_mask & MAX30101_PPG_CH_IR) ? ir : green;

            int64_t raw_sample_t_ms = next_raw_frame_t_ms;
            next_raw_frame_t_ms += PPG_SENSOR_FRAME_MS;

            /* Wear-detection metrics run on every raw 100 Hz frame */
            if (!metrics_init) {
                metrics_init = true;
                ir_dc_ema = (float)ppg_signal;
                ir_ac_env = 0.0f;
            } else {
                float ir_f = (float)ppg_signal;
                ir_dc_ema += IR_DC_ALPHA * (ir_f - ir_dc_ema);
                ir_ac_env += IR_AC_ENV_ALPHA * (fabsf(ir_f - ir_dc_ema) - ir_ac_env);
            }

            float temp_c = -1000.0f;
            int64_t temp_age_ms = -1;
#if PPG_VALIDATION_MODE && PPG_VAL_DISABLE_TEMP
            bool temp_valid = false;
#else
            bool temp_valid = as6221_get_latest_temp_c(&temp_c, &temp_age_ms) &&
                              (temp_age_ms >= 0) && (temp_age_ms <= WEAR_TEMP_STALE_MS);
#endif

            int on_score = wear_on_score(ppg_signal, ir_ac_env, temp_valid, temp_c);
            int off_score = wear_off_score(ppg_signal, ir_ac_env, temp_valid, temp_c);

            bool on_candidate;
            bool off_candidate;

            if (temp_valid) {
                on_candidate = (on_score >= 2);
                off_candidate = (off_score >= 2) || (ppg_signal <= WEAR_IR_HARD_OFF_THRESH);
            } else {
                on_candidate = (ppg_signal >= WEAR_IR_ON_THRESH) && (ir_ac_env >= WEAR_AC_ON_THRESH);
                off_candidate = ((ppg_signal <= WEAR_IR_OFF_THRESH) && (ir_ac_env <= WEAR_AC_OFF_THRESH)) ||
                                (ppg_signal <= WEAR_IR_HARD_OFF_THRESH);
            }

            if (!worn) {
                off_candidate_since_ms = -1;
                if (on_candidate) {
                    if (on_candidate_since_ms < 0) {
                        on_candidate_since_ms = raw_sample_t_ms;
                    }
                    if ((raw_sample_t_ms - on_candidate_since_ms) >= WEAR_ON_HOLD_MS) {
                        worn = true;
                        on_candidate_since_ms = -1;
                        log_wear_transition(true, ppg_signal, ir_dc_ema, ir_ac_env,
                                            temp_valid, temp_c, temp_age_ms,
                                            on_score, off_score, raw_sample_t_ms);
#if !(PPG_VALIDATION_MODE && PPG_VAL_DISABLE_NAND)
                        w25n01_set_worn(true);
#endif
                    }
                } else {
                    on_candidate_since_ms = -1;
                }
            } else {
                on_candidate_since_ms = -1;
                if (off_candidate) {
                    if (off_candidate_since_ms < 0) {
                        off_candidate_since_ms = raw_sample_t_ms;
                    }
                    if ((raw_sample_t_ms - off_candidate_since_ms) >= WEAR_OFF_HOLD_MS) {
                        worn = false;
                        off_candidate_since_ms = -1;
                        log_wear_transition(false, ppg_signal, ir_dc_ema, ir_ac_env,
                                            temp_valid, temp_c, temp_age_ms,
                                            on_score, off_score, raw_sample_t_ms);
#if !(PPG_VALIDATION_MODE && PPG_VAL_DISABLE_NAND)
                        w25n01_set_worn(false);
#endif
                    }
                } else {
                    off_candidate_since_ms = -1;
                }
            }

            if ((raw_frame_count % WEAR_DBG_EVERY_N) == 0U) {
#if !(PPG_VALIDATION_MODE && PPG_VAL_DISABLE_DEBUG_LOGS)
                LOG_INF("WEAR_DBG: state=%s mode=%s ppg_sig=%u red=%u ir=%u green=%u ppg_dc=%.0f ppg_ac=%.1f temp=%s%.2f temp_age=%lld on_score=%d off_score=%d",
                    worn ? "WORN" : "NOT_WORN",
                    max30101_channel_mask_name(active_mask),
                    ppg_signal,
                    red,
                    ir,
                    green,
                    (double)ir_dc_ema,
                    (double)ir_ac_env,
                    temp_valid ? "" : "NA/",
                    (double)(temp_valid ? temp_c : -1000.0f),
                    temp_valid ? temp_age_ms : -1,
                    on_score,
                    off_score);
#endif
            }

            /*
             * Exact effective 64 Hz output/logging from 100 Hz raw input.
             */
            decim_acc += PPG_TARGET_FS_HZ;
            if (decim_acc >= PPG_SENSOR_FS_HZ) {
                int64_t pub_t_ms = next_pub_t_ms;
                decim_acc -= PPG_SENSOR_FS_HZ;

#if PPG_VALIDATION_MODE
                /* Validation mode keeps the algorithm input explicitly on real GREEN. */
                uint32_t algo_ppg_signal = green;
                algo_v0_push_ppg(red, ir, green, pub_t_ms);
#else
                /* Existing Algo V0 processes the value passed in the IR argument.
                 * In default GREEN-only mode, feed GREEN as the selected active signal.
                 */
                uint32_t algo_ppg_signal = (active_mask & MAX30101_PPG_CH_IR) ? ir : green;
                algo_v0_push_ppg(red, algo_ppg_signal, green, pub_t_ms);
#endif

#if !(PPG_VALIDATION_MODE && PPG_VAL_DISABLE_NAND)
                /* Binary NAND logging stores only active/app-enabled PPG channels. */
                w25n01_log_raw_ppg_mask(active_mask, red, ir, green, pub_t_ms);
#endif

#if !(PPG_VALIDATION_MODE && PPG_VAL_DISABLE_DEBUG_LOGS)
                if ((pub_frame_count % PPG_LOG_EVERY_N) == 0U) {
                    LOG_INF("PPG OUT | mode=%s mask=0x%02X | RED=%u | IR=%u | GREEN=%u | stored_sig=%u | t=%lld",
                        max30101_channel_mask_name(active_mask), active_mask, red, ir, green, algo_ppg_signal, pub_t_ms);
                }
#endif

                pub_frame_count++;
                advance_64hz_time(&next_pub_t_ms, &next_pub_t_frac_us);
            }

            raw_frame_count++;
        }

        if (next_raw_frame_t_ms < (now_ms - 100)) {
            next_raw_frame_t_ms = now_ms;
            next_pub_t_ms = now_ms;
            next_pub_t_frac_us = 0U;
            decim_acc = 0U;
        }

        k_msleep(PPG_LOOP_SLEEP_MS);
    }
}

#define MAX30101_STACK_SIZE 3072
#define MAX30101_PRIORITY   5

K_THREAD_STACK_DEFINE(max30101_stack, MAX30101_STACK_SIZE);
static struct k_thread max30101_tcb;
static bool started;

void max30101_task_start(void)
{
    if (started) {
        return;
    }
    started = true;

    k_thread_create(&max30101_tcb, max30101_stack, K_THREAD_STACK_SIZEOF(max30101_stack),
            max30101_thread, NULL, NULL, NULL,
            MAX30101_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&max30101_tcb, "max30101_task");
}