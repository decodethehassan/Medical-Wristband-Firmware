#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <hal/nrf_saadc.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/reboot.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "ble_log_service.h"
#include "as6221_task.h"
#include "lsm6dso_task.h"
#include "max30101_task.h"
#include "ads1113_task.h"
#include "w25n01_task.h"
#include "algo_v0.h"
#include "ppg_validation.h"

extern void w25n01_log_processed_line(const char *line);

LOG_MODULE_REGISTER(main_all, LOG_LEVEL_INF);

/* PCBv2 soft-latch power circuit
 *
 * Schematic mapping:
 *   BTN = P0.02 / AIN0 -> push-button sense line, read by SAADC voltage
 *   PWR = P0.03        -> firmware power-hold/latch output
 *
 * Verified with minimal ADC test FW:
 *   Button released: BTN ~= 0 mV
 *   Button pressed : BTN ~= 1.42-1.46 V battery / 2.2-2.3 V USB-C
 *
 * This main FW now uses ADC threshold/hysteresis instead of raw digital GPIO:
 *   pressed  = BTN voltage >= 800 mV
 *   released = BTN voltage <= 300 mV
 *
 * Behavior:
 *   Hold button 3 sec -> PWR latch stays HIGH -> full FW starts
 *   Hold button again 3 sec, then release -> PWR is forced LOW -> board should turn OFF
 *   Hold button 10 sec, release, then short-press once/twice -> MCU cold reset
 *
 * A BLE command "OFF"/"POWER_OFF"/"SHUTDOWN" is also supported for debugging.
 */
#define POWER_ON_HOLD_TIME_MS              3000
#define POWER_OFF_HOLD_TIME_MS             3000
#define POWER_RESET_HOLD_TIME_MS           10000
#define POWER_RESET_CLICK_WINDOW_MS         2500
#define POWER_RESET_INTER_CLICK_MS           800
#define POWER_RESET_CLICK_MAX_MS             800
#define POWER_RESET_CLICK_MIN_MS              50
#define POWER_SAMPLE_MS                    50
#define POWER_BOOT_SETTLE_MS               50
#define POWER_OFF_GRACE_MS                 250
#define POWER_OFF_WAIT_RELEASE_TIMEOUT_MS  3000

#define BTN_PRESS_THRESHOLD_MV             800
#define BTN_RELEASE_THRESHOLD_MV           300

#define ADC_RESOLUTION                     12
#define ADC_GAIN_USED                      ADC_GAIN_1_6
#define ADC_REFERENCE_USED                 ADC_REF_INTERNAL
#define ADC_REF_INTERNAL_MV                600
#define ADC_GAIN_DENOMINATOR               6
#define ADC_FULL_SCALE_MV                  (ADC_REF_INTERNAL_MV * ADC_GAIN_DENOMINATOR)
#define ADC_CHANNEL_ID                     0

#ifndef NRF_SAADC_INPUT_AIN0
#define NRF_SAADC_INPUT_AIN0 SAADC_CH_PSELP_PSELP_AnalogInput0
#endif

static const struct gpio_dt_spec pwr_latch = GPIO_DT_SPEC_GET(DT_NODELABEL(pwr_latch), gpios);
static const struct device *const adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));

static const struct adc_channel_cfg btn_adc_cfg = {
    .gain = ADC_GAIN_USED,
    .reference = ADC_REFERENCE_USED,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = ADC_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
    .input_positive = NRF_SAADC_INPUT_AIN0,
#endif
};

static volatile bool g_shutdown_requested_by_ble;

struct button_reading {
    int mv;
    bool pressed;
};

static bool g_button_state_pressed;
static bool g_button_state_valid;

static void pwr_drive_high(void)
{
    if (gpio_is_ready_dt(&pwr_latch)) {
        (void)gpio_pin_configure_dt(&pwr_latch, GPIO_OUTPUT_ACTIVE);
        (void)gpio_pin_set_dt(&pwr_latch, 1);
    }
}

static void pwr_drive_low(void)
{
    if (gpio_is_ready_dt(&pwr_latch)) {
        (void)gpio_pin_configure_dt(&pwr_latch, GPIO_OUTPUT_INACTIVE);
        (void)gpio_pin_set_dt(&pwr_latch, 0);
    }
}

static void power_latch_force_low_forever(void)
{
    /* If battery-only hardware latch is correct, the board will switch off.
     * If USB-C/external power is present or Q1 is bypassed/shorted, stay here
     * with PWR forced LOW so the application does not continue running.
     */
    while (1) {
        pwr_drive_low();
        k_msleep(10);
    }
}

static void power_latch_off_forever(void)
{
    pwr_drive_low();
    power_latch_force_low_forever();
}

static void send_ble_line(const char *line)
{
    if (line) {
        (void)ble_log_send_as((const uint8_t *)line, strlen(line));
    }
}

static int btn_adc_init(void)
{
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device is not ready");
        return -ENODEV;
    }

    return adc_channel_setup(adc_dev, &btn_adc_cfg);
}

static int btn_read_mv(void)
{
    int16_t sample = 0;
    struct adc_sequence seq = {
        .channels = BIT(ADC_CHANNEL_ID),
        .buffer = &sample,
        .buffer_size = sizeof(sample),
        .resolution = ADC_RESOLUTION,
    };

    int err = adc_read(adc_dev, &seq);
    if (err) {
        return err;
    }

    /* Manual conversion for nRF SAADC with internal 0.6 V reference and gain 1/6:
     * full scale input range = 0.6 V * 6 = 3.6 V.
     * 12-bit ADC raw range = 0..4095.
     */
    int32_t mv = ((int32_t)sample * ADC_FULL_SCALE_MV) / ((1 << ADC_RESOLUTION) - 1);

    if (mv < 0) {
        mv = 0;
    }

    return (int)mv;
}

static struct button_reading button_read_adc(void)
{
    struct button_reading b = {
        .mv = btn_read_mv(),
        .pressed = false,
    };

    if (b.mv < 0) {
        /* Keep previous state on a single ADC read error so one bad sample does
         * not randomly cancel a long press or trigger release.
         */
        b.pressed = g_button_state_pressed;
        return b;
    }

    if (!g_button_state_valid) {
        g_button_state_pressed = (b.mv >= BTN_PRESS_THRESHOLD_MV);
        g_button_state_valid = true;
    } else if (!g_button_state_pressed && b.mv >= BTN_PRESS_THRESHOLD_MV) {
        g_button_state_pressed = true;
    } else if (g_button_state_pressed && b.mv <= BTN_RELEASE_THRESHOLD_MV) {
        g_button_state_pressed = false;
    }

    b.pressed = g_button_state_pressed;
    return b;
}

static void wait_for_button_release_after_off(void)
{
    int waited_ms = 0;

    LOG_INF("OFF accepted: PWR is LOW. Waiting for physical button release.");
    send_ble_line("POWER:OFF:PWR_LOW_WAIT_RELEASE\r\n");

    while (1) {
        pwr_drive_low();

        struct button_reading b = button_read_adc();
        if (b.mv < 0) {
            LOG_WRN("BTN ADC read failed while waiting release (%d)", b.mv);
        } else if (!b.pressed) {
            LOG_INF("Button released after OFF request; keeping PWR LOW now.");
            send_ble_line("POWER:OFF:BUTTON_RELEASED_KEEPING_PWR_LOW\r\n");
            break;
        }

        if ((waited_ms % 1000) == 0) {
            char msg[112];
            LOG_INF("Waiting release after OFF: waited_ms=%d BTN=%dmV pressed=%d",
                    waited_ms, b.mv, b.pressed ? 1 : 0);
            snprintk(msg, sizeof(msg),
                     "POWER:OFF:WAIT_RELEASE:%dms BTN=%dmV\r\n",
                     waited_ms, b.mv);
            send_ble_line(msg);
        }

        /* Do not wait forever. If the user holds slightly longer or the battery
         * latch keeps BTN high, still force PWR LOW and stop the application.
         */
        if (waited_ms >= POWER_OFF_WAIT_RELEASE_TIMEOUT_MS) {
            char msg[128];
            LOG_WRN("Release wait timeout. Forcing PWR LOW forever. BTN=%dmV", b.mv);
            snprintk(msg, sizeof(msg),
                     "POWER:OFF:RELEASE_TIMEOUT_FORCE_LOW BTN=%dmV\r\n",
                     b.mv);
            send_ble_line(msg);
            break;
        }

        k_msleep(POWER_SAMPLE_MS);
        waited_ms += POWER_SAMPLE_MS;
    }

    k_msleep(100);
}

static void power_latch_shutdown_now(const char *reason)
{
    char msg[96];

    LOG_INF("Power-off requested: %s", reason ? reason : "unknown");

    if (reason && strcmp(reason, "BLE") == 0) {
        send_ble_line("POWER:OFF:BLE_COMMAND_ACCEPTED\r\n");
    } else {
        snprintk(msg, sizeof(msg), "POWER:OFF:ACCEPTED:%s\r\n",
                 reason ? reason : "BUTTON_ADC_3SEC");
        send_ble_line(msg);
    }

    /* Last optional memory-status snapshot before dropping the latch. */
#if !(PPG_VALIDATION_MODE && PPG_VAL_DISABLE_NAND)
    w25n01_log_mem_snapshot();
#endif
    k_msleep(POWER_OFF_GRACE_MS);

    /* Critical: drive PWR LOW before waiting for release. While the button is
     * still physically held, the button path may still keep Q2/Q1 alive. When
     * the user releases the button, PWR is already LOW, so the latch should open.
     */
    pwr_drive_low();
    wait_for_button_release_after_off();
    power_latch_force_low_forever();
}

static int power_latch_startup_check(void)
{
    if (!gpio_is_ready_dt(&pwr_latch)) {
        LOG_ERR("PWR latch GPIO device is not ready");
        return -ENODEV;
    }

    /* Configure ADC after immediately holding the latch alive. */
    pwr_drive_high();
    k_msleep(POWER_BOOT_SETTLE_MS);

    int err = btn_adc_init();
    if (err) {
        LOG_ERR("Failed to initialize BTN ADC (%d)", err);
        return err;
    }

    LOG_INF("Power latch HIGH. Require BTN >= %dmV for %d ms to start.",
            BTN_PRESS_THRESHOLD_MV, POWER_ON_HOLD_TIME_MS);

    int elapsed_ms = 0;
    while (elapsed_ms < POWER_ON_HOLD_TIME_MS) {
        pwr_drive_high();

        struct button_reading b = button_read_adc();
        if (b.mv < 0) {
            LOG_ERR("Failed to read BTN ADC during startup (%d)", b.mv);
            return b.mv;
        }

        if (!b.pressed) {
            LOG_WRN("Startup button released early. elapsed_ms=%d BTN=%dmV. Powering off.",
                    elapsed_ms, b.mv);
            power_latch_off_forever();
        }

        k_msleep(POWER_SAMPLE_MS);
        elapsed_ms += POWER_SAMPLE_MS;
    }

    pwr_drive_high();
    LOG_INF("3-sec ON accepted. PWR latch remains HIGH. Starting full firmware.");
    return 0;
}

static char ascii_upper_char(char c)
{
    if (c >= 'a' && c <= 'z') {
        return (char)(c - 'a' + 'A');
    }
    return c;
}

static bool cmd_equals_ci(const char *a, const char *b)
{
    if (!a || !b) {
        return false;
    }

    while (*a && *b) {
        if (ascii_upper_char(*a) != ascii_upper_char(*b)) {
            return false;
        }
        a++;
        b++;
    }

    return (*a == '\0' && *b == '\0');
}

static void app_ble_command_cb(const char *cmd)
{
    if (!cmd) {
        return;
    }

    if (cmd_equals_ci(cmd, "OFF") ||
        cmd_equals_ci(cmd, "POWER_OFF") ||
        cmd_equals_ci(cmd, "SHUTDOWN")) {
        g_shutdown_requested_by_ble = true;
        send_ble_line("CMD:OFF:ACCEPTED\r\n");
        return;
    }

    if (cmd_equals_ci(cmd, "PING") || cmd_equals_ci(cmd, "STATUS")) {
        struct button_reading b = button_read_adc();
        char msg[128];
        snprintk(msg, sizeof(msg),
                 "POWER:STATUS BTN=%dmV STATE=%s TH_PRESS=%dmV TH_RELEASE=%dmV\r\n",
                 b.mv, b.pressed ? "PRESSED" : "RELEASED",
                 BTN_PRESS_THRESHOLD_MV, BTN_RELEASE_THRESHOLD_MV);
        send_ble_line(msg);
        return;
    }

    /* Keep existing MAX30101 GUI commands working. */
    max30101_handle_ble_command(cmd);
}


static void reboot_device_now(const char *reason)
{
    char msg[96];

    snprintk(msg, sizeof(msg), "POWER:RESET:REBOOT_NOW:%s\r\n",
             reason ? reason : "BUTTON_10SEC_CLICK");
    send_ble_line(msg);
    LOG_WRN("Rebooting device now: %s", reason ? reason : "BUTTON_10SEC_CLICK");

    k_msleep(300);
    sys_reboot(SYS_REBOOT_COLD);
}

static bool wait_for_reset_confirmation_after_long_hold(void)
{
    int waited_ms = 0;
    int clicks = 0;
    int first_click_wait_ms = 0;
    bool prev_pressed = true;
    bool click_press_active = false;
    int click_press_ms = 0;

    send_ble_line("POWER:RESET:LONG_HOLD_10SEC_DETECTED_RELEASE_BUTTON\r\n");
    LOG_WRN("10-sec reset hold detected. Waiting for release, then one/two short clicks.");

    /* First wait until the long hold is released. */
    while (1) {
        pwr_drive_high();

        struct button_reading b = button_read_adc();
        if (b.mv >= 0 && !b.pressed) {
            send_ble_line("POWER:RESET:LONG_HOLD_RELEASED_WAIT_CLICK\r\n");
            break;
        }

        if ((waited_ms % 1000) == 0) {
            char msg[96];
            snprintk(msg, sizeof(msg),
                     "POWER:RESET:WAIT_RELEASE:%dms BTN=%dmV\r\n",
                     waited_ms, b.mv);
            send_ble_line(msg);
        }

        k_msleep(POWER_SAMPLE_MS);
        waited_ms += POWER_SAMPLE_MS;
    }

    waited_ms = 0;
    prev_pressed = false;

    /* Now wait for one or two short confirmation presses. One valid click is
     * enough; a second click within POWER_RESET_INTER_CLICK_MS is also accepted.
     */
    while (waited_ms <= POWER_RESET_CLICK_WINDOW_MS) {
        pwr_drive_high();

        struct button_reading b = button_read_adc();
        if (b.mv < 0) {
            k_msleep(POWER_SAMPLE_MS);
            waited_ms += POWER_SAMPLE_MS;
            continue;
        }

        if (b.pressed && !prev_pressed) {
            click_press_active = true;
            click_press_ms = 0;
            send_ble_line("POWER:RESET:CONFIRM_PRESS_STARTED\r\n");
        }

        if (b.pressed && click_press_active) {
            click_press_ms += POWER_SAMPLE_MS;
            if (click_press_ms > POWER_RESET_CLICK_MAX_MS) {
                send_ble_line("POWER:RESET:CANCELLED_CONFIRM_PRESS_TOO_LONG\r\n");
                return false;
            }
        }

        if (!b.pressed && prev_pressed && click_press_active) {
            if (click_press_ms >= POWER_RESET_CLICK_MIN_MS &&
                click_press_ms <= POWER_RESET_CLICK_MAX_MS) {
                clicks++;
                char msg[96];
                snprintk(msg, sizeof(msg),
                         "POWER:RESET:CONFIRM_CLICK_%d\r\n", clicks);
                send_ble_line(msg);

                if (clicks >= 2) {
                    reboot_device_now("BUTTON_10SEC_DOUBLE_CLICK");
                }

                first_click_wait_ms = 0;
            }

            click_press_active = false;
            click_press_ms = 0;
        }

        if (clicks == 1) {
            first_click_wait_ms += POWER_SAMPLE_MS;
            if (first_click_wait_ms >= POWER_RESET_INTER_CLICK_MS) {
                reboot_device_now("BUTTON_10SEC_SINGLE_CLICK");
            }
        }

        prev_pressed = b.pressed;
        k_msleep(POWER_SAMPLE_MS);
        waited_ms += POWER_SAMPLE_MS;
    }

    send_ble_line("POWER:RESET:CANCELLED_NO_CONFIRM_CLICK\r\n");
    LOG_INF("10-sec reset gesture cancelled: no confirmation click.");
    return false;
}

static void power_latch_runtime_monitor_loop(void)
{
    bool off_armed = false;
    int held_ms = 0;
    int last_reported_sec = -1;
    int status_ms = 0;

    LOG_INF("Runtime ADC button monitor started. Waiting for release to arm OFF long press.");
    send_ble_line("POWER:INFO MAIN_FW_ADC_BTN_PRESS_800mV_RELEASE_300mV\r\n");

    while (1) {
        pwr_drive_high();

        if (g_shutdown_requested_by_ble) {
            g_shutdown_requested_by_ble = false;
            power_latch_shutdown_now("BLE");
        }

        struct button_reading b = button_read_adc();
        if (b.mv < 0) {
            LOG_WRN("Failed to read BTN ADC in runtime monitor (%d)", b.mv);
            k_msleep(POWER_SAMPLE_MS);
            continue;
        }

        if (status_ms >= 1000) {
#if !(PPG_VALIDATION_MODE && PPG_VAL_DISABLE_DEBUG_LOGS)
            char msg[128];
            snprintk(msg, sizeof(msg),
                     "POWER:STATUS BTN=%dmV STATE=%s OFF_ARMED=%d HOLD=%dms\r\n",
                     b.mv, b.pressed ? "PRESSED" : "RELEASED",
                     off_armed ? 1 : 0, held_ms);
            send_ble_line(msg);
#endif
            status_ms = 0;
        }

        /* After boot, require one release before accepting an OFF long press. */
        if (!off_armed) {
            if (!b.pressed) {
                off_armed = true;
                held_ms = 0;
                last_reported_sec = -1;
                LOG_INF("Startup button released. OFF long press is now armed.");
                send_ble_line("POWER:OFF_ARMED\r\n");
            }
            k_msleep(POWER_SAMPLE_MS);
            status_ms += POWER_SAMPLE_MS;
            continue;
        }

        if (b.pressed) {
            held_ms += POWER_SAMPLE_MS;
            int sec = held_ms / 1000;

            if (held_ms == POWER_SAMPLE_MS) {
                LOG_INF("Button hold started. Release after 3 sec for OFF, or hold 10 sec for RESET gesture.");
                send_ble_line("POWER:BUTTON_HOLD_STARTED\r\n");
            }

            if (sec != last_reported_sec && sec > 0) {
                char msg[112];
                last_reported_sec = sec;
                snprintk(msg, sizeof(msg),
                         "POWER:BUTTON_HOLD:%dms BTN=%dmV\r\n",
                         held_ms, b.mv);
                send_ble_line(msg);

                if (held_ms >= POWER_OFF_HOLD_TIME_MS &&
                    held_ms < POWER_RESET_HOLD_TIME_MS) {
                    send_ble_line("POWER:OFF_READY_RELEASE_TO_POWER_OFF_OR_KEEP_HOLDING_FOR_RESET\r\n");
                }
            }

            if (held_ms >= POWER_RESET_HOLD_TIME_MS) {
                bool reset_started = wait_for_reset_confirmation_after_long_hold();
                ARG_UNUSED(reset_started);
                held_ms = 0;
                last_reported_sec = -1;
                status_ms = 1000;
            }
        } else {
            if (held_ms >= POWER_OFF_HOLD_TIME_MS && held_ms < POWER_RESET_HOLD_TIME_MS) {
                power_latch_shutdown_now("BUTTON_ADC_3SEC_RELEASE");
            } else if (held_ms > 0) {
                LOG_INF("Button hold cancelled at %d ms.", held_ms);
                send_ble_line("POWER:BUTTON_HOLD_CANCELLED\r\n");
            }
            held_ms = 0;
            last_reported_sec = -1;
        }

        k_msleep(POWER_SAMPLE_MS);
        status_ms += POWER_SAMPLE_MS;
    }
}

int main(void)
{
    int err = power_latch_startup_check();
    if (err) {
        LOG_ERR("power_latch_startup_check failed (%d). Powering off.", err);
        power_latch_off_forever();
    }

    err = ble_log_service_init();
    if (err) {
        LOG_ERR("ble_log_service_init failed (%d)", err);
    }

    ble_log_service_set_command_callback(app_ble_command_cb);

    k_msleep(500);

#if PPG_VALIDATION_MODE
    /* Focused PPG validation path: keep BLE, MAX30101 green PPG, IMU, and algo.
     * Skip NAND/EDA/TEMP task startup to keep the BLE stream clean and reduce
     * extra load while validating the PPG waveform and peak detection.
     */
    lsm6dso_task_start();
    max30101_task_start();

    LOG_INF("PPG validation mode: MAX30101 GREEN + LSM6DSO IMU + Algo V0 only.");
#else
    w25n01_task_start();
    as6221_task_start();
    lsm6dso_task_start();
    max30101_task_start();
    ads1113_task_start();

    algo_v0_set_minute_callback(w25n01_log_processed_line);

    LOG_INF("Storage + sensor tasks started, waiting before Algo V0 start...");
#endif

    for (int i = 0; i < 80; i++) {
        if (lsm6dso_task_has_data()) {
            LOG_INF("LSM6DSO is producing samples; starting Algo V0.");
            break;
        }
        k_msleep(100);
    }

    if (!lsm6dso_task_has_data()) {
        LOG_WRN("LSM6DSO not ready after warm-up; starting Algo V0 anyway.");
    }

    algo_v0_init();
    algo_v0_start();

    LOG_INF("Algo V0 started after sensor warm-up.");

    power_latch_runtime_monitor_loop();

    return 0;
}
