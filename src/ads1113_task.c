#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <math.h>

#include "algo_v0.h"
#include "w25n01_task.h"

LOG_MODULE_REGISTER(eda_raw, LOG_LEVEL_INF);

/* Use software I2C bus on P0.07/P0.08 */
#define I2C_NODE            DT_NODELABEL(i2c_eda)
#define ADS1113_ADDR        0x49

#define REG_CONV            0x00
#define REG_CONFIG          0x01

#define FS_HZ               4
#define SAMPLE_MS           (1000 / FS_HZ)

/* Adjust these to match your actual analog front end.
 * The Algo V0 core expects EDA in µS, so we convert here before handing data off.
 */
#define EDA_ZERO_OFFSET_MV  0.0f
#define EDA_US_PER_MV       0.02f
#define EDA_USE_ABS_INPUT   1

#define FLAT_DELTA_RAW_TH   1
#define FLAT_TIME_SEC       5
#define FLAT_N_SAMPLES      (FS_HZ * FLAT_TIME_SEC)

static int ads_set_continuous(const struct device *i2c)
{
    uint8_t cfg[3] = { REG_CONFIG, 0xC2, 0x83 };
    return i2c_write(i2c, cfg, sizeof(cfg), ADS1113_ADDR);
}

static int ads_read_raw(const struct device *i2c, int16_t *raw)
{
    uint8_t reg = REG_CONV;
    uint8_t buf[2];

    int ret = i2c_write_read(i2c, ADS1113_ADDR, &reg, 1, buf, 2);
    if (ret) {
        return ret;
    }

    *raw = (int16_t)((buf[0] << 8) | buf[1]);
    return 0;
}

static int32_t raw_to_mV(int16_t raw)
{
    int32_t uv = (int32_t)raw * 125;
    return uv / 1000;
}

static float mv_to_algo_uS(float mv)
{
    float x = mv - EDA_ZERO_OFFSET_MV;
#if EDA_USE_ABS_INPUT
    x = fabsf(x);
#endif
    return x * EDA_US_PER_MV;
}

static void ads1113_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    const struct device *i2c = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c)) {
        LOG_ERR("I2C not ready");
        return;
    }

    LOG_INF("=== EDA RAW STREAM TEST (ADS1113, %d Hz) ===", FS_HZ);
    LOG_INF("EDA calibration: offset=%.3f mV scale=%.3f uS/mV abs=%d",
        (double)EDA_ZERO_OFFSET_MV, (double)EDA_US_PER_MV, EDA_USE_ABS_INPUT);

    algo_v0_set_eda_mv_to_uS(1.0f);

    int ret = ads_set_continuous(i2c);
    if (ret) {
        LOG_ERR("ADS config write failed (%d)", ret);
        return;
    }
    LOG_INF("ADS set to continuous mode");

    int16_t prev_raw = 0;
    bool have_prev = false;
    int flat_cnt = 0;

    int64_t t0 = k_uptime_get();

    while (1) {
        int16_t raw = 0;
        ret = ads_read_raw(i2c, &raw);
        if (ret) {
            LOG_ERR("ADS read failed (%d)", ret);
            k_sleep(K_MSEC(SAMPLE_MS));
            continue;
        }

        int64_t now_ms = k_uptime_get();
        float mv = (float)raw_to_mV(raw);
        float uS = mv_to_algo_uS(mv);

        algo_v0_push_eda_mv(uS, now_ms);
        w25n01_log_raw_eda(raw, (int32_t)mv, now_ms);

        int16_t d = 0;
        if (have_prev) {
            d = raw - prev_raw;
        } else {
            have_prev = true;
        }

        if (have_prev && (d <= FLAT_DELTA_RAW_TH && d >= -FLAT_DELTA_RAW_TH)) {
            flat_cnt++;
        } else {
            flat_cnt = 0;
        }

        int64_t t_ms = now_ms - t0;

        LOG_INF("t=%lldms raw=%d mv=%.3f uS=%.3f dRaw=%d flat_cnt=%d%s",
            t_ms,
            raw,
            (double)mv,
            (double)uS,
            d,
            flat_cnt,
            (flat_cnt >= FLAT_N_SAMPLES) ? " FLATLINE" : "");

        prev_raw = raw;
        k_sleep(K_MSEC(SAMPLE_MS));
    }
}

#define ADS1113_STACK_SIZE 2048
#define ADS1113_PRIORITY   5

K_THREAD_STACK_DEFINE(ads1113_stack, ADS1113_STACK_SIZE);
static struct k_thread ads1113_tcb;
static bool started;

void ads1113_task_start(void)
{
    if (started) {
        return;
    }
    started = true;

    k_thread_create(&ads1113_tcb, ads1113_stack, K_THREAD_STACK_SIZEOF(ads1113_stack),
            ads1113_thread, NULL, NULL, NULL,
            ADS1113_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&ads1113_tcb, "ads1113_task");
}
