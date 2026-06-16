#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <stdbool.h>

#include "algo_v0.h"
#include "w25n01_task.h"
#include "as6221_task.h"

LOG_MODULE_REGISTER(as6221_demo, LOG_LEVEL_INF);

#define AS6221_ADDR         0x48
#define REG_TEMP_MSB        0x00

/* Algo V0 document expects 4 Hz temperature input */
#define TEMP_SAMPLE_MS      250
#define TEMP_LOG_EVERY_N    4

static const struct device *i2c_dev;

static K_MUTEX_DEFINE(g_temp_lock);
static float g_last_temp_c;
static int64_t g_last_temp_t_ms = -1;

static float as6221_read_temp(void)
{
    uint8_t data[2];
    int ret = i2c_burst_read(i2c_dev, AS6221_ADDR, REG_TEMP_MSB, data, 2);

    if (ret < 0) {
        LOG_ERR("I2C read failed (%d)", ret);
        return -1000.0f;
    }

    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
    float temperature = raw / 100.0f;

    return temperature;
}

static void as6221_publish_temp(float temp_c, int64_t t_ms)
{
    k_mutex_lock(&g_temp_lock, K_FOREVER);
    g_last_temp_c = temp_c;
    g_last_temp_t_ms = t_ms;
    k_mutex_unlock(&g_temp_lock);
}

bool as6221_get_latest_temp_c(float *temp_c_out, int64_t *age_ms_out)
{
    float temp_c;
    int64_t t_ms;
    int64_t now_ms = k_uptime_get();

    k_mutex_lock(&g_temp_lock, K_FOREVER);
    temp_c = g_last_temp_c;
    t_ms = g_last_temp_t_ms;
    k_mutex_unlock(&g_temp_lock);

    if (t_ms < 0) {
        return false;
    }

    if (temp_c_out) {
        *temp_c_out = temp_c;
    }
    if (age_ms_out) {
        *age_ms_out = now_ms - t_ms;
    }
    return true;
}

static void as6221_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    LOG_INF("=== AS6221 CUSTOM I2C DEMO START ===");

    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C0 not ready!");
        return;
    }

    LOG_INF("I2C0 ready, addr=0x48");

    uint32_t sample_count = 0;

    while (1) {
        int64_t now_ms = k_uptime_get();
        float temperature = as6221_read_temp();

        if (temperature > -999.0f) {
            as6221_publish_temp(temperature, now_ms);
            algo_v0_push_temp_c(temperature, now_ms);
            w25n01_log_raw_temp_centi((int32_t)(temperature * 100.0f), now_ms);

            if ((sample_count % TEMP_LOG_EVERY_N) == 0U) {
                LOG_INF("[AS6221] t=%.2f C | uptime=%lld ms",
                    (double)temperature, now_ms);
            }
            sample_count++;
        }

        k_msleep(TEMP_SAMPLE_MS);
    }
}

/* thread objects */
#define AS6221_STACK_SIZE 2048
#define AS6221_PRIORITY   5

K_THREAD_STACK_DEFINE(as6221_stack, AS6221_STACK_SIZE);
static struct k_thread as6221_tcb;
static bool started;

void as6221_task_start(void)
{
    if (started) {
        return;
    }
    started = true;

    k_thread_create(&as6221_tcb, as6221_stack, K_THREAD_STACK_SIZEOF(as6221_stack),
            as6221_thread, NULL, NULL, NULL,
            AS6221_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&as6221_tcb, "as6221_task");
}
