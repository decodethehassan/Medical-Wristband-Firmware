#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "algo_v0.h"
#include "ppg_validation.h"

extern void w25n01_log_raw_imu(int32_t ax_mg, int32_t ay_mg, int32_t az_mg,
                        int32_t gx_mdps, int32_t gy_mdps, int32_t gz_mdps,
                        int64_t t_ms);

LOG_MODULE_REGISTER(lsm6dso_app, LOG_LEVEL_INF);

/* ========= LSM6DSO pins for Smartwatch PCB v2.0.1 =========
 * I2C mode wiring from schematic:
 *   CS     = P0.22  -> drive HIGH to select I2C mode
 *   SDO/SA0= P0.13  -> drive HIGH to set I2C address 0x6B
 *   INT1   = P0.28
 *   INT2   = P0.05
 * I2C1 bus pins are configured in the overlay:
 *   SCL    = P0.23
 *   SDA    = P0.24
 */
#define GPIO0_NODE DT_NODELABEL(gpio0)

#define LSM6DSO_SA0_PIN   13
#define LSM6DSO_CS_PIN    22
#define LSM6DSO_INT2_PIN  5
#define LSM6DSO_INT1_PIN  28

/* ========= LSM6DSO Registers ========= */
#define REG_WHO_AM_I          0x0F
#define WHO_AM_I_VAL          0x6C

#define REG_CTRL1_XL          0x10
#define REG_CTRL2_G           0x11
#define REG_CTRL3_C           0x12
#define REG_OUTX_L_G          0x22

#define CTRL1_XL_104HZ_2G     0x40
#define CTRL2_G_104HZ_250DPS  0x40
#define CTRL3_C_BDU_IFINC     0x44

#define ACC_MG_PER_LSB_NUM      61
#define ACC_MG_PER_LSB_DEN      1000

#define GYRO_MDPS_PER_LSB_NUM   875
#define GYRO_MDPS_PER_LSB_DEN   100

/*
 * LSM6DSO does not provide native 32 Hz ODR.
 * Keep sensor at 104 Hz and publish/log exact effective 32 Hz in firmware.
 */
#define IMU_TARGET_FS_HZ        32U
#define IMU_LOG_EVERY_N         32U
#define INIT_RETRY_MS           2000
#define READ_FAIL_LIMIT         10

static volatile bool g_lsm6_has_data;

static const struct device *i2c1;
static const struct device *gpio0;

/* ========= I2C helpers ========= */
static int reg_read_u8(uint8_t addr, uint8_t reg, uint8_t *val)
{
    return i2c_reg_read_byte(i2c1, addr, reg, val);
}

static int reg_write_u8(uint8_t addr, uint8_t reg, uint8_t val)
{
    return i2c_reg_write_byte(i2c1, addr, reg, val);
}

static int burst_read(uint8_t addr, uint8_t start_reg, uint8_t *buf, size_t len)
{
    return i2c_burst_read(i2c1, addr, start_reg, buf, len);
}

static inline int16_t le16(const uint8_t *p)
{
    return (int16_t)((p[1] << 8) | p[0]);
}

static int32_t accel_raw_to_mg(int16_t raw)
{
    return ((int32_t)raw * ACC_MG_PER_LSB_NUM) / ACC_MG_PER_LSB_DEN;
}

static int32_t gyro_raw_to_mdps(int16_t raw)
{
    return ((int32_t)raw * GYRO_MDPS_PER_LSB_NUM) / GYRO_MDPS_PER_LSB_DEN;
}

static int detect_addr(uint8_t *found)
{
    uint8_t who = 0;

    if (reg_read_u8(0x6A, REG_WHO_AM_I, &who) == 0 && who == WHO_AM_I_VAL) {
        *found = 0x6A;
        LOG_INF("WHO_AM_I @0x6A = 0x%02X (OK)", who);
        return 0;
    }

    if (reg_read_u8(0x6B, REG_WHO_AM_I, &who) == 0 && who == WHO_AM_I_VAL) {
        *found = 0x6B;
        LOG_INF("WHO_AM_I @0x6B = 0x%02X (OK)", who);
        return 0;
    }

    return -EIO;
}

static int lsm6dso_configure(uint8_t addr)
{
    int ret;

    ret = reg_write_u8(addr, REG_CTRL3_C, CTRL3_C_BDU_IFINC);
    if (ret) {
        LOG_ERR("CTRL3_C write failed (%d)", ret);
        return ret;
    }

    ret = reg_write_u8(addr, REG_CTRL1_XL, CTRL1_XL_104HZ_2G);
    if (ret) {
        LOG_ERR("CTRL1_XL write failed (%d)", ret);
        return ret;
    }

    ret = reg_write_u8(addr, REG_CTRL2_G, CTRL2_G_104HZ_250DPS);
    if (ret) {
        LOG_ERR("CTRL2_G write failed (%d)", ret);
        return ret;
    }

    return 0;
}

static void advance_32hz_time(int64_t *t_ms, uint32_t *frac_us)
{
    /*
     * 1000 / 32 = 31.25 ms
     * Use integer ms + fractional accumulator:
     * +31 ms, remainder +250 us each sample.
     */
    *t_ms += 31;
    *frac_us += 250U;
    if (*frac_us >= 1000U) {
        *t_ms += 1;
        *frac_us -= 1000U;
    }
}

static void lsm6dso_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint8_t addr = 0;
    uint32_t sample_count = 0U;
    int read_fail_streak = 0;

    LOG_INF("=== LSM6DSO FULL I2C ACC+GYRO TEST ===");

    gpio0 = DEVICE_DT_GET(GPIO0_NODE);
    i2c1  = DEVICE_DT_GET(DT_NODELABEL(i2c1));

    while (1) {
        if (!device_is_ready(gpio0)) {
            g_lsm6_has_data = false;
            LOG_ERR("GPIO0 not ready, retrying...");
            k_sleep(K_MSEC(INIT_RETRY_MS));
            continue;
        }

        if (!device_is_ready(i2c1)) {
            g_lsm6_has_data = false;
            LOG_ERR("I2C1 not ready, retrying...");
            k_sleep(K_MSEC(INIT_RETRY_MS));
            continue;
        }

        if (gpio_pin_configure(gpio0, LSM6DSO_CS_PIN, GPIO_OUTPUT_HIGH)) {
            g_lsm6_has_data = false;
            LOG_ERR("CS pin config failed, retrying...");
            k_sleep(K_MSEC(INIT_RETRY_MS));
            continue;
        }

        if (gpio_pin_configure(gpio0, LSM6DSO_SA0_PIN, GPIO_OUTPUT_HIGH)) {
            g_lsm6_has_data = false;
            LOG_ERR("SA0 pin config failed, retrying...");
            k_sleep(K_MSEC(INIT_RETRY_MS));
            continue;
        }

        (void)gpio_pin_configure(gpio0, LSM6DSO_INT1_PIN, GPIO_INPUT);
        (void)gpio_pin_configure(gpio0, LSM6DSO_INT2_PIN, GPIO_INPUT);

        k_msleep(20);

        if (detect_addr(&addr) != 0) {
            g_lsm6_has_data = false;
            LOG_ERR("LSM6DSO not detected at 0x6A/0x6B, retrying...");
            k_sleep(K_MSEC(INIT_RETRY_MS));
            continue;
        }

        if (lsm6dso_configure(addr) != 0) {
            g_lsm6_has_data = false;
            LOG_ERR("LSM6DSO config failed, retrying...");
            k_sleep(K_MSEC(INIT_RETRY_MS));
            continue;
        }

        LOG_INF("Configured: addr=0x%02X XL=104Hz(2g), G=104Hz(250dps), effective output/logging=32Hz",
                addr);

        read_fail_streak = 0;

        int64_t next_pub_t_ms = k_uptime_get();
        uint32_t next_pub_t_frac_us = 0U;

        while (1) {
            int64_t now_ms = k_uptime_get();
            if (now_ms < next_pub_t_ms) {
                k_sleep(K_MSEC((int32_t)(next_pub_t_ms - now_ms)));
            }

            uint8_t buf[12];
            int ret = burst_read(addr, REG_OUTX_L_G, buf, sizeof(buf));
            if (ret) {
                read_fail_streak++;
                LOG_ERR("Burst read failed (%d), streak=%d", ret, read_fail_streak);
                if (read_fail_streak >= READ_FAIL_LIMIT) {
                    g_lsm6_has_data = false;
                    LOG_ERR("Too many IMU read failures, reinitializing sensor...");
                    break;
                }
                k_sleep(K_MSEC(100));
                continue;
            }

            read_fail_streak = 0;

            int64_t sample_t_ms = next_pub_t_ms;

            int16_t gx = le16(&buf[0]);
            int16_t gy = le16(&buf[2]);
            int16_t gz = le16(&buf[4]);

            int16_t ax = le16(&buf[6]);
            int16_t ay = le16(&buf[8]);
            int16_t az = le16(&buf[10]);

            int32_t gx_mdps = gyro_raw_to_mdps(gx);
            int32_t gy_mdps = gyro_raw_to_mdps(gy);
            int32_t gz_mdps = gyro_raw_to_mdps(gz);

            int32_t ax_mg = accel_raw_to_mg(ax);
            int32_t ay_mg = accel_raw_to_mg(ay);
            int32_t az_mg = accel_raw_to_mg(az);

            float ax_g = (float)ax_mg / 1000.0f;
            float ay_g = (float)ay_mg / 1000.0f;
            float az_g = (float)az_mg / 1000.0f;

            float gx_dps = (float)gx_mdps / 1000.0f;
            float gy_dps = (float)gy_mdps / 1000.0f;
            float gz_dps = (float)gz_mdps / 1000.0f;

            algo_v0_push_imu(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, sample_t_ms);
#if !(PPG_VALIDATION_MODE && PPG_VAL_DISABLE_NAND)
            w25n01_log_raw_imu(ax_mg, ay_mg, az_mg, gx_mdps, gy_mdps, gz_mdps, sample_t_ms);
#endif
            g_lsm6_has_data = true;

#if !(PPG_VALIDATION_MODE && PPG_VAL_DISABLE_DEBUG_LOGS)
            if ((sample_count % IMU_LOG_EVERY_N) == 0U) {
                LOG_INF("[LSM6DSO] A[g]=[%.3f %.3f %.3f] G[dps]=[%.2f %.2f %.2f] t=%lld",
                    (double)ax_g, (double)ay_g, (double)az_g,
                    (double)gx_dps, (double)gy_dps, (double)gz_dps,
                    sample_t_ms);
            }
#endif
            sample_count++;

            advance_32hz_time(&next_pub_t_ms, &next_pub_t_frac_us);

            int64_t real_now_ms = k_uptime_get();
            if (next_pub_t_ms < real_now_ms) {
                next_pub_t_ms = real_now_ms;
                next_pub_t_frac_us = 0U;
            }
        }

        k_sleep(K_MSEC(INIT_RETRY_MS));
    }
}

/* thread objects */
#define LSM6DSO_STACK_SIZE 3072
#define LSM6DSO_PRIORITY   5

K_THREAD_STACK_DEFINE(lsm6dso_stack, LSM6DSO_STACK_SIZE);
static struct k_thread lsm6dso_tcb;
static bool started;

void lsm6dso_task_start(void)
{
    if (started) {
        return;
    }
    started = true;

    k_thread_create(&lsm6dso_tcb, lsm6dso_stack, K_THREAD_STACK_SIZEOF(lsm6dso_stack),
            lsm6dso_thread, NULL, NULL, NULL,
            LSM6DSO_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&lsm6dso_tcb, "lsm6dso_task");
}

bool lsm6dso_task_has_data(void)
{
    return g_lsm6_has_data;
}