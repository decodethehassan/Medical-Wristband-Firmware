#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================
 *  Algo V0 public API
 *  - Call algo_v0_init() once (main)
 *  - Call algo_v0_start() once (main)
 *  - In each sensor task, push new samples using the functions below
 * ========================= */

void algo_v0_init(void);
void algo_v0_start(void);

/* --- Temperature (AS6221) --- */
void algo_v0_push_temp_c(float temp_c, int64_t t_ms);

/* --- IMU (LSM6DSO) --- */
/* Units expected:
 *   accel in g (not mg), gyro in dps (not mdps)
 * If your task outputs mg/mdps, convert before pushing.
 */
void algo_v0_push_imu(float ax_g, float ay_g, float az_g,
                      float gx_dps, float gy_dps, float gz_dps,
                      int64_t t_ms);

/* --- PPG (MAX30101/2) --- */
void algo_v0_push_ppg(uint32_t red, uint32_t ir, uint32_t green, int64_t t_ms);

/* --- EDA (ADS1113) --- */
void algo_v0_push_eda_mv(float eda_mv, int64_t t_ms);

/* Optional: allow runtime calibration for EDA conversion (mV -> µS) */
void algo_v0_set_eda_mv_to_uS(float scale);

/* Optional: if you ever want to publish algo output via a callback */
typedef void (*algo_v0_minute_cb_t)(const char *line);
void algo_v0_set_minute_callback(algo_v0_minute_cb_t cb);

#ifdef __cplusplus
}
#endif