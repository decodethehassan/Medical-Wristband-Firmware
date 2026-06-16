#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define W25N01_PPG_CH_GREEN  (1u << 0)
#define W25N01_PPG_CH_IR     (1u << 1)
#define W25N01_PPG_CH_RED    (1u << 2)
#define W25N01_PPG_CH_ALL    (W25N01_PPG_CH_GREEN | W25N01_PPG_CH_IR | W25N01_PPG_CH_RED)

void w25n01_task_start(void);

void w25n01_set_worn(bool worn);
void w25n01_log_mem_snapshot(void);

/* Raw sensor logging now stores compact binary records in NAND.
 * During live BLE streaming and stored replay, records are decoded back to text
 * so the existing GUI/app can visualize them.
 */
void w25n01_log_raw_temp_centi(int32_t temp_c_x100, int64_t t_ms);
void w25n01_log_raw_imu(int32_t ax_mg, int32_t ay_mg, int32_t az_mg,
                        int32_t gx_mdps, int32_t gy_mdps, int32_t gz_mdps,
                        int64_t t_ms);
void w25n01_log_raw_ppg_mask(uint8_t channel_mask,
                             uint32_t red, uint32_t ir, uint32_t green,
                             int64_t t_ms);
void w25n01_log_raw_ppg(uint32_t red, uint32_t ir, uint32_t green, int64_t t_ms);
void w25n01_log_raw_eda(int16_t raw, int32_t mv, int64_t t_ms);
void w25n01_log_processed_line(const char *line);

#ifdef __cplusplus
}
#endif
