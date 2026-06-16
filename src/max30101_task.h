#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PPG channel mask used by MAX30101 task, NAND logger, and BLE/app commands.
 * Default firmware mode keeps GREEN enabled and RED/IR disabled.
 */
#define MAX30101_PPG_CH_GREEN  (1u << 0)
#define MAX30101_PPG_CH_IR     (1u << 1)
#define MAX30101_PPG_CH_RED    (1u << 2)
#define MAX30101_PPG_CH_ALL    (MAX30101_PPG_CH_GREEN | MAX30101_PPG_CH_IR | MAX30101_PPG_CH_RED)
#define MAX30101_PPG_CH_DEFAULT MAX30101_PPG_CH_GREEN

void max30101_task_start(void);

/* Runtime/app control. GREEN is kept on as the default/base channel.
 * Advanced mode can add IR and/or RED.
 */
int max30101_set_channel_mask(uint8_t mask);
uint8_t max30101_get_channel_mask(void);
void max30101_handle_ble_command(const char *cmd);
const char *max30101_channel_mask_name(uint8_t mask);

#ifdef __cplusplus
}
#endif
