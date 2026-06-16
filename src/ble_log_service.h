#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

int ble_log_service_init(void);

/* Optional app/GUI command callback.
 * Commands are received through the writable BLE command characteristic.
 */
typedef void (*ble_log_cmd_cb_t)(const char *cmd);
void ble_log_service_set_command_callback(ble_log_cmd_cb_t cb);

/* Send one already-framed UTF-8 text message or binary chunk.
 * This call is serialized internally so different modules cannot interleave bytes.
 */
int ble_log_send_as(const uint8_t *data, size_t len);

/* True when a BLE central is connected and notifications are enabled. */
bool ble_log_is_ready(void);

#ifdef __cplusplus
}
#endif
