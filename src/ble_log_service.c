#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#include "ble_log_service.h"

/* 128-bit UUIDs */
#define BT_UUID_LOG_SERVICE_VAL    BT_UUID_128_ENCODE(0x9f7b0000, 0x6c35, 0x4d2c, 0x9c85, 0x4a8c1a2b3c4d)
#define BT_UUID_LOG_STREAM_VAL     BT_UUID_128_ENCODE(0x9f7b0001, 0x6c35, 0x4d2c, 0x9c85, 0x4a8c1a2b3c4d)
#define BT_UUID_LOG_COMMAND_VAL    BT_UUID_128_ENCODE(0x9f7b0002, 0x6c35, 0x4d2c, 0x9c85, 0x4a8c1a2b3c4d)

static struct bt_uuid_128 log_svc_uuid = BT_UUID_INIT_128(BT_UUID_LOG_SERVICE_VAL);
static struct bt_uuid_128 log_chr_uuid = BT_UUID_INIT_128(BT_UUID_LOG_STREAM_VAL);
static struct bt_uuid_128 cmd_chr_uuid = BT_UUID_INIT_128(BT_UUID_LOG_COMMAND_VAL);

static struct bt_conn *g_conn;
static volatile bool g_notify_enabled;
static ble_log_cmd_cb_t g_cmd_cb;
K_MUTEX_DEFINE(g_tx_mutex);

static uint8_t g_last[512];
static size_t  g_last_len;

static ssize_t log_read(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    (void)attr;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, g_last, g_last_len);
}

static ssize_t cmd_write(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len,
                         uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    if (offset != 0U) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    char cmd[96];
    size_t n = MIN((size_t)len, sizeof(cmd) - 1U);
    memcpy(cmd, buf, n);
    cmd[n] = '\0';

    /* Remove trailing CR/LF from app/GUI commands. */
    while (n > 0U && (cmd[n - 1U] == '\r' || cmd[n - 1U] == '\n' || cmd[n - 1U] == ' ')) {
        cmd[n - 1U] = '\0';
        n--;
    }

    if (g_cmd_cb) {
        g_cmd_cb(cmd);
    }

    return len;
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    g_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

/* attrs index:
 * 0 = primary service
 * 1 = stream chr declaration
 * 2 = stream chr value (notify/read this)
 * 3 = stream ccc
 * 4 = command chr declaration
 * 5 = command chr value (write this)
 */
BT_GATT_SERVICE_DEFINE(log_svc,
    BT_GATT_PRIMARY_SERVICE(&log_svc_uuid),
    BT_GATT_CHARACTERISTIC(&log_chr_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           log_read, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&cmd_chr_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, cmd_write, NULL)
);

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        return;
    }

    if (g_conn) {
        bt_conn_unref(g_conn);
        g_conn = NULL;
    }
    g_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(reason);

    if (g_conn) {
        bt_conn_unref(g_conn);
        g_conn = NULL;
    }
    g_notify_enabled = false;
}

BT_CONN_CB_DEFINE(conn_cb) = {
    .connected = connected,
    .disconnected = disconnected,
};

int ble_log_service_init(void)
{
    int err = bt_enable(NULL);
    if (err) {
        return err;
    }

    return bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0);
}

void ble_log_service_set_command_callback(ble_log_cmd_cb_t cb)
{
    g_cmd_cb = cb;
}

int ble_log_send_as(const uint8_t *data, size_t len)
{
    if (!data || len == 0U) {
        return 0;
    }

    struct bt_conn *conn = g_conn;
    if (!conn || !g_notify_enabled) {
        return 0;
    }

    k_mutex_lock(&g_tx_mutex, K_FOREVER);

    uint16_t mtu = bt_gatt_get_mtu(conn);
    uint16_t max_payload = (mtu > 3U) ? (mtu - 3U) : 20U;

    g_last_len = MIN(len, sizeof(g_last));
    memcpy(g_last, data, g_last_len);

    size_t off = 0U;
    int rc = (int)len;

    while (off < len) {
        uint16_t chunk = (uint16_t)MIN((size_t)max_payload, len - off);
        int tries = 0;
        int err;

        do {
            err = bt_gatt_notify(conn, &log_svc.attrs[2], data + off, chunk);
            if (err == -ENOMEM) {
                k_msleep(5);
            }
            tries++;
        } while (err == -ENOMEM && tries < 20);

        if (err) {
            rc = err;
            break;
        }

        off += chunk;
        k_yield();
    }

    k_mutex_unlock(&g_tx_mutex);
    return rc;
}

bool ble_log_is_ready(void)
{
    return (g_conn != NULL) && g_notify_enabled;
}
