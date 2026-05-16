#include <raw_hid/raw_hid.h>
#include <raw_hid/events.h>

#include <zmk/ble.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

enum {
    HIDS_REMOTE_WAKE = BIT(0),
    HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info {
    uint16_t version;
    uint8_t code;
    uint8_t flags;
} __packed;

struct hids_report {
    uint8_t id;
    uint8_t type;
} __packed;

static struct hids_info info = {
    .version = 0x0000,
    .code = 0x00,
    .flags = HIDS_NORMALLY_CONNECTABLE | HIDS_REMOTE_WAKE,
};

enum {
    HIDS_INPUT = 0x01,
    HIDS_OUTPUT = 0x02,
    HIDS_FEATURE = 0x03,
};

static struct hids_report raw_hid_report_output = {
    .id = 0x00,
    .type = HIDS_OUTPUT,
};

static struct hids_report raw_hid_report_input = {
    .id = 0x00,
    .type = HIDS_INPUT,
};

static uint8_t ctrl_point;

static ssize_t read_hids_info(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                              uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
                             sizeof(struct hids_info));
}

static ssize_t read_hids_report_ref(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
                             sizeof(struct hids_report));
}

static ssize_t read_hids_raw_hid_report_map(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                            void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, raw_hid_report_desc,
                             sizeof(raw_hid_report_desc));
}

#if IS_ENABLED(CONFIG_RAW_HID_ENABLE_RECEIVE)
static ssize_t write_hids_raw_hid_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                         const void *buf, uint16_t len, uint16_t offset,
                                         uint8_t flags) {
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    uint8_t *data = (uint8_t *)buf;
    LOG_INF("BT - Received Raw HID report of length %i", len);
    LOG_HEXDUMP_DBG(data, len, "BT - Received Raw HID report");
    raise_raw_hid_received_event((struct raw_hid_received_event){.data = data, .length = len});

    return len;
}
#else
/* Receive path disabled - just acknowledge writes and drop data.
 * Avoids passing a pointer to a transient BT stack buffer to listeners. */
static ssize_t write_hids_raw_hid_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                         const void *buf, uint16_t len, uint16_t offset,
                                         uint8_t flags) {
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(buf);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    return len;
}
#endif

static ssize_t write_ctrl_point(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    uint8_t *value = attr->user_data;

    if (offset + len > sizeof(ctrl_point)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value + offset, buf, len);

    return len;
}

/* HID Service Declaration */
BT_GATT_SERVICE_DEFINE(
    raw_hog_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_hids_info,
                           NULL, &info),

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ, BT_GATT_PERM_READ_ENCRYPT,
                           read_hids_raw_hid_report_map, NULL, NULL),

    // send to host
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, NULL, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &raw_hid_report_input),

    // receive from host
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, NULL,
                           write_hids_raw_hid_report, NULL),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &raw_hid_report_output),

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, write_ctrl_point, &ctrl_point));

/* ---------------------------------------------------------------------------
 * Notify send path
 *
 * Original bug:
 *   send_report() built `report[]` on the stack and passed `&report` to
 *   bt_gatt_notify_cb(). That function is ASYNC: it queues the notification
 *   and returns immediately, while `report` goes out of scope. The BT host
 *   thread later reads from a freed stack location to transmit. Result:
 *   intermittent memory corruption that eventually wedges the BLE stack and
 *   requires a power cycle. This is the keyboard-freeze cause.
 *
 * Fix:
 *   - TX buffer is file-static, kept alive across the async send.
 *   - A semaphore serialises sends; the notify completion callback releases
 *     it so we never overwrite an in-flight buffer.
 *   - On error / no-connection / timeout we release the semaphore so the
 *     next send isn't blocked.
 * ------------------------------------------------------------------------- */

static uint8_t tx_report[CONFIG_RAW_HID_REPORT_SIZE];
static struct bt_gatt_notify_params tx_notify_params;
static K_SEM_DEFINE(tx_sem, 1, 1);

static void notify_complete_cb(struct bt_conn *conn, void *user_data) {
    ARG_UNUSED(conn);
    ARG_UNUSED(user_data);
    k_sem_give(&tx_sem);
}

static void send_report(const uint8_t *data, uint8_t len) {
    struct bt_conn *conn = zmk_ble_active_profile_conn();
    if (conn == NULL) {
        LOG_DBG("Not connected to active profile, dropping raw HID report");
        return;
    }

    /* Bounded wait: if the previous notify hasn't completed in 50ms, the link
     * is probably degraded. Drop this report rather than block the caller
     * (which may be the BLE thread itself). */
    if (k_sem_take(&tx_sem, K_MSEC(50)) != 0) {
        LOG_WRN("Previous raw HID notify still pending, dropping");
        bt_conn_unref(conn);
        return;
    }

    LOG_INF("BT - Sending Raw HID report of length %i", len);
    memset(tx_report, 0, sizeof(tx_report));
    memcpy(tx_report, data, MIN(len, sizeof(tx_report)));
    LOG_HEXDUMP_DBG(tx_report, sizeof(tx_report), "BT - Sending Raw HID report");

    tx_notify_params.attr = &raw_hog_svc.attrs[5];
    tx_notify_params.data = tx_report;
    tx_notify_params.len = sizeof(tx_report);
    tx_notify_params.func = notify_complete_cb;
    tx_notify_params.user_data = NULL;

    int err = bt_gatt_notify_cb(conn, &tx_notify_params);
    if (err == -EPERM) {
        bt_conn_set_security(conn, BT_SECURITY_L2);
        k_sem_give(&tx_sem);
    } else if (err) {
        LOG_ERR("Raw HID notify error %d", err);
        k_sem_give(&tx_sem);
    }
    /* Success: tx_sem is released by notify_complete_cb when BT host is done. */

    bt_conn_unref(conn);
}

static int raw_hid_sent_event_listener(const zmk_event_t *eh) {
    struct raw_hid_sent_event *event = as_raw_hid_sent_event(eh);
    if (event) {
        send_report(event->data, event->length);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(bt_process_raw_hid_sent_event, raw_hid_sent_event_listener);
ZMK_SUBSCRIPTION(bt_process_raw_hid_sent_event, raw_hid_sent_event);

K_THREAD_STACK_DEFINE(raw_hog_q_stack, CONFIG_ZMK_BLE_THREAD_STACK_SIZE);

struct k_work_q raw_hog_work_q;

static int raw_hog_init(void) {
    static const struct k_work_queue_config queue_config = {.name = "HID Over GATT Send Work"};
    k_work_queue_start(&raw_hog_work_q, raw_hog_q_stack, K_THREAD_STACK_SIZEOF(raw_hog_q_stack),
                       CONFIG_ZMK_BLE_THREAD_PRIORITY, &queue_config);

    return 0;
}

SYS_INIT(raw_hog_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);
