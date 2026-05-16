#include <raw_hid/raw_hid.h>
#include <raw_hid/events.h>

#include <zmk/ble.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

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
/* Receive path disabled - just acknowledge writes and drop data. */
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
 * Deferred notify path
 *
 * Background:
 *
 *   Original bug:  send_report() built a stack-local `report[]` and passed
 *                  `&report` to bt_gatt_notify_cb(). Although modern Zephyr
 *                  copies params->data into an ATT PDU before returning,
 *                  there are still subtle issues with calling the BT host
 *                  from event listeners (notably: the call can block on the
 *                  ATT tx semaphore if the host is busy), and the original
 *                  shared static buffer pattern in the layer-notifier could
 *                  be clobbered between back-to-back events.
 *
 *   v1 attempt:    Added a static buffer + a semaphore taken before send and
 *                  released by a notify-completion callback. This had the
 *                  unintended side effect of blocking the keymap thread for
 *                  up to 50ms per send, which broke timing-sensitive
 *                  sticky-layer / conditional-layer sequences.
 *
 *   v2 (this fix): Decouple completely. The event listener runs on the
 *                  keymap thread, copies the payload into a thread-safe
 *                  message queue, and returns. A dedicated work-queue
 *                  thread (raw_hog_work_q, declared but unused in the
 *                  original) drains the queue and performs the actual BT
 *                  notify call. The keymap thread NEVER blocks on BT.
 *
 * Queue depth: 4 messages is plenty. Layer changes happen at most a few
 * tens per second; BT notify is single-digit ms. If the queue fills (e.g.
 * BT link congested), excess reports are dropped on the producer side,
 * which is the right behaviour: dropping is preferable to blocking the
 * keymap.
 * ------------------------------------------------------------------------- */

#define TX_QUEUE_DEPTH 4

struct tx_msg {
    uint8_t data[CONFIG_RAW_HID_REPORT_SIZE];
};

K_MSGQ_DEFINE(tx_msgq, sizeof(struct tx_msg), TX_QUEUE_DEPTH, 4);

K_THREAD_STACK_DEFINE(raw_hog_q_stack, CONFIG_ZMK_BLE_THREAD_STACK_SIZE);
struct k_work_q raw_hog_work_q;
static struct k_work tx_drain_work;

static void tx_drain_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    struct tx_msg msg;

    /* Drain everything currently queued. We re-arm via k_work_submit_to_queue
     * if more arrives while we're processing; k_work handles the
     * already-pending case safely. */
    while (k_msgq_get(&tx_msgq, &msg, K_NO_WAIT) == 0) {
        struct bt_conn *conn = zmk_ble_active_profile_conn();
        if (conn == NULL) {
            LOG_DBG("Not connected to active profile, dropping raw HID report");
            continue;
        }

        struct bt_gatt_notify_params notify_params = {
            .attr = &raw_hog_svc.attrs[5],
            .data = msg.data,
            .len = CONFIG_RAW_HID_REPORT_SIZE,
            .func = NULL,
            .user_data = NULL,
        };

        LOG_INF("BT - Sending Raw HID report");
        LOG_HEXDUMP_DBG(msg.data, sizeof(msg.data), "BT - Sending Raw HID report");

        int err = bt_gatt_notify_cb(conn, &notify_params);
        if (err == -EPERM) {
            bt_conn_set_security(conn, BT_SECURITY_L2);
        } else if (err) {
            LOG_DBG("Raw HID notify error %d", err);
        }
        /* notify_params (and msg.data) are no longer referenced after
         * bt_gatt_notify_cb returns -- Zephyr copies the payload into the
         * ATT PDU before returning. */

        bt_conn_unref(conn);
    }
}

static int raw_hid_sent_event_listener(const zmk_event_t *eh) {
    struct raw_hid_sent_event *event = as_raw_hid_sent_event(eh);
    if (!event) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    struct tx_msg msg;
    memset(msg.data, 0, sizeof(msg.data));
    memcpy(msg.data, event->data, MIN(event->length, sizeof(msg.data)));

    /* Non-blocking put: if the queue is full we drop. This must NEVER
     * block, since this listener runs on the keymap thread and any block
     * here would delay sticky-layer / conditional-layer state machines. */
    int ret = k_msgq_put(&tx_msgq, &msg, K_NO_WAIT);
    if (ret != 0) {
        LOG_WRN("Raw HID TX queue full, dropping report");
        return ZMK_EV_EVENT_BUBBLE;
    }

    /* Wake the drain worker. If it's already pending or running, this is
     * a no-op; if it's idle, this schedules it. Either way, the work will
     * eventually run and drain everything we've just put. */
    k_work_submit_to_queue(&raw_hog_work_q, &tx_drain_work);

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(bt_process_raw_hid_sent_event, raw_hid_sent_event_listener);
ZMK_SUBSCRIPTION(bt_process_raw_hid_sent_event, raw_hid_sent_event);

static int raw_hog_init(void) {
    k_work_init(&tx_drain_work, tx_drain_work_handler);

    static const struct k_work_queue_config queue_config = {.name = "raw_hog_q"};
    k_work_queue_start(&raw_hog_work_q, raw_hog_q_stack, K_THREAD_STACK_SIZEOF(raw_hog_q_stack),
                       CONFIG_ZMK_BLE_THREAD_PRIORITY, &queue_config);

    return 0;
}

SYS_INIT(raw_hog_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);
