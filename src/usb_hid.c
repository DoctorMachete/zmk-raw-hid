#include <raw_hid/raw_hid.h>
#include <raw_hid/events.h>

#include <zmk/usb.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static const struct device *raw_hid_dev;

static K_SEM_DEFINE(hid_sem, 1, 1);

static void in_ready_cb(const struct device *dev) { k_sem_give(&hid_sem); }

#define HID_GET_REPORT_TYPE_MASK 0xff00
#define HID_GET_REPORT_ID_MASK 0x00ff

#define HID_REPORT_TYPE_INPUT 0x100
#define HID_REPORT_TYPE_OUTPUT 0x200
#define HID_REPORT_TYPE_FEATURE 0x300

static int get_report_cb(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
                         uint8_t **data) {
    return 0;
}

#if IS_ENABLED(CONFIG_RAW_HID_ENABLE_RECEIVE)
static int set_report_cb(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
                         uint8_t **data) {
    if ((setup->wValue & HID_GET_REPORT_TYPE_MASK) != HID_REPORT_TYPE_OUTPUT &&
        (setup->wValue & HID_GET_REPORT_TYPE_MASK) != HID_REPORT_TYPE_FEATURE) {
        LOG_ERR("[# raw-hid #] Set: Unsupported report type %d requested",
                (setup->wValue & HID_GET_REPORT_TYPE_MASK) >> 8);
        return -ENOTSUP;
    }

    LOG_INF("USB - Received Raw HID report of length %i", *len);
    LOG_HEXDUMP_DBG(*data, *len, "USB - Received Raw HID report");
    raise_raw_hid_received_event((struct raw_hid_received_event){.data = *data, .length = *len});

    return 0;
}
#else
/* Receive path disabled - reject set_report. */
static int set_report_cb(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
                         uint8_t **data) {
    ARG_UNUSED(dev);
    ARG_UNUSED(setup);
    ARG_UNUSED(len);
    ARG_UNUSED(data);
    return -ENOTSUP;
}
#endif

static const struct hid_ops ops = {
    .int_in_ready = in_ready_cb,
    .get_report = get_report_cb,
    .set_report = set_report_cb,
};

static void send_report(const uint8_t *data, uint8_t len) {
    /* If a prior send is still draining, wait briefly. If it never completes
     * (host detached, endpoint stalled), drop this report rather than blocking
     * the caller. The original code used K_MSEC(30) then unconditionally
     * called hid_int_ep_write even on semaphore timeout; that meant a stalled
     * endpoint would slowly accumulate 30ms blocks on every send. */
    if (k_sem_take(&hid_sem, K_MSEC(30)) != 0) {
        LOG_DBG("USB Raw HID busy, dropping report");
        return;
    }

    LOG_INF("USB - Sending Raw HID report of length %i", len);
    uint8_t report[CONFIG_RAW_HID_REPORT_SIZE] = {0};
    memcpy(report, data, len);
    LOG_HEXDUMP_DBG(report, CONFIG_RAW_HID_REPORT_SIZE, "USB - Sending Raw HID report");

    /* Note: hid_int_ep_write copies the data into the USB driver's own buffer
     * before returning, so a stack-local `report` is safe here -- in contrast
     * to bt_gatt_notify_cb which only queues a pointer. */
    int err = hid_int_ep_write(raw_hid_dev, report, CONFIG_RAW_HID_REPORT_SIZE, NULL);
    if (err) {
        LOG_ERR("Failed to send report: %i", err);
        k_sem_give(&hid_sem);
    }
}

static int raw_hid_sent_event_listener(const zmk_event_t *eh) {
    struct raw_hid_sent_event *event = as_raw_hid_sent_event(eh);
    if (event) {
        send_report(event->data, event->length);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(usb_process_raw_hid_sent_event, raw_hid_sent_event_listener);
ZMK_SUBSCRIPTION(usb_process_raw_hid_sent_event, raw_hid_sent_event);

static int raw_hid_init(void) {
    raw_hid_dev = device_get_binding(CONFIG_RAW_HID_DEVICE);
    if (raw_hid_dev == NULL) {
        LOG_ERR("Unable to locate HID device");
        return -EINVAL;
    }

    usb_hid_register_device(raw_hid_dev, raw_hid_report_desc, sizeof(raw_hid_report_desc), &ops);

    usb_hid_init(raw_hid_dev);

    return 0;
}

SYS_INIT(raw_hid_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
