/** @file
 *  @brief HID over GATT (HIDS) service of the Wafer Pad mouse
 *
 *  Adapted from the Zephyr samples/bluetooth/peripheral_hids "HoG Service
 *  sample" (hog.c): report map, input report with CCC and Report Reference,
 *  HID Information and HID Control Point. Changes: the shared Wafer Pad report
 *  map, encryption on every HID attribute, a readable last input report and
 *  notification with completion callback for flow control.
 */

/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2026 Wafer Pad contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <wafer_pad/hid_report.h>

#include "hid_internal.h"

LOG_MODULE_REGISTER(wp_ble_hog, CONFIG_WAFER_PAD_LOG_LEVEL);

/* HID Information flags (HIDS 1.0, 3.3). */
enum {
	HIDS_REMOTE_WAKE = BIT(0),
	HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info {
	uint16_t version; /* version number of base USB HID Specification */
	uint8_t code;     /* country HID Device hardware is localized for. */
	uint8_t flags;
} __packed;

struct hids_report {
	uint8_t id;   /* report id */
	uint8_t type; /* report type */
} __packed;

/* Report Reference report types (HIDS 1.0, 3.6.2). */
enum {
	HIDS_INPUT = 0x01,
	HIDS_OUTPUT = 0x02,
	HIDS_FEATURE = 0x03,
};

/* HID Control Point values (HIDS 1.0, 3.5). */
enum {
	HIDS_CONTROL_POINT_SUSPEND = 0x00,
	HIDS_CONTROL_POINT_EXIT_SUSPEND = 0x01,
};

static const struct hids_info info = {
	.version = sys_cpu_to_le16(0x0111), /* bcdHID 1.11 */
	.code = 0x00,
	.flags = HIDS_NORMALLY_CONNECTABLE,
};

static const struct hids_report input_ref = {
	.id = WP_HID_REPORT_ID_MOUSE,
	.type = HIDS_INPUT,
};

/* Value returned when the host READS the input Report: the button state of
 * the last notified report. Relative motion / scroll are deltas that were
 * already delivered, so they are never replayed (always 0). Cleared on
 * connect and disconnect, so a new link (or host) never reads stale buttons.
 */
static struct k_spinlock input_lock;
static uint8_t input_value[WP_HID_MOUSE_REPORT_LEN];

static uint8_t ctrl_point;

static ssize_t read_info(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			 uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &info, sizeof(info));
}

static ssize_t read_report_map(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, wp_hid_report_map,
				 wp_hid_report_map_len);
}

static ssize_t read_report_ref(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &input_ref, sizeof(input_ref));
}

static ssize_t read_input_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				 void *buf, uint16_t len, uint16_t offset)
{
	uint8_t value[WP_HID_MOUSE_REPORT_LEN];
	k_spinlock_key_t key = k_spin_lock(&input_lock);

	memcpy(value, input_value, sizeof(value));
	k_spin_unlock(&input_lock, key);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(value));
}

static void input_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	wp_ble_hog_ccc_changed((value & BT_GATT_CCC_NOTIFY) != 0U);
}

static ssize_t write_ctrl_point(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t value;

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(ctrl_point)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	value = *(const uint8_t *)buf;
	if (value != HIDS_CONTROL_POINT_SUSPEND && value != HIDS_CONTROL_POINT_EXIT_SUSPEND) {
		/* Reserved values: ignored (write without response, HIDS 3.5). */
		return len;
	}

	ctrl_point = value;
	LOG_INF("host %s", value == HIDS_CONTROL_POINT_SUSPEND ? "suspend" : "exit suspend");

	return len;
}

/* Every HID attribute requires an encrypted link (security level >= L2). */
#define HOG_PERM_READ  BT_GATT_PERM_READ_ENCRYPT
#define HOG_PERM_WRITE BT_GATT_PERM_WRITE_ENCRYPT

/* HID Service Declaration */
BT_GATT_SERVICE_DEFINE(hog_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),                          /* [0] */
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ,    /* [1],[2] */
			       HOG_PERM_READ, read_info, NULL, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ, /* [3],[4] */
			       HOG_PERM_READ, read_report_map, NULL, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,                     /* [5],[6] */
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       HOG_PERM_READ, read_input_report, NULL, NULL),
	BT_GATT_CCC(input_ccc_changed, HOG_PERM_READ | HOG_PERM_WRITE), /* [7] */
	BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, HOG_PERM_READ,      /* [8] */
			   read_report_ref, NULL, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT,                 /* [9],[10] */
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       HOG_PERM_WRITE, NULL, write_ctrl_point, &ctrl_point),
);

/* Index of the input Report characteristic value attribute in hog_svc. */
#define HOG_ATTR_INPUT_VALUE 6
#define HOG_ATTR_INPUT_CCC   7

BUILD_ASSERT(sizeof(struct hids_info) == 4, "HID Information is 4 bytes");
BUILD_ASSERT(sizeof(struct hids_report) == 2, "Report Reference is 2 bytes");

int wp_ble_hog_init(void)
{
	const struct bt_gatt_attr *val = &hog_svc.attrs[HOG_ATTR_INPUT_VALUE];
	const struct bt_gatt_attr *ccc = &hog_svc.attrs[HOG_ATTR_INPUT_CCC];

	if (hog_svc.attr_count <= HOG_ATTR_INPUT_CCC ||
	    bt_uuid_cmp(val->uuid, BT_UUID_HIDS_REPORT) != 0 ||
	    bt_uuid_cmp(ccc->uuid, BT_UUID_GATT_CCC) != 0) {
		LOG_ERR("HIDS attribute table layout changed");
		return -EFAULT;
	}

	LOG_DBG("HIDS ready, report map %u bytes", (unsigned int)wp_hid_report_map_len);
	return 0;
}

bool wp_ble_hog_is_subscribed(struct bt_conn *conn)
{
	if (conn == NULL) {
		return false;
	}
	return bt_gatt_is_subscribed(conn, &hog_svc.attrs[HOG_ATTR_INPUT_VALUE],
				     BT_GATT_CCC_NOTIFY);
}

int wp_ble_hog_notify(struct bt_conn *conn, const uint8_t *payload, uint16_t len,
		      bt_gatt_complete_func_t func, void *user_data)
{
	struct bt_gatt_notify_params params = {
		.attr = &hog_svc.attrs[HOG_ATTR_INPUT_VALUE],
		.data = payload,
		.len = len,
		.func = func,
		.user_data = user_data,
	};
	int err;

	if (conn == NULL || payload == NULL || len != WP_HID_MOUSE_REPORT_LEN) {
		return -EINVAL;
	}

	err = bt_gatt_notify_cb(conn, &params);
	if (err == 0) {
		k_spinlock_key_t key = k_spin_lock(&input_lock);

		memset(input_value, 0, sizeof(input_value));
		input_value[0] = payload[0]; /* buttons only */
		k_spin_unlock(&input_lock, key);
	}

	return err;
}

void wp_ble_hog_reset_input(void)
{
	k_spinlock_key_t key = k_spin_lock(&input_lock);

	memset(input_value, 0, sizeof(input_value));
	k_spin_unlock(&input_lock, key);
}
