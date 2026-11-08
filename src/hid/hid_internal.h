/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private interfaces between the files of src/hid (not a public contract).
 */

#ifndef WAFER_PAD_SRC_HID_INTERNAL_H_
#define WAFER_PAD_SRC_HID_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#include <wafer_pad/platform.h>

/* ble.c: the BLE HID-over-GATT transport (always present). */
extern const struct wp_hid_transport_api wp_hid_ble_transport;

/* ble_hog.c: sanity-check the static HID service. */
int wp_ble_hog_init(void);

/* ble_hog.c: true if @p conn enabled notifications on the input report. */
bool wp_ble_hog_is_subscribed(struct bt_conn *conn);

/* ble_hog.c: notify one mouse input report payload (Report ID 1, without the
 * ID byte) to @p conn. @p func runs in thread context when the controller has
 * taken the packet. Returns 0 or a negative errno from bt_gatt_notify_cb().
 */
int wp_ble_hog_notify(struct bt_conn *conn, const uint8_t *payload, uint16_t len,
		      bt_gatt_complete_func_t func, void *user_data);

/* ble_hog.c: forget the last input report (read value = all zero). Called on
 * connect and disconnect. */
void wp_ble_hog_reset_input(void);

/* ble.c: called by ble_hog.c when the input report CCC changes. */
void wp_ble_hog_ccc_changed(bool notify_enabled);

#endif /* WAFER_PAD_SRC_HID_INTERNAL_H_ */
