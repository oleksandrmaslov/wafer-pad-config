/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * HID report descriptor, report packing and the report composer
 * (pure C, host-testable; shared by every HID transport: BLE HOG, USB).
 *
 * Mouse input report, Report ID WP_HID_REPORT_ID_MOUSE, payload (without the
 * report ID byte, as carried in a HOGP Report characteristic):
 *   byte 0     : buttons (bit0 left, bit1 right, bit2 middle, bit3 back,
 *                bit4 forward, bits 5-7 constant padding)
 *   bytes 1-2  : X,  int16 little endian, -32767..32767 (relative)
 *   bytes 3-4  : Y,  int16 little endian, -32767..32767 (relative)
 *   byte 5     : wheel, int8, -127..127 (relative, + = up)
 *   byte 6     : AC Pan (Consumer page 0x0C, usage 0x0238), int8, -127..127
 *
 * Report IDs 2 (keyboard) and 3 (consumer) are RESERVED for the future Wafer
 * keyboard integration; they are not in the v0.1 descriptor.
 */

#ifndef WAFER_PAD_HID_REPORT_H_
#define WAFER_PAD_HID_REPORT_H_

#include <wafer_pad/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WP_HID_REPORT_ID_MOUSE    1
#define WP_HID_REPORT_ID_KEYBOARD 2 /* reserved */
#define WP_HID_REPORT_ID_CONSUMER 3 /* reserved */

#define WP_HID_MOUSE_REPORT_LEN   7 /* payload bytes, without report ID */

/** The HID report map (descriptor) shared by all transports. */
extern const uint8_t wp_hid_report_map[];
extern const size_t wp_hid_report_map_len;

/**
 * Serialize @p r into @p buf (payload only, no report ID).
 * @return WP_HID_MOUSE_REPORT_LEN, or 0 if @p len is too small.
 */
size_t wp_hid_mouse_report_pack(const struct wp_mouse_report *r, uint8_t *buf, size_t len);

/** Inverse of pack (used by tests and the USB loopback). */
bool wp_hid_mouse_report_unpack(const uint8_t *buf, size_t len, struct wp_mouse_report *r);

/* ------------------------------------------------------------------------ */
/* Report composer                                                           */
/* ------------------------------------------------------------------------ */

#define WP_HID_CLICK_QUEUE_LEN 4

/**
 * Accumulates fractional motion/scroll from all producers (gestures, air
 * mouse, future keyboard pointer keys), button state and one-shot clicks, and
 * emits integer HID reports when the transport can take one.
 *
 * Semantics:
 *   - Fractional parts are carried over (sub-count precision, no drift).
 *   - Pending integer motion that exceeds the report range is split over
 *     several reports (never clipped / lost), wheel/pan likewise (+-127).
 *   - A click produces exactly two reports: PRESS (held | click buttons, no
 *     motion) then RELEASE (held buttons; may carry pending motion).
 *   - A report is produced when anything changed: motion != 0, scroll != 0,
 *     or buttons differ from the last emitted report.
 *   - wp_hid_composer_release_all() guarantees that the next report has
 *     buttons == 0 if the last emitted report had any button down.
 */
struct wp_hid_composer {
	float rem_x, rem_y;          /**< Fractional remainders. */
	float rem_wheel, rem_pan;
	int32_t pend_x, pend_y;      /**< Integer motion not yet emitted. */
	int32_t pend_wheel, pend_pan;
	uint8_t held;                /**< Buttons currently held (drag). */
	uint8_t last_sent_buttons;   /**< Buttons in the last emitted report. */
	uint8_t clicks[WP_HID_CLICK_QUEUE_LEN];
	uint8_t click_head, click_count;
	bool click_pressed;          /**< PRESS of clicks[head] already emitted. */
};

void wp_hid_composer_init(struct wp_hid_composer *c);

void wp_hid_composer_add_motion(struct wp_hid_composer *c, float dx, float dy);

void wp_hid_composer_add_scroll(struct wp_hid_composer *c, float wheel, float pan);

/** Set the buttons that are held down continuously (e.g. drag). */
void wp_hid_composer_set_held(struct wp_hid_composer *c, uint8_t buttons);

/** Queue a click (press + release) of @p buttons. Dropped if queue full. */
void wp_hid_composer_click(struct wp_hid_composer *c, uint8_t buttons);

/** Drop pending motion/scroll and fractional remainders (orientation or
 *  mode change: prevents a cursor jump). Buttons are not affected. */
void wp_hid_composer_reset_motion(struct wp_hid_composer *c);

/** Drop everything and release all buttons (contact loss, disconnect). */
void wp_hid_composer_release_all(struct wp_hid_composer *c);

/** Forget what the host was last told: the next report re-sends the full
 *  button state (buttons == held), even if it equals the last emitted one.
 *  Call on every "transport became ready" transition, because a report the
 *  composer emitted may never have reached the host (e.g. a RELEASE dropped
 *  while the link flapped). */
void wp_hid_composer_force_sync(struct wp_hid_composer *c);

/** True if wp_hid_composer_next() would produce a report. */
bool wp_hid_composer_pending(const struct wp_hid_composer *c);

/** Produce the next report. Returns false if there is nothing to send. */
bool wp_hid_composer_next(struct wp_hid_composer *c, struct wp_mouse_report *r);

#ifdef __cplusplus
}
#endif

#endif /* WAFER_PAD_HID_REPORT_H_ */
