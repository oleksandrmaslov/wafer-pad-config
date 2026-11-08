/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * HID report descriptor, mouse report packing and the report composer.
 *
 * PURE C: no Zephyr includes. All state lives in struct wp_hid_composer, time
 * is not needed (the composer is driven by the transport's readiness).
 *
 * Usage references: "HID Usage Tables for USB" v1.5 (HUT 1.5):
 *   Generic Desktop page 0x01: Pointer 0x01, Mouse 0x02, X 0x30, Y 0x31 (DV, 4.2),
 *                              Wheel 0x38 (DV, 4.3)
 *   Button page 0x09:          Button 1..5 (Sel)
 *   Consumer page 0x0C:        AC Pan 0x0238 (LC, 15.16, "Set the horizontal offset of the
 *                              display in the document")
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <wafer_pad/hid_report.h>

/* ------------------------------------------------------------------------ */
/* Report map                                                                */
/* ------------------------------------------------------------------------ */

/*
 * Report ID 1, input payload = 56 bits = 7 bytes (see hid_report.h):
 *   5 x 1 bit buttons + 3 bit constant padding  =  8
 *   X, Y: 2 x 16 bit relative                    = 32
 *   Wheel: 8 bit relative                        =  8
 *   AC Pan: 8 bit relative                       =  8
 *
 * Item encoding (HID 1.11, 6.2.2.2): first byte = tag | type | size, where
 * size code 1 = 1 data byte, 2 = 2 data bytes. 0x16/0x26 are the 2-byte forms
 * of Logical Minimum/Maximum, 0x0A the 2-byte (extended page-local) Usage.
 */
const uint8_t wp_hid_report_map[] = {
	0x05, 0x01,             /* Usage Page (Generic Desktop)                    */
	0x09, 0x02,             /* Usage (Mouse)                                   */
	0xA1, 0x01,             /* Collection (Application)                        */
	0x85, WP_HID_REPORT_ID_MOUSE, /* Report ID (1)                             */
	0x09, 0x01,             /*   Usage (Pointer)                               */
	0xA1, 0x00,             /*   Collection (Physical)                         */

	/* Buttons 1..5 (left, right, middle, back, forward): 5 x 1 bit */
	0x05, 0x09,             /*     Usage Page (Button)                         */
	0x19, 0x01,             /*     Usage Minimum (Button 1)                    */
	0x29, 0x05,             /*     Usage Maximum (Button 5)                    */
	0x15, 0x00,             /*     Logical Minimum (0)                         */
	0x25, 0x01,             /*     Logical Maximum (1)                         */
	0x75, 0x01,             /*     Report Size (1)                             */
	0x95, 0x05,             /*     Report Count (5)                            */
	0x81, 0x02,             /*     Input (Data, Variable, Absolute)            */

	/* 3 bit constant padding to the byte boundary */
	0x75, 0x03,             /*     Report Size (3)                             */
	0x95, 0x01,             /*     Report Count (1)                            */
	0x81, 0x03,             /*     Input (Constant, Variable, Absolute)        */

	/* X, Y: 2 x int16, relative, -32767..32767 */
	0x05, 0x01,             /*     Usage Page (Generic Desktop)                */
	0x09, 0x30,             /*     Usage (X)                                   */
	0x09, 0x31,             /*     Usage (Y)                                   */
	0x16, 0x01, 0x80,       /*     Logical Minimum (-32767)                    */
	0x26, 0xFF, 0x7F,       /*     Logical Maximum (32767)                     */
	0x75, 0x10,             /*     Report Size (16)                            */
	0x95, 0x02,             /*     Report Count (2)                            */
	0x81, 0x06,             /*     Input (Data, Variable, Relative)            */

	/* Vertical wheel: int8, relative, -127..127 */
	0x09, 0x38,             /*     Usage (Wheel)                               */
	0x15, 0x81,             /*     Logical Minimum (-127)                      */
	0x25, 0x7F,             /*     Logical Maximum (127)                       */
	0x75, 0x08,             /*     Report Size (8)                             */
	0x95, 0x01,             /*     Report Count (1)                            */
	0x81, 0x06,             /*     Input (Data, Variable, Relative)            */

	/* Horizontal scroll: AC Pan (Consumer 0x0238), int8, relative, -127..127.
	 * Logical range / size / count are restated although they are global
	 * items still in effect, so the field reads correctly on its own.
	 */
	0x05, 0x0C,             /*     Usage Page (Consumer)                       */
	0x0A, 0x38, 0x02,       /*     Usage (AC Pan, 0x0238)                      */
	0x15, 0x81,             /*     Logical Minimum (-127)                      */
	0x25, 0x7F,             /*     Logical Maximum (127)                       */
	0x75, 0x08,             /*     Report Size (8)                             */
	0x95, 0x01,             /*     Report Count (1)                            */
	0x81, 0x06,             /*     Input (Data, Variable, Relative)            */

	0xC0,                   /*   End Collection (Physical)                     */
	0xC0,                   /* End Collection (Application)                    */
};

const size_t wp_hid_report_map_len = sizeof(wp_hid_report_map);

/* ------------------------------------------------------------------------ */
/* Packing                                                                   */
/* ------------------------------------------------------------------------ */

#define XY_LIMIT     32767
#define SCROLL_LIMIT 127

static int32_t clamp_i32(int32_t v, int32_t lim)
{
	if (v > lim) {
		return lim;
	}
	if (v < -lim) {
		return -lim;
	}
	return v;
}

static void put_le16(uint8_t *p, int16_t v)
{
	uint16_t u = (uint16_t)v;

	p[0] = (uint8_t)(u & 0xFFu);
	p[1] = (uint8_t)(u >> 8);
}

static int16_t get_le16(const uint8_t *p)
{
	uint16_t u = (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));

	return (int16_t)u;
}

size_t wp_hid_mouse_report_pack(const struct wp_mouse_report *r, uint8_t *buf, size_t len)
{
	if (r == NULL || buf == NULL || len < WP_HID_MOUSE_REPORT_LEN) {
		return 0;
	}

	/* -32768 / -128 are outside the declared logical range: clamp. */
	buf[0] = (uint8_t)(r->buttons & WP_BTN_ALL);
	put_le16(&buf[1], (int16_t)clamp_i32(r->dx, XY_LIMIT));
	put_le16(&buf[3], (int16_t)clamp_i32(r->dy, XY_LIMIT));
	buf[5] = (uint8_t)(int8_t)clamp_i32(r->wheel, SCROLL_LIMIT);
	buf[6] = (uint8_t)(int8_t)clamp_i32(r->pan, SCROLL_LIMIT);

	return WP_HID_MOUSE_REPORT_LEN;
}

bool wp_hid_mouse_report_unpack(const uint8_t *buf, size_t len, struct wp_mouse_report *r)
{
	if (r == NULL || buf == NULL || len < WP_HID_MOUSE_REPORT_LEN) {
		return false;
	}

	r->buttons = (uint8_t)(buf[0] & WP_BTN_ALL);
	r->dx = get_le16(&buf[1]);
	r->dy = get_le16(&buf[3]);
	r->wheel = (int8_t)buf[5];
	r->pan = (int8_t)buf[6];

	return true;
}

/* ------------------------------------------------------------------------ */
/* Composer                                                                  */
/* ------------------------------------------------------------------------ */

/*
 * Safety saturation of the pending integer backlog. It is far beyond anything
 * the producers generate (512 full-scale X reports); it only exists so that a
 * broken producer cannot overflow int32 arithmetic.
 */
#define PEND_LIMIT  (1L << 24)
/* Largest magnitude accepted from a single add (keeps the float->int cast defined). */
#define ADD_LIMIT_F 16777216.0f

static int32_t sat_add_pend(int32_t pend, int32_t add)
{
	int64_t s = (int64_t)pend + (int64_t)add;

	if (s > PEND_LIMIT) {
		return (int32_t)PEND_LIMIT;
	}
	if (s < -PEND_LIMIT) {
		return (int32_t)-PEND_LIMIT;
	}
	return (int32_t)s;
}

/*
 * Add @p delta to the fractional remainder @p rem, move the integer part
 * (truncated toward zero, so sub-count jitter around 0 emits nothing) into
 * @p pend and keep the fraction (|rem| < 1) in @p rem.
 */
static void accumulate(float *rem, int32_t *pend, float delta)
{
	float total;
	float whole;

	if (!isfinite(delta)) {
		return;
	}

	total = *rem + delta;
	if (total > ADD_LIMIT_F) {
		total = ADD_LIMIT_F;
	} else if (total < -ADD_LIMIT_F) {
		total = -ADD_LIMIT_F;
	}

	whole = truncf(total);
	*rem = total - whole;
	*pend = sat_add_pend(*pend, (int32_t)whole);
}

/* Take at most +-lim counts from @p pend. */
static int32_t take(int32_t *pend, int32_t lim)
{
	int32_t v = clamp_i32(*pend, lim);

	*pend -= v;
	return v;
}

static bool motion_pending(const struct wp_hid_composer *c)
{
	return c->pend_x != 0 || c->pend_y != 0 || c->pend_wheel != 0 || c->pend_pan != 0;
}

void wp_hid_composer_init(struct wp_hid_composer *c)
{
	if (c == NULL) {
		return;
	}
	memset(c, 0, sizeof(*c));
}

void wp_hid_composer_add_motion(struct wp_hid_composer *c, float dx, float dy)
{
	if (c == NULL) {
		return;
	}
	accumulate(&c->rem_x, &c->pend_x, dx);
	accumulate(&c->rem_y, &c->pend_y, dy);
}

void wp_hid_composer_add_scroll(struct wp_hid_composer *c, float wheel, float pan)
{
	if (c == NULL) {
		return;
	}
	accumulate(&c->rem_wheel, &c->pend_wheel, wheel);
	accumulate(&c->rem_pan, &c->pend_pan, pan);
}

void wp_hid_composer_set_held(struct wp_hid_composer *c, uint8_t buttons)
{
	if (c == NULL) {
		return;
	}
	c->held = (uint8_t)(buttons & WP_BTN_ALL);
}

void wp_hid_composer_click(struct wp_hid_composer *c, uint8_t buttons)
{
	uint8_t slot;

	if (c == NULL) {
		return;
	}

	buttons &= WP_BTN_ALL;
	if (buttons == 0U || c->click_count >= WP_HID_CLICK_QUEUE_LEN) {
		return; /* nothing to click / queue full: dropped as documented */
	}

	slot = (uint8_t)((c->click_head + c->click_count) % WP_HID_CLICK_QUEUE_LEN);
	c->clicks[slot] = buttons;
	c->click_count++;
}

void wp_hid_composer_reset_motion(struct wp_hid_composer *c)
{
	if (c == NULL) {
		return;
	}
	c->rem_x = 0.0f;
	c->rem_y = 0.0f;
	c->rem_wheel = 0.0f;
	c->rem_pan = 0.0f;
	c->pend_x = 0;
	c->pend_y = 0;
	c->pend_wheel = 0;
	c->pend_pan = 0;
}

void wp_hid_composer_release_all(struct wp_hid_composer *c)
{
	if (c == NULL) {
		return;
	}
	wp_hid_composer_reset_motion(c);
	c->held = 0U;
	c->click_head = 0U;
	c->click_count = 0U;
	c->click_pressed = false;
	memset(c->clicks, 0, sizeof(c->clicks));
	/* last_sent_buttons is kept: if it is non-zero, pending() is now true
	 * and the next report carries buttons == 0.
	 */
}

void wp_hid_composer_force_sync(struct wp_hid_composer *c)
{
	if (c == NULL) {
		return;
	}
	/* Impossible value (bits 5-7 are padding): next() always differs. */
	c->last_sent_buttons = 0xFFU;
}

bool wp_hid_composer_pending(const struct wp_hid_composer *c)
{
	if (c == NULL) {
		return false;
	}
	if (c->click_count > 0U) {
		return true; /* a PRESS or a RELEASE is due */
	}
	return motion_pending(c) || c->held != c->last_sent_buttons;
}

bool wp_hid_composer_next(struct wp_hid_composer *c, struct wp_mouse_report *r)
{
	struct wp_mouse_report out = {0};

	if (c == NULL || r == NULL) {
		return false;
	}

	if (c->click_count > 0U && !c->click_pressed) {
		/* PRESS: held | click buttons, no motion (pending motion is kept). */
		out.buttons = (uint8_t)(c->held | c->clicks[c->click_head]);
		c->click_pressed = true;
	} else {
		bool release = false;

		if (c->click_count > 0U) {
			/* RELEASE of the click at the head: back to the held buttons. */
			c->clicks[c->click_head] = 0U;
			c->click_head = (uint8_t)((c->click_head + 1U) % WP_HID_CLICK_QUEUE_LEN);
			c->click_count--;
			c->click_pressed = false;
			release = true;
		}

		out.buttons = c->held;
		out.dx = (int16_t)take(&c->pend_x, XY_LIMIT);
		out.dy = (int16_t)take(&c->pend_y, XY_LIMIT);
		out.wheel = (int8_t)take(&c->pend_wheel, SCROLL_LIMIT);
		out.pan = (int8_t)take(&c->pend_pan, SCROLL_LIMIT);

		if (!release && out.dx == 0 && out.dy == 0 && out.wheel == 0 && out.pan == 0 &&
		    out.buttons == c->last_sent_buttons) {
			return false; /* nothing changed */
		}
	}

	c->last_sent_buttons = out.buttons;
	*r = out;
	return true;
}
