/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for src/hid/hid_report.c: report map sanity, report packing and
 * the report composer semantics documented in include/wafer_pad/hid_report.h.
 */

#include <math.h>
#include <string.h>

#include <zephyr/ztest.h>

#include <wafer_pad/hid_report.h>

/* ------------------------------------------------------------------------ */
/* Minimal HID short-item parser                                             */
/* ------------------------------------------------------------------------ */

#define MAX_IDS 8

struct desc_stats {
	bool ok;               /* well-formed (no truncated items, no long items) */
	int depth_min;         /* lowest nesting depth seen after an End Collection */
	int depth_end;         /* nesting depth at the end */
	int collections;
	int end_collections;
	int max_depth;
	int input_items;
	int inputs_without_id; /* Input items before any Report ID */
	uint32_t input_bits[MAX_IDS]; /* indexed by report ID */
	int first_collection_type;
};

static uint32_t item_udata(const uint8_t *p, int size)
{
	uint32_t v = 0;

	for (int i = 0; i < size; i++) {
		v |= (uint32_t)p[i] << (8 * i);
	}
	return v;
}

static void parse_desc(const uint8_t *d, size_t len, struct desc_stats *s)
{
	uint32_t report_size = 0, report_count = 0, report_id = 0;
	int depth = 0;
	size_t i = 0;

	memset(s, 0, sizeof(*s));
	s->ok = true;
	s->first_collection_type = -1;

	while (i < len) {
		uint8_t prefix = d[i];
		int size = prefix & 0x03;

		if (prefix == 0xFE) { /* long item: not used in our descriptor */
			s->ok = false;
			return;
		}
		if (size == 3) {
			size = 4;
		}
		if (i + 1 + size > len) {
			s->ok = false;
			return;
		}

		uint32_t val = item_udata(&d[i + 1], size);
		uint8_t tag = prefix & 0xFC;

		switch (tag) {
		case 0xA0: /* Collection */
			if (s->first_collection_type < 0) {
				s->first_collection_type = (int)val;
			}
			s->collections++;
			depth++;
			if (depth > s->max_depth) {
				s->max_depth = depth;
			}
			break;
		case 0xC0: /* End Collection */
			s->end_collections++;
			depth--;
			if (depth < s->depth_min) {
				s->depth_min = depth;
			}
			break;
		case 0x74: /* Report Size */
			report_size = val;
			break;
		case 0x94: /* Report Count */
			report_count = val;
			break;
		case 0x84: /* Report ID */
			report_id = val;
			break;
		case 0x80: /* Input */
			s->input_items++;
			if (report_id == 0) {
				s->inputs_without_id++;
			}
			if (report_id < MAX_IDS) {
				s->input_bits[report_id] += report_size * report_count;
			}
			break;
		default:
			break;
		}
		i += 1 + size;
	}
	s->depth_end = depth;
}

static bool contains(const uint8_t *hay, size_t hay_len, const uint8_t *needle, size_t n)
{
	for (size_t i = 0; i + n <= hay_len; i++) {
		if (memcmp(&hay[i], needle, n) == 0) {
			return true;
		}
	}
	return false;
}

/* ------------------------------------------------------------------------ */
/* Descriptor                                                                */
/* ------------------------------------------------------------------------ */

ZTEST(hid_desc, test_collections_balanced)
{
	struct desc_stats s;

	parse_desc(wp_hid_report_map, wp_hid_report_map_len, &s);
	zassert_true(s.ok, "descriptor is not a sequence of complete short items");
	zassert_equal(s.collections, 2, "Application + Physical expected");
	zassert_equal(s.collections, s.end_collections, "unbalanced collections");
	zassert_equal(s.depth_end, 0);
	zassert_true(s.depth_min >= 0, "End Collection without Collection");
	zassert_equal(s.max_depth, 2);
	zassert_equal(s.first_collection_type, 0x01, "outer collection must be Application");
	/* Must open with Usage Page (Generic Desktop), Usage (Mouse). */
	zassert_equal(wp_hid_report_map[0], 0x05);
	zassert_equal(wp_hid_report_map[1], 0x01);
	zassert_equal(wp_hid_report_map[2], 0x09);
	zassert_equal(wp_hid_report_map[3], 0x02);
	zassert_equal(wp_hid_report_map[wp_hid_report_map_len - 1], 0xC0);
}

ZTEST(hid_desc, test_report1_is_56_bits)
{
	struct desc_stats s;

	parse_desc(wp_hid_report_map, wp_hid_report_map_len, &s);
	zassert_equal(s.inputs_without_id, 0, "Input item before the Report ID");
	zassert_equal(s.input_bits[WP_HID_REPORT_ID_MOUSE], 56U, "got %u bits",
		      s.input_bits[WP_HID_REPORT_ID_MOUSE]);
	zassert_equal(s.input_bits[WP_HID_REPORT_ID_MOUSE] / 8U, WP_HID_MOUSE_REPORT_LEN);
	/* No other report IDs in v0.1 (2 and 3 are reserved for the keyboard). */
	for (int id = 0; id < MAX_IDS; id++) {
		if (id != WP_HID_REPORT_ID_MOUSE) {
			zassert_equal(s.input_bits[id], 0U, "unexpected input in report %d", id);
		}
	}

	static const uint8_t report_id_item[] = {0x85, WP_HID_REPORT_ID_MOUSE};

	zassert_true(contains(wp_hid_report_map, wp_hid_report_map_len, report_id_item,
			      sizeof(report_id_item)));
}

ZTEST(hid_desc, test_usages_present)
{
	static const uint8_t ac_pan[] = {0x05, 0x0C, 0x0A, 0x38, 0x02};
	static const uint8_t xy_min[] = {0x16, 0x01, 0x80};
	static const uint8_t xy_max[] = {0x26, 0xFF, 0x7F};
	static const uint8_t xy[] = {0x09, 0x30, 0x09, 0x31};
	static const uint8_t wheel[] = {0x09, 0x38};
	static const uint8_t buttons[] = {0x05, 0x09, 0x19, 0x01, 0x29, 0x05};

	zassert_true(contains(wp_hid_report_map, wp_hid_report_map_len, ac_pan, sizeof(ac_pan)),
		     "Consumer page + AC Pan (0x0A 0x38 0x02) missing");
	zassert_true(contains(wp_hid_report_map, wp_hid_report_map_len, xy_min, sizeof(xy_min)));
	zassert_true(contains(wp_hid_report_map, wp_hid_report_map_len, xy_max, sizeof(xy_max)));
	zassert_true(contains(wp_hid_report_map, wp_hid_report_map_len, xy, sizeof(xy)));
	zassert_true(contains(wp_hid_report_map, wp_hid_report_map_len, wheel, sizeof(wheel)));
	zassert_true(contains(wp_hid_report_map, wp_hid_report_map_len, buttons,
			      sizeof(buttons)));
}

ZTEST_SUITE(hid_desc, NULL, NULL, NULL, NULL, NULL);

/* ------------------------------------------------------------------------ */
/* Packing                                                                   */
/* ------------------------------------------------------------------------ */

ZTEST(hid_pack, test_layout_little_endian)
{
	struct wp_mouse_report r = {
		.buttons = WP_BTN_LEFT | WP_BTN_FORWARD, .dx = 0x1234, .dy = -2, .wheel = -1,
		.pan = 5,
	};
	uint8_t buf[WP_HID_MOUSE_REPORT_LEN + 2];
	static const uint8_t expect[] = {0x11, 0x34, 0x12, 0xFE, 0xFF, 0xFF, 0x05};

	memset(buf, 0xAA, sizeof(buf));
	zassert_equal(wp_hid_mouse_report_pack(&r, buf, sizeof(buf)), WP_HID_MOUSE_REPORT_LEN);
	zassert_mem_equal(buf, expect, sizeof(expect));
	zassert_equal(buf[7], 0xAA, "pack wrote past the payload");
}

ZTEST(hid_pack, test_round_trip)
{
	static const struct wp_mouse_report cases[] = {
		{0, 0, 0, 0, 0},
		{WP_BTN_ALL, 32767, -32767, 127, -127},
		{WP_BTN_RIGHT, -1, 1, -1, 1},
		{WP_BTN_MIDDLE | WP_BTN_BACK, -300, 4000, 64, -64},
		{WP_BTN_LEFT, -32767, 32767, -127, 127},
	};

	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		uint8_t buf[WP_HID_MOUSE_REPORT_LEN];
		struct wp_mouse_report out;

		memset(&out, 0x5A, sizeof(out));
		zassert_equal(wp_hid_mouse_report_pack(&cases[i], buf, sizeof(buf)),
			      WP_HID_MOUSE_REPORT_LEN);
		zassert_true(wp_hid_mouse_report_unpack(buf, sizeof(buf), &out));
		zassert_equal(out.buttons, cases[i].buttons, "case %zu", i);
		zassert_equal(out.dx, cases[i].dx, "case %zu", i);
		zassert_equal(out.dy, cases[i].dy, "case %zu", i);
		zassert_equal(out.wheel, cases[i].wheel, "case %zu", i);
		zassert_equal(out.pan, cases[i].pan, "case %zu", i);
	}
}

ZTEST(hid_pack, test_out_of_range_and_short_buffers)
{
	struct wp_mouse_report r = {.buttons = 0xFF, .dx = -32768, .dy = -32768, .wheel = -128,
				    .pan = -128};
	struct wp_mouse_report out;
	uint8_t buf[WP_HID_MOUSE_REPORT_LEN];

	/* Values outside the logical range are clamped, padding bits are 0. */
	zassert_equal(wp_hid_mouse_report_pack(&r, buf, sizeof(buf)), WP_HID_MOUSE_REPORT_LEN);
	zassert_true(wp_hid_mouse_report_unpack(buf, sizeof(buf), &out));
	zassert_equal(buf[0], WP_BTN_ALL);
	zassert_equal(out.dx, -32767);
	zassert_equal(out.dy, -32767);
	zassert_equal(out.wheel, -127);
	zassert_equal(out.pan, -127);

	zassert_equal(wp_hid_mouse_report_pack(&r, buf, WP_HID_MOUSE_REPORT_LEN - 1), 0U);
	zassert_equal(wp_hid_mouse_report_pack(NULL, buf, sizeof(buf)), 0U);
	zassert_equal(wp_hid_mouse_report_pack(&r, NULL, sizeof(buf)), 0U);
	zassert_false(wp_hid_mouse_report_unpack(buf, WP_HID_MOUSE_REPORT_LEN - 1, &out));
	zassert_false(wp_hid_mouse_report_unpack(NULL, sizeof(buf), &out));
}

ZTEST_SUITE(hid_pack, NULL, NULL, NULL, NULL, NULL);

/* ------------------------------------------------------------------------ */
/* Composer                                                                  */
/* ------------------------------------------------------------------------ */

static struct wp_hid_composer comp;

static void composer_before(void *f)
{
	ARG_UNUSED(f);
	wp_hid_composer_init(&comp);
}

/* Drain everything; returns the number of reports and the summed motion. */
struct drain_sum {
	int reports;
	int32_t x, y, wheel, pan;
};

static struct drain_sum drain(struct wp_hid_composer *c)
{
	struct drain_sum s = {0};
	struct wp_mouse_report r;

	while (wp_hid_composer_next(c, &r)) {
		s.reports++;
		s.x += r.dx;
		s.y += r.dy;
		s.wheel += r.wheel;
		s.pan += r.pan;
		zassert_true(s.reports < 1000, "composer never runs dry");
	}
	zassert_false(wp_hid_composer_pending(c), "pending() true after next() returned false");
	return s;
}

ZTEST(hid_composer, test_idle_produces_nothing)
{
	struct wp_mouse_report r;

	zassert_false(wp_hid_composer_pending(&comp));
	zassert_false(wp_hid_composer_next(&comp, &r));
	wp_hid_composer_add_motion(&comp, 0.0f, 0.0f);
	wp_hid_composer_add_scroll(&comp, 0.0f, 0.0f);
	zassert_false(wp_hid_composer_pending(&comp));
}

ZTEST(hid_composer, test_fractional_accumulation)
{
	struct wp_mouse_report r;

	/* 3 x 0.25 stays below one count. */
	for (int i = 0; i < 3; i++) {
		wp_hid_composer_add_motion(&comp, 0.25f, -0.25f);
		zassert_false(wp_hid_composer_pending(&comp), "emitted at step %d", i);
	}
	/* The 4th quarter completes a count on both axes. */
	wp_hid_composer_add_motion(&comp, 0.25f, -0.25f);
	zassert_true(wp_hid_composer_pending(&comp));
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.dx, 1);
	zassert_equal(r.dy, -1);
	zassert_equal(r.buttons, 0);
	zassert_false(wp_hid_composer_next(&comp, &r));

	/* No drift: 1000 x 0.3 = 300 counts in total (+-1 for float rounding). */
	int32_t sum = 0;

	for (int i = 0; i < 1000; i++) {
		wp_hid_composer_add_motion(&comp, 0.3f, 0.0f);
		while (wp_hid_composer_next(&comp, &r)) {
			sum += r.dx;
		}
	}
	zassert_within(sum, 300, 1, "sum %d", sum);
	zassert_within((float)sum + comp.rem_x, 300.0f, 0.01f);
}

ZTEST(hid_composer, test_sub_count_jitter_cancels)
{
	/* Truncation toward zero: +-0.4 jitter never emits a report. */
	for (int i = 0; i < 50; i++) {
		wp_hid_composer_add_motion(&comp, 0.4f, -0.4f);
		zassert_false(wp_hid_composer_pending(&comp));
		wp_hid_composer_add_motion(&comp, -0.4f, 0.4f);
		zassert_false(wp_hid_composer_pending(&comp));
	}

	/* Negative fractions accumulate symmetrically. */
	struct wp_mouse_report r;

	wp_hid_composer_add_motion(&comp, -0.6f, 0.0f);
	zassert_false(wp_hid_composer_pending(&comp));
	wp_hid_composer_add_motion(&comp, -0.6f, 0.0f);
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.dx, -1);
	zassert_within(comp.rem_x, -0.2f, 1e-4f);
}

ZTEST(hid_composer, test_large_motion_is_split_not_clipped)
{
	struct wp_mouse_report r;
	static const int16_t expect[] = {32767, 32767, 32767, 100000 - 3 * 32767};

	wp_hid_composer_add_motion(&comp, 100000.0f, -100000.0f);
	for (size_t i = 0; i < ARRAY_SIZE(expect); i++) {
		zassert_true(wp_hid_composer_next(&comp, &r), "report %zu missing", i);
		zassert_equal(r.dx, expect[i], "report %zu: dx %d", i, r.dx);
		zassert_equal(r.dy, -expect[i], "report %zu: dy %d", i, r.dy);
		zassert_equal(r.wheel, 0);
		zassert_equal(r.pan, 0);
	}
	zassert_false(wp_hid_composer_pending(&comp));
	zassert_false(wp_hid_composer_next(&comp, &r));
}

ZTEST(hid_composer, test_wheel_and_pan_split)
{
	struct wp_mouse_report r;
	static const int8_t expect[] = {127, 127, 46};

	wp_hid_composer_add_scroll(&comp, 300.0f, -300.0f);
	for (size_t i = 0; i < ARRAY_SIZE(expect); i++) {
		zassert_true(wp_hid_composer_next(&comp, &r));
		zassert_equal(r.wheel, expect[i], "report %zu: wheel %d", i, r.wheel);
		zassert_equal(r.pan, -expect[i], "report %zu: pan %d", i, r.pan);
		zassert_equal(r.dx, 0);
		zassert_equal(r.dy, 0);
	}
	zassert_false(wp_hid_composer_next(&comp, &r));

	/* Fractional scroll: 2 x 0.5 detent = 1 detent. */
	wp_hid_composer_add_scroll(&comp, 0.5f, 0.0f);
	zassert_false(wp_hid_composer_pending(&comp));
	wp_hid_composer_add_scroll(&comp, 0.5f, 0.0f);
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.wheel, 1);

	/* Motion and scroll split independently but are fully delivered. */
	wp_hid_composer_add_motion(&comp, 40000.0f, 5.0f);
	wp_hid_composer_add_scroll(&comp, -1000.0f, 3.0f);
	struct drain_sum s = drain(&comp);

	zassert_equal(s.x, 40000);
	zassert_equal(s.y, 5);
	zassert_equal(s.wheel, -1000);
	zassert_equal(s.pan, 3);
	zassert_equal(s.reports, 8, "1000 / 127 needs 8 reports, got %d", s.reports);
}

ZTEST(hid_composer, test_click_press_then_release)
{
	struct wp_mouse_report r;

	wp_hid_composer_add_motion(&comp, 5.0f, -3.0f);
	wp_hid_composer_click(&comp, WP_BTN_LEFT);
	zassert_true(wp_hid_composer_pending(&comp));

	/* PRESS: no motion even though motion is pending. */
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, WP_BTN_LEFT);
	zassert_equal(r.dx, 0);
	zassert_equal(r.dy, 0);
	zassert_equal(r.wheel, 0);
	zassert_equal(r.pan, 0);

	/* RELEASE: buttons back to 0, carries the pending motion. */
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, 0);
	zassert_equal(r.dx, 5);
	zassert_equal(r.dy, -3);

	zassert_false(wp_hid_composer_pending(&comp));
	zassert_false(wp_hid_composer_next(&comp, &r));
}

ZTEST(hid_composer, test_click_without_motion_and_double_click)
{
	struct wp_mouse_report r;

	/* A click with nothing else pending still yields exactly two reports. */
	wp_hid_composer_click(&comp, WP_BTN_RIGHT);
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, WP_BTN_RIGHT);
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, 0);
	zassert_false(wp_hid_composer_next(&comp, &r));

	/* Double click: P R P R. */
	wp_hid_composer_click(&comp, WP_BTN_LEFT);
	wp_hid_composer_click(&comp, WP_BTN_LEFT);
	static const uint8_t seq[] = {WP_BTN_LEFT, 0, WP_BTN_LEFT, 0};

	for (size_t i = 0; i < ARRAY_SIZE(seq); i++) {
		zassert_true(wp_hid_composer_next(&comp, &r));
		zassert_equal(r.buttons, seq[i], "report %zu", i);
	}
	zassert_false(wp_hid_composer_next(&comp, &r));

	/* Queue holds WP_HID_CLICK_QUEUE_LEN clicks, extra clicks are dropped;
	 * a click of no buttons is ignored.
	 */
	wp_hid_composer_click(&comp, 0);
	zassert_false(wp_hid_composer_pending(&comp));
	for (int i = 0; i < WP_HID_CLICK_QUEUE_LEN + 2; i++) {
		wp_hid_composer_click(&comp, WP_BTN_MIDDLE);
	}
	struct drain_sum s = drain(&comp);

	zassert_equal(s.reports, 2 * WP_HID_CLICK_QUEUE_LEN);
}

ZTEST(hid_composer, test_held_buttons)
{
	struct wp_mouse_report r;

	/* Press-and-hold (drag): one report on change, motion carries the button. */
	wp_hid_composer_set_held(&comp, WP_BTN_LEFT);
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, WP_BTN_LEFT);
	zassert_equal(r.dx, 0);
	zassert_false(wp_hid_composer_next(&comp, &r), "unchanged held state re-sent");

	wp_hid_composer_add_motion(&comp, 10.0f, 2.0f);
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, WP_BTN_LEFT);
	zassert_equal(r.dx, 10);
	zassert_equal(r.dy, 2);

	/* Click of another button while dragging: PRESS = held | click. */
	wp_hid_composer_click(&comp, WP_BTN_RIGHT);
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, WP_BTN_LEFT | WP_BTN_RIGHT);
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, WP_BTN_LEFT);

	/* Releasing the hold produces a buttons == 0 report, then nothing. */
	wp_hid_composer_set_held(&comp, 0);
	zassert_true(wp_hid_composer_pending(&comp));
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, 0);
	zassert_false(wp_hid_composer_next(&comp, &r));

	/* Bits outside the 5 buttons are ignored. */
	wp_hid_composer_set_held(&comp, 0xE0);
	zassert_false(wp_hid_composer_pending(&comp));
}

ZTEST(hid_composer, test_release_all)
{
	struct wp_mouse_report r;

	/* Nothing pressed: release_all produces no report. */
	wp_hid_composer_add_motion(&comp, 3.5f, 0.0f);
	wp_hid_composer_release_all(&comp);
	zassert_false(wp_hid_composer_pending(&comp));

	/* Held button sent, then motion + a queued click, then release_all:
	 * next report has buttons == 0 and no motion, then silence.
	 */
	wp_hid_composer_set_held(&comp, WP_BTN_LEFT);
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, WP_BTN_LEFT);
	wp_hid_composer_add_motion(&comp, 50.0f, 50.0f);
	wp_hid_composer_click(&comp, WP_BTN_RIGHT);
	wp_hid_composer_release_all(&comp);
	zassert_true(wp_hid_composer_pending(&comp));
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, 0);
	zassert_equal(r.dx, 0);
	zassert_equal(r.dy, 0);
	zassert_false(wp_hid_composer_next(&comp, &r));

	/* release_all between the PRESS and RELEASE of a click. */
	wp_hid_composer_click(&comp, WP_BTN_LEFT);
	wp_hid_composer_click(&comp, WP_BTN_RIGHT);
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, WP_BTN_LEFT);
	wp_hid_composer_release_all(&comp);
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, 0);
	zassert_false(wp_hid_composer_next(&comp, &r), "dropped click still emitted");

	/* Fractions are dropped too. */
	wp_hid_composer_add_motion(&comp, 0.9f, 0.0f);
	wp_hid_composer_release_all(&comp);
	wp_hid_composer_add_motion(&comp, 0.2f, 0.0f);
	zassert_false(wp_hid_composer_pending(&comp));
}

ZTEST(hid_composer, test_reset_motion)
{
	struct wp_mouse_report r;

	/* Pending integer motion and remainders are dropped. */
	wp_hid_composer_add_motion(&comp, 1000.7f, -20.0f);
	wp_hid_composer_add_scroll(&comp, 3.6f, -2.0f);
	wp_hid_composer_reset_motion(&comp);
	zassert_false(wp_hid_composer_pending(&comp));
	wp_hid_composer_add_motion(&comp, 0.5f, 0.0f);
	wp_hid_composer_add_scroll(&comp, 0.5f, 0.0f);
	zassert_false(wp_hid_composer_pending(&comp), "remainder survived reset_motion");

	/* Buttons are not affected: held state and queued clicks survive. */
	wp_hid_composer_set_held(&comp, WP_BTN_LEFT);
	wp_hid_composer_click(&comp, WP_BTN_RIGHT);
	wp_hid_composer_add_motion(&comp, 7.0f, 0.0f);
	wp_hid_composer_reset_motion(&comp);
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, WP_BTN_LEFT | WP_BTN_RIGHT);
	zassert_true(wp_hid_composer_next(&comp, &r));
	zassert_equal(r.buttons, WP_BTN_LEFT);
	zassert_equal(r.dx, 0, "motion survived reset_motion");
	zassert_false(wp_hid_composer_next(&comp, &r));
}

ZTEST(hid_composer, test_non_finite_input_ignored)
{
	wp_hid_composer_add_motion(&comp, NAN, INFINITY);
	wp_hid_composer_add_scroll(&comp, -INFINITY, NAN);
	zassert_false(wp_hid_composer_pending(&comp));

	/* Absurd values saturate instead of overflowing; motion still flows. */
	wp_hid_composer_add_motion(&comp, 1e30f, -1e30f);
	zassert_true(wp_hid_composer_pending(&comp));
	zassert_true(comp.pend_x > 0);
	zassert_true(comp.pend_y < 0);
	wp_hid_composer_reset_motion(&comp);
	zassert_false(wp_hid_composer_pending(&comp));

	/* NULL safety. */
	wp_hid_composer_init(NULL);
	wp_hid_composer_add_motion(NULL, 1.0f, 1.0f);
	zassert_false(wp_hid_composer_pending(NULL));
	zassert_false(wp_hid_composer_next(&comp, NULL));
}

ZTEST(hid_composer, test_force_sync_resends_button_state)
{
	/* BLE-6: a RELEASE produced by the composer but dropped by the
	 * transport (link not ready for a moment) must be re-sent when the
	 * link becomes ready again. */
	struct wp_hid_composer c;
	struct wp_mouse_report r;

	wp_hid_composer_init(&c);
	wp_hid_composer_set_held(&c, WP_BTN_LEFT);
	zassert_true(wp_hid_composer_next(&c, &r));
	zassert_equal(r.buttons, WP_BTN_LEFT);

	wp_hid_composer_set_held(&c, 0U);
	zassert_true(wp_hid_composer_next(&c, &r)); /* RELEASE ... dropped */
	zassert_equal(r.buttons, 0U);

	/* Link ready again: the router clears stale state and forces a sync. */
	wp_hid_composer_release_all(&c);
	zassert_false(wp_hid_composer_pending(&c), "premise: nothing pending");
	wp_hid_composer_force_sync(&c);
	zassert_true(wp_hid_composer_pending(&c));
	zassert_true(wp_hid_composer_next(&c, &r));
	zassert_equal(r.buttons, 0U);
	zassert_equal(r.dx, 0);
	zassert_equal(r.dy, 0);
	zassert_false(wp_hid_composer_next(&c, &r), "exactly one sync report");

	/* With a drag still held, the sync report re-asserts it. */
	wp_hid_composer_set_held(&c, WP_BTN_LEFT);
	zassert_true(wp_hid_composer_next(&c, &r));
	wp_hid_composer_force_sync(&c);
	zassert_true(wp_hid_composer_next(&c, &r));
	zassert_equal(r.buttons, WP_BTN_LEFT);
	zassert_false(wp_hid_composer_next(&c, &r));

	wp_hid_composer_force_sync(NULL); /* tolerated */
}

ZTEST_SUITE(hid_composer, NULL, NULL, composer_before, NULL, NULL);
