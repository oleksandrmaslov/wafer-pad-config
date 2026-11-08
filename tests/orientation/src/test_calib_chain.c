/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Calibration gesture with the SHIPPED defaults, driven like the router does
 * (src/router/router.c handle_touch() + housekeeping()):
 *   orient_mgr_on_touch -> calib_update -> (COMPLETED: set_manual)
 *   -> gesture scroll inhibit -> xform_frame -> gesture_process,
 * then gesture/calib/manager ticks.
 *
 * Regression (ALG-1): "rest two fingers, then flick-scroll" must scroll and
 * must never change the orientation.
 */

#include <zephyr/ztest.h>

#include <wafer_pad/gestures.h>

#include "helpers.h"

#define CX 650
#define CY 490
#define FRAME_MS 20

static const struct wp_pad_geometry pad = {
	.max_x = CX, .max_y = CY, .width_mm = 65.0f, .height_mm = 49.0f};

static struct wp_orient_mgr mgr;
static struct wp_calib cal;
static struct wp_gesture_engine gest;
static enum wp_orientation cur;
static float wheel;
static int completed;
static int armed;
static int changes;

static void chain_reset(void)
{
	struct wp_orient_mgr_config ocfg;
	struct wp_pad_geometry ug;

	wp_orient_mgr_config_default(&ocfg);
	cur = WP_ORIENT_0;
	wp_orient_mgr_init(&mgr, &ocfg, cur);
	wp_calib_init(&cal, NULL, &pad); /* shipped defaults */
	wp_xform_geometry(cur, &pad, &ug);
	wp_gesture_init(&gest, NULL, &ug);
	wheel = 0.0f;
	completed = 0;
	armed = 0;
	changes = 0;
}

static void housekeeping(int64_t t)
{
	struct wp_gesture_output go;
	struct wp_calib_result cr;
	enum wp_orientation o;

	wp_gesture_tick(&gest, t, &go);
	wheel += go.wheel;
	wp_calib_tick(&cal, t, &cr);
	wp_gesture_set_scroll_inhibit(&gest, wp_calib_inhibits_scroll(&cal));
	wp_orient_mgr_tick(&mgr, t);
	if (wp_orient_mgr_poll_change(&mgr, &o)) {
		changes++;
		cur = o;
	}
}

static void feed(const struct wp_touch_frame *pf)
{
	struct wp_calib_result cr;
	struct wp_touch_frame uf;
	struct wp_gesture_output go;

	wp_orient_mgr_on_touch(&mgr, pf->count, pf->t_us);
	wp_calib_update(&cal, pf, &cr);
	if (cr.event == WP_CALIB_EV_ARMED) {
		armed++;
	} else if (cr.event == WP_CALIB_EV_COMPLETED) {
		completed++;
		wp_orient_mgr_set_manual(&mgr, cr.orientation, pf->t_us);
	}
	wp_gesture_set_scroll_inhibit(&gest, wp_calib_inhibits_scroll(&cal));
	wp_xform_frame(cur, pf, &uf);
	wp_gesture_process(&gest, &uf, &go);
	wheel += go.wheel;
	housekeeping(pf->t_us);
}

/* n fingers 6 mm apart in x, starting at (x0, y0) counts, moved by dy counts. */
static struct wp_touch_frame fingers(int64_t t, uint8_t n, uint16_t x0, uint16_t y0)
{
	uint16_t xy[2 * WP_TOUCH_MAX_CONTACTS];

	for (uint8_t i = 0; i < n; i++) {
		xy[2 * i] = (uint16_t)(x0 + 60U * i);
		xy[2 * i + 1] = y0;
	}
	return frame_n(t, CX, CY, n, xy);
}

/* Rest n fingers for rest_ms, flick dy_counts per 20 ms frame for k frames, lift. */
static void rest_flick_lift(uint8_t n, int rest_ms, int dy_counts, int k, uint16_t y0)
{
	int64_t t = MS(1000);
	uint16_t y = y0;
	struct wp_touch_frame f;

	for (int ms = 0; ms <= rest_ms; ms += FRAME_MS, t += MS(FRAME_MS)) {
		f = fingers(t, n, 260, y);
		feed(&f);
	}
	for (int i = 0; i < k; i++, t += MS(FRAME_MS)) {
		y = (uint16_t)(y + dy_counts);
		f = fingers(t, n, 260, y);
		feed(&f);
	}
	f = frame0(t, CX, CY);
	feed(&f);
	for (int i = 0; i < 100; i++) {
		t += MS(FRAME_MS);
		housekeeping(t);
	}
}

ZTEST(orient_calib_chain, test_rest_then_flick_scroll_never_calibrates)
{
	static const int rests[] = {300, 690, 750, 800, 1000, 2000};

	for (size_t i = 0; i < ARRAY_SIZE(rests); i++) {
		/* 20 mm towards the user in 200 ms (100 mm/s), then lift. */
		chain_reset();
		rest_flick_lift(2, rests[i], 20, 10, 150);
		zassert_equal(armed, 0, "rest %d ms armed the calibration", rests[i]);
		zassert_equal(completed, 0, "rest %d ms calibrated", rests[i]);
		zassert_equal(changes, 0, "rest %d ms changed the orientation", rests[i]);
		zassert_equal(wp_orient_mgr_get(&mgr), WP_ORIENT_0);
		zassert_true(wheel > 3.0f || wheel < -3.0f, "rest %d ms: scroll swallowed (%d/100)",
			     rests[i], (int)(wheel * 100.0f));
	}
}

ZTEST(orient_calib_chain, test_three_finger_hold_and_swipe_calibrates)
{
	/* The deliberate gesture: 3 fingers, 1.6 s hold, 30 mm in 300 ms. */
	chain_reset();
	rest_flick_lift(3, 1600, 20, 15, 80);
	zassert_equal(armed, 1);
	zassert_equal(completed, 1);
	zassert_equal(changes, 1);
	zassert_equal(wp_orient_mgr_get(&mgr), WP_ORIENT_180);

	/* Same swipe after a short (1 s) hold: just a 3-finger touch. */
	chain_reset();
	rest_flick_lift(3, 1000, 20, 15, 80);
	zassert_equal(completed, 0);
	zassert_equal(changes, 0);
}

ZTEST_SUITE(orient_calib_chain, NULL, NULL, NULL, NULL, NULL);
