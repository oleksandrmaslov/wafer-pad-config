/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests: gesture engine + pointer acceleration (pure C).
 */

#include <math.h>
#include <string.h>

#include <zephyr/ztest.h>

#include <wafer_pad/gestures.h>
#include <wafer_pad/orientation_transform.h>

/* User-frame test pad: 0.05 mm per count on both axes (20 counts per mm). */
#define UMAX_X 1280U
#define UMAX_Y 960U
static const struct wp_pad_geometry geom = {
	.max_x = UMAX_X, .max_y = UMAX_Y, .width_mm = 64.0f, .height_mm = 48.0f};

#define MS(x)   ((int64_t)(x) * 1000LL)
#define EPS     0.01f
#define CPM     16.0f /* default counts per mm */

struct pt {
	uint8_t id;
	float x;
	float y;
};

struct acc {
	float dx, dy, wheel, pan;
	float max_step;       /* largest |dx| or |dy| of any single call */
	float max_wheel_step; /* largest |wheel| or |pan| of any single call */
	int left_clicks;
	int right_clicks;
	uint32_t ev;
	uint8_t buttons;
};

static struct wp_gesture_engine eng;
static struct wp_gesture_output out;
static struct acc A;

static uint16_t cnt(float mm)
{
	return (uint16_t)(mm * 20.0f + 0.5f);
}

static void acc_clear(void)
{
	memset(&A, 0, sizeof(A));
}

static void accumulate(void)
{
	float s = fmaxf(fabsf(out.dx), fabsf(out.dy));
	float w = fmaxf(fabsf(out.wheel), fabsf(out.pan));

	A.dx += out.dx;
	A.dy += out.dy;
	A.wheel += out.wheel;
	A.pan += out.pan;
	A.max_step = fmaxf(A.max_step, s);
	A.max_wheel_step = fmaxf(A.max_wheel_step, w);
	A.left_clicks += (out.click & WP_BTN_LEFT) ? 1 : 0;
	A.right_clicks += (out.click & WP_BTN_RIGHT) ? 1 : 0;
	A.ev |= out.events;
	A.buttons = out.buttons;
}

static void frame_n(int64_t t_us, uint8_t n, const struct pt *p)
{
	struct wp_touch_frame f = {0};

	f.t_us = t_us;
	f.max_x = UMAX_X;
	f.max_y = UMAX_Y;
	f.count = n;
	for (uint8_t i = 0; i < n; i++) {
		f.contacts[i].id = p[i].id;
		f.contacts[i].x = cnt(p[i].x);
		f.contacts[i].y = cnt(p[i].y);
		f.contacts[i].strength = 200;
	}
	wp_gesture_process(&eng, &f, &out);
	accumulate();
}

static void frame1(int64_t t, float x, float y)
{
	struct pt p = {0, x, y};

	frame_n(t, 1, &p);
}

static void frame2(int64_t t, float x0, float y0, float x1, float y1)
{
	struct pt p[2] = {{0, x0, y0}, {1, x1, y1}};

	frame_n(t, 2, p);
}

static void lift(int64_t t)
{
	frame_n(t, 0, NULL);
}

static void tick(int64_t t)
{
	wp_gesture_tick(&eng, t, &out);
	accumulate();
}

/* Defaults, but with deterministic pointer maths: no deadband, no smoothing,
 * no acceleration (counts = mm * 16). */
static void plain_cfg(struct wp_gesture_config *c)
{
	wp_gesture_config_default(c);
	c->jitter_deadband_mm = 0.0f;
	c->smoothing_alpha = 1.0f;
	c->accel.max_multiplier = 1.0f;
}

static void setup(const struct wp_gesture_config *c)
{
	wp_gesture_init(&eng, c, &geom);
	memset(&out, 0, sizeof(out));
	acc_clear();
}

static void setup_plain(void)
{
	struct wp_gesture_config c;

	plain_cfg(&c);
	setup(&c);
}

/* One finger moving in a straight line: @p steps frames, 10 ms apart, each
 * moving (sx, sy) mm. Returns the final position via x/y. */
static int64_t swipe1(int64_t t, float *x, float *y, float sx, float sy, int steps)
{
	for (int i = 0; i < steps; i++) {
		t += MS(10);
		*x += sx;
		*y += sy;
		frame1(t, *x, *y);
	}
	return t;
}

static void before(void *f)
{
	ARG_UNUSED(f);
	setup(NULL);
}

ZTEST_SUITE(gestures, NULL, NULL, before, NULL, NULL);

/* ------------------------------------------------------------------------ */
/* Defaults and pointer acceleration                                         */
/* ------------------------------------------------------------------------ */

ZTEST(gestures, test_config_defaults)
{
	struct wp_gesture_config c;

	memset(&c, 0xa5, sizeof(c));
	wp_gesture_config_default(&c);
	zassert_true(c.tap_enabled);
	zassert_equal(c.tap_max_ms, 180);
	zassert_within(c.tap_max_move_mm, 1.5f, 1e-6f);
	zassert_true(c.tap_drag_enabled);
	zassert_equal(c.double_tap_window_ms, 250);
	zassert_true(c.long_press_drag_enabled);
	zassert_equal(c.long_press_ms, 450);
	zassert_within(c.long_press_max_move_mm, 1.0f, 1e-6f);
	zassert_equal(c.multi_down_skew_ms, 120);
	zassert_equal(c.two_finger_tap_max_ms, 220);
	zassert_within(c.two_finger_tap_max_move_mm, 2.0f, 1e-6f);
	zassert_true(c.three_finger_tap_enabled);
	zassert_within(c.scroll_detents_per_mm, 0.25f, 1e-6f);
	zassert_within(c.scroll_start_mm, 1.0f, 1e-6f);
	zassert_true(c.scroll_axis_lock);
	zassert_within(c.scroll_axis_lock_ratio, 2.0f, 1e-6f);
	zassert_true(c.scroll.natural);
	zassert_false(c.scroll.invert_pan);
	zassert_within(c.jitter_deadband_mm, 0.3f, 1e-6f);
	zassert_within(c.smoothing_alpha, 0.6f, 1e-6f);
	zassert_within(c.smoothing_full_speed_mm_s, 60.0f, 1e-6f);
	zassert_within(c.jump_max_step_mm, 15.0f, 1e-6f);
	zassert_within(c.jump_max_speed_mm_s, 2000.0f, 1e-6f);
	zassert_within(c.accel.counts_per_mm, 16.0f, 1e-6f);
	zassert_within(c.accel.sensitivity, 1.0f, 1e-6f);
	zassert_within(c.accel.low_speed_mm_s, 20.0f, 1e-6f);
	zassert_within(c.accel.high_speed_mm_s, 200.0f, 1e-6f);
	zassert_within(c.accel.max_multiplier, 3.0f, 1e-6f);

	zassert_equal(eng.mode, WP_GEST_MODE_IDLE);
	zassert_true(eng.pointer_enabled);
	zassert_true(eng.long_press_allowed);
	zassert_false(eng.scroll_inhibit);
	zassert_within(eng.mm_per_count_x, 0.05f, 1e-6f);
	zassert_within(eng.mm_per_count_y, 0.05f, 1e-6f);
}

ZTEST(gestures, test_accel_multiplier_curve)
{
	struct wp_pointer_accel_config a;

	wp_pointer_accel_config_default(&a);
	zassert_within(wp_pointer_accel_multiplier(&a, 0.0f), 1.0f, 1e-6f);
	zassert_within(wp_pointer_accel_multiplier(&a, 20.0f), 1.0f, 1e-6f);
	zassert_within(wp_pointer_accel_multiplier(&a, 110.0f), 2.0f, 1e-5f);
	zassert_within(wp_pointer_accel_multiplier(&a, 200.0f), 3.0f, 1e-6f);
	zassert_within(wp_pointer_accel_multiplier(&a, 5000.0f), 3.0f, 1e-6f);
	zassert_within(wp_pointer_accel_multiplier(&a, NAN), 1.0f, 1e-6f);

	/* Monotonic and continuous. */
	float prev = 1.0f;

	for (int v = 0; v <= 300; v++) {
		float m = wp_pointer_accel_multiplier(&a, (float)v);

		zassert_true(m >= prev - 1e-6f, "not monotonic at %d", v);
		zassert_true(m - prev < 0.02f, "jump at %d", v);
		prev = m;
	}

	/* Degenerate curve (high <= low) is a step, never a division by 0. */
	a.high_speed_mm_s = 20.0f;
	zassert_within(wp_pointer_accel_multiplier(&a, 20.0f), 1.0f, 1e-6f);
	zassert_within(wp_pointer_accel_multiplier(&a, 20.5f), 3.0f, 1e-6f);
}

ZTEST(gestures, test_accel_apply)
{
	struct wp_pointer_accel_config a;
	float cx, cy;

	wp_pointer_accel_config_default(&a);

	/* dt <= 0: unknown speed -> multiplier 1. */
	wp_pointer_accel_apply(&a, 3.0f, -4.0f, 0.0f, &cx, &cy);
	zassert_within(cx, 48.0f, EPS);
	zassert_within(cy, -64.0f, EPS);
	wp_pointer_accel_apply(&a, 3.0f, -4.0f, -0.01f, &cx, &cy);
	zassert_within(cx, 48.0f, EPS);

	/* 5 mm in 10 ms = 500 mm/s -> x3; signs preserved. */
	wp_pointer_accel_apply(&a, -3.0f, 4.0f, 0.01f, &cx, &cy);
	zassert_within(cx, -144.0f, EPS);
	zassert_within(cy, 192.0f, EPS);

	/* Slow: 0.1 mm in 10 ms = 10 mm/s -> x1, sensitivity applies. */
	a.sensitivity = 2.0f;
	wp_pointer_accel_apply(&a, 0.1f, 0.0f, 0.01f, &cx, &cy);
	zassert_within(cx, 3.2f, EPS);
	zassert_within(cy, 0.0f, 1e-6f);
}

/* ------------------------------------------------------------------------ */
/* Taps                                                                      */
/* ------------------------------------------------------------------------ */

ZTEST(gestures, test_tap_single_left_click)
{
	frame1(0, 30.0f, 20.0f);
	zassert_true(out.events & WP_GEST_EV_TOUCH_START);
	zassert_equal(out.mode, WP_GEST_MODE_ONE_PENDING);
	frame1(MS(10), 30.1f, 20.0f); /* jitter below the 0.3 mm deadband */
	frame1(MS(20), 30.05f, 20.1f);
	frame1(MS(60), 30.0f, 20.05f);
	zassert_equal(A.left_clicks, 0, "click before lift");
	lift(MS(90));
	/* Tap recognized at the lift; the click itself waits for the tap-drag
	 * window (a following touch could still turn it into a drag). */
	zassert_equal(out.click, 0);
	zassert_true(out.events & WP_GEST_EV_TAP);
	zassert_true(out.events & WP_GEST_EV_TOUCH_END);
	zassert_equal(out.buttons, 0);
	zassert_equal(out.mode, WP_GEST_MODE_IDLE);
	tick(MS(90 + 250));
	zassert_equal(out.click, 0, "emitted inside the window");
	tick(MS(90 + 251));
	zassert_equal(out.click, WP_BTN_LEFT);

	/* Nothing else afterwards. */
	tick(MS(400));
	lift(MS(500));
	zassert_equal(A.left_clicks, 1);
	zassert_equal(A.right_clicks, 0);
	zassert_within(A.dx, 0.0f, 1e-6f, "jitter moved the pointer");
	zassert_within(A.dy, 0.0f, 1e-6f);
	zassert_false(A.ev & (WP_GEST_EV_DRAG_START | WP_GEST_EV_POINTER));
}

ZTEST(gestures, test_tap_with_movement_no_click)
{
	float x = 30.0f, y = 20.0f;
	int64_t t;

	frame1(0, x, y);
	t = swipe1(0, &x, &y, 0.5f, 0.0f, 6); /* 3 mm in 60 ms */
	zassert_equal(out.mode, WP_GEST_MODE_POINTER);
	lift(t + MS(10));
	zassert_equal(A.left_clicks, 0);
	zassert_true(A.dx > 0.0f);
	zassert_false(A.ev & WP_GEST_EV_TAP);
}

ZTEST(gestures, test_slow_touch_no_click)
{
	frame1(0, 30.0f, 20.0f);
	for (int i = 1; i <= 30; i++) {
		frame1(MS(10 * i), 30.0f, 20.0f); /* 300 ms, still */
	}
	lift(MS(310));
	zassert_equal(A.left_clicks, 0);
	zassert_false(A.ev & (WP_GEST_EV_TAP | WP_GEST_EV_LONG_PRESS | WP_GEST_EV_DRAG_START));

	/* Just over tap_max_ms is not a tap either. */
	acc_clear();
	frame1(MS(1000), 30.0f, 20.0f);
	lift(MS(1000 + 181));
	zassert_equal(A.left_clicks, 0);
	/* ... and just under is. */
	frame1(MS(2000), 30.0f, 20.0f);
	lift(MS(2000 + 180));
	tick(MS(2000 + 180 + 300));
	zassert_equal(A.left_clicks, 1);
}

ZTEST(gestures, test_tap_disabled)
{
	struct wp_gesture_config c;

	wp_gesture_config_default(&c);
	c.tap_enabled = false;
	setup(&c);
	frame1(0, 30.0f, 20.0f);
	lift(MS(80));
	frame2(MS(500), 20.0f, 20.0f, 35.0f, 20.0f);
	lift(MS(580));
	zassert_equal(A.left_clicks, 0);
	zassert_equal(A.right_clicks, 0);
}

ZTEST(gestures, test_double_tap_two_clicks)
{
	frame1(0, 30.0f, 20.0f);
	lift(MS(70));
	frame1(MS(150), 30.0f, 20.0f);
	lift(MS(220));
	zassert_equal(A.left_clicks, 1, "first click at the second lift");
	tick(MS(230));
	zassert_equal(A.left_clicks, 2, "second click right after (no window wait)");
	zassert_false(A.ev & WP_GEST_EV_DRAG_START);
	zassert_equal(A.buttons, 0);
	tick(MS(1000));
	zassert_equal(A.left_clicks, 2);
}

/* ------------------------------------------------------------------------ */
/* Pointer motion                                                            */
/* ------------------------------------------------------------------------ */

ZTEST(gestures, test_pointer_direction_magnitude_accel_off)
{
	float x = 10.0f, y = 30.0f;
	int64_t t;

	setup_plain();
	frame1(0, x, y);
	zassert_within(out.dx, 0.0f, 1e-6f, "touch-down must not move");
	t = swipe1(0, &x, &y, 0.1f, 0.0f, 50); /* +x 5 mm */
	zassert_within(A.dx, 5.0f * CPM, 0.05f);
	zassert_within(A.dy, 0.0f, 0.05f);
	zassert_true(A.ev & WP_GEST_EV_POINTER);

	acc_clear();
	t = swipe1(t, &x, &y, 0.0f, -0.2f, 20); /* -y (away from user) 4 mm */
	zassert_within(A.dy, -4.0f * CPM, 0.05f);
	zassert_within(A.dx, 0.0f, 0.05f);

	acc_clear();
	t = swipe1(t, &x, &y, -1.5f, 1.0f, 10); /* fast diagonal, still x1 */
	zassert_within(A.dx, -15.0f * CPM, 0.1f);
	zassert_within(A.dy, 10.0f * CPM, 0.1f);
	lift(t + MS(10));
	zassert_equal(A.left_clicks, 0);
}

ZTEST(gestures, test_pointer_acceleration_on)
{
	struct wp_gesture_config c;
	float x = 5.0f, y = 30.0f;
	int64_t t;

	plain_cfg(&c);
	c.accel.max_multiplier = 3.0f;
	setup(&c);
	frame1(0, x, y);

	/* 10 mm/s: below low speed -> x1 (1.6 counts per frame). */
	t = swipe1(0, &x, &y, 0.1f, 0.0f, 10);
	zassert_within(out.dx, 0.1f * CPM, 0.01f);
	zassert_within(A.dx, 1.0f * CPM, 0.05f);

	/* 110 mm/s: halfway -> x2. */
	acc_clear();
	t = swipe1(t, &x, &y, 1.1f, 0.0f, 5);
	zassert_within(out.dx, 1.1f * CPM * 2.0f, 0.05f);

	/* 300 mm/s: saturated -> x3. */
	acc_clear();
	t = swipe1(t, &x, &y, 3.0f, 0.0f, 5);
	zassert_within(out.dx, 3.0f * CPM * 3.0f, 0.05f);
	zassert_within(A.dx, 15.0f * CPM * 3.0f, 0.2f);
	zassert_within(A.dy, 0.0f, 0.01f);
	lift(t + MS(10));
}

ZTEST(gestures, test_jitter_deadband)
{
	struct wp_gesture_config c;

	plain_cfg(&c);
	c.jitter_deadband_mm = 0.3f;
	setup(&c);
	frame1(0, 30.0f, 20.0f);
	frame1(MS(10), 30.1f, 20.0f);
	frame1(MS(20), 30.2f, 20.1f);
	frame1(MS(30), 30.0f, 20.2f);
	zassert_within(A.max_step, 0.0f, 1e-6f, "sub-deadband jitter moved the pointer");

	/* Exceeding the deadband releases the full displacement (0.5 mm over
	 * 40 ms = 12.5 mm/s -> no acceleration). */
	frame1(MS(40), 30.5f, 20.0f);
	zassert_within(out.dx, 0.5f * CPM, 0.05f);
	zassert_within(out.dy, 0.0f, 0.05f);

	/* Once passed, small movements are no longer filtered. */
	frame1(MS(50), 30.6f, 20.0f);
	zassert_within(out.dx, 0.1f * CPM, 0.05f);
	lift(MS(60));
	tick(MS(400));
	zassert_equal(A.left_clicks, 1, "0.6 mm in 60 ms is still a tap");
}

ZTEST(gestures, test_smoothing_low_speed_only)
{
	struct wp_gesture_config c;
	float x = 10.0f, y = 20.0f;
	int64_t t;

	plain_cfg(&c);
	c.smoothing_alpha = 0.6f;
	setup(&c);
	frame1(0, x, y);
	/* 10 mm/s: alpha_eff = 0.6 + 0.4 * 10/60 */
	t = swipe1(0, &x, &y, 0.1f, 0.0f, 1);
	zassert_within(out.dx, 0.1f * CPM * (0.6f + 0.4f / 6.0f), 0.02f);
	t = swipe1(t, &x, &y, 0.1f, 0.0f, 59);
	/* Converged: per-frame output equals the raw step, total lags by
	 * step * (1 - a) / a = 0.05 mm. */
	zassert_within(out.dx, 0.1f * CPM, 0.02f);
	zassert_within(A.dx, 6.0f * CPM - 0.05f * CPM, 0.1f);
	lift(t + MS(10));

	/* Above smoothing_full_speed_mm_s: no smoothing at all. */
	acc_clear();
	x = 10.0f;
	frame1(MS(2000), x, y);
	t = swipe1(MS(2000), &x, &y, 1.0f, 0.0f, 1); /* 100 mm/s */
	zassert_within(out.dx, 1.0f * CPM, 0.01f);
	lift(t + MS(10));
}

ZTEST(gestures, test_pointer_disabled_taps_still_work)
{
	float x = 10.0f, y = 20.0f;
	int64_t t;

	setup_plain();
	wp_gesture_set_pointer_enabled(&eng, false);
	frame1(0, x, y);
	t = swipe1(0, &x, &y, 0.5f, 0.0f, 10);
	zassert_within(A.max_step, 0.0f, 1e-6f, "pointer moved while disabled");
	zassert_equal(out.mode, WP_GEST_MODE_POINTER);

	/* Re-enable mid-motion: no catch-up jump, then normal motion. */
	wp_gesture_set_pointer_enabled(&eng, true);
	t = swipe1(t, &x, &y, 0.5f, 0.0f, 1);
	zassert_within(out.dx, 0.0f, 1e-6f, "re-enable produced a jump");
	t = swipe1(t, &x, &y, 0.5f, 0.0f, 1);
	zassert_within(out.dx, 0.5f * CPM, 0.01f);
	lift(t + MS(10));

	wp_gesture_set_pointer_enabled(&eng, false);
	frame1(MS(1000), 30.0f, 20.0f);
	lift(MS(1080));
	tick(MS(1400));
	zassert_equal(A.left_clicks, 1, "tap must work with pointer disabled");
}

/* ------------------------------------------------------------------------ */
/* Drags                                                                     */
/* ------------------------------------------------------------------------ */

ZTEST(gestures, test_tap_drag)
{
	float x = 30.0f, y = 20.0f;
	int64_t t;

	frame1(0, x, y);
	lift(MS(80));
	tick(MS(100));
	/* ALG-6: the tap click is held back while a tap-drag is possible. */
	zassert_equal(A.left_clicks, 0, "click sent before the tap-drag decision");

	/* Touch again 70 ms later and move. */
	frame1(MS(150), x, y);
	zassert_equal(out.buttons, 0);
	t = swipe1(MS(150), &x, &y, 0.4f, 0.0f, 3); /* 1.2 mm: still undecided */
	zassert_equal(out.mode, WP_GEST_MODE_ONE_PENDING);
	zassert_within(A.max_step, 0.0f, 1e-6f, "pointer moved before the drag decision");
	t = swipe1(t, &x, &y, 0.4f, 0.0f, 1); /* 1.6 mm > 1.5 mm */
	zassert_true(out.events & WP_GEST_EV_DRAG_START);
	zassert_equal(out.buttons, WP_BTN_LEFT);
	zassert_equal(out.mode, WP_GEST_MODE_DRAG);
	zassert_within(out.dx, 0.0f, 1e-6f, "press must come before motion");

	acc_clear();
	t = swipe1(t, &x, &y, 0.5f, 0.5f, 10);
	zassert_true(A.dx > 0.0f && A.dy > 0.0f);
	zassert_equal(A.buttons, WP_BTN_LEFT);
	lift(t + MS(10));
	zassert_true(out.events & WP_GEST_EV_DRAG_END);
	zassert_equal(out.buttons, 0);
	tick(t + MS(1000));
	/* The host sees press-move-release only: no click + press (= a double
	 * click that opens the file instead of moving it). */
	zassert_equal(A.left_clicks, 0, "no click around the tap-drag");
}

ZTEST(gestures, test_tap_drag_window_expires)
{
	float x = 30.0f, y = 20.0f;
	int64_t t;

	frame1(0, x, y);
	lift(MS(80));
	tick(MS(200));
	zassert_true(eng.last_tap_valid);
	tick(MS(80 + 251));
	zassert_false(eng.last_tap_valid, "tick must expire the tap-drag window");
	frame1(MS(400), x, y);
	t = swipe1(MS(400), &x, &y, 0.5f, 0.0f, 6);
	zassert_equal(out.mode, WP_GEST_MODE_POINTER);
	zassert_equal(out.buttons, 0);
	zassert_false(A.ev & WP_GEST_EV_DRAG_START);
	lift(t + MS(10));

	/* Window also checked against the touch-down time without ticks. */
	acc_clear();
	frame1(MS(1000), x, y);
	lift(MS(1080));
	frame1(MS(1080 + 300), x, y);
	t = swipe1(MS(1380), &x, &y, 0.5f, 0.0f, 6);
	zassert_false(A.ev & WP_GEST_EV_DRAG_START);
	lift(t + MS(10));
}

ZTEST(gestures, test_long_press_drag)
{
	float x = 30.0f, y = 20.0f;
	int64_t t;

	frame1(0, x, y);
	for (int i = 1; i <= 44; i++) {
		frame1(MS(10 * i), x + ((i & 1) ? 0.1f : 0.0f), y); /* tiny jitter */
	}
	zassert_false(A.ev & WP_GEST_EV_LONG_PRESS);
	tick(MS(449));
	zassert_false(out.events & WP_GEST_EV_LONG_PRESS);
	tick(MS(450));
	zassert_true(out.events & WP_GEST_EV_LONG_PRESS);
	/* ALG-2: the long press only arms the drag, no button yet. */
	zassert_false(out.events & WP_GEST_EV_DRAG_START);
	zassert_equal(out.buttons, 0, "button pressed without movement");
	zassert_equal(out.mode, WP_GEST_MODE_ONE_PENDING);

	acc_clear();
	t = swipe1(MS(450), &x, &y, 0.0f, 0.4f, 3); /* 1.2 mm: still armed */
	zassert_equal(A.buttons, 0);
	zassert_within(A.max_step, 0.0f, 1e-6f, "pointer moved while armed");
	t = swipe1(t, &x, &y, 0.0f, 0.4f, 1); /* 1.6 mm > tap_max_move_mm */
	zassert_true(out.events & WP_GEST_EV_DRAG_START);
	zassert_equal(out.buttons, WP_BTN_LEFT);
	zassert_equal(out.mode, WP_GEST_MODE_DRAG);
	zassert_within(out.dy, 0.0f, 1e-6f, "press must come before motion");
	t = swipe1(t, &x, &y, 0.0f, 0.4f, 10);
	zassert_true(A.dy > 0.0f);
	zassert_equal(A.buttons, WP_BTN_LEFT, "button released during drag");
	zassert_false(A.ev & WP_GEST_EV_DRAG_END);
	lift(t + MS(10));
	zassert_true(out.events & WP_GEST_EV_DRAG_END);
	zassert_equal(out.buttons, 0);
	tick(t + MS(1000));
	zassert_equal(A.left_clicks, 0);
}

ZTEST(gestures, test_long_press_then_lift_is_not_a_click)
{
	/* ALG-2 (a): rest a finger 600 ms, lift without moving -> nothing. */
	frame1(0, 30.0f, 20.0f);
	for (int i = 1; i <= 60; i++) {
		frame1(MS(10 * i), 30.0f, 20.0f);
	}
	zassert_true(A.ev & WP_GEST_EV_LONG_PRESS);
	zassert_equal(A.buttons, 0);
	lift(MS(610));
	tick(MS(2000));
	zassert_equal(A.buttons, 0);
	zassert_equal(A.left_clicks, 0, "long press + lift produced a click");
	zassert_false(A.ev & (WP_GEST_EV_DRAG_START | WP_GEST_EV_DRAG_END));
}

ZTEST(gestures, test_cancel_drag_releases_and_continues_as_pointer)
{
	float x = 30.0f, y = 20.0f;
	int64_t t;

	setup_plain();
	/* ALG-2 (c): drag in progress when the device leaves the trackpad
	 * posture (router: pointer / long press no longer allowed). */
	frame1(0, x, y);
	tick(MS(450));
	t = swipe1(MS(450), &x, &y, 0.5f, 0.0f, 5);
	zassert_equal(out.buttons, WP_BTN_LEFT);

	wp_gesture_cancel_drag(&eng, &out);
	accumulate();
	zassert_equal(out.buttons, 0, "drag not released");
	zassert_true(out.events & WP_GEST_EV_DRAG_END);
	zassert_equal(out.mode, WP_GEST_MODE_POINTER);

	acc_clear();
	t = swipe1(t, &x, &y, 0.5f, 0.0f, 1);
	zassert_within(out.dx, 0.0f, 1e-6f, "re-anchor jump after cancel");
	t = swipe1(t, &x, &y, 0.5f, 0.0f, 3);
	zassert_true(A.dx > 0.0f);
	zassert_equal(A.buttons, 0, "button re-pressed after cancel");
	zassert_false(A.ev & WP_GEST_EV_DRAG_START);
	tick(t + MS(500));
	zassert_false(A.ev & (WP_GEST_EV_LONG_PRESS | WP_GEST_EV_DRAG_START));
	lift(t + MS(510));
	zassert_equal(A.left_clicks, 0);

	/* Armed only: cancel disarms, later movement is a pointer, not a drag. */
	acc_clear();
	frame1(MS(2000), 30.0f, 20.0f);
	tick(MS(2460));
	zassert_true(eng.drag_armed);
	wp_gesture_cancel_drag(&eng, &out);
	accumulate();
	zassert_false(eng.drag_armed);
	zassert_false(out.events & WP_GEST_EV_DRAG_END);
	x = 30.0f;
	y = 20.0f;
	t = swipe1(MS(2460), &x, &y, 0.5f, 0.0f, 6);
	zassert_equal(A.buttons, 0);
	zassert_false(A.ev & WP_GEST_EV_DRAG_START);

	/* No drag active: cancel is a no-op for the session. */
	wp_gesture_cancel_drag(&eng, &out);
	zassert_equal(out.mode, WP_GEST_MODE_POINTER);
	zassert_equal(out.events, 0U);
	lift(t + MS(10));
}

ZTEST(gestures, test_long_press_disarmed_when_disallowed)
{
	float x = 30.0f, y = 20.0f;

	frame1(0, x, y);
	tick(MS(460));
	zassert_true(eng.drag_armed);
	/* Pickup: long press no longer allowed before the finger moves. */
	wp_gesture_set_long_press_allowed(&eng, false);
	(void)swipe1(MS(460), &x, &y, 0.5f, 0.0f, 6);
	zassert_equal(A.buttons, 0);
	zassert_false(A.ev & WP_GEST_EV_DRAG_START);
}

ZTEST(gestures, test_tap_hold_lift_is_one_click)
{
	/* Tap, touch again inside the window and rest, lift without moving:
	 * exactly one click, no press/drag. */
	frame1(0, 30.0f, 20.0f);
	lift(MS(70));
	frame1(MS(150), 30.0f, 20.0f);
	tick(MS(700));
	zassert_equal(A.buttons, 0);
	zassert_equal(A.left_clicks, 0);
	lift(MS(800));
	tick(MS(1500));
	zassert_equal(A.left_clicks, 1);
	zassert_false(A.ev & WP_GEST_EV_DRAG_START);
}

ZTEST(gestures, test_tap_then_two_fingers_flushes_click)
{
	/* Tap, then a two-finger tap inside the window: left then right. */
	frame1(0, 30.0f, 20.0f);
	lift(MS(70));
	frame2(MS(150), 20.0f, 20.0f, 35.0f, 20.0f);
	zassert_equal(A.left_clicks, 1, "pending click not flushed");
	lift(MS(220));
	zassert_equal(A.right_clicks, 1);
	tick(MS(1000));
	zassert_equal(A.left_clicks, 1);
}

ZTEST(gestures, test_long_press_disallowed_no_drag)
{
	wp_gesture_set_long_press_allowed(&eng, false);
	frame1(0, 30.0f, 20.0f);
	for (int i = 1; i <= 60; i++) {
		frame1(MS(10 * i), 30.0f, 20.0f);
	}
	tick(MS(700));
	zassert_equal(A.buttons, 0);
	zassert_false(A.ev & WP_GEST_EV_DRAG_START);
	zassert_true(out.single_still, "still finger must be reported (air-mouse clutch)");
	zassert_true(out.single_still_ms >= 690);
	lift(MS(710));
	zassert_equal(A.left_clicks, 0);
	zassert_false(A.ev & WP_GEST_EV_DRAG_END);

	/* Same with the feature disabled in the configuration. */
	struct wp_gesture_config c;

	wp_gesture_config_default(&c);
	c.long_press_drag_enabled = false;
	setup(&c);
	frame1(0, 30.0f, 20.0f);
	tick(MS(1000));
	zassert_equal(out.buttons, 0);
	zassert_false(A.ev & WP_GEST_EV_DRAG_START);
	lift(MS(1010));
	zassert_equal(A.left_clicks, 0);
}

ZTEST(gestures, test_long_press_requires_stillness)
{
	frame1(0, 30.0f, 20.0f);
	for (int i = 1; i <= 20; i++) {
		frame1(MS(10 * i), 30.0f, 20.0f);
	}
	/* Slide 1.2 mm (> long_press_max_move_mm, <= tap_max_move_mm): still
	 * undecided, but the long-press timer restarts from here. */
	frame1(MS(210), 30.0f, 21.2f);
	zassert_equal(out.mode, WP_GEST_MODE_ONE_PENDING);
	tick(MS(455));
	zassert_false(A.ev & WP_GEST_EV_LONG_PRESS, "timer did not restart");
	tick(MS(659));
	zassert_false(A.ev & WP_GEST_EV_LONG_PRESS);
	tick(MS(660));
	zassert_true(out.events & WP_GEST_EV_LONG_PRESS);
	zassert_equal(out.buttons, 0);
	lift(MS(700));
	zassert_false(out.events & WP_GEST_EV_DRAG_END);
	tick(MS(1500));
	zassert_equal(A.left_clicks, 0);
}

/* ------------------------------------------------------------------------ */
/* Multi-finger                                                              */
/* ------------------------------------------------------------------------ */

ZTEST(gestures, test_two_finger_tap_right_click)
{
	frame2(0, 20.0f, 20.0f, 35.0f, 20.0f);
	zassert_equal(out.mode, WP_GEST_MODE_MULTI_PENDING);
	frame2(MS(50), 20.1f, 20.0f, 35.0f, 20.1f);
	lift(MS(120));
	zassert_equal(out.click, WP_BTN_RIGHT);
	zassert_true(out.events & WP_GEST_EV_TWO_FINGER_TAP);
	zassert_equal(A.left_clicks, 0);
	zassert_within(A.max_step, 0.0f, 1e-6f);

	/* Staggered landing (40 ms) and staggered lift. */
	acc_clear();
	frame1(MS(1000), 20.0f, 20.0f);
	frame2(MS(1040), 20.0f, 20.0f, 35.0f, 20.0f);
	zassert_equal(out.mode, WP_GEST_MODE_MULTI_PENDING);
	frame_n(MS(1120), 1, &(struct pt){1, 35.0f, 20.0f});
	lift(MS(1150));
	zassert_equal(A.right_clicks, 1);
	zassert_equal(A.left_clicks, 0);
}

ZTEST(gestures, test_two_finger_tap_rejected)
{
	/* Too long. */
	frame2(0, 20.0f, 20.0f, 35.0f, 20.0f);
	lift(MS(221));
	/* Second finger too late: stays a one-finger session. */
	frame1(MS(1000), 20.0f, 20.0f);
	frame2(MS(1150), 20.0f, 20.0f, 35.0f, 20.0f);
	zassert_equal(out.mode, WP_GEST_MODE_ONE_PENDING);
	lift(MS(1200));
	/* Fingers moved too much. */
	frame2(MS(2000), 20.0f, 20.0f, 35.0f, 20.0f);
	frame2(MS(2050), 22.5f, 20.0f, 35.0f, 20.0f);
	lift(MS(2100));
	zassert_equal(A.right_clicks, 0);
	zassert_equal(A.left_clicks, 0);
}

ZTEST(gestures, test_three_finger_tap_event)
{
	struct pt p[3] = {{0, 15.0f, 20.0f}, {1, 30.0f, 20.0f}, {2, 45.0f, 20.0f}};

	frame_n(0, 3, p);
	zassert_equal(out.mode, WP_GEST_MODE_MULTI_PENDING);
	frame_n(MS(60), 3, p);
	lift(MS(150));
	zassert_true(out.events & WP_GEST_EV_THREE_FINGER_TAP);
	zassert_false(out.events & WP_GEST_EV_TWO_FINGER_TAP);
	zassert_equal(out.click, 0);
	zassert_equal(A.left_clicks + A.right_clicks, 0);

	/* Disabled by configuration. */
	struct wp_gesture_config c;

	wp_gesture_config_default(&c);
	c.three_finger_tap_enabled = false;
	setup(&c);
	frame_n(0, 3, p);
	lift(MS(100));
	zassert_false(A.ev & WP_GEST_EV_THREE_FINGER_TAP);
}

ZTEST(gestures, test_more_than_three_fingers_ignored)
{
	struct pt p[4] = {{0, 10.0f, 20.0f}, {1, 20.0f, 20.0f}, {2, 30.0f, 20.0f},
			  {3, 40.0f, 20.0f}};

	setup_plain();
	frame_n(0, 4, p);
	zassert_equal(out.mode, WP_GEST_MODE_IGNORE);
	for (int i = 1; i <= 10; i++) {
		for (int k = 0; k < 4; k++) {
			p[k].y += 0.5f;
		}
		frame_n(MS(10 * i), 4, p);
	}
	/* Dropping to two fingers keeps the session ignored. */
	frame2(MS(110), 10.0f, 25.0f, 20.0f, 25.0f);
	frame2(MS(120), 10.0f, 28.0f, 20.0f, 28.0f);
	zassert_equal(out.mode, WP_GEST_MODE_IGNORE);
	lift(MS(130));
	zassert_within(A.max_step, 0.0f, 1e-6f);
	zassert_within(A.max_wheel_step, 0.0f, 1e-6f);
	zassert_equal(A.left_clicks + A.right_clicks, 0);
	zassert_false(A.ev & (WP_GEST_EV_TWO_FINGER_TAP | WP_GEST_EV_THREE_FINGER_TAP));

	/* A held drag is released when a 4th finger arrives. */
	acc_clear();
	frame1(MS(1000), 30.0f, 20.0f);
	tick(MS(1500));
	frame1(MS(1505), 32.0f, 20.0f); /* armed long press + move: drag */
	zassert_equal(out.buttons, WP_BTN_LEFT);
	frame_n(MS(1510), 4, p);
	zassert_true(out.events & WP_GEST_EV_DRAG_END);
	zassert_equal(out.buttons, 0);
	lift(MS(1600));
	zassert_equal(A.left_clicks, 0);
}

/* ------------------------------------------------------------------------ */
/* Scroll                                                                    */
/* ------------------------------------------------------------------------ */

/* Two fingers 15 mm apart moving (sx, sy) per 10 ms frame. */
static int64_t swipe2(int64_t t, float sx, float sy, int steps)
{
	float x0 = 20.0f, y0 = 15.0f, x1 = 35.0f, y1 = 15.0f;

	frame2(t, x0, y0, x1, y1);
	for (int i = 0; i < steps; i++) {
		t += MS(10);
		x0 += sx;
		x1 += sx;
		y0 += sy;
		y1 += sy;
		frame2(t, x0, y0, x1, y1);
	}
	return t;
}

/* 20 frames x 0.4 mm: scrolling starts once the travel exceeds 1.0 mm (frame
 * 3, 1.2 mm); the following 17 frames carry 6.8 mm -> 1.7 detents. */
#define SSTEP         0.4f
#define SCROLL_EXPECT (17.0f * SSTEP * 0.25f)

ZTEST(gestures, test_scroll_vertical_natural_and_traditional)
{
	int64_t t = swipe2(0, 0.0f, SSTEP, 20); /* towards the user */

	zassert_equal(out.mode, WP_GEST_MODE_SCROLL);
	zassert_true(A.ev & WP_GEST_EV_SCROLL);
	zassert_within(A.wheel, SCROLL_EXPECT, 0.01f, "natural: fingers down -> wheel up");
	zassert_within(A.pan, 0.0f, 1e-6f);
	zassert_within(A.max_step, 0.0f, 1e-6f, "scroll moved the pointer");
	lift(t + MS(10));
	zassert_equal(A.right_clicks + A.left_clicks, 0);

	struct wp_gesture_config c;

	wp_gesture_config_default(&c);
	c.scroll.natural = false;
	setup(&c);
	t = swipe2(0, 0.0f, SSTEP, 20);
	zassert_within(A.wheel, -SCROLL_EXPECT, 0.01f);
	zassert_within(A.pan, 0.0f, 1e-6f);
	lift(t + MS(10));
}

ZTEST(gestures, test_scroll_horizontal_natural_and_traditional)
{
	int64_t t = swipe2(0, SSTEP, 0.0f, 20); /* to the right */

	zassert_within(A.pan, -SCROLL_EXPECT, 0.01f, "natural: fingers right -> pan left");
	zassert_within(A.wheel, 0.0f, 1e-6f);
	lift(t + MS(10));

	struct wp_gesture_config c;

	wp_gesture_config_default(&c);
	c.scroll.natural = false;
	setup(&c);
	t = swipe2(0, SSTEP, 0.0f, 20);
	zassert_within(A.pan, SCROLL_EXPECT, 0.01f);
	zassert_within(A.wheel, 0.0f, 1e-6f);
	lift(t + MS(10));
}

ZTEST(gestures, test_scroll_axis_lock)
{
	int64_t t = swipe2(0, 0.1f, SSTEP, 20); /* mostly vertical */

	zassert_equal(eng.scroll_axis, 1);
	zassert_within(A.wheel, SCROLL_EXPECT, 0.01f);
	zassert_within(A.pan, 0.0f, 1e-6f, "axis lock leaked pan");
	lift(t + MS(10));

	struct wp_gesture_config c;

	wp_gesture_config_default(&c);
	c.scroll_axis_lock = false;
	setup(&c);
	t = swipe2(0, 0.1f, SSTEP, 20);
	zassert_within(A.pan, -17.0f * 0.1f * 0.25f, 0.01f);
	zassert_within(A.wheel, SCROLL_EXPECT, 0.01f);
	lift(t + MS(10));

	/* Truly diagonal start with the lock on: no axis chosen. */
	setup(NULL);
	t = swipe2(0, 0.4f, 0.4f, 20);
	zassert_equal(eng.scroll_axis, 0);
	zassert_true(A.pan < 0.0f && A.wheel > 0.0f);
	lift(t + MS(10));
}

ZTEST(gestures, test_scroll_inhibit)
{
	wp_gesture_set_scroll_inhibit(&eng, true);
	int64_t t = swipe2(0, 0.0f, SSTEP, 20);

	zassert_within(A.max_wheel_step, 0.0f, 1e-6f, "scrolled while inhibited");
	zassert_false(A.ev & WP_GEST_EV_SCROLL);

	/* Lift the inhibit mid-gesture: next frame carries one frame of motion. */
	wp_gesture_set_scroll_inhibit(&eng, false);
	frame2(t + MS(10), 20.0f, 15.0f + 21.0f * SSTEP, 35.0f, 15.0f + 21.0f * SSTEP);
	zassert_within(out.wheel, SSTEP * 0.25f, 0.01f, "burst after inhibit");
	lift(t + MS(20));
	zassert_equal(A.right_clicks, 0);
}

ZTEST(gestures, test_scroll_finger_set_changes_no_burst)
{
	int64_t t = swipe2(0, 0.0f, SSTEP, 10);
	float y = 15.0f + 10.0f * SSTEP;

	/* Third finger lands far away: no output while three are down. */
	struct pt p3[3] = {{0, 20.0f, y}, {1, 35.0f, y}, {2, 55.0f, 40.0f}};

	acc_clear();
	t += MS(10);
	frame_n(t, 3, p3);
	zassert_within(out.wheel, 0.0f, 1e-6f);
	/* Back to two (third lifts): re-anchor, no burst; then resumes. */
	t += MS(10);
	y += SSTEP;
	frame2(t, 20.0f, y, 35.0f, y);
	zassert_within(out.wheel, 0.0f, 1e-6f);
	for (int i = 0; i < 4; i++) {
		t += MS(10);
		y += SSTEP;
		frame2(t, 20.0f, y, 35.0f, y);
	}
	zassert_within(A.wheel, 4.0f * SSTEP * 0.25f, 0.01f);
	zassert_true(A.max_wheel_step <= SSTEP * 0.25f + 0.01f);

	/* Drop to one finger: no pointer motion, no scroll. */
	acc_clear();
	for (int i = 0; i < 5; i++) {
		t += MS(10);
		y += SSTEP;
		frame1(t, 20.0f, y);
	}
	zassert_within(A.max_step, 0.0f, 1e-6f, "pointer moved after scroll");
	zassert_within(A.max_wheel_step, 0.0f, 1e-6f);
	zassert_equal(out.mode, WP_GEST_MODE_SCROLL);
	lift(t + MS(10));
	zassert_equal(A.left_clicks + A.right_clicks, 0);
}

/* ------------------------------------------------------------------------ */
/* Robustness                                                                */
/* ------------------------------------------------------------------------ */

ZTEST(gestures, test_finger_added_removed_no_jump)
{
	const float step = 0.5f * CPM; /* counts per frame */
	struct pt p[2];
	float ax = 10.0f, ay = 10.0f;
	int64_t t;

	setup_plain();
	frame1(0, ax, ay);
	t = swipe1(0, &ax, &ay, 0.5f, 0.0f, 10);
	zassert_equal(out.mode, WP_GEST_MODE_POINTER);

	/* A second finger lands late and far away: no jump, A keeps driving. */
	acc_clear();
	t += MS(10);
	ax += 0.5f;
	p[0] = (struct pt){0, ax, ay};
	p[1] = (struct pt){1, 50.0f, 40.0f};
	frame_n(t, 2, p);
	zassert_within(out.dx, 0.0f, 1e-6f);
	zassert_within(out.dy, 0.0f, 1e-6f);
	for (int i = 0; i < 5; i++) {
		t += MS(10);
		ax += 0.5f;
		p[0].x = ax;
		frame_n(t, 2, p);
		zassert_within(out.dx, step, 0.02f);
		zassert_within(out.dy, 0.0f, 0.02f);
	}
	zassert_equal(out.mode, WP_GEST_MODE_POINTER);

	/* The tracked finger lifts; the remaining one is 40 mm away. */
	t += MS(10);
	frame_n(t, 1, &(struct pt){1, 50.0f, 40.0f});
	zassert_within(out.dx, 0.0f, 1e-6f, "jump on anchor change");
	zassert_within(out.dy, 0.0f, 1e-6f, "jump on anchor change");

	/* Pointer now follows the remaining finger (id 1). */
	float by = 40.0f;

	for (int i = 0; i < 5; i++) {
		t += MS(10);
		by -= 0.5f;
		frame_n(t, 1, &(struct pt){1, 50.0f, by});
		zassert_within(out.dy, -step, 0.02f);
		zassert_within(out.dx, 0.0f, 0.02f);
	}
	zassert_true(A.max_step <= step + 0.02f, "bounded output violated: %f",
		     (double)A.max_step);
	lift(t + MS(10));
	zassert_equal(A.left_clicks + A.right_clicks, 0);
}

/*
 * ALG-9 regression: a contact that keeps its id but jumps 20-40 mm in one
 * frame (slot reuse / ghost coordinate) must not become pointer motion. The
 * delta is dropped and tracking continues from the new position.
 */
ZTEST(gestures, test_implausible_jump_pointer_reanchors)
{
	float x = 5.0f, y = 30.0f;
	int64_t t;

	setup_plain();
	frame1(0, x, y);
	t = swipe1(0, &x, &y, 0.5f, 0.0f, 10); /* 5 mm, POINTER */
	zassert_equal(out.mode, WP_GEST_MODE_POINTER);
	zassert_within(A.dx, 5.0f * CPM, 0.05f);

	/* Same id, +30 mm / +12 mm in one 10 ms frame. */
	acc_clear();
	t += MS(10);
	x += 30.0f;
	y += 12.0f;
	frame1(t, x, y);
	zassert_within(out.dx, 0.0f, 1e-6f, "jump became pointer motion");
	zassert_within(out.dy, 0.0f, 1e-6f, "jump became pointer motion");
	zassert_false(out.events & WP_GEST_EV_POINTER);
	zassert_equal(out.mode, WP_GEST_MODE_POINTER, "jump changed the session mode");

	/* Continues normally from the new position. */
	t = swipe1(t, &x, &y, 0.5f, 0.0f, 10);
	zassert_within(A.dx, 5.0f * CPM, 0.05f);
	zassert_within(A.dy, 0.0f, 0.05f);
	zassert_true(A.max_step <= 0.5f * CPM + 0.01f);

	/* A 20 mm jump back, and with the default (accelerated) pipeline. */
	acc_clear();
	t += MS(10);
	x -= 20.0f;
	frame1(t, x, y);
	zassert_within(out.dx, 0.0f, 1e-6f);
	lift(t + MS(10));
	zassert_equal(A.left_clicks, 0);

	setup(NULL);
	x = 5.0f;
	frame1(0, x, y);
	t = swipe1(0, &x, &y, 0.5f, 0.0f, 10);
	acc_clear();
	t += MS(10);
	x += 40.0f;
	frame1(t, x, y);
	zassert_within(out.dx, 0.0f, 1e-6f, "jump became an accelerated pointer jump");
	t = swipe1(t, &x, &y, 0.5f, 0.0f, 5);
	zassert_true(A.max_step < 0.5f * CPM * 3.0f + 0.01f, "burst after jump");
	lift(t + MS(10));

	/* Faster report rate (5 ms): a 12 mm step is below the step limit but
	 * 2.4 m/s, above the speed limit. */
	setup_plain();
	x = 5.0f;
	frame1(0, x, y);
	t = 0;
	for (int i = 0; i < 4; i++) {
		t += MS(5);
		x += 0.5f;
		frame1(t, x, y);
	}
	acc_clear();
	t += MS(5);
	x += 12.0f;
	frame1(t, x, y);
	zassert_within(out.dx, 0.0f, 1e-6f, "2.4 m/s step became pointer motion");
	lift(t + MS(5));
}

/* ALG-9 counterpart: fast but human flicks are never cut. */
ZTEST(gestures, test_fast_flick_not_rejected)
{
	/* Bell-shaped 48 mm flick, 10 ms frames, peak 14 mm/frame (1.4 m/s). */
	static const float prof[] = {3.0f, 8.0f, 12.0f, 14.0f, 8.0f, 3.0f};
	float x = 5.0f, y = 30.0f;
	int64_t t = 0;

	setup_plain();
	frame1(t, x, y);
	for (size_t i = 0; i < ARRAY_SIZE(prof); i++) {
		t += MS(10);
		x += prof[i];
		frame1(t, x, y);
		zassert_within(out.dx, prof[i] * CPM, 0.05f, "flick step %u dropped",
			       (unsigned int)i);
	}
	zassert_within(A.dx, 48.0f * CPM, 0.1f);
	lift(t + MS(10));

	/* Same flick through the default pipeline: every step is far above the
	 * acceleration knee (x3) and above the smoothing speed (alpha 1). */
	setup(NULL);
	x = 5.0f;
	t = 0;
	frame1(t, x, y);
	for (size_t i = 0; i < ARRAY_SIZE(prof); i++) {
		t += MS(10);
		x += prof[i];
		frame1(t, x, y);
	}
	zassert_within(A.dx, 48.0f * CPM * 3.0f, 0.5f);
	lift(t + MS(10));

	/* 5 ms frames at 1.4 m/s (7 mm per frame) towards -y. */
	setup_plain();
	x = 30.0f;
	y = 45.0f;
	t = 0;
	frame1(t, x, y);
	for (int i = 0; i < 6; i++) {
		t += MS(5);
		y -= 7.0f;
		frame1(t, x, y);
	}
	zassert_within(A.dy, -42.0f * CPM, 0.1f);
	lift(t + MS(5));
}

/* ALG-9: a jump of one finger during a two-finger scroll is no scroll burst. */
ZTEST(gestures, test_implausible_jump_no_scroll_burst)
{
	int64_t t = swipe2(0, 0.0f, SSTEP, 10);
	float y = 15.0f + 10.0f * SSTEP;

	zassert_equal(out.mode, WP_GEST_MODE_SCROLL);
	acc_clear();
	t += MS(10);
	y += SSTEP;
	frame2(t, 20.0f, y, 35.0f, y + 30.0f); /* finger 1 jumps 30 mm */
	zassert_within(out.wheel, 0.0f, 1e-6f, "scroll burst from a jump");
	zassert_within(out.pan, 0.0f, 1e-6f, "scroll burst from a jump");
	for (int i = 0; i < 5; i++) {
		t += MS(10);
		y += SSTEP;
		frame2(t, 20.0f, y, 35.0f, y + 30.0f);
	}
	zassert_within(A.wheel, 5.0f * SSTEP * 0.25f, 0.01f);
	zassert_true(A.max_wheel_step <= SSTEP * 0.25f + 0.01f);
	zassert_equal(out.mode, WP_GEST_MODE_SCROLL);
	lift(t + MS(10));
	zassert_equal(A.left_clicks + A.right_clicks, 0);
}

ZTEST(gestures, test_reset_releases_buttons)
{
	frame1(0, 30.0f, 20.0f);
	tick(MS(500));
	frame1(MS(505), 32.0f, 20.0f); /* long press, then move: drag */
	zassert_equal(out.buttons, WP_BTN_LEFT);

	wp_gesture_reset(&eng, &out);
	zassert_equal(out.buttons, 0);
	zassert_true(out.events & WP_GEST_EV_DRAG_END);
	zassert_equal(out.mode, WP_GEST_MODE_IDLE);
	zassert_equal(out.contacts, 0);

	/* Finger still down after the reset: fresh session, no jump. */
	frame1(MS(510), 40.0f, 30.0f);
	zassert_true(out.events & WP_GEST_EV_TOUCH_START);
	zassert_within(out.dx, 0.0f, 1e-6f);
	zassert_equal(out.buttons, 0);

	/* Second reset without a drag: no DRAG_END. */
	wp_gesture_reset(&eng, &out);
	zassert_false(out.events & WP_GEST_EV_DRAG_END);

	/* Geometry change also releases a held drag. */
	frame1(MS(1000), 30.0f, 20.0f);
	tick(MS(1500));
	frame1(MS(1505), 32.0f, 20.0f);
	zassert_equal(out.buttons, WP_BTN_LEFT);
	struct wp_pad_geometry g2;

	wp_xform_geometry(WP_ORIENT_90, &geom, &g2);
	wp_gesture_set_geometry(&eng, &g2, &out);
	zassert_equal(out.buttons, 0);
	zassert_true(out.events & WP_GEST_EV_DRAG_END);
	zassert_equal(eng.geom.max_x, UMAX_Y);
	zassert_equal(eng.geom.max_y, UMAX_X);
	zassert_within(eng.mm_per_count_x, 48.0f / 960.0f, 1e-6f);
}

ZTEST(gestures, test_single_still_timing)
{
	frame1(0, 30.0f, 20.0f);
	zassert_false(out.single_still, "no stillness at touch-down instant");
	zassert_equal(out.single_still_ms, 0);
	for (int i = 1; i <= 30; i++) {
		frame1(MS(10 * i), 30.0f + ((i & 1) ? 0.2f : 0.0f), 20.0f);
	}
	zassert_true(out.single_still);
	zassert_equal(out.single_still_ms, 300);

	/* Move 2 mm: the still timer restarts. */
	frame1(MS(310), 32.0f, 20.0f);
	zassert_false(out.single_still);
	zassert_equal(out.single_still_ms, 0);
	tick(MS(510));
	zassert_true(out.single_still);
	zassert_equal(out.single_still_ms, 200);

	/* Two fingers: never "single still". */
	frame2(MS(520), 32.0f, 20.0f, 45.0f, 20.0f);
	tick(MS(900));
	zassert_false(out.single_still);
	zassert_equal(out.single_still_ms, 0);

	/* Back to one: restarts from the change. */
	frame1(MS(910), 32.0f, 20.0f);
	tick(MS(1010));
	zassert_true(out.single_still);
	zassert_equal(out.single_still_ms, 100);
	lift(MS(1020));
	zassert_false(out.single_still);
}

ZTEST(gestures, test_invalid_geometry_safe)
{
	const struct wp_pad_geometry bad = {0};

	wp_gesture_init(&eng, NULL, &bad);
	acc_clear();
	frame1(0, 10.0f, 10.0f);
	frame1(MS(10), 20.0f, 30.0f);
	lift(MS(50));
	tick(MS(400));
	zassert_within(A.max_step, 0.0f, 1e-6f);
	zassert_equal(A.left_clicks, 1);

	/* NULL frame / NULL output are tolerated. */
	wp_gesture_process(&eng, NULL, &out);
	zassert_equal(out.contacts, 0);
	wp_gesture_tick(&eng, MS(100), NULL);
	wp_gesture_reset(&eng, NULL);
}

/* ------------------------------------------------------------------------ */
/* Orientation consistency                                                   */
/* ------------------------------------------------------------------------ */

/* Pad frame P: 64 x 48 mm, but anisotropic resolution (0.05 / 0.1 mm per
 * count) so that a wrong geometry swap would show up as a wrong magnitude. */
static const struct wp_pad_geometry pad_geom = {
	.max_x = 1280, .max_y = 480, .width_mm = 64.0f, .height_mm = 48.0f};

/* Swipe one finger in the PAD frame by (dx_mm, dy_mm) in 20 frames, feed it
 * through wp_xform_frame() for orientation @p o, return pointer counts. */
static void pad_swipe(enum wp_orientation o, float dx_mm, float dy_mm, float *ox, float *oy)
{
	struct wp_pad_geometry ug;
	struct wp_gesture_config c;
	struct wp_touch_frame pf = {0};
	struct wp_touch_frame uf;
	float px = 30.0f, py = 24.0f;

	plain_cfg(&c);
	wp_xform_geometry(o, &pad_geom, &ug);
	wp_gesture_init(&eng, &c, &ug);
	acc_clear();

	pf.max_x = pad_geom.max_x;
	pf.max_y = pad_geom.max_y;
	pf.count = 1;
	pf.contacts[0].id = 3;
	pf.contacts[0].strength = 100;
	for (int i = 0; i <= 20; i++) {
		pf.t_us = MS(10 * i);
		pf.contacts[0].x = (uint16_t)(px / 0.05f + 0.5f);
		pf.contacts[0].y = (uint16_t)(py / 0.1f + 0.5f);
		wp_xform_frame(o, &pf, &uf);
		wp_gesture_process(&eng, &uf, &out);
		accumulate();
		px += dx_mm / 20.0f;
		py += dy_mm / 20.0f;
	}
	pf.count = 0;
	pf.t_us = MS(220);
	wp_xform_frame(o, &pf, &uf);
	wp_gesture_process(&eng, &uf, &out);
	*ox = A.dx;
	*oy = A.dy;
}

ZTEST(gestures, test_orientation_90_pointer_direction)
{
	float ox, oy;

	/* 90 deg: the pad's LEFT edge faces away from the user, so a swipe
	 * towards the pad's right edge (+x_P) moves towards the user: the
	 * pointer must go DOWN (+y) by the same distance in mm. */
	pad_swipe(WP_ORIENT_90, 10.0f, 0.0f, &ox, &oy);
	zassert_within(oy, 10.0f * CPM, 0.5f, "dy=%f", (double)oy);
	zassert_within(ox, 0.0f, 0.5f);

	/* A swipe towards the pad's bottom edge (+y_P) goes to the user's LEFT. */
	pad_swipe(WP_ORIENT_90, 0.0f, 10.0f, &ox, &oy);
	zassert_within(ox, -10.0f * CPM, 0.5f, "dx=%f", (double)ox);
	zassert_within(oy, 0.0f, 0.5f);
}

ZTEST(gestures, test_orientation_all_pointer_directions)
{
	/* Delta mapping P -> U from types.h:
	 * 0: (dx, dy)  90: (-dy, dx)  180: (-dx, -dy)  270: (dy, -dx) */
	static const float ex[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}}; /* +x_P */
	static const float ey[4][2] = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}}; /* +y_P */
	float ox, oy;

	for (int o = 0; o < WP_ORIENT_COUNT; o++) {
		pad_swipe((enum wp_orientation)o, 8.0f, 0.0f, &ox, &oy);
		zassert_within(ox, ex[o][0] * 8.0f * CPM, 0.5f, "o=%d", o);
		zassert_within(oy, ex[o][1] * 8.0f * CPM, 0.5f, "o=%d", o);
		pad_swipe((enum wp_orientation)o, 0.0f, 8.0f, &ox, &oy);
		zassert_within(ox, ey[o][0] * 8.0f * CPM, 0.5f, "o=%d", o);
		zassert_within(oy, ey[o][1] * 8.0f * CPM, 0.5f, "o=%d", o);
	}
}

ZTEST(gestures, test_orientation_90_scroll_direction)
{
	struct wp_pad_geometry ug;
	struct wp_touch_frame pf = {0};
	struct wp_touch_frame uf;
	float px = 20.0f;

	/* Two fingers swiping towards the pad's right edge at 90 deg = towards
	 * the user: natural scrolling -> wheel up (+), no pan. */
	wp_xform_geometry(WP_ORIENT_90, &pad_geom, &ug);
	wp_gesture_init(&eng, NULL, &ug);
	acc_clear();
	pf.max_x = pad_geom.max_x;
	pf.max_y = pad_geom.max_y;
	pf.count = 2;
	pf.contacts[0].id = 0;
	pf.contacts[1].id = 1;
	for (int i = 0; i <= 20; i++) {
		pf.t_us = MS(10 * i);
		pf.contacts[0].x = (uint16_t)(px / 0.05f + 0.5f);
		pf.contacts[0].y = 150;
		pf.contacts[1].x = (uint16_t)(px / 0.05f + 0.5f);
		pf.contacts[1].y = 300;
		wp_xform_frame(WP_ORIENT_90, &pf, &uf);
		wp_gesture_process(&eng, &uf, &out);
		accumulate();
		px += SSTEP;
	}
	zassert_within(A.wheel, SCROLL_EXPECT, 0.02f, "wheel=%f", (double)A.wheel);
	zassert_within(A.pan, 0.0f, 1e-6f);
}
