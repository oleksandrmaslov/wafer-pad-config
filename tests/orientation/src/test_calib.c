/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Algorithm C: manual calibration gesture.
 */

#include <zephyr/ztest.h>

#include "helpers.h"

/* 0.1 mm per count on both axes. */
#define MX 650
#define MY 490
static const struct wp_pad_geometry geom = {
	.max_x = MX, .max_y = MY, .width_mm = 65.0f, .height_mm = 49.0f};

#define FRAME_MS 20

static struct wp_calib c;
static int64_t t;
static uint16_t x1, y1_, x2, y2;

struct evs {
	int armed;
	int cancelled;
	int completed;
	struct wp_calib_result last_done;
	bool inhibit_seen;
};

static struct evs ev;

static struct wp_calib_config mech_cfg;

static void before(void *f)
{
	ARG_UNUSED(f);
	calib_mech_config(&mech_cfg);
	wp_calib_init(&c, &mech_cfg, &geom);
	t = 0;
	x1 = 300;
	y1_ = 250;
	x2 = 360;
	y2 = 250;
	memset(&ev, 0, sizeof(ev));
}

static void account(const struct wp_calib_result *r)
{
	switch (r->event) {
	case WP_CALIB_EV_ARMED:
		ev.armed++;
		break;
	case WP_CALIB_EV_CANCELLED:
		ev.cancelled++;
		break;
	case WP_CALIB_EV_COMPLETED:
		ev.completed++;
		ev.last_done = *r;
		break;
	default:
		break;
	}
	if (wp_calib_inhibits_scroll(&c)) {
		ev.inhibit_seen = true;
	}
}

static void feed(const struct wp_touch_frame *f)
{
	struct wp_calib_result r;

	wp_calib_update(&c, f, &r);
	account(&r);
}

static void tick(int64_t at)
{
	struct wp_calib_result r;

	wp_calib_tick(&c, at, &r);
	account(&r);
}

/** Two fingers at the current position for @p ms (frames every 20 ms). */
static void rest(int ms)
{
	for (int k = 0; k < ms / FRAME_MS; k++) {
		struct wp_touch_frame f = frame2(t, MX, MY, x1, y1_, x2, y2);

		feed(&f);
		t += MS(FRAME_MS);
	}
}

/** Move both fingers by (dx, dy) counts per frame for @p n frames. */
static void move(int dx, int dy, int n)
{
	for (int k = 0; k < n; k++) {
		x1 = (uint16_t)(x1 + dx);
		x2 = (uint16_t)(x2 + dx);
		y1_ = (uint16_t)(y1_ + dy);
		y2 = (uint16_t)(y2 + dy);
		struct wp_touch_frame f = frame2(t, MX, MY, x1, y1_, x2, y2);

		feed(&f);
		t += MS(FRAME_MS);
	}
}

static void lift(void)
{
	struct wp_touch_frame f = frame0(t, MX, MY);

	feed(&f);
	t += MS(FRAME_MS);
}

/** Full gesture: rest 720 ms, swipe (dx, dy) per frame for n frames, lift. */
static void gesture(int dx, int dy, int n)
{
	rest(720);
	zassert_equal(ev.armed, 1, "hold must arm");
	move(dx, dy, n);
	lift();
}

ZTEST(orient_calib, test_defaults)
{
	struct wp_calib_config d;

	wp_calib_config_default(&d);
	zassert_equal(d.fingers, 3);
	zassert_equal(d.hold_ms, 1500);
	zassert_within(d.hold_max_move_mm, 1.5f, 1e-6f);
	zassert_equal(d.arm_timeout_ms, 2500);
	zassert_within(d.swipe_min_mm, 25.0f, 1e-6f);
	zassert_equal(d.swipe_max_ms, 450);
	zassert_within(d.swipe_max_angle_dev_deg, 30.0f, 1e-6f);
	zassert_equal(d.lift_max_ms, 250);
	zassert_within(d.swipe_start_mm, 2.0f, 1e-6f);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_IDLE);
}

ZTEST(orient_calib, test_four_directions)
{
	static const struct {
		int dx, dy;
		enum wp_orientation o;
		float angle;
	} cases[] = {
		{0, -20, WP_ORIENT_0, 0.0f},    /* towards top edge (-y_P) */
		{-20, 0, WP_ORIENT_90, 90.0f},  /* towards left edge (-x_P) */
		{0, 20, WP_ORIENT_180, 180.0f}, /* towards bottom edge (+y_P) */
		{20, 0, WP_ORIENT_270, 270.0f}, /* towards right edge (+x_P) */
	};

	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		before(NULL);
		gesture(cases[i].dx, cases[i].dy, 10); /* 20 mm in 200 ms */
		zassert_equal(ev.completed, 1, "case %u", (unsigned int)i);
		zassert_equal(ev.cancelled, 0, "case %u", (unsigned int)i);
		zassert_equal(ev.last_done.orientation, cases[i].o, "case %u", (unsigned int)i);
		zassert_within(ev.last_done.angle_deg, cases[i].angle, 0.5f);
		zassert_true(ev.inhibit_seen);
		zassert_equal(wp_calib_phase_get(&c), WP_CALIB_IDLE);
		zassert_false(wp_calib_inhibits_scroll(&c));
	}
}

ZTEST(orient_calib, test_phases_and_scroll_inhibit)
{
	struct wp_touch_frame f = frame2(t, MX, MY, x1, y1_, x2, y2);

	feed(&f);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_HOLDING);
	zassert_false(wp_calib_inhibits_scroll(&c), "holding must not block a scroll");
	t += MS(FRAME_MS);
	rest(700);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_ARMED);
	zassert_true(wp_calib_inhibits_scroll(&c));
	move(0, -20, 2);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_SWIPING);
	zassert_true(wp_calib_inhibits_scroll(&c));
	move(0, -20, 7);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_WAIT_LIFT);
	zassert_true(wp_calib_inhibits_scroll(&c));
	lift();
	zassert_equal(ev.completed, 1);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_IDLE);
}

ZTEST(orient_calib, test_immediate_two_finger_scroll_is_not_calibration)
{
	/* Fingers land and start scrolling right away (1 mm per frame). */
	for (int k = 0; k < 100; k++) {
		move(0, 10, 1);
		if (y1_ > 400) {
			y1_ = 100;
			y2 = 100;
		}
		zassert_false(wp_calib_inhibits_scroll(&c));
	}
	/* Then the fingers rest for a while without lifting: still the same
	 * scroll session, never arms.
	 */
	rest(2000);
	zassert_equal(ev.armed, 0);
	zassert_equal(ev.completed, 0);
	zassert_equal(ev.cancelled, 0, "never armed -> no CANCELLED event");
	zassert_false(ev.inhibit_seen);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_REJECTED);
	lift();
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_IDLE);
}

ZTEST(orient_calib, test_scroll_starting_after_short_pause_not_calibration)
{
	rest(400); /* shorter than hold_ms */
	move(0, 10, 30);
	rest(1000);
	zassert_equal(ev.armed, 0);
	zassert_false(ev.inhibit_seen);
}

ZTEST(orient_calib, test_armed_then_slow_drag_cancels_and_scroll_resumes)
{
	rest(720);
	zassert_equal(ev.armed, 1);
	/* 0.5 mm per 20 ms = 25 mm/s: a slow scroll, not a flick. */
	move(0, 5, 50);
	zassert_equal(ev.cancelled, 1);
	zassert_equal(ev.completed, 0);
	zassert_false(wp_calib_inhibits_scroll(&c), "scroll must resume");
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_REJECTED);

	/* Continuing to drag and even flicking afterwards does nothing. */
	move(0, -20, 10);
	lift();
	zassert_equal(ev.completed, 0);
	zassert_equal(ev.cancelled, 1);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_IDLE);
}

ZTEST(orient_calib, test_diagonal_swipe_no_result)
{
	gesture(-14, -14, 10); /* 45 deg */
	zassert_equal(ev.completed, 0);
	zassert_equal(ev.cancelled, 1);

	before(NULL);
	gesture(-12, -17, 10); /* ~35 deg off the -y axis */
	zassert_equal(ev.completed, 0);

	before(NULL);
	gesture(-7, -19, 10); /* ~20 deg off: accepted as 0 */
	zassert_equal(ev.completed, 1);
	zassert_equal(ev.last_done.orientation, WP_ORIENT_0);
	zassert_within(ev.last_done.angle_deg, 20.2f, 0.5f);
}

ZTEST(orient_calib, test_third_finger_rejected)
{
	static const uint16_t xy3[] = {300, 250, 360, 250, 420, 250};
	struct wp_touch_frame f;

	/* Third finger while holding: session rejected silently. */
	rest(300);
	f = frame_n(t, MX, MY, 3, xy3);
	feed(&f);
	t += MS(FRAME_MS);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_REJECTED);
	/* Back to two fingers, resting long: still rejected until all lift. */
	rest(1500);
	zassert_equal(ev.armed, 0);
	lift();
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_IDLE);

	/* Third finger once armed: CANCELLED. */
	before(NULL);
	rest(720);
	zassert_equal(ev.armed, 1);
	f = frame_n(t, MX, MY, 3, xy3);
	feed(&f);
	t += MS(FRAME_MS);
	zassert_equal(ev.cancelled, 1);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_REJECTED);
	move(0, -20, 10);
	lift();
	zassert_equal(ev.completed, 0);

	/* Three fingers landing at once. */
	before(NULL);
	f = frame_n(t, MX, MY, 3, xy3);
	feed(&f);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_REJECTED);
}

ZTEST(orient_calib, test_finger_lifted_while_armed_cancels)
{
	static const uint16_t xy1[] = {300, 250};
	struct wp_touch_frame f;

	rest(720);
	f = frame_n(t, MX, MY, 1, xy1);
	feed(&f);
	zassert_equal(ev.cancelled, 1);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_REJECTED);
}

ZTEST(orient_calib, test_arm_timeout)
{
	rest(720);
	zassert_equal(ev.armed, 1);
	int64_t armed_at = MS(700);

	tick(armed_at + MS(2500));
	zassert_equal(ev.cancelled, 0);
	zassert_true(wp_calib_inhibits_scroll(&c));
	tick(armed_at + MS(2501));
	zassert_equal(ev.cancelled, 1);
	zassert_false(wp_calib_inhibits_scroll(&c));

	/* Same through frames: resting armed for > 2.5 s. */
	before(NULL);
	rest(720 + 2600);
	zassert_equal(ev.armed, 1);
	zassert_equal(ev.cancelled, 1);
}

ZTEST(orient_calib, test_hold_arms_by_tick_without_frames)
{
	/* The controller may stay silent while fingers rest. */
	struct wp_touch_frame f = frame2(MS(0), MX, MY, x1, y1_, x2, y2);

	feed(&f);
	tick(MS(699));
	zassert_equal(ev.armed, 0);
	tick(MS(700));
	zassert_equal(ev.armed, 1);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_ARMED);

	t = MS(720);
	move(20, 0, 10);
	lift();
	zassert_equal(ev.completed, 1);
	zassert_equal(ev.last_done.orientation, WP_ORIENT_270);
}

ZTEST(orient_calib, test_late_lift_cancels)
{
	rest(720);
	move(0, 20, 10);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_WAIT_LIFT);
	rest(300); /* fingers stay down > lift_max_ms */
	zassert_equal(ev.cancelled, 1);
	lift();
	zassert_equal(ev.completed, 0);

	/* Same via tick. */
	before(NULL);
	rest(720);
	move(0, 20, 8);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_WAIT_LIFT);
	tick(t + MS(300));
	zassert_equal(ev.cancelled, 1);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_REJECTED);
}

ZTEST(orient_calib, test_staggered_landing_and_lift)
{
	static const uint16_t xy1[] = {300, 250};
	struct wp_touch_frame f;

	/* First finger lands alone, the second 40 ms later. */
	f = frame_n(t, MX, MY, 1, xy1);
	feed(&f);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_IDLE);
	t += MS(40);
	rest(700);
	zassert_equal(ev.armed, 0, "hold counts from the second finger");
	rest(40);
	zassert_equal(ev.armed, 1);

	move(-20, 0, 10);
	/* Fingers lift one after the other. */
	static const uint16_t xy_rest[] = {0, 0};

	f = frame_n(t, MX, MY, 1, xy_rest);
	f.contacts[0].id = 2;
	f.contacts[0].x = x2;
	f.contacts[0].y = y2;
	feed(&f);
	t += MS(FRAME_MS);
	zassert_equal(ev.cancelled, 0);
	lift();
	zassert_equal(ev.completed, 1);
	zassert_equal(ev.last_done.orientation, WP_ORIENT_90);
}

ZTEST(orient_calib, test_single_finger_never_arms)
{
	static const uint16_t xy1[] = {300, 250};

	for (int k = 0; k < 100; k++) {
		struct wp_touch_frame f = frame_n(t, MX, MY, 1, xy1);

		feed(&f);
		t += MS(FRAME_MS);
	}
	zassert_equal(ev.armed, 0);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_IDLE);
}

ZTEST(orient_calib, test_hold_jitter_tolerance)
{
	/* 1.4 mm peak-to-peak jitter (+-7 counts) is still a rest. */
	for (int k = 0; k < 40; k++) {
		int j = (k & 1) ? 7 : -7;
		struct wp_touch_frame f =
			frame2(t, MX, MY, x1 + j, y1_, x2, (uint16_t)(y2 + j / 2));

		feed(&f);
		t += MS(FRAME_MS);
	}
	zassert_equal(ev.armed, 1);

	/* One finger moving 2 mm is not. */
	before(NULL);
	rest(200);
	x1 += 20;
	rest(800);
	zassert_equal(ev.armed, 0);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_REJECTED);
}

ZTEST(orient_calib, test_sparse_frames_single_jump)
{
	rest(720);
	move(0, -170, 1); /* 17 mm in one frame */
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_WAIT_LIFT);
	lift();
	zassert_equal(ev.completed, 1);
	zassert_equal(ev.last_done.orientation, WP_ORIENT_0);
}

ZTEST(orient_calib, test_swipe_too_slow_but_long)
{
	rest(720);
	/* 1 mm per frame: 15 mm after 300 ms of swiping... */
	move(0, 10, 3);
	/* ...then hesitation: 450 ms budget exceeded before reaching 15 mm. */
	rest(500);
	move(0, 10, 15);
	lift();
	zassert_equal(ev.completed, 0);
	zassert_equal(ev.cancelled, 1);
}

ZTEST(orient_calib, test_new_session_after_completion_and_reset)
{
	gesture(0, 20, 10);
	zassert_equal(ev.completed, 1);
	x1 = 300;
	y1_ = 250;
	x2 = 360;
	y2 = 250;
	ev.armed = 0;
	gesture(20, 0, 10);
	zassert_equal(ev.completed, 2);
	zassert_equal(ev.last_done.orientation, WP_ORIENT_270);

	rest(720);
	zassert_true(wp_calib_inhibits_scroll(&c));
	wp_calib_reset(&c);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_IDLE);
	zassert_false(wp_calib_inhibits_scroll(&c));
}

ZTEST(orient_calib, test_mm_conversion_uses_geometry)
{
	/* Real TPS65-like geometry: 2047 x 1535 counts over 65 x 49 mm. */
	const struct wp_pad_geometry g2 = {
		.max_x = 2047, .max_y = 1535, .width_mm = 65.0f, .height_mm = 49.0f};
	struct wp_calib_result r;
	int64_t tt = 0;

	/* 40 counts per frame along x = 1.27 mm; 10 frames = 12.7 mm < 15. */
	wp_calib_init(&c, &mech_cfg, &g2);
	for (; tt <= MS(720); tt += MS(20)) {
		struct wp_touch_frame f = frame2(tt, 2047, 1535, 1000, 700, 1100, 700);

		wp_calib_update(&c, &f, &r);
	}
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_ARMED);
	for (int k = 1; k <= 10; k++, tt += MS(20)) {
		struct wp_touch_frame f =
			frame2(tt, 2047, 1535, 1000 + 40 * k, 700, 1100 + 40 * k, 700);

		wp_calib_update(&c, &f, &r);
	}
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_SWIPING, "12.7 mm is not enough");

	/* Two more frames: 15.2 mm -> accepted. */
	for (int k = 11; k <= 12; k++, tt += MS(20)) {
		struct wp_touch_frame f =
			frame2(tt, 2047, 1535, 1000 + 40 * k, 700, 1100 + 40 * k, 700);

		wp_calib_update(&c, &f, &r);
	}
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_WAIT_LIFT);
	struct wp_touch_frame f0 = frame0(tt, 2047, 1535);

	wp_calib_update(&c, &f0, &r);
	zassert_equal(r.event, WP_CALIB_EV_COMPLETED);
	zassert_equal(r.orientation, WP_ORIENT_270);
}

ZTEST(orient_calib, test_three_finger_config)
{
	struct wp_calib_config cc;
	static const uint16_t xy3[] = {300, 250, 360, 250, 420, 250};
	struct wp_calib_result r;
	int64_t tt = 0;

	calib_mech_config(&cc);
	cc.fingers = 3;
	wp_calib_init(&c, &cc, &geom);
	struct wp_touch_frame f = frame2(tt, MX, MY, 300, 250, 360, 250);

	wp_calib_update(&c, &f, &r);
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_IDLE, "2 of 3 fingers: wait");
	for (tt = MS(20); tt <= MS(760); tt += MS(20)) {
		f = frame_n(tt, MX, MY, 3, xy3);
		wp_calib_update(&c, &f, &r);
	}
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_ARMED);
}

ZTEST_SUITE(orient_calib, NULL, NULL, before, NULL, NULL);
