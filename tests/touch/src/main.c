/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests: touch tracker (frame -> DOWN/MOVE/UP events), the fixed
 * mounting correction wp_touch_mount_apply() and the IQS5xx frame
 * conversion wp_touch_from_iqs5xx().
 */

#include <string.h>

#include <zephyr/ztest.h>

#include <wafer_pad/touch_iqs5xx.h>
#include <wafer_pad/touch_tracker.h>

#define MAX_X 3072
#define MAX_Y 2048

/* ---- helpers -------------------------------------------------------------- */

struct pt {
	uint8_t id;
	uint16_t x;
	uint16_t y;
};

static struct wp_touch_frame make_frame(int64_t t_us, const struct pt *pts, uint8_t n)
{
	struct wp_touch_frame f;

	memset(&f, 0, sizeof(f));
	f.t_us = t_us;
	f.max_x = MAX_X;
	f.max_y = MAX_Y;
	f.count = n;
	for (uint8_t i = 0; i < n; i++) {
		f.contacts[i].id = pts[i].id;
		f.contacts[i].x = pts[i].x;
		f.contacts[i].y = pts[i].y;
		f.contacts[i].strength = 100;
		f.contacts[i].area = 3;
	}
	return f;
}

static size_t feed(struct wp_touch_tracker *t, int64_t t_us, const struct pt *pts, uint8_t n,
		   struct wp_touch_event *ev)
{
	struct wp_touch_frame f = make_frame(t_us, pts, n);

	return wp_touch_tracker_update(t, &f, ev, WP_TOUCH_TRACKER_MAX_EVENTS);
}

static void check_ev(const struct wp_touch_event *e, enum wp_touch_event_type type, uint8_t id,
		     uint16_t x, uint16_t y, int32_t dx, int32_t dy, int64_t t_us)
{
	zassert_equal(e->type, type, "type %d != %d (id %u)", e->type, type, id);
	zassert_equal(e->id, id, "id %u != %u", e->id, id);
	zassert_equal(e->x, x, "x %u != %u (id %u)", e->x, x, id);
	zassert_equal(e->y, y, "y %u != %u (id %u)", e->y, y, id);
	zassert_equal(e->dx, dx, "dx %d != %d (id %u)", e->dx, dx, id);
	zassert_equal(e->dy, dy, "dy %d != %d (id %u)", e->dy, dy, id);
	zassert_equal(e->t_us, t_us, "t_us mismatch (id %u)", id);
}

/* ---- tracker -------------------------------------------------------------- */

ZTEST(touch_tracker, test_init_clears_state)
{
	struct wp_touch_tracker t;

	memset(&t, 0xA5, sizeof(t));
	wp_touch_tracker_init(&t);
	zassert_equal(wp_touch_tracker_count(&t), 0);
	zassert_equal(t.last_t_us, 0);
}

ZTEST(touch_tracker, test_down_move_up_single_finger)
{
	struct wp_touch_tracker t;
	struct wp_touch_event ev[WP_TOUCH_TRACKER_MAX_EVENTS];
	const struct pt p0 = {0, 100, 200};
	const struct pt p1 = {0, 110, 190};
	size_t n;

	wp_touch_tracker_init(&t);

	n = feed(&t, 1000, &p0, 1, ev);
	zassert_equal(n, 1);
	check_ev(&ev[0], WP_TOUCH_DOWN, 0, 100, 200, 0, 0, 1000);
	zassert_equal(wp_touch_tracker_count(&t), 1);
	zassert_equal(t.last_t_us, 1000);

	n = feed(&t, 2000, &p1, 1, ev);
	zassert_equal(n, 1);
	check_ev(&ev[0], WP_TOUCH_MOVE, 0, 110, 190, 10, -10, 2000);

	n = feed(&t, 3000, NULL, 0, ev);
	zassert_equal(n, 1);
	/* UP carries the last known position and zero delta. */
	check_ev(&ev[0], WP_TOUCH_UP, 0, 110, 190, 0, 0, 3000);
	zassert_equal(wp_touch_tracker_count(&t), 0);

	/* Nothing down, nothing to report. */
	n = feed(&t, 4000, NULL, 0, ev);
	zassert_equal(n, 0);
}

ZTEST(touch_tracker, test_no_move_without_change)
{
	struct wp_touch_tracker t;
	struct wp_touch_event ev[WP_TOUCH_TRACKER_MAX_EVENTS];
	const struct pt a[2] = {{1, 500, 600}, {3, 700, 800}};
	const struct pt b[2] = {{1, 500, 600}, {3, 700, 801}};
	size_t n;

	wp_touch_tracker_init(&t);
	zassert_equal(feed(&t, 10, a, 2, ev), 2);

	/* Identical frame: no events at all. */
	n = feed(&t, 20, a, 2, ev);
	zassert_equal(n, 0);
	zassert_equal(t.last_t_us, 20);

	/* Only id 3 changed (by one count in y). */
	n = feed(&t, 30, b, 2, ev);
	zassert_equal(n, 1);
	check_ev(&ev[0], WP_TOUCH_MOVE, 3, 700, 801, 0, 1, 30);

	/* A single-axis change is still a move; x-only this time. */
	{
		const struct pt c[2] = {{1, 499, 600}, {3, 700, 801}};

		n = feed(&t, 40, c, 2, ev);
		zassert_equal(n, 1);
		check_ev(&ev[0], WP_TOUCH_MOVE, 1, 499, 600, -1, 0, 40);
	}
}

ZTEST(touch_tracker, test_event_ordering_up_down_move)
{
	struct wp_touch_tracker t;
	struct wp_touch_event ev[WP_TOUCH_TRACKER_MAX_EVENTS];
	/* Frame 1: ids 1, 2, 4 down. */
	const struct pt f1[3] = {{1, 10, 10}, {2, 20, 20}, {4, 40, 40}};
	/* Frame 2: 1 and 4 lifted, 2 moved, 0 and 3 new. Contacts are given in
	 * ascending id order, as the frame contract requires.
	 */
	const struct pt f2[3] = {{0, 5, 6}, {2, 25, 18}, {3, 33, 34}};
	size_t n;

	wp_touch_tracker_init(&t);
	n = feed(&t, 100, f1, 3, ev);
	zassert_equal(n, 3);
	check_ev(&ev[0], WP_TOUCH_DOWN, 1, 10, 10, 0, 0, 100);
	check_ev(&ev[1], WP_TOUCH_DOWN, 2, 20, 20, 0, 0, 100);
	check_ev(&ev[2], WP_TOUCH_DOWN, 4, 40, 40, 0, 0, 100);

	n = feed(&t, 200, f2, 3, ev);
	zassert_equal(n, 5);
	/* All UPs first (ascending), then DOWNs (ascending), then MOVEs. */
	check_ev(&ev[0], WP_TOUCH_UP, 1, 10, 10, 0, 0, 200);
	check_ev(&ev[1], WP_TOUCH_UP, 4, 40, 40, 0, 0, 200);
	check_ev(&ev[2], WP_TOUCH_DOWN, 0, 5, 6, 0, 0, 200);
	check_ev(&ev[3], WP_TOUCH_DOWN, 3, 33, 34, 0, 0, 200);
	check_ev(&ev[4], WP_TOUCH_MOVE, 2, 25, 18, 5, -2, 200);
	zassert_equal(wp_touch_tracker_count(&t), 3);
}

ZTEST(touch_tracker, test_large_jump_same_id_is_move)
{
	struct wp_touch_tracker t;
	struct wp_touch_event ev[WP_TOUCH_TRACKER_MAX_EVENTS];
	const struct pt a = {2, 0, 0};
	const struct pt b = {2, MAX_X, MAX_Y};
	size_t n;

	wp_touch_tracker_init(&t);
	feed(&t, 1, &a, 1, ev);

	/* Same id across the whole pad in one frame: MOVE, never UP + DOWN. */
	n = feed(&t, 2, &b, 1, ev);
	zassert_equal(n, 1);
	check_ev(&ev[0], WP_TOUCH_MOVE, 2, MAX_X, MAX_Y, MAX_X, MAX_Y, 2);

	/* And back: negative deltas over the full range. */
	n = feed(&t, 3, &a, 1, ev);
	zassert_equal(n, 1);
	check_ev(&ev[0], WP_TOUCH_MOVE, 2, 0, 0, -MAX_X, -MAX_Y, 3);
}

ZTEST(touch_tracker, test_id_reuse_after_lift)
{
	struct wp_touch_tracker t;
	struct wp_touch_event ev[WP_TOUCH_TRACKER_MAX_EVENTS];
	const struct pt first = {0, 100, 100};
	const struct pt second = {0, 2000, 1500};
	size_t n;

	wp_touch_tracker_init(&t);
	feed(&t, 10, &first, 1, ev);
	n = feed(&t, 20, NULL, 0, ev);
	zassert_equal(n, 1);
	check_ev(&ev[0], WP_TOUCH_UP, 0, 100, 100, 0, 0, 20);

	/* Slot 0 reused by a new finger: a fresh DOWN with zero delta (the old
	 * position must not leak into a MOVE delta).
	 */
	n = feed(&t, 30, &second, 1, ev);
	zassert_equal(n, 1);
	check_ev(&ev[0], WP_TOUCH_DOWN, 0, 2000, 1500, 0, 0, 30);

	n = feed(&t, 40, &(struct pt){0, 2001, 1500}, 1, ev);
	zassert_equal(n, 1);
	check_ev(&ev[0], WP_TOUCH_MOVE, 0, 2001, 1500, 1, 0, 40);
}

ZTEST(touch_tracker, test_release_all)
{
	struct wp_touch_tracker t;
	struct wp_touch_event ev[WP_TOUCH_TRACKER_MAX_EVENTS];
	const struct pt pts[3] = {{0, 1, 2}, {2, 3, 4}, {4, 5, 6}};
	const struct pt moved[3] = {{0, 11, 12}, {2, 3, 4}, {4, 5, 6}};
	size_t n;

	wp_touch_tracker_init(&t);
	feed(&t, 100, pts, 3, ev);
	feed(&t, 150, moved, 3, ev);
	zassert_equal(wp_touch_tracker_count(&t), 3);

	n = wp_touch_tracker_release_all(&t, 500, ev, WP_TOUCH_TRACKER_MAX_EVENTS);
	zassert_equal(n, 3);
	check_ev(&ev[0], WP_TOUCH_UP, 0, 11, 12, 0, 0, 500);
	check_ev(&ev[1], WP_TOUCH_UP, 2, 3, 4, 0, 0, 500);
	check_ev(&ev[2], WP_TOUCH_UP, 4, 5, 6, 0, 0, 500);
	zassert_equal(wp_touch_tracker_count(&t), 0);
	zassert_equal(t.last_t_us, 500);

	/* Idempotent. */
	zassert_equal(wp_touch_tracker_release_all(&t, 600, ev, WP_TOUCH_TRACKER_MAX_EVENTS), 0);

	/* The same fingers seen again after a reset are new DOWNs. */
	n = feed(&t, 700, moved, 3, ev);
	zassert_equal(n, 3);
	for (size_t i = 0; i < n; i++) {
		zassert_equal(ev[i].type, WP_TOUCH_DOWN);
		zassert_equal(ev[i].dx, 0);
		zassert_equal(ev[i].dy, 0);
	}
}

ZTEST(touch_tracker, test_release_all_overflow_still_clears)
{
	struct wp_touch_tracker t;
	struct wp_touch_event ev[1];
	const struct pt pts[2] = {{1, 1, 1}, {3, 3, 3}};
	struct wp_touch_event big[WP_TOUCH_TRACKER_MAX_EVENTS];
	size_t n;

	wp_touch_tracker_init(&t);
	feed(&t, 1, pts, 2, big);

	n = wp_touch_tracker_release_all(&t, 2, ev, 1);
	zassert_equal(n, 1);
	check_ev(&ev[0], WP_TOUCH_UP, 1, 1, 1, 0, 0, 2);
	/* Surplus UP dropped, but no phantom finger is left behind. */
	zassert_equal(wp_touch_tracker_count(&t), 0);
	zassert_equal(wp_touch_tracker_release_all(&t, 3, NULL, 0), 0);
}

ZTEST(touch_tracker, test_overflow_drops_surplus_keeps_state)
{
	struct wp_touch_tracker t;
	struct wp_touch_event ev[WP_TOUCH_TRACKER_MAX_EVENTS];
	struct wp_touch_frame f;
	const struct pt a[3] = {{0, 10, 10}, {1, 20, 20}, {2, 30, 30}};
	const struct pt b[3] = {{0, 11, 10}, {1, 21, 20}, {2, 31, 30}};
	size_t n;

	wp_touch_tracker_init(&t);

	/* Room for 2 of 3 DOWNs: the first two (ascending id) are kept. */
	f = make_frame(1, a, 3);
	n = wp_touch_tracker_update(&t, &f, ev, 2);
	zassert_equal(n, 2);
	zassert_equal(ev[0].id, 0);
	zassert_equal(ev[1].id, 1);
	/* State still has all three fingers. */
	zassert_equal(wp_touch_tracker_count(&t), 3);

	/* No room at all (NULL buffer): state is updated anyway. */
	f = make_frame(2, b, 3);
	n = wp_touch_tracker_update(&t, &f, NULL, 0);
	zassert_equal(n, 0);

	/* Next identical frame: no DOWN for id 2 and no MOVE (positions were
	 * taken over silently).
	 */
	n = feed(&t, 3, b, 3, ev);
	zassert_equal(n, 0);

	/* max_events = 0 with a valid buffer writes nothing. */
	memset(ev, 0x5A, sizeof(ev));
	f = make_frame(4, NULL, 0);
	n = wp_touch_tracker_update(&t, &f, ev, 0);
	zassert_equal(n, 0);
	zassert_equal(((uint8_t *)ev)[0], 0x5A, "buffer must not be touched");
	zassert_equal(wp_touch_tracker_count(&t), 0);
}

ZTEST(touch_tracker, test_max_events_bound_is_reachable)
{
	struct wp_touch_tracker t;
	struct wp_touch_event ev[WP_TOUCH_TRACKER_MAX_EVENTS];
	const struct pt low[5] = {{0, 1, 1}, {1, 2, 2}, {2, 3, 3}, {3, 4, 4}, {4, 5, 5}};
	const struct pt high[5] = {{5, 1, 1}, {6, 2, 2}, {7, 3, 3}, {8, 4, 4}, {9, 5, 5}};
	size_t n;

	wp_touch_tracker_init(&t);
	zassert_equal(feed(&t, 1, low, 5, ev), 5);

	/* Five lifts and five new fingers in one frame = 2 * MAX_CONTACTS. */
	n = feed(&t, 2, high, 5, ev);
	zassert_equal(n, WP_TOUCH_TRACKER_MAX_EVENTS);
	for (uint8_t i = 0; i < 5; i++) {
		zassert_equal(ev[i].type, WP_TOUCH_UP);
		zassert_equal(ev[i].id, i);
		zassert_equal(ev[5 + i].type, WP_TOUCH_DOWN);
		zassert_equal(ev[5 + i].id, 5 + i);
	}
}

ZTEST(touch_tracker, test_invalid_ids_and_counts)
{
	struct wp_touch_tracker t;
	struct wp_touch_event ev[WP_TOUCH_TRACKER_MAX_EVENTS];
	struct wp_touch_frame f;
	const struct pt pts[3] = {
		{WP_TOUCH_TRACKER_MAX_ID, 1, 1},     /* out of range: ignored */
		{WP_TOUCH_TRACKER_MAX_ID - 1, 2, 2}, /* highest valid id */
		{WP_TOUCH_TRACKER_MAX_ID - 1, 9, 9}, /* duplicate: first wins */
	};
	size_t n;

	wp_touch_tracker_init(&t);
	n = feed(&t, 1, pts, 3, ev);
	zassert_equal(n, 1);
	check_ev(&ev[0], WP_TOUCH_DOWN, WP_TOUCH_TRACKER_MAX_ID - 1, 2, 2, 0, 0, 1);
	zassert_equal(wp_touch_tracker_count(&t), 1);

	/* A corrupt count larger than the contact array is clamped. */
	f = make_frame(2, NULL, 0);
	f.count = 200;
	n = wp_touch_tracker_update(&t, &f, ev, WP_TOUCH_TRACKER_MAX_EVENTS);
	zassert_true(n <= WP_TOUCH_TRACKER_MAX_EVENTS);
	zassert_true(wp_touch_tracker_count(&t) <= WP_TOUCH_MAX_CONTACTS);

	/* NULL arguments are rejected without crashing. */
	zassert_equal(wp_touch_tracker_update(NULL, &f, ev, 1), 0);
	zassert_equal(wp_touch_tracker_update(&t, NULL, ev, 1), 0);
	zassert_equal(wp_touch_tracker_count(NULL), 0);
}

ZTEST_SUITE(touch_tracker, NULL, NULL, NULL, NULL, NULL);

/* ---- mounting correction ---------------------------------------------------- */

/* Sensor points: four corners and one interior point. */
static const struct pt sensor_pts[5] = {
	{0, 0, 0},         /* top-left */
	{1, MAX_X, 0},     /* top-right */
	{2, 0, MAX_Y},     /* bottom-left */
	{3, MAX_X, MAX_Y}, /* bottom-right */
	{4, 100, 200},     /* interior, asymmetric */
};

struct mount_case {
	struct wp_touch_mount m;
	uint16_t max_x, max_y;
	uint16_t x[5], y[5];
};

/* Expected results written out by hand (swap first, then invert with the
 * post-swap extents).
 */
static const struct mount_case mount_cases[8] = {
	{{false, false, false}, MAX_X, MAX_Y,
	 {0, MAX_X, 0, MAX_X, 100}, {0, 0, MAX_Y, MAX_Y, 200}},
	{{false, true, false}, MAX_X, MAX_Y,
	 {MAX_X, 0, MAX_X, 0, 2972}, {0, 0, MAX_Y, MAX_Y, 200}},
	{{false, false, true}, MAX_X, MAX_Y,
	 {0, MAX_X, 0, MAX_X, 100}, {MAX_Y, MAX_Y, 0, 0, 1848}},
	{{false, true, true}, MAX_X, MAX_Y,
	 {MAX_X, 0, MAX_X, 0, 2972}, {MAX_Y, MAX_Y, 0, 0, 1848}},
	{{true, false, false}, MAX_Y, MAX_X,
	 {0, 0, MAX_Y, MAX_Y, 200}, {0, MAX_X, 0, MAX_X, 100}},
	{{true, true, false}, MAX_Y, MAX_X,
	 {MAX_Y, MAX_Y, 0, 0, 1848}, {0, MAX_X, 0, MAX_X, 100}},
	{{true, false, true}, MAX_Y, MAX_X,
	 {0, 0, MAX_Y, MAX_Y, 200}, {MAX_X, 0, MAX_X, 0, 2972}},
	{{true, true, true}, MAX_Y, MAX_X,
	 {MAX_Y, MAX_Y, 0, 0, 1848}, {MAX_X, 0, MAX_X, 0, 2972}},
};

ZTEST(touch_mount, test_all_eight_combinations)
{
	for (size_t c = 0; c < ARRAY_SIZE(mount_cases); c++) {
		const struct mount_case *mc = &mount_cases[c];
		struct wp_touch_frame in = make_frame(777, sensor_pts, 5);
		struct wp_touch_frame out;

		memset(&out, 0xEE, sizeof(out));
		wp_touch_mount_apply(&mc->m, &in, &out);

		zassert_equal(out.max_x, mc->max_x, "case %zu max_x", c);
		zassert_equal(out.max_y, mc->max_y, "case %zu max_y", c);
		zassert_equal(out.count, 5, "case %zu count", c);
		zassert_equal(out.t_us, 777, "case %zu t_us", c);
		for (int i = 0; i < 5; i++) {
			zassert_equal(out.contacts[i].x, mc->x[i], "case %zu pt %d x=%u want %u", c,
				      i, out.contacts[i].x, mc->x[i]);
			zassert_equal(out.contacts[i].y, mc->y[i], "case %zu pt %d y=%u want %u", c,
				      i, out.contacts[i].y, mc->y[i]);
			/* Id, strength and area travel unchanged; order is kept. */
			zassert_equal(out.contacts[i].id, i, "case %zu id", c);
			zassert_equal(out.contacts[i].strength, 100, "case %zu strength", c);
			zassert_equal(out.contacts[i].area, 3, "case %zu area", c);
		}
	}
}

ZTEST(touch_mount, test_in_place_matches_copy)
{
	for (size_t c = 0; c < ARRAY_SIZE(mount_cases); c++) {
		struct wp_touch_frame a = make_frame(5, sensor_pts, 5);
		struct wp_touch_frame b = make_frame(5, sensor_pts, 5);
		struct wp_touch_frame ref;

		wp_touch_mount_apply(&mount_cases[c].m, &a, &ref);
		wp_touch_mount_apply(&mount_cases[c].m, &b, &b);
		zassert_mem_equal(&ref, &b, sizeof(ref), "case %zu in-place differs", c);
		/* Source frame untouched by the copying variant. */
		zassert_equal(a.contacts[4].x, 100);
		zassert_equal(a.max_x, MAX_X);
	}
}

ZTEST(touch_mount, test_null_mount_is_identity)
{
	struct wp_touch_frame in = make_frame(9, sensor_pts, 5);
	struct wp_touch_frame out;

	wp_touch_mount_apply(NULL, &in, &out);
	zassert_mem_equal(&in, &out, sizeof(in));

	/* NULL frames are ignored. */
	wp_touch_mount_apply(&mount_cases[7].m, NULL, &out);
	wp_touch_mount_apply(&mount_cases[7].m, &in, NULL);
}

ZTEST(touch_mount, test_applying_twice_restores_inversions)
{
	/* Inversions are involutions; swap + swap also is. */
	for (size_t c = 0; c < ARRAY_SIZE(mount_cases); c++) {
		struct wp_touch_frame in = make_frame(1, sensor_pts, 5);
		struct wp_touch_frame once, twice;
		struct wp_touch_mount inv = mount_cases[c].m;

		wp_touch_mount_apply(&mount_cases[c].m, &in, &once);
		if (inv.swap_xy) {
			/* Undo of "swap, then invert" is "swap, then invert the
			 * other axes" -> swap the invert flags.
			 */
			bool tmp = inv.invert_x;

			inv.invert_x = inv.invert_y;
			inv.invert_y = tmp;
		}
		wp_touch_mount_apply(&inv, &once, &twice);
		zassert_mem_equal(&in, &twice, sizeof(in), "case %zu not reversible", c);
	}
}

ZTEST(touch_mount, test_out_of_range_coordinates_clamp)
{
	const struct pt wild[2] = {{0, 4000, 0xFFFF}, {1, MAX_X + 1, MAX_Y + 1}};
	struct wp_touch_frame in = make_frame(1, wild, 2);
	struct wp_touch_frame out;
	const struct wp_touch_mount m = {false, true, true};

	/* Inverting a coordinate beyond max never wraps around to ~65535. */
	wp_touch_mount_apply(&m, &in, &out);
	zassert_equal(out.contacts[0].x, 0);
	zassert_equal(out.contacts[0].y, 0);
	zassert_equal(out.contacts[1].x, 0);
	zassert_equal(out.contacts[1].y, 0);

	/* A corrupt count is clamped to the array size. */
	in = make_frame(1, sensor_pts, 5);
	in.count = 250;
	wp_touch_mount_apply(&m, &in, &out);
	zassert_equal(out.count, WP_TOUCH_MAX_CONTACTS);
}

ZTEST_SUITE(touch_mount, NULL, NULL, NULL, NULL, NULL);

/* ---- IQS5xx frame conversion (HW-6) ------------------------------------- */

static struct iqs5xx_frame ifr;

static void ifr_init(void)
{
	memset(&ifr, 0, sizeof(ifr));
	ifr.timestamp_us = 123456;
	ifr.num_fingers = 2;
	/* Slots 0 and 2 in touch, 1/3/4 idle with the reserved coordinate. */
	for (int i = 0; i < IQS5XX_MAX_FINGERS; i++) {
		ifr.fingers[i].x = IQS5XX_COORD_NONE;
		ifr.fingers[i].y = IQS5XX_COORD_NONE;
	}
	ifr.fingers[0] = (struct iqs5xx_finger){.x = 100, .y = 200, .strength = 300, .area = 4};
	ifr.fingers[2] = (struct iqs5xx_finger){.x = 1500, .y = 900, .strength = 250, .area = 3};
}

static void expect_two(const struct wp_touch_frame *f)
{
	zassert_equal(f->count, 2);
	zassert_equal(f->contacts[0].id, 0);
	zassert_equal(f->contacts[0].x, 100);
	zassert_equal(f->contacts[0].y, 200);
	zassert_equal(f->contacts[0].strength, 300);
	zassert_equal(f->contacts[0].area, 4);
	zassert_equal(f->contacts[1].id, 2);
	zassert_equal(f->contacts[1].x, 1500);
	zassert_equal(f->contacts[1].y, 900);
}

ZTEST(touch_iqs5xx, test_normal_frame)
{
	struct wp_touch_frame f;

	ifr_init();
	zassert_equal(wp_touch_from_iqs5xx(&ifr, MAX_X, MAX_Y, &f), 2);
	expect_two(&f);
	zassert_equal(f.t_us, 123456);
	zassert_equal(f.max_x, MAX_X);
	zassert_equal(f.max_y, MAX_Y);

	/* Unrelated System Info 1 bits (TP_MOVEMENT, RR_MISSED, SNAP_TOGGLE,
	 * SWITCH_STATE) do not drop anything. */
	ifr.sys_info1 = (1U << 0) | (1U << 3) | (1U << 4) | (1U << 5);
	zassert_equal(wp_touch_from_iqs5xx(&ifr, MAX_X, MAX_Y, &f), 2);
	expect_two(&f);
}

ZTEST(touch_iqs5xx, test_palm_and_too_many_fingers_give_zero_contacts)
{
	struct wp_touch_frame f;

	ifr_init();
	ifr.sys_info1 = IQS5XX_SYS_INFO1_PALM_DETECT;
	zassert_equal(wp_touch_from_iqs5xx(&ifr, MAX_X, MAX_Y, &f), 0, "palm frame has contacts");
	zassert_equal(f.count, 0);
	zassert_equal(f.t_us, 123456, "timestamp lost");

	ifr.sys_info1 = IQS5XX_SYS_INFO1_TOO_MANY_FINGERS | (1U << 0);
	zassert_equal(wp_touch_from_iqs5xx(&ifr, MAX_X, MAX_Y, &f), 0,
		      "too-many-fingers frame has contacts");

	/* The next clean frame reports the fingers again. */
	ifr.sys_info1 = 0U;
	zassert_equal(wp_touch_from_iqs5xx(&ifr, MAX_X, MAX_Y, &f), 2);
}

ZTEST(touch_iqs5xx, test_reserved_coordinate_slots_dropped)
{
	struct wp_touch_frame f;

	/* Non-zero strength but X and/or Y = 0xFFFF: never a contact (it would
	 * otherwise be clamped to a phantom finger on the pad edge). */
	ifr_init();
	ifr.fingers[1] = (struct iqs5xx_finger){.x = IQS5XX_COORD_NONE, .y = 500, .strength = 90};
	ifr.fingers[3] = (struct iqs5xx_finger){.x = 700, .y = IQS5XX_COORD_NONE, .strength = 90};
	ifr.fingers[4] = (struct iqs5xx_finger){
		.x = IQS5XX_COORD_NONE, .y = IQS5XX_COORD_NONE, .strength = 90};
	zassert_equal(wp_touch_from_iqs5xx(&ifr, MAX_X, MAX_Y, &f), 2);
	expect_two(&f);
}

ZTEST(touch_iqs5xx, test_clamp_inactive_and_null)
{
	struct wp_touch_frame f;

	ifr_init();
	ifr.fingers[2].x = MAX_X + 5;
	ifr.fingers[2].y = MAX_Y + 1;
	ifr.fingers[0].strength = 0U; /* inactive: coordinates ignored */
	zassert_equal(wp_touch_from_iqs5xx(&ifr, MAX_X, MAX_Y, &f), 1);
	zassert_equal(f.contacts[0].id, 2);
	zassert_equal(f.contacts[0].x, MAX_X);
	zassert_equal(f.contacts[0].y, MAX_Y);

	zassert_equal(wp_touch_from_iqs5xx(NULL, MAX_X, MAX_Y, &f), 0);
	zassert_equal(f.count, 0);
	zassert_equal(wp_touch_from_iqs5xx(&ifr, MAX_X, MAX_Y, NULL), 0);
}

ZTEST_SUITE(touch_iqs5xx, NULL, NULL, NULL, NULL, NULL);
