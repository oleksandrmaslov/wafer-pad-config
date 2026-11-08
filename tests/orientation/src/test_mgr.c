/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Orientation manager (B: lock / touch deferral, manual & forced, gates).
 */

#include <zephyr/ztest.h>

#include "helpers.h"

static struct wp_orient_mgr m;
static struct wp_orient_mgr_config cfg;
static int64_t now;

static void before(void *f)
{
	ARG_UNUSED(f);
	wp_orient_mgr_config_default(&cfg);
	wp_orient_mgr_init(&m, &cfg, WP_ORIENT_0);
	now = 0;
}

static void reinit(void)
{
	wp_orient_mgr_init(&m, &cfg, WP_ORIENT_0);
	now = 0;
}

static void hold(float tilt, float heading, float mag, int ms)
{
	for (int k = 0; k < ms / T_IMU_MS; k++) {
		struct wp_imu_sample s = imu_tilt(now, tilt, heading, mag);

		wp_orient_mgr_on_imu(&m, &s);
		now += MS(T_IMU_MS);
	}
}

static int count_changes(void)
{
	enum wp_orientation o;
	int n = 0;

	while (wp_orient_mgr_poll_change(&m, &o)) {
		n++;
	}
	return n;
}

ZTEST(orient_mgr, test_defaults_and_init)
{
	zassert_true(cfg.auto_enabled);
	zassert_equal(cfg.manual_lock_mode, WP_MANUAL_LOCK_TIMED);
	zassert_equal(cfg.manual_lock_ms, 30000);
	zassert_equal(cfg.touch_release_settle_ms, 150);
	zassert_false(cfg.gyro.enabled);
	zassert_within(cfg.gyro.step_deg, 70.0f, 1e-6f);
	zassert_within(cfg.gyro.min_rate_dps, 8.0f, 1e-6f);
	zassert_equal(cfg.gyro.max_window_ms, 1500);
	zassert_within(cfg.gyro.max_tilt_deg, 50.0f, 1e-6f);
	zassert_within(cfg.gravity.min_tilt_deg, 20.0f, 1e-6f);

	wp_orient_mgr_init(&m, &cfg, WP_ORIENT_180);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_180);

	struct wp_orient_status st;

	wp_orient_mgr_get_status(&m, &st);
	zassert_equal(st.current, WP_ORIENT_180);
	zassert_equal(st.source, WP_ORIENT_SRC_DEFAULT);
	zassert_false(st.pending);
	zassert_equal(st.block, WP_ORIENT_BLOCK_NONE);
	zassert_false(wp_orient_mgr_poll_change(&m, NULL));

	/* NULL config = defaults */
	wp_orient_mgr_init(&m, NULL, WP_ORIENT_90);
	zassert_true(m.cfg.auto_enabled);
	zassert_equal(m.cfg.touch_release_settle_ms, 150);
}

ZTEST(orient_mgr, test_tilt_changes_orientation_and_poll_once)
{
	enum wp_orientation o = WP_ORIENT_0;

	hold(30.0f, 90.0f, 1.0f, 290);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0, "not before confirm_ms");
	hold(30.0f, 90.0f, 1.0f, 100);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_90);

	zassert_true(wp_orient_mgr_poll_change(&m, &o));
	zassert_equal(o, WP_ORIENT_90);
	zassert_false(wp_orient_mgr_poll_change(&m, &o), "exactly once");

	struct wp_orient_status st;

	wp_orient_mgr_get_status(&m, &st);
	zassert_equal(st.source, WP_ORIENT_SRC_GRAVITY);
	zassert_true(st.confidence > 0.9f);

	/* Keeping the pose does not re-trigger. */
	hold(30.0f, 90.0f, 1.0f, 2000);
	zassert_equal(count_changes(), 0);
	zassert_equal(m.gravity.reference, WP_ORIENT_90, "estimator re-referenced");
}

ZTEST(orient_mgr, test_flat_keeps_last_orientation)
{
	hold(30.0f, 180.0f, 1.0f, 500);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_180);
	zassert_equal(count_changes(), 1);

	/* Laid flat (and slightly wobbly) for 10 s: no information, keep 180. */
	for (int i = 0; i < 10; i++) {
		hold(5.0f + (float)i, (float)(i * 37), 1.0f, 1000);
	}
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_180);
	zassert_equal(count_changes(), 0);
}

ZTEST(orient_mgr, test_boot_reconfirmation_updates_source_without_change)
{
	hold(30.0f, 0.0f, 1.0f, 500);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	zassert_equal(count_changes(), 0);

	struct wp_orient_status st;

	wp_orient_mgr_get_status(&m, &st);
	zassert_equal(st.source, WP_ORIENT_SRC_GRAVITY, "default became confirmed");
	zassert_true(st.confidence > 0.9f);
}

ZTEST(orient_mgr, test_change_during_touch_deferred_until_lift_and_settle)
{
	struct wp_orient_status st;

	wp_orient_mgr_on_touch(&m, 2, now); /* two-finger gesture starts */
	hold(30.0f, 270.0f, 1.0f, 1000);   /* pad tilted towards 270 meanwhile */
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0, "never during a gesture");
	zassert_equal(count_changes(), 0);

	wp_orient_mgr_get_status(&m, &st);
	zassert_true(st.pending);
	zassert_equal(st.pending_orientation, WP_ORIENT_270);
	zassert_equal(st.block, WP_ORIENT_BLOCK_TOUCH);

	/* One finger lifts: still a gesture. */
	wp_orient_mgr_on_touch(&m, 1, now);
	hold(30.0f, 270.0f, 1.0f, 200);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);

	/* All up: settle time not elapsed yet. */
	int64_t up = now;

	wp_orient_mgr_on_touch(&m, 0, up);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	hold(30.0f, 270.0f, 1.0f, 140);
	wp_orient_mgr_tick(&m, up + MS(149));
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0, "149 ms < settle");

	wp_orient_mgr_tick(&m, up + MS(150));
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_270);
	zassert_equal(count_changes(), 1);
	wp_orient_mgr_get_status(&m, &st);
	zassert_false(st.pending);
	zassert_equal(st.block, WP_ORIENT_BLOCK_NONE);
}

ZTEST(orient_mgr, test_touch_down_again_during_settle_restarts_it)
{
	wp_orient_mgr_on_touch(&m, 1, now);
	hold(30.0f, 90.0f, 1.0f, 500);
	wp_orient_mgr_on_touch(&m, 0, now);
	hold(30.0f, 90.0f, 1.0f, 100);
	wp_orient_mgr_on_touch(&m, 1, now); /* tap again within settle */
	hold(30.0f, 90.0f, 1.0f, 500);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	wp_orient_mgr_on_touch(&m, 0, now);
	hold(30.0f, 90.0f, 1.0f, 100);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	hold(30.0f, 90.0f, 1.0f, 60);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_90);
}

ZTEST(orient_mgr, test_deferred_change_dropped_if_no_longer_confirmed)
{
	wp_orient_mgr_on_touch(&m, 2, now);
	hold(30.0f, 180.0f, 1.0f, 600);
	zassert_true(m.pending);

	/* The pad is laid flat before the fingers lift. */
	hold(3.0f, 0.0f, 1.0f, 500);
	wp_orient_mgr_on_touch(&m, 0, now);
	hold(3.0f, 0.0f, 1.0f, 1000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	zassert_false(m.pending);
	zassert_equal(count_changes(), 0);
}

ZTEST(orient_mgr, test_deferred_change_follows_latest_candidate)
{
	wp_orient_mgr_on_touch(&m, 2, now);
	hold(30.0f, 180.0f, 1.0f, 600);
	zassert_equal(m.pending_orientation, WP_ORIENT_180);
	hold(30.0f, 270.0f, 1.0f, 600);
	zassert_equal(m.pending_orientation, WP_ORIENT_270);
	/* Back to the current orientation: nothing pending any more. */
	hold(30.0f, 0.0f, 1.0f, 600);
	zassert_false(m.pending);
	wp_orient_mgr_on_touch(&m, 0, now);
	hold(30.0f, 0.0f, 1.0f, 500);
	zassert_equal(count_changes(), 0);
}

ZTEST(orient_mgr, test_lifting_and_noisy_samples_never_change)
{
	/* The pad is picked up: each sample alternates between 0.8 and 1.3 g
	 * with a heading that would mean 90. Never a change.
	 */
	for (int k = 0; k < 300; k++) {
		struct wp_imu_sample s = imu_tilt(now, 30.0f, 90.0f, (k & 1) ? 1.3f : 0.8f);

		wp_orient_mgr_on_imu(&m, &s);
		now += MS(T_IMU_MS);
	}
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);

	/* A single good sample in between noise does not change it either. */
	struct wp_imu_sample s = imu_tilt(now, 30.0f, 90.0f, 1.0f);

	wp_orient_mgr_on_imu(&m, &s);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	zassert_equal(count_changes(), 0);
}

ZTEST(orient_mgr, test_manual_overrides_auto_with_timed_lock)
{
	struct wp_orient_status st;

	/* Pad tilted towards 0 (gravity says 0), user calibrates 270. */
	hold(30.0f, 0.0f, 1.0f, 500);
	wp_orient_mgr_set_manual(&m, WP_ORIENT_270, now);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_270);
	zassert_equal(count_changes(), 1);
	wp_orient_mgr_get_status(&m, &st);
	zassert_equal(st.source, WP_ORIENT_SRC_MANUAL);
	zassert_within(st.confidence, 1.0f, 1e-6f);

	/* Gravity keeps saying 0 for 29 s: locked. */
	int64_t t_manual = now;

	hold(30.0f, 0.0f, 1.0f, 29000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_270);
	wp_orient_mgr_get_status(&m, &st);
	zassert_equal(st.block, WP_ORIENT_BLOCK_MANUAL_LOCK);
	zassert_true(st.last_gravity.confirmed);
	zassert_equal(st.last_gravity.candidate, WP_ORIENT_0);

	/* Lock expires at t_manual + 30 s, then auto takes over again. */
	hold(30.0f, 0.0f, 1.0f, 1100);
	zassert_true(now > t_manual + MS(30000));
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	zassert_equal(count_changes(), 1);
	wp_orient_mgr_get_status(&m, &st);
	zassert_equal(st.source, WP_ORIENT_SRC_GRAVITY);
}

ZTEST(orient_mgr, test_manual_flat_pad_is_kept)
{
	/* Typical demo: pad flat on a table, user calibrates 90. */
	hold(0.0f, 0.0f, 1.0f, 200);
	wp_orient_mgr_set_manual(&m, WP_ORIENT_90, now);
	hold(2.0f, 0.0f, 1.0f, 60000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_90, "flat: kept after lock expiry");
	zassert_equal(count_changes(), 1);
}

ZTEST(orient_mgr, test_manual_lock_until_recal)
{
	cfg.manual_lock_mode = WP_MANUAL_LOCK_UNTIL_RECAL;
	reinit();

	wp_orient_mgr_set_manual(&m, WP_ORIENT_180, now);
	zassert_equal(count_changes(), 1);
	hold(30.0f, 90.0f, 1.0f, 60000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_180, "locked for good");
	zassert_equal(m.block, WP_ORIENT_BLOCK_MANUAL_LOCK);

	/* A new calibration keeps the lock and applies the new value. */
	wp_orient_mgr_set_manual(&m, WP_ORIENT_270, now);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_270);
	hold(30.0f, 90.0f, 1.0f, 2000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_270);

	/* Re-enabling auto ends the lock. */
	wp_orient_mgr_set_auto(&m, true);
	hold(30.0f, 90.0f, 1.0f, 500);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_90);
}

ZTEST(orient_mgr, test_manual_lock_none_lets_auto_override)
{
	cfg.manual_lock_mode = WP_MANUAL_LOCK_NONE;
	reinit();

	hold(30.0f, 0.0f, 1.0f, 500);
	wp_orient_mgr_set_manual(&m, WP_ORIENT_90, now);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_90);
	zassert_equal(count_changes(), 1);
	zassert_false(m.manual_lock_active);
	/* Estimator was reset: needs a fresh confirmation before overriding. */
	hold(30.0f, 0.0f, 1.0f, 200);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_90);
	hold(30.0f, 0.0f, 1.0f, 300);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	zassert_equal(count_changes(), 1);
}

ZTEST(orient_mgr, test_manual_during_touch_applied_on_lift)
{
	wp_orient_mgr_on_touch(&m, 2, now);
	wp_orient_mgr_set_manual(&m, WP_ORIENT_180, now);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	zassert_true(m.pending);
	zassert_equal(m.block, WP_ORIENT_BLOCK_TOUCH);

	/* An auto candidate meanwhile does not replace the manual request. */
	hold(30.0f, 90.0f, 1.0f, 600);
	zassert_equal(m.pending_orientation, WP_ORIENT_180);
	zassert_equal(m.pending_source, WP_ORIENT_SRC_MANUAL);

	/* Applied immediately when all fingers are up (no settle needed). */
	wp_orient_mgr_on_touch(&m, 0, now);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_180);
	zassert_equal(m.source, WP_ORIENT_SRC_MANUAL);
	zassert_equal(count_changes(), 1);

	/* ... and locked: the gravity 90 does not take over. */
	hold(30.0f, 90.0f, 1.0f, 2000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_180);
}

ZTEST(orient_mgr, test_manual_same_orientation_no_change_event)
{
	wp_orient_mgr_set_manual(&m, WP_ORIENT_0, now);
	zassert_equal(count_changes(), 0);
	zassert_equal(m.source, WP_ORIENT_SRC_MANUAL);
	zassert_true(m.manual_lock_active);
}

ZTEST(orient_mgr, test_force)
{
	wp_orient_mgr_force(&m, WP_ORIENT_270, now);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_270);
	zassert_equal(m.source, WP_ORIENT_SRC_FORCED);
	zassert_equal(count_changes(), 1);
	/* Forced also locks (applied like manual). */
	hold(30.0f, 0.0f, 1.0f, 1000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_270);

	/* Out-of-range enum values are normalised. */
	wp_orient_mgr_force(&m, (enum wp_orientation)6, now);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_180);
}

ZTEST(orient_mgr, test_allow_auto_false_blocks)
{
	wp_orient_mgr_allow_auto(&m, false); /* e.g. AIR_MOUSE */
	hold(30.0f, 90.0f, 1.0f, 2000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	zassert_equal(m.block, WP_ORIENT_BLOCK_EXTERNAL);
	zassert_equal(count_changes(), 0);

	wp_orient_mgr_allow_auto(&m, true);
	hold(30.0f, 90.0f, 1.0f, 20);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_90);
	zassert_equal(count_changes(), 1);
}

ZTEST(orient_mgr, test_allow_auto_false_drops_pending)
{
	wp_orient_mgr_on_touch(&m, 1, now);
	hold(30.0f, 90.0f, 1.0f, 500);
	zassert_true(m.pending);
	wp_orient_mgr_allow_auto(&m, false);
	zassert_false(m.pending);
	wp_orient_mgr_on_touch(&m, 0, now);
	hold(30.0f, 90.0f, 1.0f, 1000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
}

ZTEST(orient_mgr, test_set_auto_false_blocks)
{
	wp_orient_mgr_set_auto(&m, false);
	hold(30.0f, 180.0f, 1.0f, 2000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	zassert_equal(m.block, WP_ORIENT_BLOCK_AUTO_OFF);

	/* Manual still works while auto is off. */
	wp_orient_mgr_set_manual(&m, WP_ORIENT_90, now);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_90);

	wp_orient_mgr_set_auto(&m, true); /* also clears the manual lock */
	hold(30.0f, 180.0f, 1.0f, 500);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_180);
}

ZTEST(orient_mgr, test_source_names)
{
	zassert_str_equal(wp_orient_source_name(WP_ORIENT_SRC_DEFAULT), "default");
	zassert_str_equal(wp_orient_source_name(WP_ORIENT_SRC_GRAVITY), "gravity");
	zassert_str_equal(wp_orient_source_name(WP_ORIENT_SRC_MANUAL), "manual");
	zassert_str_equal(wp_orient_source_name(WP_ORIENT_SRC_GYRO), "gyro");
	zassert_str_equal(wp_orient_source_name(WP_ORIENT_SRC_FORCED), "forced");
	zassert_str_equal(wp_orient_source_name((enum wp_orient_source)42), "unknown");
}

ZTEST(orient_mgr, test_calibration_drives_manager)
{
	/* End-to-end C -> manager: hold + swipe towards the pad's right edge
	 * (+x_P) means the right edge is away from the user -> 270.
	 */
	const struct wp_pad_geometry geom = {
		.max_x = 650, .max_y = 490, .width_mm = 65.0f, .height_mm = 49.0f};
	struct wp_calib c;
	struct wp_calib_result r;
	int64_t t = 0;
	bool done = false;
	struct wp_calib_config cc;

	calib_mech_config(&cc);
	wp_calib_init(&c, &cc, &geom);
	for (; t <= MS(720); t += MS(20)) {
		struct wp_touch_frame f = frame2(t, 650, 490, 300, 250, 350, 250);

		wp_orient_mgr_on_touch(&m, f.count, t);
		wp_calib_update(&c, &f, &r);
	}
	zassert_equal(wp_calib_phase_get(&c), WP_CALIB_ARMED);
	for (int k = 1; k <= 10; k++, t += MS(20)) {
		struct wp_touch_frame f =
			frame2(t, 650, 490, 300 + 20 * k, 250, 350 + 20 * k, 250);

		wp_orient_mgr_on_touch(&m, f.count, t);
		wp_calib_update(&c, &f, &r);
	}
	struct wp_touch_frame f0 = frame0(t, 650, 490);

	wp_calib_update(&c, &f0, &r);
	/* Router order: completion is handled while the manager still thinks
	 * fingers are down -> pending, applied on the lift notification.
	 */
	if (r.event == WP_CALIB_EV_COMPLETED) {
		wp_orient_mgr_set_manual(&m, r.orientation, t);
		done = true;
	}
	zassert_true(done);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	wp_orient_mgr_on_touch(&m, 0, t);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_270);
	zassert_equal(count_changes(), 1);
}

/* Bottom (near) edge lifted 0 -> peak -> 0 at rate_dps (heading 180),
 * then lying flat for 1 s. */
static void sweep(float peak_deg, float rate_dps, bool gyro)
{
	const float step = rate_dps * (float)T_IMU_MS / 1000.0f;
	float tilt = 0.0f;
	int dir = 1;

	while (dir > 0 || tilt > 0.0f) {
		tilt += (float)dir * step;
		if (tilt >= peak_deg) {
			tilt = peak_deg;
			dir = -1;
		}
		if (tilt < 0.0f) {
			tilt = 0.0f;
		}
		struct wp_imu_sample s = imu_tilt(now, tilt, 180.0f, 1.0f);

		s.gyro[0] = (float)dir * rate_dps * WP_DEG_TO_RAD;
		s.gyro_valid = gyro;
		wp_orient_mgr_on_imu(&m, &s);
		wp_orient_mgr_tick(&m, now);
		now += MS(T_IMU_MS);
	}
	for (int k = 0; k < 1000 / T_IMU_MS; k++) {
		struct wp_imu_sample s = imu_flat(now);

		s.gyro_valid = gyro;
		wp_orient_mgr_on_imu(&m, &s);
		wp_orient_mgr_tick(&m, now);
		now += MS(T_IMU_MS);
	}
}

ZTEST(orient_mgr, test_pickup_putdown_sweep_never_changes)
{
	/* ALG-7: lifting the near edge to pick the pad up (or tipping it to
	 * look underneath) and laying it down again must not latch 180 deg. */
	static const struct {
		float peak, rate;
		bool gyro;
	} cases[] = {
		{60.0f, 40.0f, true},  {60.0f, 40.0f, false}, {35.0f, 50.0f, true},
		{35.0f, 50.0f, false}, {30.0f, 20.0f, false}, {60.0f, 25.0f, false},
	};

	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		reinit();
		hold(0.0f, 0.0f, 1.0f, 500);
		sweep(cases[i].peak, cases[i].rate, cases[i].gyro);
		zassert_equal(count_changes(), 0, "case %u: sweep changed the orientation",
			      (unsigned int)i);
		zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0, "case %u", (unsigned int)i);
	}

	/* A posture that is actually held still confirms (gyro on and off). */
	reinit();
	hold(35.0f, 180.0f, 1.0f, 600);
	zassert_equal(count_changes(), 1);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_180);

	reinit();
	for (int k = 0; k < 1000 / T_IMU_MS; k++) {
		struct wp_imu_sample s = imu_tilt(now, 35.0f, 180.0f, 1.0f);

		s.gyro_valid = false;
		wp_orient_mgr_on_imu(&m, &s);
		wp_orient_mgr_tick(&m, now);
		now += MS(T_IMU_MS);
	}
	zassert_equal(count_changes(), 1);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_180);
}

ZTEST_SUITE(orient_mgr, NULL, NULL, before, NULL, NULL);
