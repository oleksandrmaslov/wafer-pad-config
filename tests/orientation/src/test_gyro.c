/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Algorithm D: gyro-assisted relative rotation (inside the manager).
 */

#include <zephyr/ztest.h>

#include "helpers.h"

static struct wp_orient_mgr m;
static struct wp_orient_mgr_config cfg;
static int64_t now;
static int changes;

static void before(void *f)
{
	ARG_UNUSED(f);
	wp_orient_mgr_config_default(&cfg);
	cfg.gyro.enabled = true;
	/* No manual lock so that the manual "fix" below does not gate D. */
	cfg.manual_lock_mode = WP_MANUAL_LOCK_NONE;
	wp_orient_mgr_init(&m, &cfg, WP_ORIENT_0);
	now = 0;
	changes = 0;
}

static void poll(void)
{
	enum wp_orientation o;

	while (wp_orient_mgr_poll_change(&m, &o)) {
		changes++;
	}
}

/** Pad at @p tilt (heading 0), yaw rate @p dps about z_B, for @p ms. */
static void spin(float tilt, float dps, int ms)
{
	for (int k = 0; k < ms / T_IMU_MS; k++) {
		struct wp_imu_sample s = imu_tilt(now, tilt, 0.0f, 1.0f);

		s.gyro[2] = dps * WP_DEG_TO_RAD;
		wp_orient_mgr_on_imu(&m, &s);
		poll();
		now += MS(T_IMU_MS);
	}
}

/** A confident fix while flat: manual calibration to the current value. */
static void fix_flat(enum wp_orientation o)
{
	spin(0.0f, 0.0f, 100);
	wp_orient_mgr_set_manual(&m, o, now);
	poll();
	changes = 0;
}

/** Orientation whose "forward" body axis equals @p v (2D). */
static enum wp_orientation orientation_with_fwd(float vx, float vy)
{
	for (int o = 0; o < 4; o++) {
		float fwd[3], right[3];

		wp_orientation_axes_body((enum wp_orientation)o, fwd, right);
		if (fabsf(fwd[0] - vx) < 1e-3f && fabsf(fwd[1] - vy) < 1e-3f) {
			return (enum wp_orientation)o;
		}
	}
	return (enum wp_orientation)-1;
}

ZTEST(orient_gyro, test_sign_convention_derivation)
{
	/* Physical derivation, independent of the implementation: the world
	 * direction "away from the user" is +y_B at orientation 0. When the pad
	 * turns by theta about +z_B (right-hand rule, counter-clockwise seen
	 * from above), a world-fixed vector appears rotated by -theta in body
	 * coordinates. The orientation whose forward axis matches it is the
	 * new orientation.
	 */
	float th = 90.0f * WP_DEG_TO_RAD;
	float vx = -sinf(-th) * 1.0f; /* R(-theta) * (0, 1) */
	float vy = cosf(-th) * 1.0f;

	zassert_equal(orientation_with_fwd(vx, vy), WP_ORIENT_270,
		      "+90 about z_B (CCW) -> 270, i.e. orientation - 90");
	vx = -sinf(th);
	vy = cosf(th);
	zassert_equal(orientation_with_fwd(vx, vy), WP_ORIENT_90,
		      "-90 about z_B (CW) -> 90, i.e. orientation + 90");
}

ZTEST(orient_gyro, test_positive_yaw_90_is_one_step_ccw)
{
	fix_flat(WP_ORIENT_0);
	spin(0.0f, 90.0f, 1000); /* +90 deg about z_B at 90 dps */
	spin(0.0f, 0.0f, 500);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_270);
	zassert_equal(changes, 1, "exactly one step");
	zassert_equal(m.source, WP_ORIENT_SRC_GYRO);
}

ZTEST(orient_gyro, test_negative_yaw_90_is_one_step_cw)
{
	fix_flat(WP_ORIENT_0);
	spin(0.0f, -90.0f, 1000);
	spin(0.0f, 0.0f, 500);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_90);
	zassert_equal(changes, 1);
}

ZTEST(orient_gyro, test_composes_from_any_orientation)
{
	fix_flat(WP_ORIENT_180);
	spin(0.0f, -90.0f, 1000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_270);
	spin(0.0f, 0.0f, 2000);
	spin(0.0f, -90.0f, 1000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0, "wraps 270 -> 0");
	zassert_equal(changes, 2);
}

ZTEST(orient_gyro, test_half_turn_two_steps)
{
	fix_flat(WP_ORIENT_0);
	spin(0.0f, 90.0f, 2000); /* 180 deg */
	spin(0.0f, 0.0f, 500);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_180);
	zassert_equal(changes, 2);
}

ZTEST(orient_gyro, test_small_rotation_no_step)
{
	fix_flat(WP_ORIENT_0);
	spin(0.0f, 90.0f, 600); /* 54 deg < step 70 */
	spin(0.0f, 0.0f, 2000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	zassert_equal(changes, 0);
	/* The stale 54 deg must have expired: another 54 deg is not a step. */
	spin(0.0f, 90.0f, 600);
	spin(0.0f, 0.0f, 500);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
}

ZTEST(orient_gyro, test_slow_drift_below_min_rate_ignored)
{
	fix_flat(WP_ORIENT_0);
	spin(0.0f, 5.0f, 60000); /* 300 deg of drift over a minute */
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	zassert_equal(changes, 0);
	zassert_within(m.gyro_accum_deg, 0.0f, 1e-6f);
}

ZTEST(orient_gyro, test_accumulator_bounded_by_window)
{
	fix_flat(WP_ORIENT_0);
	/* 12 dps (above min rate) for 30 s = 360 deg, but never 70 deg within
	 * the 1.5 s window.
	 */
	float worst = 0.0f;

	for (int k = 0; k < 3000; k++) {
		spin(0.0f, 12.0f, T_IMU_MS);
		if (fabsf(m.gyro_accum_deg) > worst) {
			worst = fabsf(m.gyro_accum_deg);
		}
	}
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	zassert_true(worst <= 12.0f * 1.5f + 0.5f, "accumulator %f unbounded", (double)worst);
}

ZTEST(orient_gyro, test_disabled_by_default)
{
	cfg.gyro.enabled = false;
	wp_orient_mgr_init(&m, &cfg, WP_ORIENT_0);
	fix_flat(WP_ORIENT_0);
	spin(0.0f, 90.0f, 1000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	zassert_equal(changes, 0);
}

ZTEST(orient_gyro, test_requires_a_confident_fix)
{
	/* Boot default was never confirmed: relative tracking has no base. */
	spin(0.0f, 90.0f, 1000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	zassert_false(m.gyro_ref_valid);
}

ZTEST(orient_gyro, test_gravity_fix_enables_tracking)
{
	spin(30.0f, 0.0f, 500); /* gravity confirms 0 (top edge raised) */
	zassert_true(m.gyro_ref_valid);
	spin(0.0f, 0.0f, 300); /* laid flat */
	spin(0.0f, -90.0f, 1000);
	spin(0.0f, 0.0f, 300);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_90);
}

ZTEST(orient_gyro, test_step_during_touch_deferred)
{
	fix_flat(WP_ORIENT_0);
	wp_orient_mgr_on_touch(&m, 1, now);
	spin(0.0f, 90.0f, 1000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	zassert_true(m.pending);
	zassert_equal(m.pending_orientation, WP_ORIENT_270);
	zassert_equal(m.pending_source, WP_ORIENT_SRC_GYRO);

	wp_orient_mgr_on_touch(&m, 0, now);
	int64_t up = now;

	wp_orient_mgr_tick(&m, up + MS(100));
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_0);
	wp_orient_mgr_tick(&m, up + MS(150));
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_270);
}

ZTEST(orient_gyro, test_too_tilted_not_integrated)
{
	fix_flat(WP_ORIENT_0);
	spin(60.0f, 0.0f, 200);
	spin(60.0f, 90.0f, 1000); /* 60 deg > gyro max_tilt 50 */
	zassert_equal(changes, 0);
	zassert_within(m.gyro_accum_deg, 0.0f, 1e-6f);

	/* Moderately tilted (30 deg) is fine (gravity rejects the fast spin). */
	spin(30.0f, 90.0f, 1000);
	zassert_equal(changes, 1);
}

ZTEST(orient_gyro, test_external_gate_blocks)
{
	fix_flat(WP_ORIENT_0);
	wp_orient_mgr_allow_auto(&m, false); /* AIR_MOUSE: pad waved around */
	spin(0.0f, 90.0f, 1000);
	zassert_equal(changes, 0);
	zassert_within(m.gyro_accum_deg, 0.0f, 1e-6f);
	wp_orient_mgr_allow_auto(&m, true);
	spin(0.0f, 90.0f, 500); /* 45 deg after re-enable: no stale history */
	zassert_equal(changes, 0);
}

ZTEST(orient_gyro, test_manual_lock_blocks)
{
	cfg.manual_lock_mode = WP_MANUAL_LOCK_TIMED;
	wp_orient_mgr_init(&m, &cfg, WP_ORIENT_0);
	fix_flat(WP_ORIENT_90);
	spin(0.0f, 90.0f, 1000);
	zassert_equal(wp_orient_mgr_get(&m), WP_ORIENT_90);
	zassert_equal(changes, 0);
}

ZTEST(orient_gyro, test_gap_not_integrated)
{
	fix_flat(WP_ORIENT_0);
	spin(0.0f, 90.0f, 500); /* 45 deg */
	now += MS(400);          /* sample gap > 100 ms */
	spin(0.0f, 90.0f, 400); /* 36 deg: 81 total but split by the gap */
	zassert_equal(changes, 0);
}

ZTEST_SUITE(orient_gyro, NULL, NULL, before, NULL, NULL);
