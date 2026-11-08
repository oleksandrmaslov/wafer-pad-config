/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Algorithm A: gravity estimator.
 */

#include <zephyr/ztest.h>

#include "helpers.h"

static struct wp_orient_gravity g;
static int64_t now;

static void before(void *f)
{
	ARG_UNUSED(f);
	wp_orient_gravity_init(&g, NULL, WP_ORIENT_0);
	now = 0;
}

/** Feed a constant pose for @p ms (one sample every T_IMU_MS), return last. */
static struct wp_orient_gravity_result feed(float tilt, float heading, float mag, int ms)
{
	struct wp_orient_gravity_result r = {0};

	for (int k = 0; k < ms / T_IMU_MS; k++) {
		struct wp_imu_sample s = imu_tilt(now, tilt, heading, mag);

		wp_orient_gravity_update(&g, &s, &r);
		now += MS(T_IMU_MS);
	}
	return r;
}

ZTEST(orient_gravity, test_defaults)
{
	struct wp_orient_gravity_config c;

	wp_orient_gravity_config_default(&c);
	zassert_within(c.min_tilt_deg, 20.0f, 1e-6f);
	zassert_within(c.max_tilt_deg, 75.0f, 1e-6f);
	zassert_within(c.max_mag_dev_g, 0.12f, 1e-6f);
	zassert_within(c.hysteresis_deg, 15.0f, 1e-6f);
	zassert_within(c.max_gyro_dps, 20.0f, 1e-6f);
	zassert_equal(c.confirm_ms, 300);
	zassert_equal(c.confirm_ms_no_gyro, 800);
	zassert_within(c.max_tilt_change_deg, 6.0f, 1e-6f);
	zassert_within(c.min_confidence, 0.6f, 1e-6f);
	zassert_within(c.lpf_tau_ms, 50.0f, 1e-6f);
}

ZTEST(orient_gravity, test_tilt_30_towards_each_edge)
{
	static const struct {
		float heading;
		enum wp_orientation o;
	} cases[] = {
		{0.0f, WP_ORIENT_0},     /* top edge raised */
		{90.0f, WP_ORIENT_90},   /* left edge raised */
		{180.0f, WP_ORIENT_180}, /* bottom edge raised */
		{270.0f, WP_ORIENT_270}, /* right edge raised */
	};

	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		before(NULL);
		struct wp_orient_gravity_result r = feed(30.0f, cases[i].heading, 1.0f, 500);

		zassert_true(r.valid, "case %u", (unsigned int)i);
		zassert_within(r.tilt_deg, 30.0f, 0.05f);
		zassert_within(r.heading_deg, cases[i].heading, 0.05f);
		zassert_equal(r.candidate, cases[i].o, "case %u", (unsigned int)i);
		zassert_true(r.confirmed, "case %u", (unsigned int)i);
		zassert_true(r.confidence > 0.95f, "conf %f", (double)r.confidence);
	}
}

ZTEST(orient_gravity, test_explicit_axis_signs)
{
	/* Top edge raised = +y_B up: a_y > 0 -> heading 0. Right edge raised =
	 * +x_B up: a_x > 0 -> heading 270.
	 */
	struct wp_imu_sample s = {.t_us = 0, .accel = {0.0f, 4.9f, 8.5f}};
	struct wp_orient_gravity_result r;

	wp_orient_gravity_update(&g, &s, &r);
	zassert_within(r.heading_deg, 0.0f, 0.01f);

	wp_orient_gravity_reset(&g, WP_ORIENT_0);
	s = (struct wp_imu_sample){.t_us = MS(1), .accel = {4.9f, 0.0f, 8.5f}};
	wp_orient_gravity_update(&g, &s, &r);
	zassert_within(r.heading_deg, 270.0f, 0.01f);
	zassert_equal(wp_orientation_from_deg(r.heading_deg), WP_ORIENT_270);
}

ZTEST(orient_gravity, test_flat_gives_no_candidate)
{
	struct wp_orient_gravity_result r = feed(5.0f, 90.0f, 1.0f, 2000);

	zassert_false(r.valid);
	zassert_false(r.confirmed);
	zassert_equal(r.candidate, WP_ORIENT_0, "candidate stays the reference");
	zassert_within(r.confidence, 0.0f, 1e-6f);
	zassert_within(r.tilt_deg, 5.0f, 0.05f);

	/* Exactly flat */
	r = feed(0.0f, 0.0f, 1.0f, 500);
	zassert_false(r.valid);
}

ZTEST(orient_gravity, test_lifting_magnitude_invalid)
{
	struct wp_orient_gravity_result r = feed(30.0f, 90.0f, 1.3f, 1000);

	zassert_false(r.valid);
	zassert_false(r.confirmed);

	/* Free fall-ish / pulled down */
	before(NULL);
	r = feed(30.0f, 90.0f, 0.8f, 1000);
	zassert_false(r.valid);

	/* Inside tolerance (1.08 g) is still valid. */
	before(NULL);
	r = feed(30.0f, 90.0f, 1.08f, 1000);
	zassert_true(r.valid);
	zassert_true(r.confirmed);
}

ZTEST(orient_gravity, test_single_lifting_sample_restarts_timer)
{
	struct wp_orient_gravity_result r = feed(30.0f, 90.0f, 1.0f, 250);

	zassert_false(r.confirmed);
	r = feed(30.0f, 90.0f, 1.3f, T_IMU_MS); /* one spike */
	zassert_false(r.valid);
	/* The spike also disturbs the low-passed magnitude for a while; the
	 * timer must restart, so 250 ms more are NOT enough.
	 */
	r = feed(30.0f, 90.0f, 1.0f, 250);
	zassert_false(r.confirmed);
	r = feed(30.0f, 90.0f, 1.0f, 400);
	zassert_true(r.confirmed);
	zassert_equal(r.candidate, WP_ORIENT_90);
}

ZTEST(orient_gravity, test_confirm_only_after_confirm_ms)
{
	struct wp_orient_gravity_result r;

	/* Samples at t = 0, 10, ..., 290 ms: not yet confirmed. */
	r = feed(30.0f, 180.0f, 1.0f, 300);
	zassert_equal(r.candidate, WP_ORIENT_180);
	zassert_true(r.valid);
	zassert_false(r.confirmed, "only 290 ms elapsed");

	/* Sample at t = 300 ms: confirmed. */
	r = feed(30.0f, 180.0f, 1.0f, T_IMU_MS);
	zassert_true(r.confirmed);
}

ZTEST(orient_gravity, test_candidate_change_restarts_timer)
{
	struct wp_orient_gravity_result r = feed(30.0f, 90.0f, 1.0f, 200);

	zassert_equal(r.candidate, WP_ORIENT_90);
	r = feed(30.0f, 180.0f, 1.0f, 200);
	/* The filter needs a few tau to swing over; meanwhile candidate moves. */
	zassert_equal(r.candidate, WP_ORIENT_180);
	zassert_false(r.confirmed);
	r = feed(30.0f, 180.0f, 1.0f, 300);
	zassert_true(r.confirmed);
}

ZTEST(orient_gravity, test_hysteresis_from_reference_0)
{
	struct wp_orient_gravity_result r;

	/* Reference 0. 40 deg: nearest is 0 -> stays 0. */
	r = feed(35.0f, 40.0f, 1.0f, 500);
	zassert_true(r.valid);
	zassert_equal(r.candidate, WP_ORIENT_0);

	/* 50 deg: nearest is 90 but 40 deg from its centre (> 45 - 15 = 30):
	 * switching away from the reference is not allowed yet.
	 */
	before(NULL);
	r = feed(35.0f, 50.0f, 1.0f, 1000);
	zassert_true(r.valid);
	zassert_equal(r.candidate, WP_ORIENT_0);

	/* 59 deg: still inside the widened reference zone. */
	before(NULL);
	r = feed(35.0f, 59.0f, 1.0f, 1000);
	zassert_equal(r.candidate, WP_ORIENT_0);
	zassert_false(r.confirmed, "far from the reference centre: low confidence");

	/* 61 deg: within 29 deg of 90 -> switch candidate, and it confirms. */
	before(NULL);
	r = feed(35.0f, 61.0f, 1.0f, 500);
	zassert_equal(r.candidate, WP_ORIENT_90);
	zassert_true(r.confirmed);

	/* Mirror side: 310 deg (= -50) stays 0, 295 deg switches to 270. */
	before(NULL);
	r = feed(35.0f, 310.0f, 1.0f, 500);
	zassert_equal(r.candidate, WP_ORIENT_0);
	before(NULL);
	r = feed(35.0f, 295.0f, 1.0f, 500);
	zassert_equal(r.candidate, WP_ORIENT_270);
}

ZTEST(orient_gravity, test_hysteresis_from_reference_90)
{
	struct wp_orient_gravity_result r;

	wp_orient_gravity_init(&g, NULL, WP_ORIENT_90);
	r = feed(35.0f, 70.0f, 1.0f, 500);
	zassert_equal(r.candidate, WP_ORIENT_90);
	zassert_true(r.confirmed, "20 deg from the reference centre");

	/* 50 deg: nearest is 90 = reference, but 40 deg off-centre -> the
	 * candidate stays 90 with a low confidence (no switch either way).
	 */
	wp_orient_gravity_init(&g, NULL, WP_ORIENT_90);
	r = feed(35.0f, 50.0f, 1.0f, 500);
	zassert_equal(r.candidate, WP_ORIENT_90);
	zassert_within(r.confidence, 1.0f - 0.5f * 40.0f / 45.0f, 0.02f);

	wp_orient_gravity_init(&g, NULL, WP_ORIENT_90);
	r = feed(35.0f, 40.0f, 1.0f, 1000);
	zassert_equal(r.candidate, WP_ORIENT_90, "40 deg is still sticky for reference 90");

	wp_orient_gravity_init(&g, NULL, WP_ORIENT_90);
	r = feed(35.0f, 25.0f, 1.0f, 500);
	zassert_equal(r.candidate, WP_ORIENT_0);
	zassert_true(r.confirmed);
}

ZTEST(orient_gravity, test_ambiguous_band_between_non_reference_quadrants)
{
	/* Reference 0, heading 135: 45 deg from both 90 and 180, both are new
	 * orientations and neither is within the switch zone -> no candidate.
	 */
	struct wp_orient_gravity_result r = feed(35.0f, 135.0f, 1.0f, 1000);

	zassert_true(r.valid);
	zassert_false(r.confirmed);
	zassert_equal(r.candidate, WP_ORIENT_0);
	zassert_within(r.confidence, 0.0f, 1e-6f);
}

ZTEST(orient_gravity, test_gyro_too_fast_invalid)
{
	struct wp_orient_gravity_result r = {0};

	for (int k = 0; k < 100; k++) {
		struct wp_imu_sample s = imu_tilt(now, 30.0f, 90.0f, 1.0f);

		s.gyro[2] = 90.0f * WP_DEG_TO_RAD; /* 90 dps > 20 */
		wp_orient_gravity_update(&g, &s, &r);
		now += MS(T_IMU_MS);
	}
	zassert_false(r.valid);
	zassert_false(r.confirmed);

	/* Slow rotation (15 dps) is fine. */
	before(NULL);
	for (int k = 0; k < 50; k++) {
		struct wp_imu_sample s = imu_tilt(now, 30.0f, 90.0f, 1.0f);

		s.gyro[0] = 15.0f * WP_DEG_TO_RAD;
		wp_orient_gravity_update(&g, &s, &r);
		now += MS(T_IMU_MS);
	}
	zassert_true(r.valid);

	/* gyro_valid == false: the rate is ignored (gyro powered down), and
	 * the longer confirm_ms_no_gyro applies instead. */
	before(NULL);
	for (int k = 0; k < 90; k++) {
		struct wp_imu_sample s = imu_tilt(now, 30.0f, 90.0f, 1.0f);

		s.gyro[2] = 500.0f;
		s.gyro_valid = false;
		wp_orient_gravity_update(&g, &s, &r);
		now += MS(T_IMU_MS);
		if (k == 50) {
			zassert_true(r.valid);
			zassert_false(r.confirmed, "500 ms is not enough without gyro");
		}
	}
	zassert_true(r.valid);
	zassert_true(r.confirmed);
}

ZTEST(orient_gravity, test_face_down_and_vertical_invalid)
{
	struct wp_orient_gravity_result r = feed(180.0f, 0.0f, 1.0f, 500);

	zassert_false(r.valid);
	zassert_within(r.tilt_deg, 180.0f, 0.1f);

	before(NULL);
	r = feed(120.0f, 90.0f, 1.0f, 500); /* tilted beyond vertical */
	zassert_false(r.valid);

	before(NULL);
	r = feed(80.0f, 90.0f, 1.0f, 500); /* above max_tilt 75 */
	zassert_false(r.valid);

	before(NULL);
	r = feed(70.0f, 90.0f, 1.0f, 500); /* steep but allowed */
	zassert_true(r.valid);
	zassert_true(r.confirmed);
}

ZTEST(orient_gravity, test_confidence_factors)
{
	struct wp_orient_gravity_result r;

	/* Barely above min tilt: f_tilt = 0.55 -> never confirms. */
	r = feed(21.0f, 90.0f, 1.0f, 2000);
	zassert_true(r.valid);
	zassert_equal(r.candidate, WP_ORIENT_90);
	zassert_within(r.confidence, 0.55f, 0.02f);
	zassert_false(r.confirmed);

	/* Fully tilted, on-centre, 1 g: ~1.0. */
	before(NULL);
	r = feed(40.0f, 180.0f, 1.0f, 500);
	zassert_within(r.confidence, 1.0f, 0.01f);

	/* Heading 20 deg off the centre: f_angle = 1 - 0.5 * 20 / 45. */
	before(NULL);
	r = feed(40.0f, 200.0f, 1.0f, 500);
	zassert_within(r.confidence, 1.0f - 0.5f * 20.0f / 45.0f, 0.02f);
	zassert_true(r.confirmed);

	/* Magnitude error 0.1 g: f_mag = 1 - 0.5 * 0.1 / 0.12. */
	before(NULL);
	r = feed(40.0f, 180.0f, 1.1f, 500);
	zassert_within(r.confidence, 1.0f - 0.5f * 0.1f / 0.12f, 0.02f);
	zassert_false(r.confirmed, "0.58 < 0.6");
}

ZTEST(orient_gravity, test_heading_stability_factor)
{
	struct wp_orient_gravity_result r = {0};
	float h = 75.0f;

	/* Pad slowly turning through the 90 quadrant at 50 deg/s (gyro below the
	 * 60 dps rejection): the heading never stays put, confidence decays
	 * with the drift since the candidate started.
	 */
	float min_conf = 1.0f;

	for (int k = 0; k < 60; k++) {
		struct wp_imu_sample s = imu_tilt(now, 40.0f, h, 1.0f);

		wp_orient_gravity_update(&g, &s, &r);
		if (k > 20 && r.candidate == WP_ORIENT_90 && r.confidence < min_conf) {
			min_conf = r.confidence;
		}
		now += MS(T_IMU_MS);
		h += 0.5f;
	}
	zassert_true(min_conf < 0.95f, "drift must lower the confidence (%f)", (double)min_conf);
}

ZTEST(orient_gravity, test_nan_sample_rejected)
{
	struct wp_orient_gravity_result r = feed(30.0f, 90.0f, 1.0f, 250);
	struct wp_imu_sample s = imu_tilt(now, 30.0f, 90.0f, 1.0f);

	s.accel[1] = NAN;
	wp_orient_gravity_update(&g, &s, &r);
	now += MS(T_IMU_MS);
	zassert_false(r.valid);
	zassert_false(r.confirmed);

	/* Filter not poisoned: recovers normally. */
	r = feed(30.0f, 90.0f, 1.0f, 400);
	zassert_true(r.valid);
	zassert_true(r.confirmed);
	zassert_true(isfinite(r.heading_deg));
}

ZTEST(orient_gravity, test_lowpass_and_gap_reseed)
{
	struct wp_orient_gravity_result r = feed(0.0f, 0.0f, 1.0f, 200);

	zassert_false(r.valid);
	/* Step to 40 deg: the low-pass (tau 50 ms) needs time to follow. */
	r = feed(40.0f, 90.0f, 1.0f, T_IMU_MS);
	zassert_true(r.tilt_deg < 15.0f, "filtered tilt %f", (double)r.tilt_deg);
	r = feed(40.0f, 90.0f, 1.0f, 300);
	zassert_within(r.tilt_deg, 40.0f, 0.5f);

	/* After a 2 s gap the filter is re-seeded with the new pose at once and
	 * the confirmation does not span the gap.
	 */
	now += MS(2000);
	r = feed(40.0f, 270.0f, 1.0f, T_IMU_MS);
	zassert_within(r.heading_deg, 270.0f, 0.05f);
	zassert_equal(r.candidate, WP_ORIENT_270);
	zassert_false(r.confirmed);
}

ZTEST(orient_gravity, test_reset_sets_reference)
{
	struct wp_orient_gravity_result r = feed(30.0f, 90.0f, 1.0f, 500);

	zassert_true(r.confirmed);
	wp_orient_gravity_reset(&g, WP_ORIENT_90);
	zassert_equal(g.reference, WP_ORIENT_90);
	zassert_false(g.have_candidate);
	zassert_false(g.g_init);

	/* With reference 90, heading 50 is sticky (would have been 0 otherwise). */
	r = feed(35.0f, 50.0f, 1.0f, 500);
	zassert_equal(r.candidate, WP_ORIENT_90);
}

ZTEST_SUITE(orient_gravity, NULL, NULL, before, NULL, NULL);
