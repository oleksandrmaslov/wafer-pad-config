/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for src/air_mouse/air_mouse.c (with orientation_transform.c).
 */

#include <math.h>

#include <zephyr/ztest.h>

#include <wafer_pad/air_mouse.h>
#include <wafer_pad/orientation_transform.h>

#define DT_US 10000LL /* 100 Hz */
#define DT_S  0.01f
#define G     WP_GRAVITY_MS2

static struct wp_air_mouse am;
static struct wp_air_mouse_config cfg;
static int64_t now_us;

static const float FLAT[3] = {0.0f, 0.0f, G};
static const float ZERO3[3] = {0.0f, 0.0f, 0.0f};

/** One sample at now_us + DT; gyro given in deg/s (body frame). */
static void sample(const float gyro_dps[3], const float accel[3], bool gyro_valid, float *dx,
		   float *dy)
{
	struct wp_imu_sample s = {0};

	now_us += DT_US;
	s.t_us = now_us;
	for (int i = 0; i < 3; i++) {
		s.gyro[i] = gyro_dps[i] * WP_DEG_TO_RAD;
		s.accel[i] = accel[i];
	}
	s.gyro_valid = gyro_valid;
	wp_air_mouse_update(&am, &s, dx, dy);
}

/** Feed @p n identical samples, accumulate the output. */
static void feed(int n, const float gyro_dps[3], const float accel[3], float *sx, float *sy)
{
	float x = 0.0f;
	float y = 0.0f;

	for (int i = 0; i < n; i++) {
		float dx;
		float dy;

		sample(gyro_dps, accel, true, &dx, &dy);
		x += dx;
		y += dy;
	}
	if (sx != NULL) {
		*sx = x;
	}
	if (sy != NULL) {
		*sy = y;
	}
}

static void scale3(const float v[3], float k, float out[3])
{
	for (int i = 0; i < 3; i++) {
		out[i] = v[i] * k;
	}
}

/** Fresh instance with @p c, gravity settled flat for 0.5 s while inactive. */
static void reset_with(const struct wp_air_mouse_config *c)
{
	now_us = 1000000LL;
	wp_air_mouse_init(&am, c);
	feed(50, ZERO3, FLAT, NULL, NULL);
}

static void before(void *f)
{
	ARG_UNUSED(f);
	wp_air_mouse_config_default(&cfg);
	reset_with(&cfg);
}

/** Expected per-sample output magnitude for a steady rate (after ramp). */
static float steady(float rate_dps)
{
	float w = fabsf(rate_dps) - cfg.deadzone_dps;

	if (w < 0.0f) {
		w = 0.0f;
	}
	if (w > cfg.max_rate_dps) {
		w = cfg.max_rate_dps;
	}
	return w * cfg.counts_per_deg * DT_S;
}

ZTEST(air_mouse, test_config_defaults)
{
	struct wp_air_mouse_config c;

	wp_air_mouse_config_default(&c);
	zassert_within(c.counts_per_deg, 12.0f, 1e-6f);
	zassert_within(c.deadzone_dps, 1.5f, 1e-6f);
	zassert_within(c.lpf_cutoff_hz, 15.0f, 1e-6f);
	zassert_within(c.max_rate_dps, 400.0f, 1e-6f);
	zassert_equal(c.ramp_in_ms, 120U);
	zassert_true(c.gravity_compensated);
	zassert_within(c.accel_lp_tau_ms, 150.0f, 1e-6f);
	zassert_false(c.invert_x);
	zassert_false(c.invert_y);

	/* NULL config selects the defaults. */
	wp_air_mouse_init(&am, NULL);
	zassert_within(am.cfg.counts_per_deg, 12.0f, 1e-6f);
	zassert_false(wp_air_mouse_active(&am));
}

ZTEST(air_mouse, test_zero_when_inactive_or_gyro_invalid)
{
	const float fast[3] = {50.0f, -80.0f, 120.0f};
	float dx;
	float dy;

	/* Never started: exactly zero. */
	for (int i = 0; i < 20; i++) {
		sample(fast, FLAT, true, &dx, &dy);
		zassert_equal(dx, 0.0f);
		zassert_equal(dy, 0.0f);
	}

	wp_air_mouse_start(&am, now_us);
	zassert_true(wp_air_mouse_active(&am));
	feed(30, fast, FLAT, NULL, NULL);

	/* Gyro powered down while active: exactly zero. */
	for (int i = 0; i < 10; i++) {
		sample(fast, FLAT, false, &dx, &dy);
		zassert_equal(dx, 0.0f);
		zassert_equal(dy, 0.0f);
	}

	/* Non-finite gyro data is treated as invalid. */
	const float bad[3] = {NAN, 0.0f, 0.0f};

	sample(bad, FLAT, true, &dx, &dy);
	zassert_equal(dx, 0.0f);
	zassert_equal(dy, 0.0f);

	/* Gyro back: output resumes. */
	sample(fast, FLAT, true, &dx, &dy);
	sample(fast, FLAT, true, &dx, &dy);
	zassert_true(fabsf(dx) > 0.1f && fabsf(dy) > 0.1f);

	/* NULL output pointers / sample are tolerated. */
	wp_air_mouse_update(&am, NULL, &dx, &dy);
	zassert_equal(dx, 0.0f);
	zassert_equal(dy, 0.0f);
	sample(fast, FLAT, true, NULL, NULL);
}

ZTEST(air_mouse, test_samples_before_start_time_are_zero)
{
	const float yaw[3] = {0.0f, 0.0f, 40.0f};
	float dx;
	float dy;

	/* Clutch pressed "in the future" relative to queued samples. */
	wp_air_mouse_start(&am, now_us + 5 * DT_US);
	for (int i = 0; i < 5; i++) {
		sample(yaw, FLAT, true, &dx, &dy);
		zassert_equal(dx, 0.0f);
		zassert_equal(dy, 0.0f);
	}
	sample(yaw, FLAT, true, &dx, &dy);
	zassert_true(dx < 0.0f);
}

ZTEST(air_mouse, test_yaw_left_moves_cursor_left)
{
	const float yaw_left[3] = {0.0f, 0.0f, 30.0f}; /* +z_B, flat pad */
	float sx;
	float sy;

	wp_air_mouse_start(&am, now_us);
	feed(50, yaw_left, FLAT, &sx, &sy);
	zassert_true(sx < -10.0f, "dx sum %f", (double)sx);
	zassert_within(sy, 0.0f, 1e-3f);

	/* Turning right: cursor right. */
	const float yaw_right[3] = {0.0f, 0.0f, -30.0f};

	feed(50, yaw_right, FLAT, &sx, &sy);
	zassert_true(sx > 10.0f);
}

ZTEST(air_mouse, test_pitch_forward_edge_up_moves_cursor_up)
{
	/* Orientation 0: right = +x_B; forward edge up = positive about right. */
	const float pitch_up[3] = {30.0f, 0.0f, 0.0f};
	float sx;
	float sy;

	wp_air_mouse_start(&am, now_us);
	feed(50, pitch_up, FLAT, &sx, &sy);
	zassert_true(sy < -10.0f, "dy sum %f", (double)sy);
	zassert_within(sx, 0.0f, 1e-3f);
}

ZTEST(air_mouse, test_same_motion_all_orientations)
{
	const float rate = 25.0f;
	float ref_x = 0.0f;
	float ref_y = 0.0f;

	for (int o = 0; o < WP_ORIENT_COUNT; o++) {
		float fwd[3];
		float right[3];
		float pitch[3];
		float roll[3];
		const float yaw[3] = {0.0f, 0.0f, rate};
		float sx;
		float sy;

		wp_orientation_axes_body((enum wp_orientation)o, fwd, right);
		scale3(right, rate, pitch);
		scale3(fwd, rate, roll);

		reset_with(&cfg);
		wp_air_mouse_set_orientation(&am, (enum wp_orientation)o);

		/* Yaw left about the vertical: cursor left. */
		wp_air_mouse_start(&am, now_us);
		feed(40, yaw, FLAT, &sx, &sy);
		zassert_true(sx < -5.0f, "o=%d dx=%f", o, (double)sx);
		zassert_within(sy, 0.0f, 1e-3f, "o=%d", o);
		if (o == 0) {
			ref_x = sx;
		} else {
			zassert_within(sx, ref_x, 1e-3f, "o=%d", o);
		}
		wp_air_mouse_stop(&am);

		/* User's forward edge up: cursor up, regardless of which pad edge it is. */
		wp_air_mouse_start(&am, now_us);
		feed(40, pitch, FLAT, &sx, &sy);
		zassert_true(sy < -5.0f, "o=%d dy=%f", o, (double)sy);
		zassert_within(sx, 0.0f, 1e-3f, "o=%d", o);
		if (o == 0) {
			ref_y = sy;
		} else {
			zassert_within(sy, ref_y, 1e-3f, "o=%d", o);
		}
		wp_air_mouse_stop(&am);

		/* Roll about the user's forward axis is ignored. */
		wp_air_mouse_start(&am, now_us);
		feed(40, roll, FLAT, &sx, &sy);
		zassert_within(sx, 0.0f, 1e-3f, "o=%d", o);
		zassert_within(sy, 0.0f, 1e-3f, "o=%d", o);
		wp_air_mouse_stop(&am);
	}
}

ZTEST(air_mouse, test_deadzone)
{
	const float slow[3] = {1.0f, 0.0f, 1.2f}; /* below 1.5 dps on both axes */
	const float med[3] = {0.0f, 0.0f, 5.0f};
	float dx;
	float dy;

	wp_air_mouse_start(&am, now_us);
	for (int i = 0; i < 60; i++) {
		sample(slow, FLAT, true, &dx, &dy);
		zassert_equal(dx, 0.0f);
		zassert_equal(dy, 0.0f);
	}

	/* Above the deadzone: reduced by the deadzone (soft), not just gated. */
	feed(60, med, FLAT, NULL, NULL);
	sample(med, FLAT, true, &dx, &dy);
	zassert_within(dx, -steady(5.0f), 1e-4f);
	zassert_within(dx, -(5.0f - 1.5f) * 12.0f * DT_S, 1e-4f);
}

ZTEST(air_mouse, test_lpf_step_response_monotonic)
{
	const float rate = 60.0f;
	const float step[3] = {0.0f, 0.0f, rate};
	float prev = 0.0f;
	float dx;
	float dy;

	wp_air_mouse_start(&am, now_us);
	feed(30, ZERO3, FLAT, NULL, NULL); /* past the ramp, filter at zero */

	float rc = 1.0f / (2.0f * WP_PI * cfg.lpf_cutoff_hz);
	float alpha = DT_S / (DT_S + rc);
	float lp = 0.0f;

	for (int i = 0; i < 40; i++) {
		sample(step, FLAT, true, &dx, &dy);
		lp += alpha * (rate - lp);

		float mag = -dx;

		zassert_true(mag >= prev - 1e-6f, "not monotonic at %d", i);
		zassert_within(mag, (lp - cfg.deadzone_dps) * cfg.counts_per_deg * DT_S, 1e-3f,
			       "i=%d", i);
		if (i == 0) {
			/* Filter lag: first sample well below the final value. */
			zassert_true(mag < 0.6f * steady(rate));
		}
		prev = mag;
	}
	zassert_within(prev, steady(rate), 0.001f * steady(rate));
}

ZTEST(air_mouse, test_lpf_disabled_passes_through)
{
	const float step[3] = {0.0f, 0.0f, 60.0f};
	float dx;
	float dy;

	cfg.lpf_cutoff_hz = 0.0f;
	reset_with(&cfg);
	wp_air_mouse_start(&am, now_us);
	feed(30, ZERO3, FLAT, NULL, NULL);
	sample(step, FLAT, true, &dx, &dy);
	zassert_within(-dx, steady(60.0f), 1e-4f);
}

ZTEST(air_mouse, test_ramp_in)
{
	const float yaw[3] = {0.0f, 0.0f, 20.0f};
	const int ramp_n = (int)(cfg.ramp_in_ms * 1000U / DT_US); /* 12 samples */
	float dx;
	float dy;

	wp_air_mouse_start(&am, now_us);
	for (int k = 1; k <= ramp_n + 10; k++) {
		sample(yaw, FLAT, true, &dx, &dy);

		float r = (k < ramp_n) ? (float)k / (float)ramp_n : 1.0f;

		zassert_within(-dx, steady(20.0f) * r, 1e-4f, "k=%d", k);
	}

	/* No ramp configured: full gain on the first sample. */
	cfg.ramp_in_ms = 0U;
	reset_with(&cfg);
	wp_air_mouse_start(&am, now_us);
	sample(yaw, FLAT, true, &dx, &dy);
	zassert_within(-dx, steady(20.0f), 1e-4f);
}

/** Rotation about world vertical with the pad tilted so that "up" is @p up_b. */
static void tilted_yaw(const float up_b[3], float rate, float *sx, float *sy)
{
	float accel[3];
	float gyro[3];

	scale3(up_b, G, accel);
	scale3(up_b, rate, gyro);
	/* Gravity estimate settles while INACTIVE (from the flat posture). */
	feed(100, ZERO3, accel, NULL, NULL);
	wp_air_mouse_start(&am, now_us);
	feed(50, gyro, accel, sx, sy);
	wp_air_mouse_stop(&am);
}

ZTEST(air_mouse, test_gravity_compensation_tilted_40deg)
{
	const float a = 40.0f * WP_DEG_TO_RAD;
	/* Right edge raised 40 deg (roll): world up in body = (sin, 0, cos). */
	const float up_roll[3] = {sinf(a), 0.0f, cosf(a)};
	/* Forward edge raised 40 deg (pitch): world up in body = (0, sin, cos). */
	const float up_pitch[3] = {0.0f, sinf(a), cosf(a)};
	/* Raised about a diagonal axis. */
	const float up_diag[3] = {sinf(a) * 0.70710678f, sinf(a) * 0.70710678f, cosf(a)};
	const float rate = 30.0f;
	/* 50 samples, 12-sample linear ramp: effective 50 - 5.5 samples. */
	const float expect = -steady(rate) * (50.0f - 5.5f);
	float sx;
	float sy;

	tilted_yaw(up_roll, rate, &sx, &sy);
	zassert_within(sx, expect, 0.02f * fabsf(expect), "dx %f", (double)sx);
	zassert_true(fabsf(sy) < 0.02f * fabsf(sx), "dy %f", (double)sy);

	reset_with(&cfg);
	tilted_yaw(up_pitch, rate, &sx, &sy);
	zassert_within(sx, expect, 0.02f * fabsf(expect));
	zassert_true(fabsf(sy) < 0.02f * fabsf(sx));

	reset_with(&cfg);
	tilted_yaw(up_diag, rate, &sx, &sy);
	zassert_within(sx, expect, 0.02f * fabsf(expect));
	zassert_true(fabsf(sy) < 0.02f * fabsf(sx));

	/* Pitch about the HORIZONTAL right axis while tilted: mostly dy. */
	reset_with(&cfg);
	{
		float accel[3];
		float fwd[3] = {0.0f, 1.0f, 0.0f};
		float h[3] = {fwd[1] * up_diag[2] - fwd[2] * up_diag[1],
			      fwd[2] * up_diag[0] - fwd[0] * up_diag[2],
			      fwd[0] * up_diag[1] - fwd[1] * up_diag[0]};
		float hn = sqrtf(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
		float gyro[3];

		scale3(up_diag, G, accel);
		scale3(h, rate / hn, gyro);
		feed(100, ZERO3, accel, NULL, NULL);
		wp_air_mouse_start(&am, now_us);
		feed(50, gyro, accel, &sx, &sy);
		zassert_true(sy < 0.0f);
		zassert_true(fabsf(sx) < 0.02f * fabsf(sy));
	}

	/* Without compensation the roll-tilted yaw leaks strongly into dy. */
	cfg.gravity_compensated = false;
	reset_with(&cfg);
	tilted_yaw(up_roll, rate, &sx, &sy);
	zassert_true(fabsf(sy) > 0.5f * fabsf(sx), "dx %f dy %f", (double)sx, (double)sy);
}

ZTEST(air_mouse, test_yaw_about_vertical_with_forward_edge_up)
{
	/* ALG-8: forward edge raised by `tilt` (up = (0, sin, cos) in the body
	 * frame); turning left about WORLD vertical must give the same dx at
	 * 85 deg as at 30 deg, and pitch must stay ~0. */
	static const float tilts[] = {30.0f, 78.0f, 80.0f, 85.0f, 89.0f};
	const float rate = 10.0f;
	float ref = 0.0f;

	for (size_t i = 0; i < ARRAY_SIZE(tilts); i++) {
		const float a = tilts[i] * WP_DEG_TO_RAD;
		const float up[3] = {0.0f, sinf(a), cosf(a)};
		float sx;
		float sy;

		reset_with(&cfg);
		tilted_yaw(up, rate, &sx, &sy);
		if (i == 0) {
			ref = sx;
			zassert_true(ref < -10.0f, "dx %f", (double)ref);
		}
		zassert_within(sx, ref, 0.03f * fabsf(ref), "tilt %d: dx %f vs %f", (int)tilts[i],
			       (double)sx, (double)ref);
		zassert_true(fabsf(sy) < 0.03f * fabsf(ref), "tilt %d: dy %f", (int)tilts[i],
			     (double)sy);
	}
}

ZTEST(air_mouse, test_vertical_and_free_fall_fallback)
{
	const float up_fwd[3] = {0.0f, G, 0.0f}; /* forward edge points straight up */
	const float pitch[3] = {30.0f, 0.0f, 0.0f};
	float sx;
	float sy;

	/* |fwd x up| ~ 0: falls back to the uncompensated axes (right = +x_B). */
	feed(100, ZERO3, up_fwd, NULL, NULL);
	wp_air_mouse_start(&am, now_us);
	feed(30, pitch, up_fwd, &sx, &sy);
	zassert_true(isfinite(sx) && isfinite(sy));
	zassert_true(sy < -1.0f);
	zassert_within(sx, 0.0f, 1e-3f);
	wp_air_mouse_stop(&am);

	/* Free fall (no usable gravity): still finite, uncompensated axes. */
	reset_with(&cfg);
	feed(100, ZERO3, ZERO3, NULL, NULL);
	wp_air_mouse_start(&am, now_us);
	const float yaw[3] = {0.0f, 0.0f, 30.0f};

	feed(30, yaw, ZERO3, &sx, &sy);
	zassert_true(isfinite(sx) && isfinite(sy));
	zassert_true(sx < -1.0f);
	zassert_within(sy, 0.0f, 1e-3f);
}

ZTEST(air_mouse, test_stop_gives_zero_and_restart_ramps)
{
	const float yaw[3] = {0.0f, 0.0f, 50.0f};
	float dx;
	float dy;

	wp_air_mouse_start(&am, now_us);
	feed(30, yaw, FLAT, NULL, NULL);
	sample(yaw, FLAT, true, &dx, &dy);
	zassert_true(dx < -1.0f);

	wp_air_mouse_stop(&am);
	zassert_false(wp_air_mouse_active(&am));
	for (int i = 0; i < 20; i++) {
		sample(yaw, FLAT, true, &dx, &dy);
		zassert_equal(dx, 0.0f);
		zassert_equal(dy, 0.0f);
	}

	/* Restart: the ramp-in starts again from zero gain. */
	wp_air_mouse_start(&am, now_us);
	sample(yaw, FLAT, true, &dx, &dy);
	zassert_within(-dx, steady(50.0f) / 12.0f, 1e-3f);
}

ZTEST(air_mouse, test_magnitude_10dps_for_1s)
{
	const float yaw[3] = {0.0f, 0.0f, 10.0f};
	float sx;
	float sy;

	wp_air_mouse_start(&am, now_us);
	feed(100, yaw, FLAT, &sx, &sy);

	/* 10 deg turned -> 10 deg * 12 counts/deg = 120 counts, minus the soft
	 * deadzone (1.5 dps -> 8.5 effective) and the 120 ms linear ramp
	 * (5.5 samples lost of 100): 8.5 * 12 * 0.945 = 96.39 counts.
	 */
	zassert_within(sx, -96.39f, 0.5f, "sum dx %f", (double)sx);
	zassert_within(sy, 0.0f, 1e-3f);

	/* Same without deadzone/ramp: exactly 10 deg * counts_per_deg. */
	cfg.deadzone_dps = 0.0f;
	cfg.ramp_in_ms = 0U;
	reset_with(&cfg);
	wp_air_mouse_start(&am, now_us);
	feed(100, yaw, FLAT, &sx, &sy);
	zassert_within(sx, -10.0f * cfg.counts_per_deg, 0.5f, "sum dx %f", (double)sx);
}

ZTEST(air_mouse, test_clamp_max_rate)
{
	const float huge[3] = {0.0f, 0.0f, 2000.0f};
	float dx;
	float dy;

	wp_air_mouse_start(&am, now_us);
	feed(40, huge, FLAT, NULL, NULL);
	sample(huge, FLAT, true, &dx, &dy);
	zassert_within(-dx, cfg.max_rate_dps * cfg.counts_per_deg * DT_S, 1e-3f);
}

ZTEST(air_mouse, test_invert_axes)
{
	const float both[3] = {20.0f, 0.0f, 20.0f};
	float sx;
	float sy;

	wp_air_mouse_start(&am, now_us);
	feed(30, both, FLAT, &sx, &sy);
	zassert_true(sx < 0.0f && sy < 0.0f);

	cfg.invert_x = true;
	cfg.invert_y = true;
	reset_with(&cfg);
	wp_air_mouse_start(&am, now_us);

	float ix;
	float iy;

	feed(30, both, FLAT, &ix, &iy);
	zassert_within(ix, -sx, 1e-4f);
	zassert_within(iy, -sy, 1e-4f);
}

ZTEST(air_mouse, test_time_gaps_and_duplicates)
{
	const float yaw[3] = {0.0f, 0.0f, 40.0f};
	struct wp_imu_sample s = {0};
	float dx;
	float dy;

	wp_air_mouse_start(&am, now_us);
	feed(30, yaw, FLAT, NULL, NULL);

	/* Duplicate timestamp: no output (dt == 0). */
	s.t_us = now_us;
	s.accel[2] = G;
	s.gyro[2] = 40.0f * WP_DEG_TO_RAD;
	s.gyro_valid = true;
	wp_air_mouse_update(&am, &s, &dx, &dy);
	zassert_equal(dx, 0.0f);

	/* Gap of 250 ms: filter reset, no output for that sample. */
	now_us += 250000LL;
	s.t_us = now_us;
	wp_air_mouse_update(&am, &s, &dx, &dy);
	zassert_equal(dx, 0.0f);
	zassert_equal(dy, 0.0f);

	/* Next regular sample: normal output again (filter re-seeded). */
	sample(yaw, FLAT, true, &dx, &dy);
	zassert_within(-dx, steady(40.0f), 1e-3f);

	/* Timestamp going backwards: treated as a discontinuity. */
	s.t_us = now_us - 50000LL;
	wp_air_mouse_update(&am, &s, &dx, &dy);
	zassert_equal(dx, 0.0f);
	now_us = s.t_us;
	sample(yaw, FLAT, true, &dx, &dy);
	zassert_within(-dx, steady(40.0f), 1e-3f);
}

ZTEST(air_mouse, test_set_orientation_normalizes_and_resets)
{
	const float yaw[3] = {0.0f, 0.0f, 40.0f};
	float dx;
	float dy;

	wp_air_mouse_set_orientation(&am, (enum wp_orientation)6); /* -> 180 */
	zassert_equal(am.orientation, WP_ORIENT_180);

	wp_air_mouse_start(&am, now_us);
	feed(30, yaw, FLAT, NULL, NULL);
	zassert_true(am.lp_init);
	wp_air_mouse_set_orientation(&am, WP_ORIENT_90);
	zassert_false(am.lp_init);
	zassert_true(wp_air_mouse_active(&am));
	sample(yaw, FLAT, true, &dx, &dy);
	zassert_true(dx < 0.0f);
}

ZTEST_SUITE(air_mouse, NULL, NULL, before, NULL, NULL);
