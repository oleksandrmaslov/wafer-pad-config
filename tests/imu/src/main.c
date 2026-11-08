/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host tests for src/imu/imu_algo.c (gyro bias estimator, motion features).
 */

#include <math.h>
#include <string.h>

#include <zephyr/ztest.h>

#include <wafer_pad/imu_algo.h>

#define ODR_HZ    104
#define DT_US     (1000000LL / ODR_HZ)
#define G         WP_GRAVITY_MS2
#define DPS       WP_DEG_TO_RAD

/* Deterministic pseudo-random noise in [-1, 1]. */
static uint32_t rng_state;

static void rng_seed(uint32_t s)
{
	rng_state = s;
}

static float rnd(void)
{
	rng_state = rng_state * 1664525u + 1013904223u;
	return ((float)(rng_state >> 8) / (float)(1u << 24)) * 2.0f - 1.0f;
}

/* ------------------------------------------------------------------------ */
/* Gyro bias                                                                 */
/* ------------------------------------------------------------------------ */

struct bias_sim {
	struct wp_gyro_bias b;
	int64_t t;
	float true_bias_dps[3];
	float gyro_noise_dps;
	float accel_noise_g;
};

static void bias_sim_init(struct bias_sim *s, float bx, float by, float bz)
{
	memset(s, 0, sizeof(*s));
	wp_gyro_bias_init(&s->b, NULL);
	s->t = 1000000;
	s->true_bias_dps[0] = bx;
	s->true_bias_dps[1] = by;
	s->true_bias_dps[2] = bz;
	s->gyro_noise_dps = 0.05f;
	s->accel_noise_g = 0.002f;
	rng_seed(12345u);
}

/* Feed n samples; extra_rate_dps is added on z, accel_kick_g on x. */
static bool bias_sim_run(struct bias_sim *s, int n, float extra_rate_dps, float accel_kick_g)
{
	bool still = false;

	for (int i = 0; i < n; i++) {
		float raw[3];
		float acc[3];

		for (int k = 0; k < 3; k++) {
			raw[k] = (s->true_bias_dps[k] + s->gyro_noise_dps * rnd()) * DPS;
			acc[k] = s->accel_noise_g * rnd() * G;
		}
		raw[2] += extra_rate_dps * DPS;
		acc[2] += G;
		acc[0] += accel_kick_g * G;
		still = wp_gyro_bias_update(&s->b, s->t, raw, acc);
		s->t += DT_US;
	}
	return still;
}

static void expect_bias(const struct wp_gyro_bias *b, float x, float y, float z, float tol_dps)
{
	zassert_true(wp_gyro_bias_valid(b));
	zassert_within(b->bias[0] / DPS, x, tol_dps, "x %f", (double)(b->bias[0] / DPS));
	zassert_within(b->bias[1] / DPS, y, tol_dps, "y %f", (double)(b->bias[1] / DPS));
	zassert_within(b->bias[2] / DPS, z, tol_dps, "z %f", (double)(b->bias[2] / DPS));
}

ZTEST(imu_bias, test_defaults)
{
	struct wp_gyro_bias_config c;

	wp_gyro_bias_config_default(&c);
	zassert_within(c.still_gyro_dps, 1.0f, 1e-6f);
	zassert_within(c.still_accel_dev_g, 0.03f, 1e-6f);
	zassert_equal(c.still_min_ms, 800u);
	zassert_within(c.alpha, 0.02f, 1e-6f);
	zassert_within(c.max_bias_dps, 10.0f, 1e-6f);
	zassert_within(c.max_noise_dps, 0.15f, 1e-6f);
	zassert_equal(c.relearn_ms, 5000u);
	zassert_within(c.relearn_accel_rms_g, 0.002f, 1e-7f);
}

ZTEST(imu_bias, test_converges_while_still)
{
	static struct bias_sim s;

	bias_sim_init(&s, 1.2f, -0.7f, 0.4f);

	/* Not valid before the first still window (800 ms) completed. */
	zassert_true(bias_sim_run(&s, 70, 0.0f, 0.0f));
	zassert_false(wp_gyro_bias_valid(&s.b));

	(void)bias_sim_run(&s, 40, 0.0f, 0.0f); /* ~1.06 s total */
	expect_bias(&s.b, 1.2f, -0.7f, 0.4f, 0.03f);

	/* Slow drift of the true bias is tracked by the EMA while still. */
	s.true_bias_dps[0] = 1.6f;
	(void)bias_sim_run(&s, 4 * ODR_HZ, 0.0f, 0.0f);
	expect_bias(&s.b, 1.6f, -0.7f, 0.4f, 0.03f);

	/* Applying the bias removes it. */
	float raw[3] = {1.6f * DPS, -0.7f * DPS, 0.4f * DPS};

	wp_gyro_bias_apply(&s.b, raw, raw); /* in-place (aliasing) allowed */
	for (int k = 0; k < 3; k++) {
		zassert_within(raw[k] / DPS, 0.0f, 0.03f);
	}
}

ZTEST(imu_bias, test_no_estimate_while_rotating)
{
	static struct bias_sim s;

	bias_sim_init(&s, 0.5f, 0.5f, 0.5f);
	zassert_false(bias_sim_run(&s, 3 * ODR_HZ, 30.0f, 0.0f));
	zassert_false(wp_gyro_bias_valid(&s.b));

	/* Slow-but-not-still rotation that varies (wobble) is not still either. */
	for (int i = 0; i < 3 * ODR_HZ; i++) {
		float w = 8.0f * sinf(2.0f * WP_PI * 1.0f * (float)i / ODR_HZ);

		(void)bias_sim_run(&s, 1, w, 0.0f);
	}
	zassert_false(wp_gyro_bias_valid(&s.b));
}

ZTEST(imu_bias, test_no_update_while_moving)
{
	static struct bias_sim s;
	float before[3];

	bias_sim_init(&s, 1.0f, 0.0f, -1.0f);
	(void)bias_sim_run(&s, 2 * ODR_HZ, 0.0f, 0.0f);
	expect_bias(&s.b, 1.0f, 0.0f, -1.0f, 0.03f);
	memcpy(before, s.b.bias, sizeof(before));

	/* Rotation: clearly moving -> not still, bias untouched. */
	zassert_false(bias_sim_run(&s, 2 * ODR_HZ, 25.0f, 0.0f));
	zassert_mem_equal(before, s.b.bias, sizeof(before));

	/* Small rate offset (inside the gyro still band) with accel spikes every
	 * 200 ms: stillness never lasts still_min_ms, so no EMA update.
	 */
	for (int i = 0; i < 20; i++) {
		(void)bias_sim_run(&s, 20, 2.5f, 0.0f);
		zassert_false(bias_sim_run(&s, 1, 2.5f, 0.3f));
	}
	zassert_mem_equal(before, s.b.bias, sizeof(before));

	/* A held, slightly vibrating device (accel jitter > 0.03 g) also blocks updates. */
	for (int i = 0; i < 2 * ODR_HZ; i++) {
		(void)bias_sim_run(&s, 1, 2.5f, (i & 1) ? 0.05f : -0.05f);
	}
	zassert_mem_equal(before, s.b.bias, sizeof(before));
}

ZTEST(imu_bias, test_rejects_implausible_bias)
{
	static struct bias_sim s;

	bias_sim_init(&s, 12.0f, 0.0f, 0.0f); /* > max_bias_dps = 10 */
	(void)bias_sim_run(&s, 4 * ODR_HZ, 0.0f, 0.0f);
	zassert_false(wp_gyro_bias_valid(&s.b));
}

ZTEST(imu_bias, test_large_offset_first_estimate)
{
	static struct bias_sim s;

	/* Offset above still_gyro_dps (1) but below max_bias_dps (10) is still learnt. */
	bias_sim_init(&s, 0.0f, 6.0f, 0.0f);
	(void)bias_sim_run(&s, 2 * ODR_HZ, 0.0f, 0.0f);
	expect_bias(&s.b, 0.0f, 6.0f, 0.0f, 0.03f);
}

ZTEST(imu_bias, test_set_restore_and_restart)
{
	static struct bias_sim s;
	const float restored[3] = {0.5f * DPS, -0.25f * DPS, 2.0f * DPS};
	const float too_big[3] = {20.0f * DPS, 0.0f, 0.0f};
	const float nan_bias[3] = {NAN, 0.0f, 0.0f};
	float out[3];
	const float raw[3] = {1.0f * DPS, 1.0f * DPS, 1.0f * DPS};

	bias_sim_init(&s, 0.5f, -0.25f, 2.0f);

	/* Not valid: apply passes the raw value through. */
	wp_gyro_bias_apply(&s.b, raw, out);
	zassert_mem_equal(out, raw, sizeof(out));

	wp_gyro_bias_set(&s.b, restored);
	expect_bias(&s.b, 0.5f, -0.25f, 2.0f, 1e-4f);

	wp_gyro_bias_apply(&s.b, raw, out);
	zassert_within(out[0] / DPS, 0.5f, 1e-4f);
	zassert_within(out[1] / DPS, 1.25f, 1e-4f);
	zassert_within(out[2] / DPS, -1.0f, 1e-4f);

	/* Implausible / corrupt persisted values are ignored. */
	wp_gyro_bias_set(&s.b, too_big);
	wp_gyro_bias_set(&s.b, nan_bias);
	expect_bias(&s.b, 0.5f, -0.25f, 2.0f, 1e-4f);

	/* Restart resets stillness tracking but keeps the bias. */
	(void)bias_sim_run(&s, ODR_HZ, 0.0f, 0.0f);
	zassert_true(s.b.still);
	wp_gyro_bias_restart(&s.b);
	zassert_false(s.b.still);
	zassert_false(s.b.acc_lp_init);
	expect_bias(&s.b, 0.5f, -0.25f, 2.0f, 0.03f);
}

ZTEST(imu_bias, test_gap_resets_window)
{
	static struct bias_sim s;

	bias_sim_init(&s, 1.0f, 1.0f, 1.0f);
	(void)bias_sim_run(&s, 55, 0.0f, 0.0f); /* ~520 ms */
	s.t += 300000;                           /* 300 ms gap > 200 ms */
	(void)bias_sim_run(&s, 55, 0.0f, 0.0f); /* another ~520 ms */
	zassert_false(wp_gyro_bias_valid(&s.b), "window must restart after a gap");
	(void)bias_sim_run(&s, 40, 0.0f, 0.0f); /* > 800 ms since the gap */
	expect_bias(&s.b, 1.0f, 1.0f, 1.0f, 0.03f);

	/* Duplicate timestamp: ignored, no crash, state unchanged. */
	float raw[3] = {100.0f * DPS, 0, 0};
	float acc[3] = {0, 0, G};
	bool st = s.b.still;

	zassert_equal(wp_gyro_bias_update(&s.b, s.t - DT_US, raw, acc), st);
	expect_bias(&s.b, 1.0f, 1.0f, 1.0f, 0.03f);
}

/* Held pad: per-sample random accel jitter of +-jit_g on x/y. */
static void bias_sim_held(struct bias_sim *s, int n, float rate_z_dps, float jit_g,
			  float tremor_dps, int *i0)
{
	for (int i = 0; i < n; i++, (*i0)++) {
		float ph = 2.0f * WP_PI * 9.0f * (float)(*i0) / (float)ODR_HZ;
		float raw[3];
		float acc[3];

		for (int k = 0; k < 3; k++) {
			raw[k] = (s->true_bias_dps[k] + s->gyro_noise_dps * rnd()) * DPS;
		}
		raw[0] += tremor_dps * sinf(ph) * DPS;
		raw[1] += tremor_dps * cosf(ph) * DPS;
		raw[2] += rate_z_dps * DPS;
		acc[0] = jit_g * rnd() * G;
		acc[1] = jit_g * rnd() * G;
		acc[2] = G;
		(void)wp_gyro_bias_update(&s->b, s->t, raw, acc);
		s->t += DT_US;
	}
}

ZTEST(imu_bias, test_slow_steady_rotation_not_absorbed)
{
	/* ALG-5: air-mouse aiming = slow steady yaw (3 dps) with a held pad's
	 * 4 mg accel jitter. The cursor must keep moving: bias unchanged. */
	static struct bias_sim s;
	int i0 = 0;

	bias_sim_init(&s, 0.3f, -0.2f, 0.1f);
	(void)bias_sim_run(&s, 2 * ODR_HZ, 0.0f, 0.0f);
	expect_bias(&s.b, 0.3f, -0.2f, 0.1f, 0.03f);

	bias_sim_held(&s, 3 * ODR_HZ, 3.0f, 0.004f, 0.0f, &i0);
	expect_bias(&s.b, 0.3f, -0.2f, 0.1f, 0.2f);
	/* ... also when it lasts long (no "re-learning" on a held pad). */
	bias_sim_held(&s, 8 * ODR_HZ, 3.0f, 0.004f, 0.0f, &i0);
	expect_bias(&s.b, 0.3f, -0.2f, 0.1f, 0.2f);

	/* Very slow pan (0.8 dps) of a hand with tremor: not absorbed either. */
	bias_sim_held(&s, 4 * ODR_HZ, 0.8f, 0.004f, 0.5f, &i0);
	expect_bias(&s.b, 0.3f, -0.2f, 0.1f, 0.1f);
}

ZTEST(imu_bias, test_hand_tremor_blocks_first_estimate)
{
	/* Held still in the hand (tremor) with a small steady drift: never a
	 * "stationary" window, so no first estimate from hand motion. */
	static struct bias_sim s;
	int i0 = 0;

	bias_sim_init(&s, 0.2f, 0.2f, 0.2f);
	bias_sim_held(&s, 4 * ODR_HZ, 0.6f, 0.004f, 0.5f, &i0);
	zassert_false(wp_gyro_bias_valid(&s.b));
}

ZTEST(imu_bias, test_relearns_wrong_persisted_bias)
{
	/* A stale persisted bias 3 dps off is replaced after relearn_ms of
	 * true rest (table: 2 mg-quiet accel, sensor noise only). */
	static struct bias_sim s;
	const float wrong[3] = {0.0f, 0.0f, 3.0f * DPS};

	bias_sim_init(&s, 0.4f, 0.0f, -0.3f);
	s.accel_noise_g = 0.0005f;
	wp_gyro_bias_set(&s.b, wrong);
	(void)bias_sim_run(&s, 4 * ODR_HZ, 0.0f, 0.0f);
	expect_bias(&s.b, 0.0f, 0.0f, 3.0f, 1e-3f); /* not before relearn_ms */
	(void)bias_sim_run(&s, 3 * ODR_HZ, 0.0f, 0.0f);
	expect_bias(&s.b, 0.4f, 0.0f, -0.3f, 0.05f);
	/* ... and is then tracked normally. */
	(void)bias_sim_run(&s, 2 * ODR_HZ, 0.0f, 0.0f);
	expect_bias(&s.b, 0.4f, 0.0f, -0.3f, 0.03f);
}

ZTEST_SUITE(imu_bias, NULL, NULL, NULL, NULL, NULL);

/* ------------------------------------------------------------------------ */
/* Motion features                                                           */
/* ------------------------------------------------------------------------ */

struct motion_sim {
	struct wp_motion m;
	struct wp_motion_features f;
	int64_t t;
	int i;
};

static void motion_sim_init(struct motion_sim *s)
{
	memset(s, 0, sizeof(*s));
	wp_motion_init(&s->m, NULL);
	s->t = 5000000;
	rng_seed(777u);
}

static void motion_step(struct motion_sim *s, const float gyro_dps[3], const float acc_g[3],
			bool gyro_valid)
{
	struct wp_imu_sample smp = {.t_us = s->t, .gyro_valid = gyro_valid};

	for (int k = 0; k < 3; k++) {
		smp.gyro[k] = gyro_dps[k] * DPS;
		smp.accel[k] = acc_g[k] * G;
	}
	wp_motion_update(&s->m, &smp, &s->f);
	s->t += DT_US;
	s->i++;
}

/* Quiet table: 0.03 dps gyro noise, 0.001 g accel noise, face up. */
static void motion_quiet(struct motion_sim *s, int n)
{
	for (int j = 0; j < n; j++) {
		const float g[3] = {0.03f * rnd(), 0.03f * rnd(), 0.03f * rnd()};
		const float a[3] = {0.001f * rnd(), 0.001f * rnd(), 1.0f + 0.001f * rnd()};

		motion_step(s, g, a, true);
	}
}

ZTEST(imu_motion, test_defaults)
{
	struct wp_motion_config c;

	wp_motion_config_default(&c);
	zassert_within(c.still_gyro_dps, 1.5f, 1e-6f);
	zassert_within(c.still_accel_dev_g, 0.015f, 1e-6f);
	zassert_equal(c.still_window_ms, 300u);
	zassert_within(c.move_gyro_dps, 20.0f, 1e-6f);
	zassert_within(c.move_accel_dev_g, 0.06f, 1e-6f);
	zassert_within(c.tremor_tau_ms, 250.0f, 1e-6f);
	zassert_within(c.accel_lp_tau_ms, 400.0f, 1e-6f);
	zassert_within(c.handheld_tremor_dps, 0.35f, 1e-6f);
	zassert_within(c.handheld_accel_jitter_g, 0.004f, 1e-7f);
}

ZTEST(imu_motion, test_still_after_window)
{
	static struct motion_sim s;

	motion_sim_init(&s);
	motion_quiet(&s, 1);
	zassert_true(s.f.valid);
	zassert_false(s.f.still);
	zassert_equal(s.f.still_ms, 0u);

	motion_quiet(&s, 28); /* ~280 ms: still below the 300 ms window */
	zassert_false(s.f.still);
	zassert_equal(s.f.still_ms, 0u);

	motion_quiet(&s, 4); /* > 300 ms */
	zassert_true(s.f.still);
	zassert_true(s.f.still_ms >= 300u && s.f.still_ms < 340u, "%u", s.f.still_ms);
	zassert_false(s.f.moving);
	zassert_false(s.f.hand_tremor);

	motion_quiet(&s, ODR_HZ);
	zassert_true(s.f.still);
	zassert_true(s.f.still_ms > 1200u);
}

ZTEST(imu_motion, test_moving_on_gyro_spike)
{
	static struct motion_sim s;
	const float spin[3] = {0.0f, 0.0f, 30.0f};
	const float a[3] = {0.0f, 0.0f, 1.0f};

	motion_sim_init(&s);
	motion_quiet(&s, ODR_HZ);
	zassert_true(s.f.still);

	motion_step(&s, spin, a, true);
	zassert_true(s.f.moving);
	zassert_false(s.f.still);
	zassert_equal(s.f.still_ms, 0u);
	zassert_within(s.f.gyro_mag_dps, 30.0f, 0.01f);

	/* Needs a full new window to become still again. */
	motion_quiet(&s, 20);
	zassert_false(s.f.still);
	zassert_false(s.f.moving);
	motion_quiet(&s, 20);
	zassert_true(s.f.still);

	/* Moderate rotation (above still, below move): neither still nor moving. */
	const float slow[3] = {5.0f, 0.0f, 0.0f};

	motion_step(&s, slow, a, true);
	zassert_false(s.f.still);
	zassert_false(s.f.moving);
}

ZTEST(imu_motion, test_moving_on_accel_spike)
{
	static struct motion_sim s;
	const float g0[3] = {0};
	const float bump[3] = {0.1f, 0.0f, 1.0f};

	motion_sim_init(&s);
	motion_quiet(&s, ODR_HZ);
	zassert_true(s.f.still);

	motion_step(&s, g0, bump, true);
	zassert_true(s.f.moving);
	zassert_false(s.f.still);
	zassert_within(s.f.accel_dev_g, 0.1f, 0.005f);

	/* Works with the gyro off as well. */
	motion_quiet(&s, ODR_HZ);
	motion_step(&s, g0, bump, false);
	zassert_true(s.f.moving);
	zassert_equal(s.f.gyro_mag_dps, 0.0f);
	zassert_equal(s.f.tremor_dps, 0.0f);
}

ZTEST(imu_motion, test_hand_tremor_gyro)
{
	static struct motion_sim s;
	const float a[3] = {0.0f, 0.0f, 1.0f};

	/* ~1 dps, 9 Hz physiological tremor on two axes. */
	motion_sim_init(&s);
	for (int i = 0; i < 2 * ODR_HZ; i++) {
		float ph = 2.0f * WP_PI * 9.0f * (float)i / (float)ODR_HZ;
		const float g[3] = {1.0f * sinf(ph), 0.5f * cosf(ph), 0.0f};

		motion_step(&s, g, a, true);
	}
	zassert_true(s.f.hand_tremor, "tremor %f", (double)s.f.tremor_dps);
	zassert_true(s.f.tremor_dps > 0.6f && s.f.tremor_dps < 1.0f, "%f",
		     (double)s.f.tremor_dps);
	zassert_false(s.f.moving);

	/* Table noise only (0.03 dps): no tremor. */
	motion_sim_init(&s);
	motion_quiet(&s, 2 * ODR_HZ);
	zassert_false(s.f.hand_tremor, "tremor %f", (double)s.f.tremor_dps);
	zassert_true(s.f.tremor_dps < 0.06f, "%f", (double)s.f.tremor_dps);
	zassert_true(s.f.tremor_from_gyro);

	/* A residual bias error / slow steady rotation (constant 0.6 dps) is
	 * not tremor: tremor is the AC part of the rate. */
	motion_sim_init(&s);
	for (int i = 0; i < 2 * ODR_HZ; i++) {
		const float g[3] = {0.6f + 0.03f * rnd(), 0.03f * rnd(), 0.03f * rnd()};

		motion_step(&s, g, a, true);
	}
	zassert_false(s.f.hand_tremor, "DC offset taken for tremor (%f dps)",
		      (double)s.f.tremor_dps);
	zassert_true(s.f.still);
}

ZTEST(imu_motion, test_hand_tremor_accel_when_gyro_off)
{
	static struct motion_sim s;
	const float g0[3] = {0};

	motion_sim_init(&s);
	for (int i = 0; i < 2 * ODR_HZ; i++) {
		float ph = 2.0f * WP_PI * 9.0f * (float)i / (float)ODR_HZ;
		const float a[3] = {0.01f * sinf(ph), 0.0f, 1.0f};

		motion_step(&s, g0, a, false);
	}
	zassert_true(s.f.hand_tremor, "jitter %f", (double)s.f.accel_jitter_g);
	zassert_false(s.f.tremor_from_gyro);
	zassert_equal(s.f.tremor_dps, 0.0f);

	motion_sim_init(&s);
	for (int i = 0; i < 2 * ODR_HZ; i++) {
		const float a[3] = {0.0005f * rnd(), 0.0005f * rnd(), 1.0f};

		motion_step(&s, g0, a, false);
	}
	zassert_false(s.f.hand_tremor, "jitter %f", (double)s.f.accel_jitter_g);
	zassert_true(s.f.still, "accel-only stillness with gyro off");
}

static void tilt_case(float deg, float exp_tilt, bool exp_up)
{
	static struct motion_sim s;
	const float g0[3] = {0};
	const float r = deg * WP_DEG_TO_RAD;
	const float a[3] = {0.0f, sinf(r), cosf(r)};

	motion_sim_init(&s);
	for (int i = 0; i < 10; i++) {
		motion_step(&s, g0, a, true);
	}
	zassert_within(s.f.tilt_deg, exp_tilt, 0.1f, "deg %f -> %f", (double)deg,
		       (double)s.f.tilt_deg);
	zassert_equal(s.f.face_up, exp_up, "deg %f", (double)deg);
}

ZTEST(imu_motion, test_tilt_and_face_up)
{
	tilt_case(0.0f, 0.0f, true);
	tilt_case(45.0f, 45.0f, true);
	tilt_case(90.0f, 90.0f, false); /* a_z == 0 is not face up */
	tilt_case(135.0f, 135.0f, false);
	tilt_case(180.0f, 180.0f, false);

	/* Tilt around x_B as well (pure x component). */
	static struct motion_sim s;
	const float g0[3] = {0};
	const float a[3] = {-0.7071068f, 0.0f, 0.7071068f};

	motion_sim_init(&s);
	motion_step(&s, g0, a, true);
	zassert_within(s.f.tilt_deg, 45.0f, 0.1f);
	zassert_true(s.f.face_up);

	/* Free fall (|a| == 0): defined output, no NaN. */
	const float zero[3] = {0};

	motion_sim_init(&s);
	motion_step(&s, g0, zero, true);
	zassert_false(isnan(s.f.tilt_deg));
	zassert_false(s.f.face_up);
}

ZTEST(imu_motion, test_gap_and_bad_timestamps)
{
	static struct motion_sim s;

	motion_sim_init(&s);
	motion_quiet(&s, ODR_HZ);
	zassert_true(s.f.still);

	/* Duplicate timestamp: still evaluated, state not advanced, no crash. */
	s.t -= DT_US;
	motion_quiet(&s, 1);
	zassert_true(s.f.valid);
	zassert_true(s.f.still);

	/* Gap > 200 ms resets filters: stillness restarts. */
	s.t += 250000;
	motion_quiet(&s, 1);
	zassert_false(s.f.still);
	zassert_equal(s.f.still_ms, 0u);
	motion_quiet(&s, 35);
	zassert_true(s.f.still);

	/* Non-finite input is rejected without corrupting the state. */
	struct wp_imu_sample bad = {.t_us = s.t, .accel = {NAN, 0, G}, .gyro_valid = true};

	wp_motion_update(&s.m, &bad, &s.f);
	zassert_false(s.f.valid);
	motion_quiet(&s, 1);
	zassert_true(s.f.valid);
	zassert_true(s.f.still);
	zassert_false(isnan(s.f.tilt_deg));
}

ZTEST(imu_motion, test_jitter_rms_value)
{
	static struct motion_sim s;
	const float g0[3] = {0};

	/* Square-wave dynamic accel of +-0.02 g around 1 g: RMS ~0.02 g. */
	motion_sim_init(&s);
	for (int i = 0; i < 3 * ODR_HZ; i++) {
		const float a[3] = {(i & 1) ? 0.02f : -0.02f, 0.0f, 1.0f};

		motion_step(&s, g0, a, true);
	}
	zassert_within(s.f.accel_jitter_g, 0.02f, 0.003f, "%f", (double)s.f.accel_jitter_g);
	zassert_false(s.f.still);
}

ZTEST_SUITE(imu_motion, NULL, NULL, NULL, NULL, NULL);

/* ------------------------------------------------------------------------ */
/* Plausibility watchdog (HW-8: silent reset -> zeros forever)              */
/* ------------------------------------------------------------------------ */

static struct wp_imu_plaus plaus;
static int64_t plaus_t;

/* Feed @p n samples at ODR_HZ; returns the OR of all results and the index
 * of the first sample that reported a dead sensor in @p first (-1 if none). */
static uint32_t plaus_run(int n, const float acc[3], const float *gyro, int *first)
{
	uint32_t all = 0U;

	if (first != NULL) {
		*first = -1;
	}
	for (int i = 0; i < n; i++) {
		uint32_t r;

		plaus_t += DT_US;
		r = wp_imu_plaus_update(&plaus, plaus_t, acc, gyro);
		if (r != 0U && first != NULL && *first < 0) {
			*first = i;
		}
		all |= r;
	}
	return all;
}

static void plaus_before(void *f)
{
	ARG_UNUSED(f);
	wp_imu_plaus_init(&plaus, NULL);
	plaus_t = 0;
}

ZTEST(imu_plaus, test_defaults)
{
	struct wp_imu_plaus_config c;

	memset(&c, 0xa5, sizeof(c));
	wp_imu_plaus_config_default(&c);
	zassert_within(c.min_accel_g, 0.2f, 1e-6f);
	zassert_equal(c.dead_ms, 500U);
}

ZTEST(imu_plaus, test_accel_zeros_flagged_after_dead_ms)
{
	static const float zero[3] = {0.0f, 0.0f, 0.0f};
	static const float rest[3] = {0.0f, 0.0f, G};
	static const float tilted[3] = {0.5f * G, -0.6f * G, 0.62f * G};
	int first;

	/* Live sensor at rest / tilted: never flagged. */
	zassert_equal(plaus_run(5 * ODR_HZ, rest, NULL, NULL), 0U);
	zassert_equal(plaus_run(5 * ODR_HZ, tilted, NULL, NULL), 0U);

	/* Silent reset: zeros. Flagged only after MORE than 500 ms. */
	zassert_equal(plaus_run(ODR_HZ, zero, NULL, &first), WP_IMU_PLAUS_ACCEL_DEAD);
	zassert_true(first >= 52 && first <= 53, "flagged at sample %d", first);
	/* Keeps reporting while the readings stay implausible. */
	zassert_equal(plaus_run(1, zero, NULL, NULL), WP_IMU_PLAUS_ACCEL_DEAD);

	/* A plausible sample clears it. */
	zassert_equal(plaus_run(1, rest, NULL, NULL), 0U);
	zassert_equal(plaus_run(ODR_HZ / 4, zero, NULL, NULL), 0U);
}

ZTEST(imu_plaus, test_free_fall_from_table_not_flagged)
{
	/* 0.4 s of free fall (|a| ~ 0.02 g), impact, rest. */
	static const float fall[3] = {0.01f * G, 0.0f, 0.02f * G};
	static const float impact[3] = {3.0f * G, 1.0f, 5.0f * G};
	static const float rest[3] = {0.0f, 0.0f, G};

	zassert_equal(plaus_run(ODR_HZ * 4 / 10, fall, NULL, NULL), 0U);
	zassert_equal(plaus_run(2, impact, NULL, NULL), 0U);
	zassert_equal(plaus_run(ODR_HZ, rest, NULL, NULL), 0U);
	/* 0.19 g sustained IS implausible; 0.21 g is not. */
	static const float low[3] = {0.0f, 0.0f, 0.19f * G};
	static const float ok[3] = {0.0f, 0.0f, 0.21f * G};

	zassert_equal(plaus_run(ODR_HZ, ok, NULL, NULL), 0U);
	zassert_equal(plaus_run(ODR_HZ, low, NULL, NULL), WP_IMU_PLAUS_ACCEL_DEAD);
}

ZTEST(imu_plaus, test_gyro_exact_zero_flagged_only_when_enabled)
{
	static const float rest[3] = {0.0f, 0.0f, G};
	static const float g_zero[3] = {0.0f, 0.0f, 0.0f};
	static const float g_noise[3] = {0.0003f, 0.0f, -0.0002f}; /* a few LSB */
	int first;

	/* Live gyro: tiny noise is never flagged, even for seconds. */
	zassert_equal(plaus_run(5 * ODR_HZ, rest, g_noise, NULL), 0U);
	/* Gyro off / settling (NULL): not checked. */
	zassert_equal(plaus_run(5 * ODR_HZ, rest, NULL, NULL), 0U);

	/* Enabled and settled, exactly 0,0,0: flagged after > 500 ms. */
	zassert_equal(plaus_run(ODR_HZ, rest, g_zero, &first), WP_IMU_PLAUS_GYRO_DEAD);
	zassert_true(first >= 52 && first <= 53, "flagged at sample %d", first);

	/* Gyro powered down in between restarts the timer. */
	wp_imu_plaus_reset(&plaus);
	zassert_equal(plaus_run(ODR_HZ / 3, rest, g_zero, NULL), 0U);
	zassert_equal(plaus_run(1, rest, NULL, NULL), 0U);
	zassert_equal(plaus_run(ODR_HZ / 3, rest, g_zero, NULL), 0U);

	/* Both dead at once (whole sensor in power-down). */
	wp_imu_plaus_reset(&plaus);
	zassert_equal(plaus_run(ODR_HZ, g_zero, g_zero, NULL),
		      WP_IMU_PLAUS_ACCEL_DEAD | WP_IMU_PLAUS_GYRO_DEAD);
}

ZTEST(imu_plaus, test_reset_and_disabled)
{
	static const float zero[3] = {0.0f, 0.0f, 0.0f};
	struct wp_imu_plaus_config c;

	/* Reset (after a re-init) restarts the full dead_ms window. */
	zassert_equal(plaus_run(ODR_HZ / 2, zero, NULL, NULL), 0U);
	wp_imu_plaus_reset(&plaus);
	zassert_equal(plaus_run(ODR_HZ / 2, zero, NULL, NULL), 0U);

	/* dead_ms = 0 disables the watchdog. */
	wp_imu_plaus_config_default(&c);
	c.dead_ms = 0U;
	wp_imu_plaus_init(&plaus, &c);
	zassert_equal(plaus_run(5 * ODR_HZ, zero, zero, NULL), 0U);

	/* NULL safety. */
	zassert_equal(wp_imu_plaus_update(NULL, 0, zero, NULL), 0U);
	zassert_equal(wp_imu_plaus_update(&plaus, 0, NULL, NULL), 0U);
}

ZTEST_SUITE(imu_plaus, NULL, NULL, plaus_before, NULL, NULL);
