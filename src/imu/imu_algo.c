/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMU signal processing (pure C, no Zephyr includes): gyroscope bias
 * estimation while stationary, the motion features used by the state
 * machine and the sensor plausibility watchdog (silent reset detection).
 * See <wafer_pad/imu_algo.h> for the contract.
 *
 * All state lives in the caller-owned structs; time is passed in (t_us).
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <wafer_pad/imu_algo.h>

/* A gap between two samples larger than this resets all filters. */
#define WP_IMU_GAP_RESET_US (200LL * WP_US_PER_MS)

/* Time constant of the accel low-pass used for the bias stillness test. */
#define WP_BIAS_ACC_LP_TAU_MS 250.0f

/* Minimum number of samples in the first stationary window. */
#define WP_BIAS_MIN_SAMPLES 8u

/* Rest detection: gyro low-pass (AC part = noise) and noise / jitter EMAs. */
#define WP_BIAS_GYRO_LP_TAU_MS 500.0f
#define WP_BIAS_NOISE_TAU_MS   250.0f

#define WP_RAD_S_PER_DPS WP_DEG_TO_RAD

static float vec3_norm(const float v[3])
{
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static float vec3_dist(const float a[3], const float b[3])
{
	const float d[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};

	return vec3_norm(d);
}

static bool vec3_finite(const float v[3])
{
	return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

/* First-order low-pass factor for a step of dt_ms with time constant tau_ms. */
static float lp_alpha(float dt_ms, float tau_ms)
{
	if (dt_ms <= 0.0f) {
		return 0.0f;
	}
	if (tau_ms <= 0.0f) {
		return 1.0f;
	}
	return dt_ms / (tau_ms + dt_ms);
}

/* ------------------------------------------------------------------------ */
/* Gyroscope bias estimator                                                  */
/* ------------------------------------------------------------------------ */

void wp_gyro_bias_config_default(struct wp_gyro_bias_config *cfg)
{
	if (cfg == NULL) {
		return;
	}
	cfg->still_gyro_dps = 1.0f;
	cfg->still_accel_dev_g = 0.03f;
	cfg->still_min_ms = 800u;
	cfg->alpha = 0.02f;
	cfg->max_bias_dps = 10.0f;
	cfg->max_noise_dps = 0.15f;
	cfg->relearn_ms = 5000u;
	cfg->relearn_accel_rms_g = 0.002f;
}

static void bias_reset_window(struct wp_gyro_bias *b)
{
	b->sum[0] = 0.0f;
	b->sum[1] = 0.0f;
	b->sum[2] = 0.0f;
	b->n = 0u;
}

void wp_gyro_bias_init(struct wp_gyro_bias *b, const struct wp_gyro_bias_config *cfg)
{
	if (b == NULL) {
		return;
	}
	memset(b, 0, sizeof(*b));
	if (cfg != NULL) {
		b->cfg = *cfg;
	} else {
		wp_gyro_bias_config_default(&b->cfg);
	}
	if (!(b->cfg.alpha >= 0.0f)) {
		b->cfg.alpha = 0.0f;
	} else if (b->cfg.alpha > 1.0f) {
		b->cfg.alpha = 1.0f;
	}
}

void wp_gyro_bias_restart(struct wp_gyro_bias *b)
{
	if (b == NULL) {
		return;
	}
	b->still = false;
	b->still_since_us = 0;
	b->acc_lp_init = false;
	b->last_t_us = 0;
	b->g_lp_init = false;
	b->noise_dps2 = 0.0f;
	b->jitter_g2 = 0.0f;
	b->quiet = false;
	b->quiet_since_us = 0;
	bias_reset_window(b);
}

void wp_gyro_bias_set(struct wp_gyro_bias *b, const float bias[3])
{
	if (b == NULL || bias == NULL || !vec3_finite(bias)) {
		return;
	}
	/* A persisted value above the plausibility limit is ignored. */
	if (vec3_norm(bias) > b->cfg.max_bias_dps * WP_RAD_S_PER_DPS) {
		return;
	}
	b->bias[0] = bias[0];
	b->bias[1] = bias[1];
	b->bias[2] = bias[2];
	b->valid = true;
}

bool wp_gyro_bias_valid(const struct wp_gyro_bias *b)
{
	return (b != NULL) && b->valid;
}

void wp_gyro_bias_apply(const struct wp_gyro_bias *b, const float raw[3], float out[3])
{
	const bool use = (b != NULL) && b->valid;

	for (int i = 0; i < 3; i++) {
		out[i] = use ? (raw[i] - b->bias[i]) : raw[i];
	}
}

bool wp_gyro_bias_update(struct wp_gyro_bias *b, int64_t t_us, const float raw_gyro[3],
			 const float accel[3])
{
	const struct wp_gyro_bias_config *c;
	float ref[3];
	float dt_ms;
	float dev_g;
	float alpha;
	bool gyro_ok;
	bool still_now;

	if (b == NULL || raw_gyro == NULL || accel == NULL) {
		return false;
	}
	c = &b->cfg;

	if (!vec3_finite(raw_gyro) || !vec3_finite(accel)) {
		b->still = false;
		bias_reset_window(b);
		return false;
	}

	if (b->acc_lp_init) {
		int64_t dt_us = t_us - b->last_t_us;

		if (dt_us <= 0) {
			/* Duplicate or out-of-order timestamp: ignore the sample. */
			return b->still;
		}
		if (dt_us > WP_IMU_GAP_RESET_US) {
			wp_gyro_bias_restart(b);
		}
	}

	if (!b->acc_lp_init) {
		memcpy(b->acc_lp, accel, sizeof(b->acc_lp));
		b->acc_lp_init = true;
		b->last_t_us = t_us;
		dt_ms = 0.0f;
	} else {
		dt_ms = (float)(t_us - b->last_t_us) / (float)WP_US_PER_MS;
		b->last_t_us = t_us;
	}

	/* Dynamic acceleration vs. the (pre-update) low-pass, in g. */
	dev_g = vec3_dist(accel, b->acc_lp) / WP_GRAVITY_MS2;
	alpha = lp_alpha(dt_ms, WP_BIAS_ACC_LP_TAU_MS);
	for (int i = 0; i < 3; i++) {
		b->acc_lp[i] += alpha * (accel[i] - b->acc_lp[i]);
	}

	/* Gyro AC noise: a hand always shows tremor, a table only sensor noise. */
	if (!b->g_lp_init) {
		memcpy(b->g_lp, raw_gyro, sizeof(b->g_lp));
		b->g_lp_init = true;
		b->noise_dps2 = 0.0f;
		b->jitter_g2 = 0.0f;
	}
	const float ac_dps = vec3_dist(raw_gyro, b->g_lp) * WP_RAD_TO_DEG;
	const float a_lp = lp_alpha(dt_ms, WP_BIAS_GYRO_LP_TAU_MS);
	const float a_n = lp_alpha(dt_ms, WP_BIAS_NOISE_TAU_MS);

	for (int i = 0; i < 3; i++) {
		b->g_lp[i] += a_lp * (raw_gyro[i] - b->g_lp[i]);
	}
	b->noise_dps2 += a_n * (ac_dps * ac_dps - b->noise_dps2);
	b->jitter_g2 += a_n * (dev_g * dev_g - b->jitter_g2);

	const bool noise_ok = sqrtf(fmaxf(b->noise_dps2, 0.0f)) < c->max_noise_dps;
	const bool accel_ok = dev_g < c->still_accel_dev_g;

	/* Re-learn a bias that is clearly wrong (stale persisted value): the
	 * device has been quiet for relearn_ms, yet the rate is not near it.
	 */
	if (b->valid && noise_ok && accel_ok &&
	    sqrtf(fmaxf(b->jitter_g2, 0.0f)) < c->relearn_accel_rms_g) {
		if (!b->quiet) {
			b->quiet = true;
			b->quiet_since_us = t_us;
		} else if ((t_us - b->quiet_since_us) >= (int64_t)c->relearn_ms * WP_US_PER_MS &&
			   vec3_dist(b->g_lp, b->bias) * WP_RAD_TO_DEG >= c->still_gyro_dps &&
			   vec3_norm(b->g_lp) <= c->max_bias_dps * WP_RAD_S_PER_DPS) {
			memcpy(b->bias, b->g_lp, sizeof(b->bias));
			b->quiet_since_us = t_us;
			b->still = false; /* next window starts from the new bias */
			bias_reset_window(b);
			return false;
		}
	} else {
		b->quiet = false;
	}

	/*
	 * Gyro stillness reference: the current bias once valid; before that
	 * the running mean of the current window (or the sample itself), so a
	 * zero-rate offset up to max_bias_dps does not prevent the first
	 * estimate. A hard limit rejects anything faster than bias + noise.
	 */
	if (b->valid) {
		memcpy(ref, b->bias, sizeof(ref));
	} else if (b->n > 0u) {
		for (int i = 0; i < 3; i++) {
			ref[i] = b->sum[i] / (float)b->n;
		}
	} else {
		memcpy(ref, raw_gyro, sizeof(ref));
	}
	gyro_ok = (vec3_dist(raw_gyro, ref) * WP_RAD_TO_DEG < c->still_gyro_dps) &&
		  (vec3_norm(raw_gyro) * WP_RAD_TO_DEG < c->max_bias_dps + c->still_gyro_dps);
	still_now = gyro_ok && accel_ok && noise_ok;

	if (!still_now) {
		b->still = false;
		bias_reset_window(b);
		return false;
	}

	if (!b->still) {
		b->still = true;
		b->still_since_us = t_us;
		bias_reset_window(b);
	}

	const bool window_done =
		(t_us - b->still_since_us) >= (int64_t)c->still_min_ms * WP_US_PER_MS;

	if (!b->valid) {
		for (int i = 0; i < 3; i++) {
			b->sum[i] += raw_gyro[i];
		}
		b->n++;
		if (window_done && b->n >= WP_BIAS_MIN_SAMPLES) {
			float mean[3];

			for (int i = 0; i < 3; i++) {
				mean[i] = b->sum[i] / (float)b->n;
			}
			if (vec3_norm(mean) <= c->max_bias_dps * WP_RAD_S_PER_DPS) {
				memcpy(b->bias, mean, sizeof(b->bias));
				b->valid = true;
			}
			/* Accepted or rejected: start a fresh window. */
			bias_reset_window(b);
			b->still_since_us = t_us;
		}
	} else if (window_done) {
		float cand[3];

		for (int i = 0; i < 3; i++) {
			cand[i] = b->bias[i] + c->alpha * (raw_gyro[i] - b->bias[i]);
		}
		if (vec3_norm(cand) <= c->max_bias_dps * WP_RAD_S_PER_DPS) {
			memcpy(b->bias, cand, sizeof(b->bias));
		}
	}

	return true;
}

/* ------------------------------------------------------------------------ */
/* Motion features                                                           */
/* ------------------------------------------------------------------------ */

void wp_motion_config_default(struct wp_motion_config *cfg)
{
	if (cfg == NULL) {
		return;
	}
	cfg->still_gyro_dps = 1.5f;
	cfg->still_accel_dev_g = 0.015f;
	cfg->still_window_ms = 300u;
	cfg->move_gyro_dps = 20.0f;
	cfg->move_accel_dev_g = 0.06f;
	cfg->tremor_tau_ms = 250.0f;
	cfg->tremor_hp_tau_ms = 500.0f;
	cfg->accel_lp_tau_ms = 400.0f;
	cfg->handheld_tremor_dps = 0.35f;
	cfg->handheld_accel_jitter_g = 0.004f;
}

void wp_motion_reset(struct wp_motion *m)
{
	if (m == NULL) {
		return;
	}
	m->acc_lp[0] = 0.0f;
	m->acc_lp[1] = 0.0f;
	m->acc_lp[2] = 0.0f;
	m->init = false;
	m->last_t_us = 0;
	m->still_candidate_since_us = 0;
	m->still_candidate = false;
	m->tremor_ms2 = 0.0f;
	m->jitter_ms2 = 0.0f;
	m->gyro_lp[0] = 0.0f;
	m->gyro_lp[1] = 0.0f;
	m->gyro_lp[2] = 0.0f;
	m->gyro_lp_init = false;
}

void wp_motion_init(struct wp_motion *m, const struct wp_motion_config *cfg)
{
	if (m == NULL) {
		return;
	}
	memset(m, 0, sizeof(*m));
	if (cfg != NULL) {
		m->cfg = *cfg;
	} else {
		wp_motion_config_default(&m->cfg);
	}
	wp_motion_reset(m);
}

void wp_motion_update(struct wp_motion *m, const struct wp_imu_sample *s,
		      struct wp_motion_features *out)
{
	const struct wp_motion_config *c;
	float dt_ms = 0.0f;
	float gyro_mag_dps = 0.0f;
	float gyro_ac_dps = 0.0f;
	float dev_g;
	bool advance = true;

	if (m == NULL || s == NULL || out == NULL) {
		return;
	}
	c = &m->cfg;
	memset(out, 0, sizeof(*out));

	if (!vec3_finite(s->accel) || (s->gyro_valid && !vec3_finite(s->gyro))) {
		/* Corrupt sample: report "not still, not moving", keep state. */
		out->valid = false;
		return;
	}

	if (m->init) {
		int64_t dt_us = s->t_us - m->last_t_us;

		if (dt_us > WP_IMU_GAP_RESET_US) {
			wp_motion_reset(m);
		} else if (dt_us <= 0) {
			/* Duplicate / out-of-order: evaluate but do not advance. */
			advance = false;
		} else {
			dt_ms = (float)dt_us / (float)WP_US_PER_MS;
		}
	}

	if (!m->init) {
		memcpy(m->acc_lp, s->accel, sizeof(m->acc_lp));
		m->init = true;
		m->last_t_us = s->t_us;
		m->tremor_ms2 = 0.0f;
		m->jitter_ms2 = 0.0f;
		m->still_candidate = false;
	} else if (advance) {
		m->last_t_us = s->t_us;
	}

	if (s->gyro_valid) {
		float w_dps[3];

		for (int i = 0; i < 3; i++) {
			w_dps[i] = s->gyro[i] * WP_RAD_TO_DEG;
		}
		gyro_mag_dps = vec3_norm(w_dps);
		if (!m->gyro_lp_init) {
			memcpy(m->gyro_lp, w_dps, sizeof(m->gyro_lp));
			m->gyro_lp_init = true;
		}
		/* Tremor = AC part: a constant offset (residual bias) or a slow
		 * steady rotation is not a hand. */
		gyro_ac_dps = vec3_dist(w_dps, m->gyro_lp);
		if (advance && dt_ms > 0.0f) {
			const float a_hp = lp_alpha(dt_ms, c->tremor_hp_tau_ms);

			for (int i = 0; i < 3; i++) {
				m->gyro_lp[i] += a_hp * (w_dps[i] - m->gyro_lp[i]);
			}
		}
	} else {
		m->gyro_lp_init = false;
	}
	dev_g = vec3_dist(s->accel, m->acc_lp) / WP_GRAVITY_MS2;

	if (advance && dt_ms > 0.0f) {
		const float a_lp = lp_alpha(dt_ms, c->accel_lp_tau_ms);
		const float a_tr = lp_alpha(dt_ms, c->tremor_tau_ms);

		for (int i = 0; i < 3; i++) {
			m->acc_lp[i] += a_lp * (s->accel[i] - m->acc_lp[i]);
		}
		m->jitter_ms2 += a_tr * (dev_g * dev_g - m->jitter_ms2);
		if (s->gyro_valid) {
			m->tremor_ms2 += a_tr * (gyro_ac_dps * gyro_ac_dps - m->tremor_ms2);
		}
	}
	if (!s->gyro_valid) {
		/* Stale while the gyro is off: restart from zero on power-up. */
		m->tremor_ms2 = 0.0f;
	}

	/* Stillness: gyro (when available) and dynamic acceleration quiet. */
	const bool quiet = (!s->gyro_valid || gyro_mag_dps < c->still_gyro_dps) &&
			   (dev_g < c->still_accel_dev_g);

	if (quiet) {
		if (!m->still_candidate) {
			m->still_candidate = true;
			m->still_candidate_since_us = s->t_us;
		}
	} else {
		m->still_candidate = false;
	}

	uint32_t still_ms = 0u;

	if (m->still_candidate) {
		int64_t d_ms = (s->t_us - m->still_candidate_since_us) / WP_US_PER_MS;

		if (d_ms < 0) {
			d_ms = 0;
		} else if (d_ms > (int64_t)UINT32_MAX) {
			d_ms = (int64_t)UINT32_MAX;
		}
		still_ms = (uint32_t)d_ms;
	}

	out->valid = true;
	out->still = m->still_candidate && (still_ms >= c->still_window_ms);
	out->still_ms = out->still ? still_ms : 0u;
	out->moving = (gyro_mag_dps > c->move_gyro_dps) || (dev_g > c->move_accel_dev_g);
	out->gyro_mag_dps = gyro_mag_dps;
	out->accel_dev_g = dev_g;
	out->tremor_dps = s->gyro_valid ? sqrtf(fmaxf(m->tremor_ms2, 0.0f)) : 0.0f;
	out->accel_jitter_g = sqrtf(fmaxf(m->jitter_ms2, 0.0f));
	out->hand_tremor = s->gyro_valid ? (out->tremor_dps > c->handheld_tremor_dps)
					 : (out->accel_jitter_g > c->handheld_accel_jitter_g);
	out->tremor_from_gyro = s->gyro_valid;

	/* Tilt / face-up from the gravity estimate (low-passed accel). */
	const float *g = m->acc_lp;
	const float horiz = sqrtf(g[0] * g[0] + g[1] * g[1]);

	out->tilt_deg = atan2f(horiz, g[2]) * WP_RAD_TO_DEG; /* 0..180, 0 if |g| == 0 */
	out->face_up = g[2] > 0.0f;
}

/* ------------------------------------------------------------------------ */
/* Sensor plausibility watchdog                                              */
/* ------------------------------------------------------------------------ */

void wp_imu_plaus_config_default(struct wp_imu_plaus_config *cfg)
{
	if (cfg == NULL) {
		return;
	}
	cfg->min_accel_g = 0.2f;
	cfg->dead_ms = 500U;
}

void wp_imu_plaus_init(struct wp_imu_plaus *p, const struct wp_imu_plaus_config *cfg)
{
	if (p == NULL) {
		return;
	}
	if (cfg != NULL) {
		p->cfg = *cfg;
	} else {
		wp_imu_plaus_config_default(&p->cfg);
	}
	wp_imu_plaus_reset(p);
}

void wp_imu_plaus_reset(struct wp_imu_plaus *p)
{
	if (p == NULL) {
		return;
	}
	p->accel_bad = false;
	p->accel_bad_since_us = 0;
	p->gyro_bad = false;
	p->gyro_bad_since_us = 0;
}

/* Track how long @p bad has been true; true once it exceeds @p ms. */
static bool plaus_held(bool bad, bool *state, int64_t *since_us, int64_t t_us, uint32_t ms)
{
	if (!bad) {
		*state = false;
		return false;
	}
	if (!*state) {
		*state = true;
		*since_us = t_us;
		return false;
	}
	return (t_us - *since_us) > (int64_t)ms * WP_US_PER_MS;
}

uint32_t wp_imu_plaus_update(struct wp_imu_plaus *p, int64_t t_us, const float accel_ms2[3],
			     const float *gyro_raw)
{
	uint32_t dead = 0U;
	bool bad;

	if (p == NULL || accel_ms2 == NULL || p->cfg.dead_ms == 0U) {
		return 0U;
	}

	/* Compare squared magnitudes; NaN compares false (left to wp_motion). */
	const float lim = p->cfg.min_accel_g * WP_GRAVITY_MS2;
	const float a2 = accel_ms2[0] * accel_ms2[0] + accel_ms2[1] * accel_ms2[1] +
			 accel_ms2[2] * accel_ms2[2];

	bad = a2 < lim * lim;
	if (plaus_held(bad, &p->accel_bad, &p->accel_bad_since_us, t_us, p->cfg.dead_ms)) {
		dead |= WP_IMU_PLAUS_ACCEL_DEAD;
	}

	bad = gyro_raw != NULL && gyro_raw[0] == 0.0f && gyro_raw[1] == 0.0f &&
	      gyro_raw[2] == 0.0f;
	if (plaus_held(bad, &p->gyro_bad, &p->gyro_bad_since_us, t_us, p->cfg.dead_ms)) {
		dead |= WP_IMU_PLAUS_GYRO_DEAD;
	}
	return dead;
}
