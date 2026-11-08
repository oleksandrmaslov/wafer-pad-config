/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Algorithm A: gravity-based orientation estimator (pure C, host-testable).
 *
 * Pipeline per IMU sample (body frame B, see <wafer_pad/types.h>):
 *
 *  1. Low-pass the specific force: g_lp += alpha * (a - g_lp),
 *     alpha = dt / (tau + dt). The filter is (re)seeded with the raw sample
 *     after init/reset, after a time gap > WP_GRAV_MAX_GAP_US or when time
 *     runs backwards.
 *
 *  2. Validity (all must hold, otherwise the candidate timer restarts):
 *       mag_err = max(| |a_raw|/g - 1 |, | |g_lp|/g - 1 |) <= max_mag_dev_g
 *         (the RAW magnitude catches lifting/shaking immediately, the filtered
 *          one keeps the sample invalid until the filter has settled again)
 *       min_tilt_deg <= tilt <= max_tilt_deg, tilt = acos(g_lp_z / |g_lp|)
 *         (flat -> no information; near vertical / face down -> rejected)
 *       |gyro| <= max_gyro_dps                    (only if gyro_valid)
 *
 *  3. heading = atan2(-g_lp_x, g_lp_y) in degrees, wrapped to [0, 360).
 *
 *  4. Candidate with hysteresis relative to `reference` (= the orientation
 *     currently in use, set by init/reset):
 *       - nearest quadrant == reference              -> candidate = reference
 *       - nearest quadrant != reference and the heading is within
 *         (45 - hysteresis) deg of its centre        -> candidate = nearest
 *       - otherwise, if the heading is within (45 + hysteresis) deg of the
 *         reference centre                           -> candidate = reference
 *       - otherwise (band between two non-reference quadrants)
 *                                                    -> no candidate
 *
 *  5. Confidence = f_tilt * f_mag * f_angle * f_stab, each factor in [0, 1]:
 *       f_tilt  = 0.5 + 0.5 * clamp((tilt - min_tilt) / 10 deg, 0, 1)
 *       f_mag   = 1 - 0.5 * clamp(mag_err / max_mag_dev_g, 0, 1)
 *       f_angle = clamp(1 - 0.5 * d / 45 deg, 0, 1)
 *                 d = angular distance heading <-> candidate quadrant centre
 *       f_stab  = 1 - 0.5 * clamp(dh / 30 deg, 0, 1)
 *                 dh = heading change since the candidate (timer) started
 *     A perfect sample gives 1.0; tilt just at min_tilt, a magnitude error at
 *     the limit, a heading 45 deg off-centre or a 30 deg heading drift each
 *     halve it, so any single bad factor alone drops it below the default
 *     min_confidence (0.6).
 *
 *  6. confirmed = same candidate on every sample for >= confirm_ms
 *     (confirm_ms_no_gyro when the sample carries no gyro data, i.e. no rate
 *     check was possible) with confidence >= min_confidence. An invalid
 *     sample, a candidate change, a missing candidate, a sample below
 *     min_confidence, or a tilt that moved more than max_tilt_change_deg
 *     since the timer started restarts the timer. The tilt check rejects the
 *     steadily changing tilt of a pick-up / put-down (a sweep through
 *     25..60 deg at 20..50 dps), which would otherwise latch an orientation
 *     that then persists while the pad lies flat.
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <wafer_pad/orientation.h>
#include <wafer_pad/orientation_transform.h>

/** last_t_us value meaning "no sample seen yet". */
#define WP_GRAV_NO_TIME INT64_MIN

/** Gap between samples after which the filter is re-seeded. */
#define WP_GRAV_MAX_GAP_US (500LL * WP_US_PER_MS)

#define WP_GRAV_TILT_RAMP_DEG   10.0f
#define WP_GRAV_STAB_RANGE_DEG  30.0f
#define WP_GRAV_MIN_NORM        1e-3f

void wp_orient_gravity_config_default(struct wp_orient_gravity_config *cfg)
{
	cfg->min_tilt_deg = 20.0f;
	cfg->max_tilt_deg = 75.0f;
	cfg->max_mag_dev_g = 0.12f;
	cfg->hysteresis_deg = 15.0f;
	cfg->max_gyro_dps = 20.0f;
	cfg->confirm_ms = 300U;
	cfg->confirm_ms_no_gyro = 800U;
	cfg->max_tilt_change_deg = 6.0f;
	cfg->min_confidence = 0.6f;
	cfg->lpf_tau_ms = 50.0f;
}

static float clampf(float v, float lo, float hi)
{
	if (v < lo) {
		return lo;
	}
	if (v > hi) {
		return hi;
	}
	return v;
}

static float wrap360(float deg)
{
	float r = fmodf(deg, 360.0f);

	if (r < 0.0f) {
		r += 360.0f;
	}
	if (r >= 360.0f) {
		r = 0.0f;
	}
	return r;
}

/** Absolute angular distance in degrees, 0..180. */
static float ang_dist(float a, float b)
{
	float d = wrap360(a - b);

	return (d > 180.0f) ? (360.0f - d) : d;
}

static float norm3(const float v[3])
{
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void wp_orient_gravity_init(struct wp_orient_gravity *g, const struct wp_orient_gravity_config *cfg,
			    enum wp_orientation reference)
{
	memset(g, 0, sizeof(*g));
	if (cfg != NULL) {
		g->cfg = *cfg;
	} else {
		wp_orient_gravity_config_default(&g->cfg);
	}
	g->last_t_us = WP_GRAV_NO_TIME;
	wp_orient_gravity_reset(g, reference);
}

void wp_orient_gravity_reset(struct wp_orient_gravity *g, enum wp_orientation reference)
{
	/* last_t_us is kept on purpose: it is the time base of the sample
	 * stream (the manager's gyro integration uses it), not filter state.
	 */
	g->g_lp[0] = 0.0f;
	g->g_lp[1] = 0.0f;
	g->g_lp[2] = 0.0f;
	g->g_init = false;
	g->reference = wp_orientation_norm((int)reference);
	g->have_candidate = false;
	g->candidate = g->reference;
	g->candidate_since_us = 0;
	g->heading_ref_deg = 0.0f;
	g->tilt_ref_deg = 0.0f;
}

static bool sample_finite(const struct wp_imu_sample *s)
{
	for (int i = 0; i < 3; i++) {
		if (!isfinite(s->accel[i])) {
			return false;
		}
		if (s->gyro_valid && !isfinite(s->gyro[i])) {
			return false;
		}
	}
	return true;
}

static void lowpass(struct wp_orient_gravity *g, const struct wp_imu_sample *s)
{
	int64_t dt_us = s->t_us - g->last_t_us;
	bool gap = (g->last_t_us == WP_GRAV_NO_TIME) || (dt_us < 0) ||
		   (dt_us > WP_GRAV_MAX_GAP_US);

	if (!g->g_init || gap) {
		for (int i = 0; i < 3; i++) {
			g->g_lp[i] = s->accel[i];
		}
		g->g_init = true;
		if (gap) {
			/* Unknown history: do not let a candidate span the gap. */
			g->have_candidate = false;
		}
		return;
	}

	float dt_ms = (float)dt_us / (float)WP_US_PER_MS;
	float tau = g->cfg.lpf_tau_ms;
	float alpha = (tau > 0.0f && (tau + dt_ms) > 0.0f) ? (dt_ms / (tau + dt_ms)) : 1.0f;

	for (int i = 0; i < 3; i++) {
		g->g_lp[i] += alpha * (s->accel[i] - g->g_lp[i]);
	}
}

void wp_orient_gravity_update(struct wp_orient_gravity *g, const struct wp_imu_sample *s,
			      struct wp_orient_gravity_result *out)
{
	const struct wp_orient_gravity_config *c = &g->cfg;
	struct wp_orient_gravity_result r = {
		.valid = false,
		.tilt_deg = 0.0f,
		.heading_deg = 0.0f,
		.candidate = g->reference,
		.confidence = 0.0f,
		.confirmed = false,
	};

	if (!sample_finite(s)) {
		g->have_candidate = false;
		g->last_t_us = s->t_us;
		*out = r;
		return;
	}

	lowpass(g, s);
	g->last_t_us = s->t_us;

	/* ---- tilt / heading from the filtered vector ---- */
	float raw_mag = norm3(s->accel);
	float lp_mag = norm3(g->g_lp);
	float tilt = 180.0f;

	if (lp_mag > WP_GRAV_MIN_NORM) {
		tilt = acosf(clampf(g->g_lp[2] / lp_mag, -1.0f, 1.0f)) * WP_RAD_TO_DEG;
	}
	float heading = wrap360(atan2f(-g->g_lp[0], g->g_lp[1]) * WP_RAD_TO_DEG);

	r.tilt_deg = tilt;
	r.heading_deg = heading;

	/* ---- validity ---- */
	float err_raw = fabsf(raw_mag / WP_GRAVITY_MS2 - 1.0f);
	float err_lp = fabsf(lp_mag / WP_GRAVITY_MS2 - 1.0f);
	float mag_err = (err_raw > err_lp) ? err_raw : err_lp;
	bool valid = (mag_err <= c->max_mag_dev_g) && (tilt >= c->min_tilt_deg) &&
		     (tilt <= c->max_tilt_deg);

	if (valid && s->gyro_valid) {
		float rate_dps = norm3(s->gyro) * WP_RAD_TO_DEG;

		valid = (rate_dps <= c->max_gyro_dps);
	}
	r.valid = valid;
	if (!valid) {
		g->have_candidate = false;
		*out = r;
		return;
	}

	/* ---- candidate with hysteresis ---- */
	float hyst = clampf(c->hysteresis_deg, 0.0f, 45.0f);
	enum wp_orientation nearest = wp_orientation_from_deg(heading);
	float d_near = ang_dist(heading, (float)wp_orientation_deg(nearest));
	float d_ref = ang_dist(heading, (float)wp_orientation_deg(g->reference));
	enum wp_orientation cand;
	float d;

	if (nearest == g->reference) {
		cand = g->reference;
		d = d_near;
	} else if (d_near <= 45.0f - hyst) {
		cand = nearest;
		d = d_near;
	} else if (d_ref <= 45.0f + hyst) {
		cand = g->reference;
		d = d_ref;
	} else {
		/* Ambiguous band between two non-reference quadrants. */
		g->have_candidate = false;
		*out = r;
		return;
	}

	/* ---- confidence ---- */
	bool continuing = g->have_candidate && (g->candidate == cand);

	if (continuing && c->max_tilt_change_deg > 0.0f &&
	    fabsf(tilt - g->tilt_ref_deg) > c->max_tilt_change_deg) {
		continuing = false; /* tilt still changing: not a stable posture */
	}
	float f_tilt = 0.5f + 0.5f * clampf((tilt - c->min_tilt_deg) / WP_GRAV_TILT_RAMP_DEG,
					    0.0f, 1.0f);
	float f_mag = (c->max_mag_dev_g > 0.0f)
			      ? (1.0f - 0.5f * clampf(mag_err / c->max_mag_dev_g, 0.0f, 1.0f))
			      : 1.0f;
	float f_angle = clampf(1.0f - 0.5f * d / 45.0f, 0.0f, 1.0f);
	float f_stab = 1.0f;

	if (continuing) {
		float dh = ang_dist(heading, g->heading_ref_deg);

		f_stab = 1.0f - 0.5f * clampf(dh / WP_GRAV_STAB_RANGE_DEG, 0.0f, 1.0f);
	}

	float conf = clampf(f_tilt * f_mag * f_angle * f_stab, 0.0f, 1.0f);

	if (!continuing || conf < c->min_confidence) {
		/* New candidate, or this sample does not count: restart timer. */
		g->have_candidate = true;
		g->candidate = cand;
		g->candidate_since_us = s->t_us;
		g->heading_ref_deg = heading;
		g->tilt_ref_deg = tilt;
	}

	uint32_t confirm_ms = c->confirm_ms;

	if (!s->gyro_valid && c->confirm_ms_no_gyro > confirm_ms) {
		confirm_ms = c->confirm_ms_no_gyro;
	}
	r.candidate = cand;
	r.confidence = conf;
	r.confirmed = (conf >= c->min_confidence) &&
		      ((s->t_us - g->candidate_since_us) >= (int64_t)confirm_ms * WP_US_PER_MS);
	*out = r;
}
