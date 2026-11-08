/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Air mouse: gyroscope angular rate -> relative cursor motion (pure C).
 * Specification: <wafer_pad/air_mouse.h>. Frames: <wafer_pad/types.h>.
 *
 * Implementation notes
 *   - am->up[] holds the low-passed specific force (m/s^2, body frame); it is
 *     normalized when used. It is updated on every sample, active or not.
 *   - am->last_t_us is the timestamp of the last accepted sample (any state).
 *     A gap > AM_MAX_GAP_US or a backwards timestamp is a discontinuity:
 *     the gravity estimate snaps to the current sample, the rate filter is
 *     re-seeded and that sample produces no output (dt unknown).
 *   - The rate filter (lp_yaw_dps / lp_pitch_dps) is seeded with the first
 *     projected rate after a reset; ramp-in after start provides the soft
 *     engagement. Per-sample output = rate(deg/s) * dt(s) * counts_per_deg.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <wafer_pad/air_mouse.h>
#include <wafer_pad/orientation_transform.h>

/** last_t_us value meaning "no sample seen yet". */
#define AM_T_NONE INT64_MIN

/** Sample gaps above this reset the filters (no output for that sample). */
#define AM_MAX_GAP_US 100000LL

/** Low-passed specific force below this is unusable as "up" (free fall). */
#define AM_MIN_UP_MS2 (0.5f * WP_GRAVITY_MS2)

/** |fwd x up| below this (~11.5 deg from vertical) -> uncompensated axes. */
#define AM_MIN_HORIZ 0.2f

void wp_air_mouse_config_default(struct wp_air_mouse_config *cfg)
{
	if (cfg == NULL) {
		return;
	}

	cfg->counts_per_deg = 12.0f;
	cfg->deadzone_dps = 1.5f;
	cfg->lpf_cutoff_hz = 15.0f;
	cfg->max_rate_dps = 400.0f;
	cfg->ramp_in_ms = 120U;
	cfg->gravity_compensated = true;
	cfg->accel_lp_tau_ms = 150.0f;
	cfg->invert_x = false;
	cfg->invert_y = false;
}

static void reset_rate_filter(struct wp_air_mouse *am)
{
	am->lp_yaw_dps = 0.0f;
	am->lp_pitch_dps = 0.0f;
	am->lp_init = false;
}

void wp_air_mouse_init(struct wp_air_mouse *am, const struct wp_air_mouse_config *cfg)
{
	if (am == NULL) {
		return;
	}

	memset(am, 0, sizeof(*am));
	if (cfg != NULL) {
		am->cfg = *cfg;
	} else {
		wp_air_mouse_config_default(&am->cfg);
	}
	am->orientation = WP_ORIENT_0;
	am->active = false;
	am->start_us = 0;
	am->last_t_us = AM_T_NONE;
	am->up_init = false;
	reset_rate_filter(am);
}

void wp_air_mouse_set_orientation(struct wp_air_mouse *am, enum wp_orientation o)
{
	if (am == NULL) {
		return;
	}

	am->orientation = wp_orientation_norm((int)o);
	reset_rate_filter(am);
}

void wp_air_mouse_start(struct wp_air_mouse *am, int64_t t_us)
{
	if (am == NULL) {
		return;
	}

	am->active = true;
	am->start_us = t_us;
	reset_rate_filter(am);
}

void wp_air_mouse_stop(struct wp_air_mouse *am)
{
	if (am == NULL) {
		return;
	}

	am->active = false;
	reset_rate_filter(am);
}

bool wp_air_mouse_active(const struct wp_air_mouse *am)
{
	return (am != NULL) && am->active;
}

/* ---- small vector helpers ----------------------------------------------- */

static float dot3(const float a[3], const float b[3])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static float norm3(const float v[3])
{
	return sqrtf(dot3(v, v));
}

static void cross3(const float a[3], const float b[3], float out[3])
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

static bool finite3(const float v[3])
{
	return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

/* ---- processing stages -------------------------------------------------- */

static void update_gravity(struct wp_air_mouse *am, const float accel[3], bool continuous,
			   float dt_s)
{
	if (!finite3(accel)) {
		return;
	}

	if (!am->up_init || !continuous) {
		for (int i = 0; i < 3; i++) {
			am->up[i] = accel[i];
		}
		am->up_init = true;
		return;
	}

	float tau_s = am->cfg.accel_lp_tau_ms / 1000.0f;
	float alpha = 1.0f;

	if (tau_s > 0.0f) {
		alpha = dt_s / (tau_s + dt_s);
	}
	for (int i = 0; i < 3; i++) {
		am->up[i] += alpha * (accel[i] - am->up[i]);
	}
}

/**
 * Body-frame unit axes for yaw (cursor X) and pitch (cursor Y).
 * See the axis derivation in <wafer_pad/air_mouse.h>.
 */
static void rate_axes(const struct wp_air_mouse *am, float yaw_axis[3], float pitch_axis[3])
{
	float fwd[3];
	float right[3];

	wp_orientation_axes_body(am->orientation, fwd, right);

	/* Uncompensated axes: pad normal and the user's "right" in the pad plane. */
	yaw_axis[0] = 0.0f;
	yaw_axis[1] = 0.0f;
	yaw_axis[2] = 1.0f;
	for (int i = 0; i < 3; i++) {
		pitch_axis[i] = right[i];
	}

	if (!am->cfg.gravity_compensated || !am->up_init) {
		return;
	}

	float up_n = norm3(am->up);

	if (!(up_n >= AM_MIN_UP_MS2)) {
		return;
	}

	float up[3] = {am->up[0] / up_n, am->up[1] / up_n, am->up[2] / up_n};
	float horiz[3];

	/* Gravity is usable: turning left/right is ALWAYS about world up. */
	for (int i = 0; i < 3; i++) {
		yaw_axis[i] = up[i];
	}

	cross3(fwd, up, horiz);

	float h_n = norm3(horiz);

	if (h_n >= AM_MIN_HORIZ) {
		for (int i = 0; i < 3; i++) {
			pitch_axis[i] = horiz[i] / h_n;
		}
		return;
	}

	/* Forward edge (nearly) vertical: fwd x up degenerates, but the pad's
	 * "right" is then (nearly) horizontal; use its horizontal part. */
	float r_up = right[0] * up[0] + right[1] * up[1] + right[2] * up[2];

	for (int i = 0; i < 3; i++) {
		horiz[i] = right[i] - r_up * up[i];
	}
	h_n = norm3(horiz);
	if (h_n >= AM_MIN_HORIZ) {
		for (int i = 0; i < 3; i++) {
			pitch_axis[i] = horiz[i] / h_n;
		}
	}
	/* else: keep the pad's right axis for pitch (no usable horizontal). */
}

static float lpf_alpha(float dt_s, float cutoff_hz)
{
	if (!(cutoff_hz > 0.0f) || !(dt_s > 0.0f)) {
		return 1.0f;
	}

	float rc = 1.0f / (2.0f * WP_PI * cutoff_hz);

	return dt_s / (dt_s + rc);
}

/** Soft deadzone then clamp, deg/s. */
static float shape_rate(float w, float dz, float max_rate)
{
	float mag = fabsf(w);

	if (dz < 0.0f) {
		dz = 0.0f;
	}
	if (!(mag > dz)) {
		return 0.0f;
	}

	mag -= dz;
	if (max_rate > 0.0f && mag > max_rate) {
		mag = max_rate;
	}

	return (w < 0.0f) ? -mag : mag;
}

static float ramp_factor(const struct wp_air_mouse *am, int64_t t_us)
{
	if (t_us <= am->start_us) {
		return 0.0f;
	}

	int64_t ramp_us = (int64_t)am->cfg.ramp_in_ms * WP_US_PER_MS;
	int64_t since_us = t_us - am->start_us;

	if (ramp_us <= 0 || since_us >= ramp_us) {
		return 1.0f;
	}

	return (float)since_us / (float)ramp_us;
}

void wp_air_mouse_update(struct wp_air_mouse *am, const struct wp_imu_sample *s, float *dx,
			 float *dy)
{
	float out_x = 0.0f;
	float out_y = 0.0f;

	if (dx != NULL) {
		*dx = 0.0f;
	}
	if (dy != NULL) {
		*dy = 0.0f;
	}
	if (am == NULL || s == NULL) {
		return;
	}

	/* ---- time base ---- */
	bool have_prev = (am->last_t_us != AM_T_NONE);
	int64_t dt_us = have_prev ? (s->t_us - am->last_t_us) : 0;

	if (have_prev && dt_us == 0) {
		/* Duplicate timestamp: nothing new to integrate. */
		return;
	}

	bool continuous = have_prev && dt_us > 0 && dt_us <= AM_MAX_GAP_US;
	float dt_s = continuous ? (float)dt_us / 1e6f : 0.0f;

	am->last_t_us = s->t_us;

	/* ---- gravity estimate: always, even while inactive ---- */
	update_gravity(am, s->accel, continuous, dt_s);

	if (!am->active) {
		return;
	}
	if (!s->gyro_valid || !finite3(s->gyro)) {
		reset_rate_filter(am);
		return;
	}

	/* ---- axis projection (rad/s -> deg/s) ---- */
	float yaw_axis[3];
	float pitch_axis[3];

	rate_axes(am, yaw_axis, pitch_axis);

	float yaw_dps = dot3(s->gyro, yaw_axis) * WP_RAD_TO_DEG;
	float pitch_dps = dot3(s->gyro, pitch_axis) * WP_RAD_TO_DEG;

	/* ---- 1-pole low-pass ---- */
	if (!am->lp_init || !continuous) {
		am->lp_yaw_dps = yaw_dps;
		am->lp_pitch_dps = pitch_dps;
		am->lp_init = true;
		if (!continuous) {
			return;
		}
	} else {
		float a = lpf_alpha(dt_s, am->cfg.lpf_cutoff_hz);

		am->lp_yaw_dps += a * (yaw_dps - am->lp_yaw_dps);
		am->lp_pitch_dps += a * (pitch_dps - am->lp_pitch_dps);
	}

	/* ---- deadzone, clamp, gain, ramp-in ---- */
	float ramp = ramp_factor(am, s->t_us);

	if (!(ramp > 0.0f)) {
		return;
	}

	float w_yaw = shape_rate(am->lp_yaw_dps, am->cfg.deadzone_dps, am->cfg.max_rate_dps);
	float w_pitch = shape_rate(am->lp_pitch_dps, am->cfg.deadzone_dps, am->cfg.max_rate_dps);
	float gain = am->cfg.counts_per_deg * dt_s * ramp;

	if (w_yaw != 0.0f) {
		out_x = -w_yaw * gain;
	}
	if (w_pitch != 0.0f) {
		out_y = -w_pitch * gain;
	}
	if (am->cfg.invert_x) {
		out_x = -out_x;
	}
	if (am->cfg.invert_y) {
		out_y = -out_y;
	}
	if (!isfinite(out_x)) {
		out_x = 0.0f;
	}
	if (!isfinite(out_y)) {
		out_y = 0.0f;
	}

	if (dx != NULL) {
		*dx = out_x;
	}
	if (dy != NULL) {
		*dy = out_y;
	}
}
