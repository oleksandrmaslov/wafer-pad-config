/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Orientation manager: combines A (gravity), B (lock / touch deferral),
 * C (manual calibration result) and D (gyro assist). Pure C, host-testable.
 *
 * Rules (B):
 *  - Automatic sources (A, D) may change the orientation only when
 *    auto_enabled, the external gate (wp_orient_mgr_allow_auto) is open and
 *    no manual lock is active. Otherwise the change is dropped and `block`
 *    reports why. A only proposes after its own confirmation (confirm_ms of a
 *    continuously identical, confident candidate), so a single noisy or
 *    lifting sample can never change the orientation.
 *  - While any finger is down, or until touch_release_settle_ms after the
 *    last finger lifted, an automatic change is stored as pending (block =
 *    TOUCH). It is applied once settled, and for A only if the latest
 *    gravity result still confirms the same orientation; otherwise it is
 *    dropped (A re-proposes it as soon as it confirms again).
 *  - Manual / forced orientations are applied immediately when no finger is
 *    down, else as soon as all fingers are up (no settle time). They start
 *    the manual lock (manual_lock_mode) and are never overwritten by a
 *    pending automatic change.
 *  - Every applied change resets the gravity estimator with the new
 *    orientation as hysteresis reference and clears the gyro accumulator.
 *    wp_orient_mgr_poll_change() returns true once per change of `current`.
 *
 * Gyro assist (D), experimental, cfg.gyro.enabled (default off):
 *  - Active only after a confident fix (gravity confirmation, manual, forced
 *    or a previous gyro step), while the gyro is valid, the automatic gates
 *    are open and the filtered tilt is <= gyro.max_tilt_deg.
 *  - Sign convention: types.h defines orientation as the CLOCKWISE rotation
 *    of the pad seen from above, and z_B points out of the touch surface.
 *    A positive gyro z rate (right-hand rule) is a counter-clockwise rotation
 *    seen from above, so the accumulator integrates the clockwise angle
 *        accum_deg += -gyro_z_dps * dt
 *    and accum >= +step_deg -> orientation + 90, accum <= -step_deg ->
 *    orientation - 90.
 *  - Only |rate| >= min_rate_dps is integrated (drift rejection). The
 *    accumulation window opens with the first integrated sample and the
 *    accumulator is cleared when it is older than max_window_ms, after each
 *    step, on gaps > 100 ms and whenever a precondition fails, so it is
 *    bounded by max_rate * max_window_ms and drift cannot build up.
 *  - A step is discarded while gravity (A) currently confirms an orientation:
 *    A is absolute and wins whenever it is informative. Steps are otherwise
 *    treated like a confirmed automatic candidate (same gates and touch
 *    deferral). Consecutive steps compose on top of a pending gyro step.
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <wafer_pad/orientation.h>
#include <wafer_pad/orientation_transform.h>

/** "Never" timestamp; far enough from INT64_MIN that differences cannot overflow. */
#define WP_MGR_NO_TIME (INT64_MIN / 4)

/** Must match the "no sample yet" marker of orientation_gravity.c. */
#define WP_GRAV_NO_TIME INT64_MIN

/** Larger gaps between IMU samples are not integrated by the gyro assist. */
#define WP_GYRO_MAX_DT_US (100LL * WP_US_PER_MS)

/** Confidence reported for a gyro step (relative tracking, no absolute fix). */
#define WP_GYRO_FIX_CONFIDENCE 0.5f

void wp_orient_gyro_config_default(struct wp_orient_gyro_config *cfg)
{
	cfg->enabled = false;
	cfg->step_deg = 70.0f;
	cfg->min_rate_dps = 8.0f;
	cfg->max_window_ms = 1500U;
	cfg->max_tilt_deg = 50.0f;
}

void wp_orient_mgr_config_default(struct wp_orient_mgr_config *cfg)
{
	wp_orient_gravity_config_default(&cfg->gravity);
	wp_orient_gyro_config_default(&cfg->gyro);
	cfg->auto_enabled = true;
	cfg->manual_lock_mode = WP_MANUAL_LOCK_TIMED;
	cfg->manual_lock_ms = 30000U;
	cfg->touch_release_settle_ms = 150U;
}

static bool is_auto_src(enum wp_orient_source src)
{
	return (src == WP_ORIENT_SRC_GRAVITY) || (src == WP_ORIENT_SRC_GYRO);
}

static void advance_time(struct wp_orient_mgr *m, int64_t t_us)
{
	if (t_us > m->now_us) {
		m->now_us = t_us;
	}
	if (m->manual_lock_active && m->cfg.manual_lock_mode == WP_MANUAL_LOCK_TIMED &&
	    m->now_us >= m->manual_lock_until_us) {
		m->manual_lock_active = false;
	}
}

static enum wp_orient_block_reason auto_gate(const struct wp_orient_mgr *m)
{
	if (!m->cfg.auto_enabled) {
		return WP_ORIENT_BLOCK_AUTO_OFF;
	}
	if (m->manual_lock_active) {
		return WP_ORIENT_BLOCK_MANUAL_LOCK;
	}
	if (!m->external_allow) {
		return WP_ORIENT_BLOCK_EXTERNAL;
	}
	return WP_ORIENT_BLOCK_NONE;
}

static bool touch_settled(const struct wp_orient_mgr *m)
{
	if (m->contacts > 0) {
		return false;
	}
	if (m->last_touch_up_us == WP_MGR_NO_TIME) {
		return true;
	}
	return (m->now_us - m->last_touch_up_us) >=
	       (int64_t)m->cfg.touch_release_settle_ms * WP_US_PER_MS;
}

static void gyro_clear(struct wp_orient_mgr *m)
{
	m->gyro_accum_deg = 0.0f;
	m->gyro_window_start_us = WP_MGR_NO_TIME;
}

static void apply(struct wp_orient_mgr *m, enum wp_orientation o, enum wp_orient_source src,
		  float conf)
{
	o = wp_orientation_norm((int)o);
	if (o != m->current) {
		m->changed = true;
	}
	m->current = o;
	m->source = src;
	m->confidence = conf;
	m->pending = false;
	m->block = WP_ORIENT_BLOCK_NONE;
	wp_orient_gravity_reset(&m->gravity, o);
	gyro_clear(m);
	m->gyro_ref_valid = true;
}

static void drop_auto_pending(struct wp_orient_mgr *m, enum wp_orient_block_reason why)
{
	if (m->pending && is_auto_src(m->pending_source)) {
		m->pending = false;
	}
	m->block = why;
}

/** Apply a deferred change if its conditions are met now. */
static void process_pending(struct wp_orient_mgr *m)
{
	if (!m->pending) {
		return;
	}
	if (!is_auto_src(m->pending_source)) {
		if (m->contacts == 0) {
			apply(m, m->pending_orientation, m->pending_source, 1.0f);
		} else {
			m->block = WP_ORIENT_BLOCK_TOUCH;
		}
		return;
	}

	enum wp_orient_block_reason gate = auto_gate(m);

	if (gate != WP_ORIENT_BLOCK_NONE) {
		drop_auto_pending(m, gate);
		return;
	}
	if (!touch_settled(m)) {
		m->block = WP_ORIENT_BLOCK_TOUCH;
		return;
	}
	if (m->pending_source == WP_ORIENT_SRC_GRAVITY) {
		if (m->last_gravity.confirmed &&
		    m->last_gravity.candidate == m->pending_orientation) {
			apply(m, m->pending_orientation, WP_ORIENT_SRC_GRAVITY,
			      m->last_gravity.confidence);
		} else {
			/* No longer confirmed: drop, A re-proposes when it confirms again. */
			drop_auto_pending(m, WP_ORIENT_BLOCK_NONE);
		}
		return;
	}
	apply(m, m->pending_orientation, m->pending_source, WP_GYRO_FIX_CONFIDENCE);
}

/** A confirmed automatic candidate (A or D). */
static void propose_auto(struct wp_orient_mgr *m, enum wp_orientation o, enum wp_orient_source src,
			 float conf)
{
	if (m->pending && !is_auto_src(m->pending_source)) {
		/* A manual / forced change is waiting for the lift: it wins. */
		return;
	}

	enum wp_orient_block_reason gate = auto_gate(m);

	if (o == m->current) {
		/* Re-confirmation of the active orientation: cancel any pending switch. */
		if (m->pending) {
			m->pending = false;
		}
		m->block = WP_ORIENT_BLOCK_NONE;
		if (src == WP_ORIENT_SRC_GRAVITY) {
			m->gyro_ref_valid = true;
			if (gate == WP_ORIENT_BLOCK_NONE &&
			    (m->source == WP_ORIENT_SRC_DEFAULT ||
			     m->source == WP_ORIENT_SRC_GRAVITY)) {
				m->source = WP_ORIENT_SRC_GRAVITY;
				m->confidence = conf;
			}
		}
		return;
	}
	if (gate != WP_ORIENT_BLOCK_NONE) {
		drop_auto_pending(m, gate);
		return;
	}
	if (!touch_settled(m)) {
		m->pending = true;
		m->pending_orientation = o;
		m->pending_source = src;
		m->block = WP_ORIENT_BLOCK_TOUCH;
		return;
	}
	apply(m, o, src, conf);
}

/** Algorithm D. Returns +1 / -1 quarter turns (clockwise positive) or 0. */
static int gyro_assist(struct wp_orient_mgr *m, const struct wp_imu_sample *s, int64_t prev_t_us,
		       const struct wp_orient_gravity_result *res)
{
	const struct wp_orient_gyro_config *gc = &m->cfg.gyro;

	if (!gc->enabled || !(gc->step_deg > 0.0f) || !s->gyro_valid || !m->gyro_ref_valid ||
	    auto_gate(m) != WP_ORIENT_BLOCK_NONE || !(res->tilt_deg <= gc->max_tilt_deg)) {
		gyro_clear(m);
		return 0;
	}
	if (prev_t_us == WP_GRAV_NO_TIME) {
		return 0;
	}

	int64_t dt_us = s->t_us - prev_t_us;

	if (dt_us == 0) {
		return 0;
	}
	if (dt_us < 0 || dt_us > WP_GYRO_MAX_DT_US || !isfinite(s->gyro[2])) {
		gyro_clear(m);
		return 0;
	}
	if (m->gyro_window_start_us != WP_MGR_NO_TIME &&
	    (s->t_us - m->gyro_window_start_us) > (int64_t)gc->max_window_ms * WP_US_PER_MS) {
		gyro_clear(m);
	}

	float rate_dps = s->gyro[2] * WP_RAD_TO_DEG;

	if (fabsf(rate_dps) < gc->min_rate_dps) {
		return 0;
	}
	if (m->gyro_window_start_us == WP_MGR_NO_TIME) {
		m->gyro_window_start_us = prev_t_us;
	}
	/* Clockwise degrees seen from above (positive z_B rate = counter-clockwise). */
	m->gyro_accum_deg += -rate_dps * ((float)dt_us / (float)(1000LL * WP_US_PER_MS));

	if (m->gyro_accum_deg >= gc->step_deg) {
		gyro_clear(m);
		return 1;
	}
	if (m->gyro_accum_deg <= -gc->step_deg) {
		gyro_clear(m);
		return -1;
	}
	return 0;
}

void wp_orient_mgr_init(struct wp_orient_mgr *m, const struct wp_orient_mgr_config *cfg,
			enum wp_orientation initial)
{
	memset(m, 0, sizeof(*m));
	if (cfg != NULL) {
		m->cfg = *cfg;
	} else {
		wp_orient_mgr_config_default(&m->cfg);
	}
	m->current = wp_orientation_norm((int)initial);
	wp_orient_gravity_init(&m->gravity, &m->cfg.gravity, m->current);
	m->source = WP_ORIENT_SRC_DEFAULT;
	m->confidence = 0.0f;
	m->pending = false;
	m->pending_orientation = m->current;
	m->pending_source = WP_ORIENT_SRC_DEFAULT;
	m->changed = false;
	m->contacts = 0;
	m->last_touch_up_us = WP_MGR_NO_TIME;
	m->external_allow = true;
	m->manual_lock_active = false;
	m->manual_lock_until_us = 0;
	m->now_us = WP_MGR_NO_TIME;
	m->gyro_ref_valid = false;
	gyro_clear(m);
	m->last_gravity.candidate = m->current;
	m->block = WP_ORIENT_BLOCK_NONE;
}

void wp_orient_mgr_on_imu(struct wp_orient_mgr *m, const struct wp_imu_sample *s)
{
	struct wp_orient_gravity_result res;
	int64_t prev_t_us = m->gravity.last_t_us;

	advance_time(m, s->t_us);
	wp_orient_gravity_update(&m->gravity, s, &res);
	m->last_gravity = res;

	if (!m->pending) {
		m->block = WP_ORIENT_BLOCK_NONE;
	}

	/* A: gravity */
	if (res.confirmed) {
		propose_auto(m, res.candidate, WP_ORIENT_SRC_GRAVITY, res.confidence);
	}

	/* D: gyro assist (uses the result computed before any reset by A). */
	int step = gyro_assist(m, s, prev_t_us, &res);

	if (step != 0 && !res.confirmed) {
		enum wp_orientation base = m->current;

		if (m->pending && m->pending_source == WP_ORIENT_SRC_GYRO) {
			base = m->pending_orientation;
		}
		propose_auto(m, wp_orientation_norm((int)base + step), WP_ORIENT_SRC_GYRO,
			     WP_GYRO_FIX_CONFIDENCE);
	}

	process_pending(m);
}

void wp_orient_mgr_on_touch(struct wp_orient_mgr *m, uint8_t contacts, int64_t t_us)
{
	advance_time(m, t_us);
	if (contacts == 0 && m->contacts > 0) {
		m->last_touch_up_us = m->now_us;
	}
	m->contacts = contacts;
	process_pending(m);
}

void wp_orient_mgr_tick(struct wp_orient_mgr *m, int64_t t_us)
{
	advance_time(m, t_us);
	process_pending(m);
}

void wp_orient_mgr_allow_auto(struct wp_orient_mgr *m, bool allow)
{
	m->external_allow = allow;
	if (!allow) {
		gyro_clear(m);
		if (m->pending && is_auto_src(m->pending_source)) {
			drop_auto_pending(m, WP_ORIENT_BLOCK_EXTERNAL);
		}
	}
}

void wp_orient_mgr_set_auto(struct wp_orient_mgr *m, bool enabled)
{
	m->cfg.auto_enabled = enabled;
	if (enabled) {
		/* Explicitly re-enabling automatic detection ends any manual lock. */
		m->manual_lock_active = false;
	} else {
		gyro_clear(m);
		if (m->pending && is_auto_src(m->pending_source)) {
			drop_auto_pending(m, WP_ORIENT_BLOCK_AUTO_OFF);
		}
	}
}

static void set_explicit(struct wp_orient_mgr *m, enum wp_orientation o, enum wp_orient_source src,
			 int64_t t_us)
{
	advance_time(m, t_us);
	o = wp_orientation_norm((int)o);

	switch (m->cfg.manual_lock_mode) {
	case WP_MANUAL_LOCK_TIMED:
		m->manual_lock_active = true;
		m->manual_lock_until_us = m->now_us + (int64_t)m->cfg.manual_lock_ms * WP_US_PER_MS;
		break;
	case WP_MANUAL_LOCK_UNTIL_RECAL:
		m->manual_lock_active = true;
		break;
	case WP_MANUAL_LOCK_NONE:
	default:
		m->manual_lock_active = false;
		break;
	}
	/* A zero-length timed lock expires right away. */
	advance_time(m, m->now_us);

	if (m->contacts == 0) {
		apply(m, o, src, 1.0f);
	} else {
		m->pending = true;
		m->pending_orientation = o;
		m->pending_source = src;
		m->block = WP_ORIENT_BLOCK_TOUCH;
	}
}

void wp_orient_mgr_set_manual(struct wp_orient_mgr *m, enum wp_orientation o, int64_t t_us)
{
	set_explicit(m, o, WP_ORIENT_SRC_MANUAL, t_us);
}

void wp_orient_mgr_force(struct wp_orient_mgr *m, enum wp_orientation o, int64_t t_us)
{
	set_explicit(m, o, WP_ORIENT_SRC_FORCED, t_us);
}

bool wp_orient_mgr_poll_change(struct wp_orient_mgr *m, enum wp_orientation *new_o)
{
	if (!m->changed) {
		return false;
	}
	m->changed = false;
	if (new_o != NULL) {
		*new_o = m->current;
	}
	return true;
}

enum wp_orientation wp_orient_mgr_get(const struct wp_orient_mgr *m)
{
	return m->current;
}

void wp_orient_mgr_get_status(const struct wp_orient_mgr *m, struct wp_orient_status *st)
{
	st->current = m->current;
	st->source = m->source;
	st->confidence = m->confidence;
	st->pending = m->pending;
	st->pending_orientation = m->pending_orientation;
	st->block = m->block;
	st->last_gravity = m->last_gravity;
}

const char *wp_orient_source_name(enum wp_orient_source src)
{
	switch (src) {
	case WP_ORIENT_SRC_DEFAULT:
		return "default";
	case WP_ORIENT_SRC_GRAVITY:
		return "gravity";
	case WP_ORIENT_SRC_MANUAL:
		return "manual";
	case WP_ORIENT_SRC_GYRO:
		return "gyro";
	case WP_ORIENT_SRC_FORCED:
		return "forced";
	default:
		return "unknown";
	}
}
