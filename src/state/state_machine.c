/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Device-level finite state machine (pure C). Specification and transition
 * table: <wafer_pad/state_machine.h>.
 *
 * Implementation notes
 *   - At most ONE transition is taken per wp_sm_update(). A transition caused
 *     by wp_sm_event() is applied immediately (wp_sm_state() sees it) and is
 *     reported by the next wp_sm_update(), which then evaluates no further
 *     rules; the following update continues from the new state.
 *   - state_changed / prev_state compare against the state reported by the
 *     previous update (A -> B -> A between two updates is no change).
 *   - Motion data is usable only when in->motion_valid && in->motion.valid.
 *     Without it the machine is a plain trackpad: no pickup, and
 *     PICKUP_CANDIDATE/HANDHELD fall back to TRACKPAD, AIR_MOUSE to HANDHELD
 *     (FORCE_HANDHELD keeps HANDHELD).
 *   - "still for X ms" = motion.still continuously true for X ms, measured by
 *     the machine itself or reported by motion.still_ms, whichever is longer.
 *   - The idle timer counts from the later of the last activity and the
 *     entry into TRACKPAD, so TRACKPAD always lasts idle_timeout_ms.
 *   - A zero idle/sleep/calibration timeout disables that timeout
 *     (disconnected_sleep_ms = 0 means "never", as documented).
 *   - PICKUP_CANDIDATE -> HANDHELD additionally requires !motion.still: a
 *     tremor estimate that is still decaying after a bump must not promote a
 *     device that is already lying still. Only gyro-based tremor counts there
 *     (the accel-jitter fallback rings for ~1 s after a knock), and "moving"
 *     counts only from pickup_settle_ms after the state was entered.
 *   - Put-down (HANDHELD / AIR_MOUSE -> TRACKPAD) needs "calm" = still AND no
 *     hand tremor, continuously for handheld_putdown_ms (tracked here).
 *   - CALIBRATING keeps the handheld context of the state it returns to
 *     (calib_return HANDHELD: gyro_needed stays true, long press stays
 *     disallowed), and while in it accel-only samples (tremor_from_gyro ==
 *     false) never count as still, so the still/calm timers cannot run up on
 *     accel-only data and trigger a put-down right after the return.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <wafer_pad/state_machine.h>

/** Timestamp meaning "never happened". */
#define SM_NEVER INT64_MIN

void wp_sm_config_default(struct wp_sm_config *cfg)
{
	if (cfg == NULL) {
		return;
	}

	cfg->pickup_confirm_ms = 400U;
	cfg->pickup_settle_ms = 150U;
	cfg->pickup_timeout_ms = 1500U;
	cfg->putdown_still_ms = 300U;
	cfg->handheld_putdown_ms = 1200U;
	cfg->air_mouse_enabled = true;
	cfg->air_hold_ms = 300U;
	cfg->idle_timeout_ms = 30000U;
	cfg->sleep_enabled = false;
	cfg->sleep_timeout_ms = 900000U;
	cfg->disconnected_sleep_ms = 0U;
	cfg->calib_timeout_ms = 5000U;
	cfg->error_retry_ms = 500U;
	cfg->error_retry_max_ms = 8000U;
	cfg->gyro_always_on = false;
}

/* ---- time helpers ------------------------------------------------------- */

static int64_t ms_to_us(uint32_t ms)
{
	return (int64_t)ms * WP_US_PER_MS;
}

/** now - since >= ms (false if @p since never happened). */
static bool elapsed_ge(int64_t now, int64_t since, uint32_t ms)
{
	if (since == SM_NEVER) {
		return false;
	}

	return (now - since) >= ms_to_us(ms);
}

/** @p t lies within the last @p ms (strictly): now - t < ms. */
static bool within_last(int64_t now, int64_t t, uint32_t ms)
{
	if (t == SM_NEVER) {
		return false;
	}

	return (now - t) < ms_to_us(ms);
}

static void set_now(struct wp_sm *sm, int64_t t_us)
{
	if (t_us > sm->now_us) {
		sm->now_us = t_us;
	}
}

static uint32_t retry_base_ms(const struct wp_sm *sm)
{
	return (sm->cfg.error_retry_ms > 0U) ? sm->cfg.error_retry_ms : 1U;
}

static uint32_t retry_max_ms(const struct wp_sm *sm)
{
	uint32_t base = retry_base_ms(sm);

	return (sm->cfg.error_retry_max_ms > base) ? sm->cfg.error_retry_max_ms : base;
}

/* ---- transitions -------------------------------------------------------- */

static void enter(struct wp_sm *sm, enum wp_state st)
{
	if (st == sm->state) {
		return;
	}

	if (!sm->changed_pending) {
		sm->changed_from = sm->state;
		sm->changed_pending = true;
	}
	if (sm->state == WP_STATE_ERROR_RECOVERY) {
		sm->reinit_pending = false;
	}

	sm->state = st;
	sm->state_since_us = sm->now_us;

	if (st == WP_STATE_ERROR_RECOVERY) {
		sm->retry_delay_ms = retry_base_ms(sm);
		sm->next_retry_us = sm->now_us + ms_to_us(sm->retry_delay_ms);
		sm->reinit_pending = true;
	}
}

/** Mode override effects that apply immediately (also re-checked in update). */
static void apply_mode_override(struct wp_sm *sm)
{
	switch (sm->mode) {
	case WP_MODE_FORCE_HANDHELD:
		if (sm->state == WP_STATE_TRACKPAD || sm->state == WP_STATE_PICKUP_CANDIDATE) {
			enter(sm, WP_STATE_HANDHELD);
		}
		break;
	case WP_MODE_FORCE_TRACKPAD:
		if (sm->state == WP_STATE_HANDHELD || sm->state == WP_STATE_AIR_MOUSE ||
		    sm->state == WP_STATE_PICKUP_CANDIDATE) {
			enter(sm, WP_STATE_TRACKPAD);
		}
		break;
	case WP_MODE_AUTO:
	default:
		break;
	}
}

static bool is_operational(enum wp_state st)
{
	switch (st) {
	case WP_STATE_TRACKPAD:
	case WP_STATE_PICKUP_CANDIDATE:
	case WP_STATE_HANDHELD:
	case WP_STATE_AIR_MOUSE:
	case WP_STATE_IDLE:
	case WP_STATE_CALIBRATING:
		return true;
	default:
		return false;
	}
}

void wp_sm_init(struct wp_sm *sm, const struct wp_sm_config *cfg, int64_t t_us)
{
	if (sm == NULL) {
		return;
	}

	memset(sm, 0, sizeof(*sm));
	if (cfg != NULL) {
		sm->cfg = *cfg;
	} else {
		wp_sm_config_default(&sm->cfg);
	}

	sm->state = WP_STATE_BOOT;
	sm->prev_state = WP_STATE_BOOT;
	sm->calib_return = WP_STATE_TRACKPAD;
	sm->mode = WP_MODE_AUTO;
	sm->now_us = t_us;
	sm->state_since_us = t_us;
	sm->last_activity_us = t_us;
	sm->last_moving_us = SM_NEVER;
	sm->still_since_us = t_us;
	sm->still_tracking = false;
	sm->calm_since_us = t_us;
	sm->calm_tracking = false;
	sm->ble_connected = false;
	sm->ble_disconnected_since_us = t_us;
	sm->next_retry_us = SM_NEVER;
	sm->retry_delay_ms = retry_base_ms(sm);
	sm->reinit_pending = false;
	sm->changed_pending = false;
	sm->changed_from = WP_STATE_BOOT;
}

void wp_sm_event(struct wp_sm *sm, enum wp_sm_event ev, int64_t t_us)
{
	if (sm == NULL) {
		return;
	}

	set_now(sm, t_us);

	if (sm->state == WP_STATE_SLEEP) {
		return; /* terminal: wake = reset */
	}

	switch (ev) {
	case WP_SM_EV_INIT_START:
		if (sm->state == WP_STATE_BOOT) {
			enter(sm, WP_STATE_INITIALIZING);
		}
		break;
	case WP_SM_EV_INIT_OK:
		if (sm->state == WP_STATE_INITIALIZING || sm->state == WP_STATE_ERROR_RECOVERY) {
			enter(sm, WP_STATE_TRACKPAD);
		}
		break;
	case WP_SM_EV_INIT_FAIL:
		if (sm->state == WP_STATE_INITIALIZING) {
			enter(sm, WP_STATE_ERROR_RECOVERY);
		}
		break;
	case WP_SM_EV_ERROR:
		/* INITIALIZING treats a generic error like EV_INIT_FAIL. */
		if (is_operational(sm->state) || sm->state == WP_STATE_INITIALIZING) {
			enter(sm, WP_STATE_ERROR_RECOVERY);
		}
		break;
	case WP_SM_EV_RECOVERED:
		if (sm->state == WP_STATE_ERROR_RECOVERY) {
			enter(sm, WP_STATE_TRACKPAD);
		}
		break;
	case WP_SM_EV_BLE_CONNECTED:
		sm->ble_connected = true;
		break;
	case WP_SM_EV_BLE_DISCONNECTED:
		if (sm->ble_connected) {
			sm->ble_connected = false;
			sm->ble_disconnected_since_us = sm->now_us;
		}
		break;
	case WP_SM_EV_CALIB_ARMED:
		if (sm->state == WP_STATE_TRACKPAD || sm->state == WP_STATE_HANDHELD ||
		    sm->state == WP_STATE_AIR_MOUSE) {
			sm->calib_return = (sm->state == WP_STATE_AIR_MOUSE) ? WP_STATE_HANDHELD
									      : sm->state;
			enter(sm, WP_STATE_CALIBRATING);
		}
		break;
	case WP_SM_EV_CALIB_DONE:
	case WP_SM_EV_CALIB_CANCEL:
		if (sm->state == WP_STATE_CALIBRATING) {
			enter(sm, sm->calib_return);
		}
		break;
	case WP_SM_EV_MODE_TOGGLE:
		switch (sm->mode) {
		case WP_MODE_AUTO:
			sm->mode = WP_MODE_FORCE_HANDHELD;
			break;
		case WP_MODE_FORCE_HANDHELD:
			sm->mode = WP_MODE_FORCE_TRACKPAD;
			break;
		case WP_MODE_FORCE_TRACKPAD:
		default:
			sm->mode = WP_MODE_AUTO;
			break;
		}
		apply_mode_override(sm);
		break;
	default:
		break;
	}
}

/* ---- handheld context --------------------------------------------------- */

/** States in which the device is (or may be) held in the hand. */
static bool is_handheld_state(enum wp_state st)
{
	return st == WP_STATE_PICKUP_CANDIDATE || st == WP_STATE_HANDHELD ||
	       st == WP_STATE_AIR_MOUSE;
}

/**
 * Handheld context: a handheld state, or CALIBRATING entered from one (the
 * calibration is a short excursion that returns to calib_return; powering the
 * gyro down for it would make stillness accel-only and drop a held device
 * back to TRACKPAD on return).
 */
static bool in_handheld_context(const struct wp_sm *sm)
{
	if (sm->state == WP_STATE_CALIBRATING) {
		return is_handheld_state(sm->calib_return);
	}
	return is_handheld_state(sm->state);
}

/* ---- per-update evaluation ---------------------------------------------- */

struct sm_ctx {
	uint8_t contacts;
	bool single_still;
	uint32_t single_still_ms;
	bool motion_ok;  /**< IMU data usable. */
	bool moving;
	bool still;
	bool tremor;
	bool gyro_tremor; /**< Tremor measured by the gyroscope. */
	bool activity;
	int64_t still_us; /**< Duration of continuous stillness, -1 if not still. */
	int64_t calm_us;  /**< Duration of still && !tremor, -1 if not calm. */
};

static bool still_for(const struct sm_ctx *c, uint32_t ms)
{
	return c->still && c->still_us >= ms_to_us(ms);
}

/** Put down: lying still with no hand tremor for @p ms. */
static bool calm_for(const struct sm_ctx *c, uint32_t ms)
{
	return c->calm_us >= 0 && c->calm_us >= ms_to_us(ms);
}

static void eval_error_recovery(struct wp_sm *sm)
{
	if (sm->next_retry_us == SM_NEVER || sm->now_us < sm->next_retry_us) {
		return;
	}

	uint32_t max = retry_max_ms(sm);

	sm->reinit_pending = true;
	if (sm->retry_delay_ms >= max / 2U) {
		sm->retry_delay_ms = max;
	} else {
		sm->retry_delay_ms *= 2U;
	}
	sm->next_retry_us = sm->now_us + ms_to_us(sm->retry_delay_ms);
}

static void eval_trackpad(struct wp_sm *sm, const struct sm_ctx *c)
{
	const struct wp_sm_config *cfg = &sm->cfg;

	if (sm->mode == WP_MODE_FORCE_HANDHELD) {
		enter(sm, WP_STATE_HANDHELD);
		return;
	}
	if (sm->mode == WP_MODE_AUTO && c->moving) {
		enter(sm, WP_STATE_PICKUP_CANDIDATE);
		return;
	}

	int64_t idle_ref = (sm->last_activity_us > sm->state_since_us) ? sm->last_activity_us
									 : sm->state_since_us;

	if (cfg->idle_timeout_ms > 0U && elapsed_ge(sm->now_us, idle_ref, cfg->idle_timeout_ms)) {
		enter(sm, WP_STATE_IDLE);
	}
}

static void eval_pickup(struct wp_sm *sm, const struct sm_ctx *c)
{
	const struct wp_sm_config *cfg = &sm->cfg;

	if (sm->mode == WP_MODE_FORCE_HANDHELD) {
		enter(sm, WP_STATE_HANDHELD);
		return;
	}
	if (sm->mode == WP_MODE_FORCE_TRACKPAD || !c->motion_ok) {
		enter(sm, WP_STATE_TRACKPAD);
		return;
	}
	if (still_for(c, cfg->putdown_still_ms)) {
		enter(sm, WP_STATE_TRACKPAD); /* bump */
		return;
	}
	/* Evidence must come after the knock that started the candidate. */
	bool moved_after_settle =
		sm->last_moving_us != SM_NEVER &&
		sm->last_moving_us >= sm->state_since_us + ms_to_us(cfg->pickup_settle_ms) &&
		within_last(sm->now_us, sm->last_moving_us, cfg->pickup_confirm_ms);

	if (elapsed_ge(sm->now_us, sm->state_since_us, cfg->pickup_confirm_ms) && !c->still &&
	    (c->gyro_tremor || moved_after_settle)) {
		enter(sm, WP_STATE_HANDHELD);
		return;
	}
	if (elapsed_ge(sm->now_us, sm->state_since_us, cfg->pickup_timeout_ms)) {
		enter(sm, WP_STATE_TRACKPAD);
	}
}

static void eval_handheld(struct wp_sm *sm, const struct sm_ctx *c)
{
	const struct wp_sm_config *cfg = &sm->cfg;
	bool forced = (sm->mode == WP_MODE_FORCE_HANDHELD);

	if (sm->mode == WP_MODE_FORCE_TRACKPAD) {
		enter(sm, WP_STATE_TRACKPAD);
		return;
	}
	if (!c->motion_ok) {
		if (!forced) {
			enter(sm, WP_STATE_TRACKPAD);
		}
		return;
	}
	if (!forced && calm_for(c, cfg->handheld_putdown_ms)) {
		enter(sm, WP_STATE_TRACKPAD);
		return;
	}
	if (cfg->air_mouse_enabled && c->contacts == 1U && c->single_still &&
	    c->single_still_ms >= cfg->air_hold_ms) {
		enter(sm, WP_STATE_AIR_MOUSE);
	}
}

static void eval_air_mouse(struct wp_sm *sm, const struct sm_ctx *c)
{
	const struct wp_sm_config *cfg = &sm->cfg;

	if (sm->mode == WP_MODE_FORCE_TRACKPAD) {
		enter(sm, WP_STATE_TRACKPAD);
		return;
	}
	if (c->contacts != 1U || !c->motion_ok || !cfg->air_mouse_enabled) {
		enter(sm, WP_STATE_HANDHELD);
		return;
	}
	if (sm->mode != WP_MODE_FORCE_HANDHELD && calm_for(c, cfg->handheld_putdown_ms)) {
		enter(sm, WP_STATE_TRACKPAD);
	}
}

static void eval_idle(struct wp_sm *sm, const struct sm_ctx *c)
{
	const struct wp_sm_config *cfg = &sm->cfg;

	if (c->activity) {
		enter(sm, WP_STATE_TRACKPAD);
		return;
	}
	if (!cfg->sleep_enabled) {
		return;
	}
	if (cfg->sleep_timeout_ms > 0U &&
	    elapsed_ge(sm->now_us, sm->state_since_us, cfg->sleep_timeout_ms)) {
		enter(sm, WP_STATE_SLEEP);
		return;
	}
	if (cfg->disconnected_sleep_ms > 0U && !sm->ble_connected &&
	    elapsed_ge(sm->now_us, sm->ble_disconnected_since_us, cfg->disconnected_sleep_ms)) {
		enter(sm, WP_STATE_SLEEP);
	}
}

static void eval_calibrating(struct wp_sm *sm)
{
	if (sm->cfg.calib_timeout_ms > 0U &&
	    elapsed_ge(sm->now_us, sm->state_since_us, sm->cfg.calib_timeout_ms)) {
		enter(sm, sm->calib_return);
	}
}

static void evaluate(struct wp_sm *sm, const struct sm_ctx *c)
{
	switch (sm->state) {
	case WP_STATE_TRACKPAD:
		eval_trackpad(sm, c);
		break;
	case WP_STATE_PICKUP_CANDIDATE:
		eval_pickup(sm, c);
		break;
	case WP_STATE_HANDHELD:
		eval_handheld(sm, c);
		break;
	case WP_STATE_AIR_MOUSE:
		eval_air_mouse(sm, c);
		break;
	case WP_STATE_IDLE:
		eval_idle(sm, c);
		break;
	case WP_STATE_CALIBRATING:
		eval_calibrating(sm);
		break;
	case WP_STATE_ERROR_RECOVERY:
		eval_error_recovery(sm);
		break;
	case WP_STATE_BOOT:
	case WP_STATE_INITIALIZING:
	case WP_STATE_SLEEP:
	default:
		break;
	}
}

static void fill_outputs(struct wp_sm *sm, struct wp_sm_outputs *out)
{
	enum wp_state st = sm->state;

	out->state = st;
	out->mode = sm->mode;

	out->state_changed = false;
	if (sm->changed_pending) {
		sm->changed_pending = false;
		if (st != sm->changed_from) {
			sm->prev_state = sm->changed_from;
			out->state_changed = true;
		}
	}
	out->prev_state = sm->prev_state;

	out->gyro_needed = in_handheld_context(sm) ||
			   (sm->cfg.gyro_always_on && st != WP_STATE_SLEEP);
	out->imu_low_rate = (st == WP_STATE_IDLE);
	out->touch_low_power = (st == WP_STATE_IDLE);
	out->pointer_from_touch = (st != WP_STATE_AIR_MOUSE);
	out->air_mouse_active = (st == WP_STATE_AIR_MOUSE);
	out->long_press_allowed = !(in_handheld_context(sm) || sm->mode == WP_MODE_FORCE_HANDHELD);
	out->orientation_auto_allowed =
		!(st == WP_STATE_AIR_MOUSE || st == WP_STATE_CALIBRATING ||
		  st == WP_STATE_ERROR_RECOVERY || st == WP_STATE_BOOT ||
		  st == WP_STATE_INITIALIZING);
	out->request_reinit = (st == WP_STATE_ERROR_RECOVERY) && sm->reinit_pending;
	sm->reinit_pending = false;
	out->request_sleep = (st == WP_STATE_SLEEP);
	out->ble_low_latency = (st == WP_STATE_TRACKPAD || st == WP_STATE_PICKUP_CANDIDATE ||
				st == WP_STATE_HANDHELD || st == WP_STATE_AIR_MOUSE ||
				st == WP_STATE_CALIBRATING);
}

void wp_sm_update(struct wp_sm *sm, const struct wp_sm_inputs *in, struct wp_sm_outputs *out)
{
	struct wp_sm_outputs local_out;

	if (out == NULL) {
		out = &local_out;
	}
	memset(out, 0, sizeof(*out));
	if (sm == NULL) {
		return;
	}

	struct sm_ctx c = {0};

	c.still_us = -1;
	c.calm_us = -1;
	if (in != NULL) {
		set_now(sm, in->t_us);
		c.contacts = in->contacts;
		c.single_still = in->single_still;
		c.single_still_ms = in->single_still_ms;
		c.motion_ok = in->motion_valid && in->motion.valid;
	}
	if (c.motion_ok) {
		c.moving = in->motion.moving;
		c.still = in->motion.still;
		c.tremor = in->motion.hand_tremor;
		c.gyro_tremor = in->motion.hand_tremor && in->motion.tremor_from_gyro;
		/* CALIBRATING in the handheld context: accel-only stillness
		 * (gyro sample not valid, e.g. still settling) is not evidence of
		 * a put-down, so it must not start the still/calm timers that
		 * HANDHELD evaluates right after the return. */
		if (sm->state == WP_STATE_CALIBRATING && in_handheld_context(sm) &&
		    !in->motion.tremor_from_gyro) {
			c.still = false;
		}
	}
	c.activity = (c.contacts > 0U) || c.moving;

	if (c.activity) {
		sm->last_activity_us = sm->now_us;
	}
	if (c.moving) {
		sm->last_moving_us = sm->now_us;
	}
	if (c.still) {
		if (!sm->still_tracking) {
			sm->still_tracking = true;
			sm->still_since_us = sm->now_us;
		}
		c.still_us = sm->now_us - sm->still_since_us;
		if (ms_to_us(in->motion.still_ms) > c.still_us) {
			c.still_us = ms_to_us(in->motion.still_ms);
		}
	} else {
		sm->still_tracking = false;
	}
	if (c.still && !c.tremor) {
		if (!sm->calm_tracking) {
			sm->calm_tracking = true;
			sm->calm_since_us = sm->now_us;
		}
		c.calm_us = sm->now_us - sm->calm_since_us;
	} else {
		sm->calm_tracking = false;
	}

	/* One transition per update: an event-driven change is reported first. */
	if (!sm->changed_pending) {
		evaluate(sm, &c);
	}

	fill_outputs(sm, out);
}

enum wp_state wp_sm_state(const struct wp_sm *sm)
{
	return (sm != NULL) ? sm->state : WP_STATE_BOOT;
}

const char *wp_state_name(enum wp_state st)
{
	switch (st) {
	case WP_STATE_BOOT:
		return "BOOT";
	case WP_STATE_INITIALIZING:
		return "INITIALIZING";
	case WP_STATE_TRACKPAD:
		return "TRACKPAD";
	case WP_STATE_PICKUP_CANDIDATE:
		return "PICKUP_CANDIDATE";
	case WP_STATE_HANDHELD:
		return "HANDHELD";
	case WP_STATE_AIR_MOUSE:
		return "AIR_MOUSE";
	case WP_STATE_IDLE:
		return "IDLE";
	case WP_STATE_SLEEP:
		return "SLEEP";
	case WP_STATE_CALIBRATING:
		return "CALIBRATING";
	case WP_STATE_ERROR_RECOVERY:
		return "ERROR_RECOVERY";
	default:
		return "UNKNOWN";
	}
}

const char *wp_mode_name(enum wp_mode_override mode)
{
	switch (mode) {
	case WP_MODE_AUTO:
		return "AUTO";
	case WP_MODE_FORCE_HANDHELD:
		return "FORCE_HANDHELD";
	case WP_MODE_FORCE_TRACKPAD:
		return "FORCE_TRACKPAD";
	default:
		return "UNKNOWN";
	}
}
