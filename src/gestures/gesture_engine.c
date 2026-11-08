/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Trackpad gesture engine (pure C, host-testable). Spec: <wafer_pad/gestures.h>.
 *
 * Input frames are in the USER frame; positions are converted to millimetres
 * with the user-frame geometry (mm_per_count = size_mm / max_count per axis).
 *
 * Session model (a session lasts from the first finger down to the last
 * finger up):
 *
 *   IDLE --1 finger--> ONE_PENDING --travel > tap_max_move_mm--> POINTER
 *                                  --travel > tap_max_move_mm, session
 *                                    started inside the tap-drag window--> DRAG
 *                                  --still >= long_press_ms (drag enabled and
 *                                    allowed, or tap-drag candidate): drag
 *                                    ARMED; then travel > tap_max_move_mm
 *                                    from the armed position--> DRAG
 *                                  --2-3 fingers within multi_down_skew_ms of
 *                                    the first one--> MULTI_PENDING
 *   IDLE --2-3 fingers--> MULTI_PENDING --exactly 2 fingers, centroid travel >
 *                                         scroll_start_mm--> SCROLL
 *   any  --more than 3 fingers--> IGNORE (held buttons released)
 *   any  --last finger up--> IDLE (tap evaluation, drag released)
 *
 * Design decisions (documented contract of this implementation):
 *   - Pointer motion is already emitted in ONE_PENDING once the jitter
 *     deadband has been exceeded (responsive pointer, like libinput); a tap
 *     still counts while the travel stays <= tap_max_move_mm. Exception: in a
 *     session that may become a tap-drag the pointer is held still until the
 *     decision, so the drag starts exactly where the user tapped.
 *   - The deadband delays motion; it does not discard it: when it is exceeded
 *     the full movement since touch-down is fed through the filter.
 *   - Fingers that land LATER than multi_down_skew_ms after the first one do
 *     not turn a one-finger session into a multi-finger one: they are ignored
 *     for motion (resting thumb) and the pointer keeps following the tracked
 *     (anchor) finger. If the anchor lifts, the pointer re-anchors on the
 *     lowest-id remaining finger without emitting the jump.
 *   - A multi-finger session (MULTI_PENDING / SCROLL) never moves the pointer,
 *     even after it drops to one finger, until every finger has lifted. Scroll
 *     output is produced only while exactly two fingers are down; dropping to
 *     one or adding a third pauses it, and returning to two resumes it from a
 *     fresh anchor.
 *   - Every change of the finger set re-anchors pointer, scroll and still
 *     detection (no output in that frame).
 *   - So does an implausible step of a tracked contact (same id moved more
 *     than jump_max_step_mm, or faster than jump_max_speed_mm_s between two
 *     frames): slot reuse or a ghost coordinate. The delta is dropped (no
 *     pointer jump, no scroll burst, a long-press arm is cleared) and the
 *     contact continues from its new position; the session mode, the tap
 *     travel and the touch-down times are not affected.
 *   - "Still" is a rolling detector: the reference point moves to the current
 *     position whenever the single finger leaves the long_press_max_move_mm
 *     radius around it, and single_still_ms counts from that moment. This
 *     lets the air-mouse clutch engage after the finger has moved the pointer
 *     and then stopped. single_still is reported for any session mode while
 *     exactly one finger is down. In one-finger sessions the reference point
 *     is kept in scroll_cx_mm/scroll_cy_mm, which are only used for scrolling
 *     while two or more fingers are down (every finger-set change re-anchors
 *     them).
 *   - The one-finger tap click is emitted on lift when tap-drag is disabled.
 *     With tap-drag enabled it is kept pending until double_tap_window_ms
 *     passes without a new touch (wp_gesture_tick() or the next frame emits
 *     it). A new touch inside the window keeps it pending until the outcome:
 *     a tap-drag (movement, or long press then movement) DROPS it, so the
 *     host only sees press-move-release (never click + press = a double
 *     click); a second tap emits it at the lift and keeps its own click
 *     pending for the next call (two clicks, the host sees a double click);
 *     anything else (lift without drag, second finger, cancelled drag,
 *     reset) emits it.
 *   - A long press never presses the button by itself: it arms the drag and
 *     holds the pointer still; the press follows when the finger moves more
 *     than tap_max_move_mm from where it was armed. A lift while only armed
 *     emits nothing (no click). wp_gesture_cancel_drag() releases / disarms.
 *   - tap_enabled gates one- and two-finger taps (and therefore tap-drag);
 *     the three-finger tap is gated only by three_finger_tap_enabled.
 *   - Smoothing speed is the raw per-frame speed of the tracked finger; when
 *     the frame interval is unknown (dt <= 0) smoothing and acceleration are
 *     bypassed (alpha 1, multiplier 1).
 *   - wp_gesture_reset() forgets the contacts; fingers that are still down
 *     start a new session on the next frame (re-anchored, no jump).
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <wafer_pad/gestures.h>
#include <wafer_pad/orientation_transform.h>
#include <wafer_pad/types.h>

#define MULTI_MAX_FINGERS 3U
#define ALPHA_MIN         0.05f
#define US_PER_S_F        1000000.0f

enum scroll_axis {
	AXIS_FREE = 0,
	AXIS_VERTICAL = 1,
	AXIS_HORIZONTAL = 2,
};

void wp_gesture_config_default(struct wp_gesture_config *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->tap_enabled = true;
	cfg->tap_max_ms = 180U;
	cfg->tap_max_move_mm = 1.5f;
	cfg->tap_drag_enabled = true;
	cfg->double_tap_window_ms = 250U;
	cfg->long_press_drag_enabled = true;
	cfg->long_press_ms = 450U;
	cfg->long_press_max_move_mm = 1.0f;
	cfg->multi_down_skew_ms = 120U;
	cfg->two_finger_tap_max_ms = 220U;
	cfg->two_finger_tap_max_move_mm = 2.0f;
	cfg->three_finger_tap_enabled = true;
	cfg->scroll_detents_per_mm = 0.25f;
	cfg->scroll_start_mm = 1.0f;
	cfg->scroll_axis_lock = true;
	cfg->scroll_axis_lock_ratio = 2.0f;
	cfg->scroll.natural = true;
	cfg->scroll.invert_pan = false;
	cfg->jitter_deadband_mm = 0.3f;
	cfg->smoothing_alpha = 0.6f;
	cfg->smoothing_full_speed_mm_s = 60.0f;
	/* Implausible-step limits (ALG-9). A finger sliding on a pad moves at
	 * < 0.3 m/s when pointing carefully, 0.5-1 m/s when pointing fast, and
	 * peaks at roughly 1-1.5 m/s in a hard flick while it is still in
	 * contact (the 65 mm TPS65 stroke is too short to go much faster).
	 *  - 2000 mm/s is 1.3-2x that peak, which also absorbs RDY timestamp
	 *    jitter on the frame interval.
	 *  - 15 mm per frame is 1.5 m/s at the 10 ms active report period and
	 *    ~23 % of the pad width; a slot reuse or ghost coordinate jumps
	 *    20-40 mm (a third to half of the pad). After dropped frames a
	 *    genuine step > 15 mm only loses that one delta (re-anchor).
	 */
	cfg->jump_max_step_mm = 15.0f;
	cfg->jump_max_speed_mm_s = 2000.0f;
	wp_pointer_accel_config_default(&cfg->accel);
}

/* ------------------------------------------------------------------------ */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------ */

static int64_t ms_to_us(uint32_t ms)
{
	return (int64_t)ms * WP_US_PER_MS;
}

static int64_t elapsed_us(int64_t from, int64_t to)
{
	return (to > from) ? (to - from) : 0;
}

static float vlen(float x, float y)
{
	return sqrtf(x * x + y * y);
}

static void compute_scale(struct wp_gesture_engine *e)
{
	e->mm_per_count_x = 0.0f;
	e->mm_per_count_y = 0.0f;
	/* Invalid geometry -> no motion at all (taps still work). */
	if (e->geom.max_x > 0U && e->geom.width_mm > 0.0f) {
		e->mm_per_count_x = e->geom.width_mm / (float)e->geom.max_x;
	}
	if (e->geom.max_y > 0U && e->geom.height_mm > 0.0f) {
		e->mm_per_count_y = e->geom.height_mm / (float)e->geom.max_y;
	}
}

/* A per-frame step of one contact (same id) that no finger can make: slot
 * reuse or a ghost coordinate. @p dt_us <= 0 (unknown) skips the speed test. */
static bool step_implausible(const struct wp_gesture_config *cfg, float step_mm, int64_t dt_us)
{
	if (cfg->jump_max_step_mm > 0.0f && step_mm > cfg->jump_max_step_mm) {
		return true;
	}
	if (cfg->jump_max_speed_mm_s > 0.0f && dt_us > 0 &&
	    step_mm > cfg->jump_max_speed_mm_s * ((float)dt_us / US_PER_S_F)) {
		return true;
	}
	return false;
}

static void centroid(const float *x, const float *y, uint8_t n, float *cx, float *cy)
{
	float sx = 0.0f;
	float sy = 0.0f;

	for (uint8_t i = 0; i < n; i++) {
		sx += x[i];
		sy += y[i];
	}
	*cx = (n > 0U) ? sx / (float)n : 0.0f;
	*cy = (n > 0U) ? sy / (float)n : 0.0f;
}

static void flush_pending_click(struct wp_gesture_engine *e, struct wp_gesture_output *out)
{
	out->click |= e->pending_click;
	e->pending_click = 0U;
}

static void start_drag(struct wp_gesture_engine *e, struct wp_gesture_output *out)
{
	/* Tap-and-drag: the press replaces the deferred tap click. */
	e->pending_click = 0U;
	e->drag_armed = false;
	e->mode = WP_GEST_MODE_DRAG;
	if (!e->drag_active) {
		e->drag_active = true;
		out->events |= WP_GEST_EV_DRAG_START;
	}
}

static void end_drag(struct wp_gesture_engine *e, struct wp_gesture_output *out)
{
	if (e->drag_active) {
		e->drag_active = false;
		out->events |= WP_GEST_EV_DRAG_END;
	}
}

static void enter_ignore(struct wp_gesture_engine *e, struct wp_gesture_output *out)
{
	end_drag(e, out);
	flush_pending_click(e, out);
	e->drag_armed = false;
	e->mode = WP_GEST_MODE_IGNORE;
	e->last_tap_valid = false;
	e->anchor_valid = false;
}

/* Clear all per-session state (contacts, anchors, flags). Buttons must have
 * been released by the caller. */
static void clear_session(struct wp_gesture_engine *e)
{
	e->mode = WP_GEST_MODE_IDLE;
	e->contacts = 0U;
	e->max_contacts = 0U;
	e->max_travel_mm = 0.0f;
	memset(e->ids, 0, sizeof(e->ids));
	e->anchor_valid = false;
	e->deadband_passed = false;
	e->long_press_fired = false;
	e->drag_armed = false;
	e->scroll_travel_mm = 0.0f;
	e->scroll_started = false;
	e->scroll_axis = AXIS_FREE;
}

static void out_finish(const struct wp_gesture_engine *e, struct wp_gesture_output *out)
{
	out->buttons = e->drag_active ? (uint8_t)WP_BTN_LEFT : 0U;
	out->contacts = e->contacts;
	out->mode = e->mode;
	out->single_still = false;
	out->single_still_ms = 0U;
	if (e->contacts == 1U && e->now_us > e->still_since_us) {
		int64_t ms = (e->now_us - e->still_since_us) / WP_US_PER_MS;

		out->single_still = true;
		out->single_still_ms = (ms > (int64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)ms;
	}
	if (out->dx != 0.0f || out->dy != 0.0f) {
		out->events |= WP_GEST_EV_POINTER;
	}
	if (out->wheel != 0.0f || out->pan != 0.0f) {
		out->events |= WP_GEST_EV_SCROLL;
	}
}

/* ------------------------------------------------------------------------ */
/* Pointer                                                                   */
/* ------------------------------------------------------------------------ */

static float smoothing_alpha(const struct wp_gesture_config *cfg, float step_mm, float dt_s)
{
	float a0 = cfg->smoothing_alpha;
	float k;

	if (!(a0 < 1.0f) || !(dt_s > 0.0f) || !(cfg->smoothing_full_speed_mm_s > 0.0f)) {
		return 1.0f;
	}
	if (a0 < ALPHA_MIN) {
		a0 = ALPHA_MIN;
	}
	k = (step_mm / dt_s) / cfg->smoothing_full_speed_mm_s;
	if (k > 1.0f) {
		k = 1.0f;
	}
	return a0 + (1.0f - a0) * k;
}

/*
 * Track the anchor finger (index @p a in the current arrays) and, when
 * @p emit is set, add the resulting pointer motion to @p out.
 */
static void pointer_track(struct wp_gesture_engine *e, uint8_t a, const float *cur_x,
			  const float *cur_y, const float *prev_x, const float *prev_y, int64_t t,
			  bool emit, struct wp_gesture_output *out)
{
	const struct wp_gesture_config *cfg = &e->cfg;
	float rx = cur_x[a];
	float ry = cur_y[a];
	float dt_s = 0.0f;
	float alpha;
	float nx, ny, dx, dy;
	int64_t dt_us;

	if (!e->anchor_valid) {
		e->filt_x_mm = rx;
		e->filt_y_mm = ry;
		e->last_motion_us = t;
		e->anchor_valid = true;
		return;
	}
	if (!e->deadband_passed) {
		if (vlen(rx - e->filt_x_mm, ry - e->filt_y_mm) < cfg->jitter_deadband_mm) {
			return;
		}
		e->deadband_passed = true;
	}

	dt_us = t - e->last_motion_us;
	if (dt_us > 0) {
		dt_s = (float)dt_us / US_PER_S_F;
	}
	alpha = smoothing_alpha(cfg, vlen(rx - prev_x[a], ry - prev_y[a]), dt_s);
	nx = e->filt_x_mm + alpha * (rx - e->filt_x_mm);
	ny = e->filt_y_mm + alpha * (ry - e->filt_y_mm);
	dx = nx - e->filt_x_mm;
	dy = ny - e->filt_y_mm;
	e->filt_x_mm = nx;
	e->filt_y_mm = ny;
	e->last_motion_us = t;

	if (emit && (dx != 0.0f || dy != 0.0f)) {
		float cx, cy;

		wp_pointer_accel_apply(&cfg->accel, dx, dy, dt_s, &cx, &cy);
		out->dx += cx;
		out->dy += cy;
	}
}

static void single_step(struct wp_gesture_engine *e, const float *cur_x, const float *cur_y,
			const float *prev_x, const float *prev_y, int64_t t, bool reanchor,
			struct wp_gesture_output *out)
{
	bool emit = e->pointer_enabled && !reanchor;
	uint8_t a = 0U;
	bool found = false;

	for (uint8_t i = 0; i < e->contacts; i++) {
		if (e->ids[i] == e->anchor_id) {
			a = i;
			found = true;
			break;
		}
	}
	if (!found) {
		/* Tracked finger lifted: follow the lowest-id remaining finger. */
		e->anchor_id = e->ids[0];
		e->anchor_valid = false;
	}
	if (reanchor) {
		e->anchor_valid = false;
		/* The armed position belongs to the finger that was held:
		 * never turn a finger-set change (or a jump) into a press. */
		e->drag_armed = false;
	}

	if (e->mode == WP_GEST_MODE_ONE_PENDING) {
		bool tap_drag = e->last_tap_valid && e->max_contacts == 1U;

		if (e->drag_armed) {
			/* Long press seen: hold the pointer still; the press comes
			 * only with a deliberate movement from the armed position.
			 */
			if (vlen(cur_x[a] - e->arm_x_mm, cur_y[a] - e->arm_y_mm) >
			    e->cfg.tap_max_move_mm) {
				start_drag(e, out); /* press first, move from the next frame on */
			}
			emit = false;
		} else if (e->max_travel_mm > e->cfg.tap_max_move_mm) {
			e->last_tap_valid = false;
			if (tap_drag) {
				/* Press first, move from the next frame on. */
				start_drag(e, out);
				emit = false;
			} else {
				flush_pending_click(e, out); /* defensive: none expected */
				e->mode = WP_GEST_MODE_POINTER;
			}
		} else if (tap_drag) {
			emit = false;
		}
	}

	pointer_track(e, a, cur_x, cur_y, prev_x, prev_y, t, emit, out);
}

/* ------------------------------------------------------------------------ */
/* Scroll                                                                    */
/* ------------------------------------------------------------------------ */

static int8_t scroll_axis_decide(const struct wp_gesture_config *cfg, float ux, float uy)
{
	float ratio = cfg->scroll_axis_lock_ratio;
	float ax = fabsf(ux);
	float ay = fabsf(uy);

	if (!cfg->scroll_axis_lock) {
		return AXIS_FREE;
	}
	if (!(ratio >= 1.0f)) {
		ratio = 1.0f;
	}
	if (ay >= ratio * ax) {
		return AXIS_VERTICAL;
	}
	if (ax >= ratio * ay) {
		return AXIS_HORIZONTAL;
	}
	return AXIS_FREE;
}

static void multi_step(struct wp_gesture_engine *e, const float *cur_x, const float *cur_y,
		       bool reanchor, struct wp_gesture_output *out)
{
	const struct wp_gesture_config *cfg = &e->cfg;
	uint8_t n = e->contacts;
	float cx, cy, dx, dy, wheel, pan;

	/* One finger: scroll_cx/cy belong to the still detector. Set change or jump:
	 * the anchor was already re-set by the caller. */
	if (n == 1U || reanchor) {
		return;
	}
	centroid(cur_x, cur_y, n, &cx, &cy);
	if (n != 2U) {
		e->scroll_cx_mm = cx;
		e->scroll_cy_mm = cy;
		return;
	}

	if (!e->scroll_started) {
		/* Mean finger displacement from each finger's own landing point:
		 * immune to jitter accumulation and to finger-set changes. */
		float mx = 0.0f;
		float my = 0.0f;

		for (uint8_t i = 0; i < n; i++) {
			mx += cur_x[i] - e->start_x_mm[i];
			my += cur_y[i] - e->start_y_mm[i];
		}
		mx /= (float)n;
		my /= (float)n;
		e->scroll_travel_mm = vlen(mx, my);
		e->scroll_cx_mm = cx;
		e->scroll_cy_mm = cy;
		if (e->scroll_travel_mm > cfg->scroll_start_mm) {
			e->scroll_started = true;
			e->mode = WP_GEST_MODE_SCROLL;
			e->scroll_axis = scroll_axis_decide(cfg, mx, my);
		}
		return;
	}

	dx = cx - e->scroll_cx_mm;
	dy = cy - e->scroll_cy_mm;
	e->scroll_cx_mm = cx;
	e->scroll_cy_mm = cy;
	e->scroll_travel_mm += vlen(dx, dy);
	if (e->scroll_inhibit) {
		return;
	}
	if (e->scroll_axis == AXIS_VERTICAL) {
		dx = 0.0f;
	} else if (e->scroll_axis == AXIS_HORIZONTAL) {
		dy = 0.0f;
	}
	wp_scroll_map(&cfg->scroll, dx, dy, &wheel, &pan);
	out->wheel += wheel * cfg->scroll_detents_per_mm;
	out->pan += pan * cfg->scroll_detents_per_mm;
}

/* ------------------------------------------------------------------------ */
/* Session start / end, long press                                          */
/* ------------------------------------------------------------------------ */

static void check_long_press(struct wp_gesture_engine *e, int64_t now,
			     struct wp_gesture_output *out)
{
	if (e->mode != WP_GEST_MODE_ONE_PENDING || e->contacts != 1U || e->long_press_fired) {
		return;
	}
	if (elapsed_us(e->still_since_us, now) < ms_to_us(e->cfg.long_press_ms)) {
		return;
	}
	/* A tap-drag candidate (tap, then touch and hold) arms like a long
	 * press: the deferred click stays pending until the outcome.
	 */
	bool tap_hold = e->last_tap_valid && e->max_contacts == 1U;

	e->long_press_fired = true;
	e->last_tap_valid = false;
	out->events |= WP_GEST_EV_LONG_PRESS;
	if ((e->cfg.long_press_drag_enabled && e->long_press_allowed) || tap_hold) {
		e->drag_armed = true;
		/* Rolling still reference: within long_press_max_move_mm of the finger. */
		e->arm_x_mm = e->scroll_cx_mm;
		e->arm_y_mm = e->scroll_cy_mm;
	} else {
		flush_pending_click(e, out);
	}
}

/* No finger down: emit a deferred click once the tap-drag window is over
 * (or at once if this click can no longer start a tap-drag). */
static void expire_tap_window(struct wp_gesture_engine *e, int64_t now,
			      struct wp_gesture_output *out)
{
	if (e->last_tap_valid &&
	    elapsed_us(e->last_tap_us, now) > ms_to_us(e->cfg.double_tap_window_ms)) {
		e->last_tap_valid = false;
	}
	if (!e->last_tap_valid) {
		flush_pending_click(e, out);
	}
}

static void start_session(struct wp_gesture_engine *e, const struct wp_touch_frame *f, uint8_t n,
			  int64_t t, struct wp_gesture_output *out)
{
	const struct wp_gesture_config *cfg = &e->cfg;

	end_drag(e, out); /* defensive: cannot be active while no finger is down */
	clear_session(e);
	out->events |= WP_GEST_EV_TOUCH_START;

	/* last_tap_valid now means "this session may become a tap-drag". */
	e->last_tap_valid = e->last_tap_valid && cfg->tap_enabled && cfg->tap_drag_enabled &&
			    n == 1U &&
			    elapsed_us(e->last_tap_us, t) <= ms_to_us(cfg->double_tap_window_ms);
	if (!e->last_tap_valid) {
		/* This session cannot turn the deferred click into a drag. */
		flush_pending_click(e, out);
	}

	e->contacts = n;
	e->max_contacts = n;
	for (uint8_t i = 0; i < n; i++) {
		const struct wp_touch_contact *c = &f->contacts[i];

		e->ids[i] = c->id;
		e->start_x_mm[i] = (float)c->x * e->mm_per_count_x;
		e->start_y_mm[i] = (float)c->y * e->mm_per_count_y;
		e->last_x_mm[i] = e->start_x_mm[i];
		e->last_y_mm[i] = e->start_y_mm[i];
	}
	e->session_start_us = t;
	e->last_frame_us = t;
	e->first_down_us = t;
	e->last_down_us = t;

	e->anchor_id = e->ids[0];
	e->filt_x_mm = e->start_x_mm[0];
	e->filt_y_mm = e->start_y_mm[0];
	e->last_motion_us = t;
	e->anchor_valid = true;
	e->deadband_passed = !(cfg->jitter_deadband_mm > 0.0f);

	e->still_since_us = t;
	centroid(e->start_x_mm, e->start_y_mm, n, &e->scroll_cx_mm, &e->scroll_cy_mm);

	if (n == 1U) {
		e->mode = WP_GEST_MODE_ONE_PENDING;
	} else if (n <= MULTI_MAX_FINGERS) {
		e->mode = WP_GEST_MODE_MULTI_PENDING;
		e->last_tap_valid = false;
	} else {
		enter_ignore(e, out);
	}
}

static void end_session(struct wp_gesture_engine *e, int64_t t, struct wp_gesture_output *out)
{
	const struct wp_gesture_config *cfg = &e->cfg;
	int64_t dur = elapsed_us(e->first_down_us, t);
	bool tapped = false;

	out->events |= WP_GEST_EV_TOUCH_END;

	/* A deferred click that did not become a tap-drag is due now. */
	bool had_pending = e->pending_click != 0U;

	flush_pending_click(e, out);

	if (e->mode == WP_GEST_MODE_ONE_PENDING && cfg->tap_enabled && e->max_contacts == 1U &&
	    !e->long_press_fired && dur <= ms_to_us(cfg->tap_max_ms) &&
	    e->max_travel_mm <= cfg->tap_max_move_mm) {
		out->events |= WP_GEST_EV_TAP;
		tapped = true;
	} else if (e->mode == WP_GEST_MODE_MULTI_PENDING &&
		   dur <= ms_to_us(cfg->two_finger_tap_max_ms) &&
		   e->max_travel_mm <= cfg->two_finger_tap_max_move_mm &&
		   elapsed_us(e->first_down_us, e->last_down_us) <=
			   ms_to_us(cfg->multi_down_skew_ms)) {
		if (e->max_contacts == 2U && cfg->tap_enabled) {
			out->click |= (uint8_t)WP_BTN_RIGHT;
			out->events |= WP_GEST_EV_TWO_FINGER_TAP;
		} else if (e->max_contacts == 3U && cfg->three_finger_tap_enabled) {
			out->events |= WP_GEST_EV_THREE_FINGER_TAP;
		}
	}

	end_drag(e, out);
	clear_session(e);
	e->still_since_us = t;
	e->last_tap_valid = false;
	if (tapped) {
		if (!cfg->tap_drag_enabled) {
			out->click |= (uint8_t)WP_BTN_LEFT;
		} else if (had_pending) {
			/* Second tap: the first click went out above; this one is
			 * a separate click, emitted by the next tick / frame (no
			 * tap-drag after a double tap). */
			e->pending_click = (uint8_t)WP_BTN_LEFT;
		} else {
			e->pending_click = (uint8_t)WP_BTN_LEFT;
			e->last_tap_valid = true;
			e->last_tap_us = t;
		}
	}
}

/* ------------------------------------------------------------------------ */
/* Public API                                                                */
/* ------------------------------------------------------------------------ */

void wp_gesture_init(struct wp_gesture_engine *e, const struct wp_gesture_config *cfg,
		     const struct wp_pad_geometry *user_geom)
{
	memset(e, 0, sizeof(*e));
	if (cfg != NULL) {
		e->cfg = *cfg;
	} else {
		wp_gesture_config_default(&e->cfg);
	}
	if (user_geom != NULL) {
		e->geom = *user_geom;
	}
	compute_scale(e);
	e->mode = WP_GEST_MODE_IDLE;
	e->pointer_enabled = true;
	e->scroll_inhibit = false;
	e->long_press_allowed = true;
}

void wp_gesture_reset(struct wp_gesture_engine *e, struct wp_gesture_output *out)
{
	struct wp_gesture_output dummy;

	if (out == NULL) {
		out = &dummy;
	}
	memset(out, 0, sizeof(*out));
	end_drag(e, out);
	flush_pending_click(e, out);
	clear_session(e);
	e->last_tap_valid = false;
	e->still_since_us = e->now_us;
	out_finish(e, out);
}

void wp_gesture_set_geometry(struct wp_gesture_engine *e, const struct wp_pad_geometry *user_geom,
			     struct wp_gesture_output *out)
{
	wp_gesture_reset(e, out);
	if (user_geom != NULL) {
		e->geom = *user_geom;
	}
	compute_scale(e);
}

void wp_gesture_process(struct wp_gesture_engine *e, const struct wp_touch_frame *user_frame,
			struct wp_gesture_output *out)
{
	const struct wp_touch_frame *f = user_frame;
	float cur_x[WP_TOUCH_MAX_CONTACTS];
	float cur_y[WP_TOUCH_MAX_CONTACTS];
	float prev_x[WP_TOUCH_MAX_CONTACTS];
	float prev_y[WP_TOUCH_MAX_CONTACTS];
	float start_x[WP_TOUCH_MAX_CONTACTS];
	float start_y[WP_TOUCH_MAX_CONTACTS];
	uint8_t ids[WP_TOUCH_MAX_CONTACTS];
	struct wp_gesture_output dummy;
	bool set_changed;
	bool reanchor;
	bool added = false;
	bool jumped = false;
	int64_t frame_dt_us;
	int64_t t;
	uint8_t n;

	if (out == NULL) {
		out = &dummy;
	}
	memset(out, 0, sizeof(*out));
	if (f == NULL) {
		out_finish(e, out);
		return;
	}

	t = f->t_us;
	if (t > e->now_us) {
		e->now_us = t;
	}
	n = (f->count > WP_TOUCH_MAX_CONTACTS) ? (uint8_t)WP_TOUCH_MAX_CONTACTS : f->count;

	if (n == 0U) {
		if (e->contacts > 0U) {
			end_session(e, t, out);
		} else {
			expire_tap_window(e, e->now_us, out);
		}
		out_finish(e, out);
		return;
	}
	if (e->contacts == 0U) {
		start_session(e, f, n, t, out);
		out_finish(e, out);
		return;
	}

	/* Match the new contacts to the tracked ones by id. */
	set_changed = (n != e->contacts);
	frame_dt_us = t - e->last_frame_us;
	e->last_frame_us = t;
	for (uint8_t i = 0; i < n; i++) {
		const struct wp_touch_contact *c = &f->contacts[i];
		bool known = false;

		ids[i] = c->id;
		cur_x[i] = (float)c->x * e->mm_per_count_x;
		cur_y[i] = (float)c->y * e->mm_per_count_y;
		for (uint8_t j = 0; j < e->contacts; j++) {
			if (e->ids[j] == c->id) {
				start_x[i] = e->start_x_mm[j];
				start_y[i] = e->start_y_mm[j];
				prev_x[i] = e->last_x_mm[j];
				prev_y[i] = e->last_y_mm[j];
				known = true;
				break;
			}
		}
		if (known && step_implausible(&e->cfg,
					      vlen(cur_x[i] - prev_x[i], cur_y[i] - prev_y[i]),
					      frame_dt_us)) {
			/* Slot reuse / ghost coordinate: not motion. Re-anchor
			 * this contact at its new position (it is not a new
			 * finger: no touch-down time, no tap/skew effects). */
			start_x[i] = cur_x[i];
			start_y[i] = cur_y[i];
			prev_x[i] = cur_x[i];
			prev_y[i] = cur_y[i];
			jumped = true;
		}
		if (!known) {
			start_x[i] = cur_x[i];
			start_y[i] = cur_y[i];
			prev_x[i] = cur_x[i];
			prev_y[i] = cur_y[i];
			added = true;
			set_changed = true;
		}
	}
	/* An implausible step is handled like a finger-set change for motion,
	 * scroll and still detection (drop the delta, continue from the new
	 * position), but it never changes the session mode. */
	reanchor = set_changed || jumped;
	e->contacts = n;
	for (uint8_t i = 0; i < n; i++) {
		float d = vlen(cur_x[i] - start_x[i], cur_y[i] - start_y[i]);

		e->ids[i] = ids[i];
		e->start_x_mm[i] = start_x[i];
		e->start_y_mm[i] = start_y[i];
		if (d > e->max_travel_mm) {
			e->max_travel_mm = d;
		}
	}
	if (added) {
		e->last_down_us = t;
	}
	if (n > e->max_contacts) {
		e->max_contacts = n;
	}

	/* Re-anchor scroll centroid / still reference on finger-set change;
	 * otherwise run the rolling still detector for a single finger. */
	if (reanchor) {
		centroid(cur_x, cur_y, n, &e->scroll_cx_mm, &e->scroll_cy_mm);
		e->still_since_us = t;
	} else if (n == 1U) {
		if (vlen(cur_x[0] - e->scroll_cx_mm, cur_y[0] - e->scroll_cy_mm) >
		    e->cfg.long_press_max_move_mm) {
			e->scroll_cx_mm = cur_x[0];
			e->scroll_cy_mm = cur_y[0];
			e->still_since_us = t;
		}
	}

	if (n > MULTI_MAX_FINGERS && e->mode != WP_GEST_MODE_IGNORE) {
		enter_ignore(e, out);
	}
	if (e->mode == WP_GEST_MODE_ONE_PENDING && set_changed && n >= 2U &&
	    elapsed_us(e->first_down_us, t) <= ms_to_us(e->cfg.multi_down_skew_ms)) {
		e->mode = WP_GEST_MODE_MULTI_PENDING;
		e->last_tap_valid = false;
		e->drag_armed = false;
		flush_pending_click(e, out);
	}

	switch (e->mode) {
	case WP_GEST_MODE_ONE_PENDING:
	case WP_GEST_MODE_POINTER:
	case WP_GEST_MODE_DRAG:
		single_step(e, cur_x, cur_y, prev_x, prev_y, t, reanchor, out);
		check_long_press(e, t, out);
		break;
	case WP_GEST_MODE_MULTI_PENDING:
	case WP_GEST_MODE_SCROLL:
		multi_step(e, cur_x, cur_y, reanchor, out);
		break;
	default:
		break;
	}

	for (uint8_t i = 0; i < n; i++) {
		e->last_x_mm[i] = cur_x[i];
		e->last_y_mm[i] = cur_y[i];
	}
	out_finish(e, out);
}

void wp_gesture_tick(struct wp_gesture_engine *e, int64_t now_us, struct wp_gesture_output *out)
{
	struct wp_gesture_output dummy;

	if (out == NULL) {
		out = &dummy;
	}
	memset(out, 0, sizeof(*out));
	if (now_us > e->now_us) {
		e->now_us = now_us;
	}
	if (e->contacts == 0U) {
		expire_tap_window(e, e->now_us, out);
	} else {
		check_long_press(e, e->now_us, out);
	}
	out_finish(e, out);
}

void wp_gesture_set_pointer_enabled(struct wp_gesture_engine *e, bool enabled)
{
	if (e->pointer_enabled != enabled) {
		e->pointer_enabled = enabled;
		e->anchor_valid = false; /* resume from the current position */
	}
}

void wp_gesture_set_scroll_inhibit(struct wp_gesture_engine *e, bool inhibit)
{
	/* The scroll anchor keeps tracking the centroid while inhibited, so the
	 * first frame after the inhibit is lifted only carries one frame of
	 * motion (no burst). */
	e->scroll_inhibit = inhibit;
}

void wp_gesture_set_long_press_allowed(struct wp_gesture_engine *e, bool allowed)
{
	e->long_press_allowed = allowed;
	if (!allowed && e->drag_armed) {
		/* Armed but not pressed yet: nothing to release. A deferred
		 * click (tap-hold) is emitted at the lift. */
		e->drag_armed = false;
	}
}

void wp_gesture_cancel_drag(struct wp_gesture_engine *e, struct wp_gesture_output *out)
{
	struct wp_gesture_output dummy;
	bool active = e->drag_active || e->drag_armed;

	if (out == NULL) {
		out = &dummy;
	}
	memset(out, 0, sizeof(*out));
	if (active) {
		end_drag(e, out);
		flush_pending_click(e, out);
		e->drag_armed = false;
		e->last_tap_valid = false;
		e->long_press_fired = true; /* no new long press in this session */
		if (e->contacts > 0U && e->mode != WP_GEST_MODE_IGNORE) {
			e->mode = WP_GEST_MODE_POINTER;
			e->anchor_valid = false; /* re-anchor: no jump */
		}
	}
	out_finish(e, out);
}
