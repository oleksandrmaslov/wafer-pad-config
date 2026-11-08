/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Algorithm C: manual orientation calibration gesture (pure C, host-testable).
 * Specification: <wafer_pad/orientation_calib.h>.
 *
 * Phase machine (all positions converted to millimetres with the pad-frame
 * geometry given to wp_calib_init()):
 *
 *   IDLE      0 < fingers down < cfg.fingers: wait (fingers rarely land in
 *             the same frame); == cfg.fingers: anchor every finger -> HOLDING;
 *             more than cfg.fingers -> REJECTED.
 *   HOLDING   finger set changed -> REJECTED (IDLE if all lifted); any finger
 *             farther than hold_max_move_mm from its anchor -> REJECTED (this
 *             is how an immediate scroll is told apart); hold_ms elapsed
 *             (frame or tick) -> ARMED, EV_ARMED, centroid0 = centroid.
 *   ARMED     finger set changed or arm_timeout_ms exceeded -> EV_CANCELLED;
 *             centroid moved >= swipe_start_mm from centroid0 -> SWIPING
 *             (swipe timer starts) and the same frame is evaluated as below.
 *   SWIPING   finger set changed, swipe_max_ms exceeded before the centroid
 *             travelled swipe_min_mm (slow drag = scrolling) or a direction
 *             farther than swipe_max_angle_dev_deg from every axis
 *             -> EV_CANCELLED; else once >= swipe_min_mm -> WAIT_LIFT.
 *   WAIT_LIFT fingers may lift one by one; while all are still down the swipe
 *             vector keeps following the centroid. All fingers up within
 *             lift_max_ms of the swipe end and the total swipe vector still
 *             axis-aligned -> EV_COMPLETED; else EV_CANCELLED. A new finger
 *             -> EV_CANCELLED.
 *   REJECTED  ignore everything until a frame with no fingers -> IDLE.
 *
 * After EV_CANCELLED the session continues in REJECTED (or IDLE if no finger
 * is left), where wp_calib_inhibits_scroll() is false again: the scroll that
 * the user apparently intended resumes. Swipe angle (P frame, +y = bottom):
 * angle = atan2(-dx, -dy) wrapped to [0, 360), orientation = nearest 90.
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <wafer_pad/orientation_calib.h>
#include <wafer_pad/orientation_transform.h>

void wp_calib_config_default(struct wp_calib_config *cfg)
{
	/* Three fingers held 1.5 s: no trackpad gesture uses a 3-finger hold
	 * (the 3-finger tap is <= 220 ms, scrolling is two fingers), so a
	 * resting hand followed by a flick-scroll can never calibrate. */
	cfg->fingers = 3U;
	cfg->hold_ms = 1500U;
	cfg->hold_max_move_mm = 1.5f;
	cfg->arm_timeout_ms = 2500U;
	cfg->swipe_min_mm = 25.0f;
	cfg->swipe_max_ms = 450U;
	cfg->swipe_max_angle_dev_deg = 30.0f;
	cfg->lift_max_ms = 250U;
	cfg->swipe_start_mm = 2.0f;
}

static int64_t ms_to_us(uint32_t ms)
{
	return (int64_t)ms * WP_US_PER_MS;
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

/** Swipe angle per the table in orientation_calib.h, degrees [0, 360). */
static float swipe_angle(float dx, float dy)
{
	return wrap360(atan2f(-dx, -dy) * WP_RAD_TO_DEG);
}

/** Deviation (deg, 0..45) of @p angle from the nearest axis; nearest axis in @p q. */
static float axis_dev(float angle, enum wp_orientation *q)
{
	enum wp_orientation n = wp_orientation_from_deg(angle);
	float d = wrap360(angle - (float)wp_orientation_deg(n));

	if (d > 180.0f) {
		d = 360.0f - d;
	}
	*q = n;
	return d;
}

static float scale_mm(float extent_mm, uint16_t geom_max, uint16_t frame_max)
{
	uint16_t max = (geom_max != 0U) ? geom_max : frame_max;

	if (max == 0U || !(extent_mm > 0.0f)) {
		return 0.0f;
	}
	return extent_mm / (float)max;
}

static uint8_t frame_count(const struct wp_touch_frame *f)
{
	return (f->count > WP_TOUCH_MAX_CONTACTS) ? WP_TOUCH_MAX_CONTACTS : f->count;
}

static void contact_mm(const struct wp_calib *c, const struct wp_touch_frame *f,
		       const struct wp_touch_contact *ct, float *x_mm, float *y_mm)
{
	*x_mm = (float)ct->x * scale_mm(c->geom.width_mm, c->geom.max_x, f->max_x);
	*y_mm = (float)ct->y * scale_mm(c->geom.height_mm, c->geom.max_y, f->max_y);
}

static void centroid_mm(const struct wp_calib *c, const struct wp_touch_frame *f, float *cx,
			float *cy)
{
	uint8_t n = frame_count(f);
	float sx = 0.0f, sy = 0.0f;

	for (uint8_t i = 0; i < n; i++) {
		float x, y;

		contact_mm(c, f, &f->contacts[i], &x, &y);
		sx += x;
		sy += y;
	}
	*cx = (n > 0U) ? (sx / (float)n) : 0.0f;
	*cy = (n > 0U) ? (sy / (float)n) : 0.0f;
}

static int find_id(const struct wp_touch_frame *f, uint8_t id)
{
	uint8_t n = frame_count(f);

	for (uint8_t i = 0; i < n; i++) {
		if (f->contacts[i].id == id) {
			return (int)i;
		}
	}
	return -1;
}

/** Exactly the anchored fingers are down (by tracking id). */
static bool same_fingers(const struct wp_calib *c, const struct wp_touch_frame *f)
{
	if (frame_count(f) != c->anchor_count) {
		return false;
	}
	for (uint8_t i = 0; i < c->anchor_count; i++) {
		if (find_id(f, c->anchor_id[i]) < 0) {
			return false;
		}
	}
	return true;
}

/** Every finger down is one of the anchored fingers (fingers may only lift). */
static bool subset_of_anchors(const struct wp_calib *c, const struct wp_touch_frame *f)
{
	uint8_t n = frame_count(f);

	for (uint8_t i = 0; i < n; i++) {
		bool found = false;

		for (uint8_t k = 0; k < c->anchor_count; k++) {
			if (c->anchor_id[k] == f->contacts[i].id) {
				found = true;
				break;
			}
		}
		if (!found) {
			return false;
		}
	}
	return true;
}

static void enter(struct wp_calib *c, enum wp_calib_phase phase, int64_t t_us)
{
	c->phase = phase;
	c->phase_start_us = t_us;
}

static void start_hold(struct wp_calib *c, const struct wp_touch_frame *f)
{
	uint8_t n = frame_count(f);

	c->anchor_count = n;
	for (uint8_t i = 0; i < n; i++) {
		c->anchor_id[i] = f->contacts[i].id;
		contact_mm(c, f, &f->contacts[i], &c->anchor_x_mm[i], &c->anchor_y_mm[i]);
	}
	centroid_mm(c, f, &c->last_cx_mm, &c->last_cy_mm);
	c->hold_start_us = f->t_us;
	enter(c, WP_CALIB_HOLDING, f->t_us);
}

static float max_finger_move_mm(const struct wp_calib *c, const struct wp_touch_frame *f)
{
	float worst = 0.0f;

	for (uint8_t i = 0; i < c->anchor_count; i++) {
		int k = find_id(f, c->anchor_id[i]);
		float x, y;

		if (k < 0) {
			continue;
		}
		contact_mm(c, f, &f->contacts[k], &x, &y);
		float d = hypotf(x - c->anchor_x_mm[i], y - c->anchor_y_mm[i]);

		if (d > worst) {
			worst = d;
		}
	}
	return worst;
}

static void arm(struct wp_calib *c, int64_t t_us, struct wp_calib_result *out)
{
	c->centroid0_x_mm = c->last_cx_mm;
	c->centroid0_y_mm = c->last_cy_mm;
	c->swipe_dx_mm = 0.0f;
	c->swipe_dy_mm = 0.0f;
	enter(c, WP_CALIB_ARMED, t_us);
	out->event = WP_CALIB_EV_ARMED;
}

static void cancel(struct wp_calib *c, int64_t t_us, uint8_t fingers_down,
		   struct wp_calib_result *out)
{
	out->event = WP_CALIB_EV_CANCELLED;
	enter(c, (fingers_down == 0U) ? WP_CALIB_IDLE : WP_CALIB_REJECTED, t_us);
}

/** SWIPING: evaluate the centroid displacement (dx, dy) from centroid0 at t. */
static void swipe_eval(struct wp_calib *c, int64_t t_us, float dx, float dy, uint8_t n,
		       struct wp_calib_result *out)
{
	if ((t_us - c->swipe_start_us) > ms_to_us(c->cfg.swipe_max_ms)) {
		cancel(c, t_us, n, out); /* too slow: this is a scroll */
		return;
	}
	if (hypotf(dx, dy) < c->cfg.swipe_min_mm) {
		return;
	}

	enum wp_orientation q;

	if (axis_dev(swipe_angle(dx, dy), &q) > c->cfg.swipe_max_angle_dev_deg) {
		cancel(c, t_us, n, out); /* diagonal */
		return;
	}
	c->swipe_dx_mm = dx;
	c->swipe_dy_mm = dy;
	c->swipe_end_us = t_us;
	enter(c, WP_CALIB_WAIT_LIFT, t_us);
}

static void finish(struct wp_calib *c, int64_t t_us, struct wp_calib_result *out)
{
	enum wp_orientation q;
	float angle = swipe_angle(c->swipe_dx_mm, c->swipe_dy_mm);

	if ((t_us - c->swipe_end_us) > ms_to_us(c->cfg.lift_max_ms) ||
	    axis_dev(angle, &q) > c->cfg.swipe_max_angle_dev_deg) {
		cancel(c, t_us, 0U, out);
		return;
	}
	out->event = WP_CALIB_EV_COMPLETED;
	out->orientation = q;
	out->angle_deg = angle;
	enter(c, WP_CALIB_IDLE, t_us);
}

void wp_calib_init(struct wp_calib *c, const struct wp_calib_config *cfg,
		   const struct wp_pad_geometry *pad_geom)
{
	memset(c, 0, sizeof(*c));
	if (cfg != NULL) {
		c->cfg = *cfg;
	} else {
		wp_calib_config_default(&c->cfg);
	}
	if (pad_geom != NULL) {
		c->geom = *pad_geom;
	}
	wp_calib_reset(c);
}

void wp_calib_reset(struct wp_calib *c)
{
	c->phase = WP_CALIB_IDLE;
	c->phase_start_us = 0;
	c->hold_start_us = 0;
	c->anchor_count = 0U;
	c->centroid0_x_mm = 0.0f;
	c->centroid0_y_mm = 0.0f;
	c->last_cx_mm = 0.0f;
	c->last_cy_mm = 0.0f;
	c->swipe_start_us = 0;
	c->swipe_end_us = 0;
	c->swipe_dx_mm = 0.0f;
	c->swipe_dy_mm = 0.0f;
}

void wp_calib_update(struct wp_calib *c, const struct wp_touch_frame *f,
		     struct wp_calib_result *out)
{
	uint8_t n = frame_count(f);
	int64_t t = f->t_us;
	float cx, cy;

	out->event = WP_CALIB_EV_NONE;
	out->orientation = WP_ORIENT_0;
	out->angle_deg = 0.0f;

	switch (c->phase) {
	case WP_CALIB_IDLE:
		if (n == 0U) {
			break;
		}
		if (n == c->cfg.fingers) {
			start_hold(c, f);
		} else if (n > c->cfg.fingers) {
			enter(c, WP_CALIB_REJECTED, t);
		}
		break;

	case WP_CALIB_HOLDING:
		if (!same_fingers(c, f) ||
		    max_finger_move_mm(c, f) > c->cfg.hold_max_move_mm) {
			/* Finger change, or moved right away (scroll): not for us. */
			enter(c, (n == 0U) ? WP_CALIB_IDLE : WP_CALIB_REJECTED, t);
			break;
		}
		centroid_mm(c, f, &c->last_cx_mm, &c->last_cy_mm);
		if ((t - c->hold_start_us) >= ms_to_us(c->cfg.hold_ms)) {
			arm(c, t, out);
		}
		break;

	case WP_CALIB_ARMED:
		if (!same_fingers(c, f) ||
		    (t - c->phase_start_us) > ms_to_us(c->cfg.arm_timeout_ms)) {
			cancel(c, t, n, out);
			break;
		}
		centroid_mm(c, f, &cx, &cy);
		c->last_cx_mm = cx;
		c->last_cy_mm = cy;
		if (hypotf(cx - c->centroid0_x_mm, cy - c->centroid0_y_mm) >=
		    c->cfg.swipe_start_mm) {
			c->swipe_start_us = t;
			enter(c, WP_CALIB_SWIPING, t);
			swipe_eval(c, t, cx - c->centroid0_x_mm, cy - c->centroid0_y_mm, n, out);
		}
		break;

	case WP_CALIB_SWIPING:
		if (!same_fingers(c, f)) {
			cancel(c, t, n, out);
			break;
		}
		centroid_mm(c, f, &cx, &cy);
		c->last_cx_mm = cx;
		c->last_cy_mm = cy;
		swipe_eval(c, t, cx - c->centroid0_x_mm, cy - c->centroid0_y_mm, n, out);
		break;

	case WP_CALIB_WAIT_LIFT:
		if (n == 0U) {
			finish(c, t, out);
			break;
		}
		if (!subset_of_anchors(c, f) ||
		    (t - c->swipe_end_us) > ms_to_us(c->cfg.lift_max_ms)) {
			cancel(c, t, n, out);
			break;
		}
		if (same_fingers(c, f)) {
			centroid_mm(c, f, &cx, &cy);
			c->last_cx_mm = cx;
			c->last_cy_mm = cy;
			c->swipe_dx_mm = cx - c->centroid0_x_mm;
			c->swipe_dy_mm = cy - c->centroid0_y_mm;
		}
		break;

	case WP_CALIB_REJECTED:
	default:
		if (n == 0U) {
			enter(c, WP_CALIB_IDLE, t);
		}
		break;
	}
}

void wp_calib_tick(struct wp_calib *c, int64_t t_us, struct wp_calib_result *out)
{
	out->event = WP_CALIB_EV_NONE;
	out->orientation = WP_ORIENT_0;
	out->angle_deg = 0.0f;

	/* No frame: the fingers are assumed to be where the last frame saw them
	 * (the controller may stay silent while fingers rest). Timeouts move to
	 * REJECTED, which waits for a frame without fingers.
	 */
	switch (c->phase) {
	case WP_CALIB_HOLDING:
		if ((t_us - c->hold_start_us) >= ms_to_us(c->cfg.hold_ms)) {
			arm(c, t_us, out);
		}
		break;
	case WP_CALIB_ARMED:
		if ((t_us - c->phase_start_us) > ms_to_us(c->cfg.arm_timeout_ms)) {
			cancel(c, t_us, 1U, out);
		}
		break;
	case WP_CALIB_SWIPING:
		if ((t_us - c->swipe_start_us) > ms_to_us(c->cfg.swipe_max_ms)) {
			cancel(c, t_us, 1U, out);
		}
		break;
	case WP_CALIB_WAIT_LIFT:
		if ((t_us - c->swipe_end_us) > ms_to_us(c->cfg.lift_max_ms)) {
			cancel(c, t_us, 1U, out);
		}
		break;
	default:
		break;
	}
}

bool wp_calib_inhibits_scroll(const struct wp_calib *c)
{
	return (c->phase == WP_CALIB_ARMED) || (c->phase == WP_CALIB_SWIPING) ||
	       (c->phase == WP_CALIB_WAIT_LIFT);
}

enum wp_calib_phase wp_calib_phase_get(const struct wp_calib *c)
{
	return c->phase;
}
