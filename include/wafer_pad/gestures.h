/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Trackpad gesture engine (pure C, host-testable).
 *
 * Input:  USER-frame touch frames (already passed through
 *         wp_xform_frame(), see orientation_transform.h) and the matching
 *         user-frame geometry (wp_xform_geometry()). Everything internal is
 *         computed in millimetres, so thresholds are controller-independent.
 * Output: pointer motion (fractional HID counts), scroll (fractional wheel
 *         detents), button state and one-shot clicks, gesture event flags.
 *
 * Supported gestures
 *   1 finger move              -> pointer motion (with acceleration)
 *   1 finger tap               -> left click (deferred, see below)
 *   tap, then touch+move within double_tap_window -> left-button drag
 *   1 finger long press (still for long_press_ms) ARMS a drag; the left
 *   button is pressed only when the finger then moves more than
 *   tap_max_move_mm (a long press followed by a lift does nothing)
 *   2 finger tap               -> right click
 *   2 finger move              -> scroll (vertical wheel + horizontal pan)
 *   3 finger tap               -> WP_GEST_EV_THREE_FINGER_TAP (mode toggle)
 *
 * Robustness rules
 *   - Whenever the set of fingers changes, motion is re-anchored (no jumps).
 *   - A contact whose per-frame step (same id) is implausible for a human
 *     finger (longer than jump_max_step_mm, or faster than
 *     jump_max_speed_mm_s) is a slot reuse / ghost coordinate: that frame is
 *     handled like a finger-set change (no pointer motion, no scroll, still
 *     detection and a long-press arm restart) and tracking continues from
 *     the new position.
 *   - A drag button is ALWAYS released when the last finger lifts, on
 *     wp_gesture_reset(), and when contact is lost (caller calls reset).
 *   - Double tap = two separate taps -> two clicks (the host decides that it
 *     is a double click from timing).
 *   - With tap_drag_enabled, a one-finger tap click is DEFERRED until
 *     double_tap_window_ms expired without a new touch (emitted by
 *     wp_gesture_tick() / the next frame). If the next touch becomes a
 *     tap-drag, the click is dropped and only the drag press is sent, so the
 *     host never sees "click + press" (a double click) before a tap-drag.
 *     A second tap emits both clicks (the second one on the next call).
 */

#ifndef WAFER_PAD_GESTURES_H_
#define WAFER_PAD_GESTURES_H_

#include <wafer_pad/types.h>
#include <wafer_pad/orientation_transform.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Pointer acceleration                                                      */
/* ------------------------------------------------------------------------ */

/**
 * Speed-dependent gain, piecewise linear, monotonic and continuous:
 *   v <= low_speed_mm_s  : mult = 1
 *   v >= high_speed_mm_s : mult = max_multiplier
 *   between              : linear interpolation
 *   counts = mm * counts_per_mm * sensitivity * mult
 */
struct wp_pointer_accel_config {
	float counts_per_mm;     /**< Base resolution. Default 16 (~400 CPI). */
	float sensitivity;       /**< User multiplier. Default 1.0. */
	float low_speed_mm_s;    /**< Default 20. */
	float high_speed_mm_s;   /**< Default 200. */
	float max_multiplier;    /**< Default 3.0 (1.0 disables acceleration). */
};

void wp_pointer_accel_config_default(struct wp_pointer_accel_config *cfg);

/** Speed-dependent multiplier (>= 1 when max_multiplier >= 1). */
float wp_pointer_accel_multiplier(const struct wp_pointer_accel_config *cfg, float speed_mm_s);

/** Convert a movement in mm over dt_s seconds into HID counts. dt_s <= 0
 *  means "unknown" -> multiplier 1. Sign is preserved per axis. */
void wp_pointer_accel_apply(const struct wp_pointer_accel_config *cfg, float dx_mm, float dy_mm,
			    float dt_s, float *dx_counts, float *dy_counts);

/* ------------------------------------------------------------------------ */
/* Gesture engine                                                            */
/* ------------------------------------------------------------------------ */

struct wp_gesture_config {
	/* taps */
	bool tap_enabled;                 /**< Default true. */
	uint32_t tap_max_ms;              /**< Default 180. */
	float tap_max_move_mm;            /**< Default 1.5. */
	bool tap_drag_enabled;            /**< Default true. */
	uint32_t double_tap_window_ms;    /**< Default 250. */
	/* long press drag */
	bool long_press_drag_enabled;     /**< Default true. */
	uint32_t long_press_ms;           /**< Default 450. */
	float long_press_max_move_mm;     /**< Default 1.0. */
	/* multi finger */
	uint32_t multi_down_skew_ms;      /**< Fingers landing within this count as
					   *   simultaneous. Default 120. */
	uint32_t two_finger_tap_max_ms;   /**< Default 220. */
	float two_finger_tap_max_move_mm; /**< Default 2.0. */
	bool three_finger_tap_enabled;    /**< Default true. */
	/* scroll */
	float scroll_detents_per_mm;      /**< Default 0.25 (4 mm per detent). */
	float scroll_start_mm;            /**< Centroid travel before scrolling
					   *   starts. Default 1.0. */
	bool scroll_axis_lock;            /**< Lock to the dominant axis per
					   *   scroll session. Default true. */
	float scroll_axis_lock_ratio;     /**< Dominance ratio. Default 2.0. */
	struct wp_scroll_config scroll;   /**< natural = true by default. */
	/* pointer */
	float jitter_deadband_mm;         /**< Movement < this right after touch
					   *   down is ignored. Default 0.3. */
	float smoothing_alpha;            /**< 1-pole low-pass on finger position
					   *   at low speed, 0..1, 1 = off.
					   *   Default 0.6. */
	float smoothing_full_speed_mm_s;  /**< Above this speed smoothing is off.
					   *   Default 60. */
	/* plausibility of a contact's per-frame step (same id); 0 disables */
	float jump_max_step_mm;           /**< A step longer than this is a
					   *   re-anchor, not motion. Default 15. */
	float jump_max_speed_mm_s;        /**< A step faster than this (frame
					   *   timestamps) is a re-anchor, not
					   *   motion. Default 2000. */
	struct wp_pointer_accel_config accel;
};

void wp_gesture_config_default(struct wp_gesture_config *cfg);

/* Event flags reported in wp_gesture_output.events (one-shot per call). */
#define WP_GEST_EV_TAP              WP_BIT(0)
#define WP_GEST_EV_TWO_FINGER_TAP   WP_BIT(1)
#define WP_GEST_EV_THREE_FINGER_TAP WP_BIT(2)
#define WP_GEST_EV_DRAG_START       WP_BIT(3)
#define WP_GEST_EV_DRAG_END         WP_BIT(4)
#define WP_GEST_EV_SCROLL           WP_BIT(5)
#define WP_GEST_EV_POINTER          WP_BIT(6)
#define WP_GEST_EV_LONG_PRESS       WP_BIT(7)
#define WP_GEST_EV_TOUCH_START      WP_BIT(8)  /**< first finger down */
#define WP_GEST_EV_TOUCH_END        WP_BIT(9)  /**< last finger up */

enum wp_gesture_mode {
	WP_GEST_MODE_IDLE = 0,
	WP_GEST_MODE_ONE_PENDING,   /**< 1 finger, undecided (tap / long press / move). */
	WP_GEST_MODE_POINTER,
	WP_GEST_MODE_DRAG,          /**< Left button held. */
	WP_GEST_MODE_MULTI_PENDING, /**< 2-3 fingers, undecided (tap / scroll). */
	WP_GEST_MODE_SCROLL,
	WP_GEST_MODE_IGNORE,        /**< Session invalid (e.g. >3 fingers) until all up. */
};

struct wp_gesture_output {
	float dx;               /**< Pointer motion, HID counts (fractional), user frame. */
	float dy;
	float wheel;            /**< Wheel detents (fractional), + = up. */
	float pan;              /**< Pan detents (fractional), + = right. */
	uint8_t buttons;        /**< Buttons that must be HELD now (drag). */
	uint8_t click;          /**< Buttons to click (press+release) once. */
	uint32_t events;        /**< WP_GEST_EV_* raised during this call. */
	uint8_t contacts;       /**< Fingers currently down. */
	enum wp_gesture_mode mode;
	bool single_still;      /**< Exactly one finger down and it has not moved
				 *   more than long_press_max_move_mm. */
	uint32_t single_still_ms; /**< How long it has been still (0 if not). */
};

struct wp_gesture_engine {
	struct wp_gesture_config cfg;
	struct wp_pad_geometry geom;     /**< USER-frame geometry. */
	float mm_per_count_x;
	float mm_per_count_y;
	enum wp_gesture_mode mode;
	int64_t now_us;
	/* session */
	int64_t session_start_us;
	uint8_t max_contacts;            /**< Max fingers seen this session. */
	uint8_t contacts;
	uint8_t ids[WP_TOUCH_MAX_CONTACTS];
	float start_x_mm[WP_TOUCH_MAX_CONTACTS];
	float start_y_mm[WP_TOUCH_MAX_CONTACTS];
	float last_x_mm[WP_TOUCH_MAX_CONTACTS];
	float last_y_mm[WP_TOUCH_MAX_CONTACTS];
	float max_travel_mm;             /**< Max travel of any finger this session. */
	int64_t last_frame_us;           /**< Timestamp of the last frame with contacts. */
	int64_t first_down_us;
	int64_t last_down_us;
	/* pointer */
	bool anchor_valid;
	uint8_t anchor_id;
	float filt_x_mm, filt_y_mm;      /**< Smoothed position of the tracked finger. */
	int64_t last_motion_us;
	bool deadband_passed;
	/* long press / drag */
	int64_t still_since_us;
	bool drag_active;
	bool long_press_fired;
	bool drag_armed;                 /**< Long press seen: press LEFT on movement. */
	float arm_x_mm, arm_y_mm;        /**< Finger position when the drag was armed. */
	uint8_t pending_click;           /**< Deferred tap click (WP_BTN_*), 0 = none. */
	/* tap-drag */
	int64_t last_tap_us;
	bool last_tap_valid;
	/* scroll */
	float scroll_cx_mm, scroll_cy_mm;   /**< Last centroid. */
	float scroll_travel_mm;
	bool scroll_started;
	int8_t scroll_axis;                 /**< 0 none, 1 vertical, 2 horizontal. */
	/* flags */
	bool pointer_enabled;
	bool scroll_inhibit;
	bool long_press_allowed;
};

void wp_gesture_init(struct wp_gesture_engine *e, const struct wp_gesture_config *cfg,
		     const struct wp_pad_geometry *user_geom);

/** Change geometry (after an orientation change). Implies wp_gesture_reset(). */
void wp_gesture_set_geometry(struct wp_gesture_engine *e, const struct wp_pad_geometry *user_geom,
			     struct wp_gesture_output *out);

/** Process one user-frame touch frame. Always fills @p out (zeroed first). */
void wp_gesture_process(struct wp_gesture_engine *e, const struct wp_touch_frame *user_frame,
			struct wp_gesture_output *out);

/** Advance time without a new frame (long press, tap-drag window expiry).
 *  Always fills @p out (zeroed first). */
void wp_gesture_tick(struct wp_gesture_engine *e, int64_t now_us, struct wp_gesture_output *out);

/**
 * Abort the session: forget all contacts and release every held button
 * (out->buttons = 0, WP_GEST_EV_DRAG_END if a drag was active). Use on
 * orientation change, contact loss, controller reset, mode switch.
 */
void wp_gesture_reset(struct wp_gesture_engine *e, struct wp_gesture_output *out);

/** false: finger motion does not move the pointer (air mouse clutch). Taps
 *  and scrolling keep working. Default true. */
void wp_gesture_set_pointer_enabled(struct wp_gesture_engine *e, bool enabled);

/** true: two-finger motion produces no scroll (calibration armed). When the
 *  inhibit is lifted the scroll anchor is re-set (no jump). */
void wp_gesture_set_scroll_inhibit(struct wp_gesture_engine *e, bool inhibit);

/** false: long-press drag disabled (handheld mode uses a still finger to
 *  activate the air mouse); also disarms a long press that has not pressed
 *  the button yet. Default true. */
void wp_gesture_set_long_press_allowed(struct wp_gesture_engine *e, bool allowed);

/**
 * End an active (or armed) drag without ending the touch session: the left
 * button is released (out->buttons = 0, WP_GEST_EV_DRAG_END) and the session
 * continues as plain pointer motion (re-anchored, no jump). A deferred tap
 * click is emitted. No-op (no mode change) if no drag is active or armed.
 * The router calls it when the device leaves the trackpad posture (pointer
 * or long press no longer allowed), so a drag can never continue into the
 * air mouse.
 */
void wp_gesture_cancel_drag(struct wp_gesture_engine *e, struct wp_gesture_output *out);

#ifdef __cplusplus
}
#endif

#endif /* WAFER_PAD_GESTURES_H_ */
