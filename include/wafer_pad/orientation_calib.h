/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Algorithm C: manual orientation calibration gesture (pure C).
 *
 *   1. Exactly `fingers` (default 3) fingers rest on the pad and stay within
 *      `hold_max_move_mm` of where they landed for `hold_ms` (default 1500 ms)
 *      -> ARMED (caller should give feedback, e.g. LED, and inhibit scrolling).
 *   2. Within `arm_timeout_ms`, the fingers perform a deliberate swipe: the
 *      centroid travels >= `swipe_min_mm` within `swipe_max_ms`, in a
 *      direction within `swipe_max_angle_dev_deg` of one of the four pad axes.
 *   3. The fingers lift within `lift_max_ms` after the swipe ends
 *      -> COMPLETED; the swipe direction is "away from the user".
 *
 * Anything else (slow movement = scrolling, a finger added/removed, timeout,
 * diagonal swipe) -> CANCELLED, and the touch session is ignored by the
 * detector until all fingers lift.
 *
 * The defaults (3 fingers, 1.5 s hold, 25 mm swipe) are chosen so that no
 * normal trackpad use can complete the gesture: two resting fingers followed
 * by a quick flick are a SCROLL, never a calibration (with 2 fingers and a
 * 0.7 s hold, "rest, then flick-scroll" flipped the pad by 180 deg).
 *
 * Direction -> orientation (P frame, +y towards bottom edge):
 *   swipe towards -y_P (top edge)    -> WP_ORIENT_0
 *   swipe towards -x_P (left edge)   -> WP_ORIENT_90
 *   swipe towards +y_P (bottom edge) -> WP_ORIENT_180
 *   swipe towards +x_P (right edge)  -> WP_ORIENT_270
 *   i.e. angle = atan2(-dx, -dy) in degrees, nearest multiple of 90.
 *
 * Input frames MUST be pad-frame (P) frames, NOT orientation-transformed.
 */

#ifndef WAFER_PAD_ORIENTATION_CALIB_H_
#define WAFER_PAD_ORIENTATION_CALIB_H_

#include <wafer_pad/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct wp_calib_config {
	uint8_t fingers;               /**< Default 3. */
	uint32_t hold_ms;              /**< Default 1500. */
	float hold_max_move_mm;        /**< Default 1.5. */
	uint32_t arm_timeout_ms;       /**< Default 2500. */
	float swipe_min_mm;            /**< Default 25. */
	uint32_t swipe_max_ms;         /**< Default 450. */
	float swipe_max_angle_dev_deg; /**< Default 30. */
	uint32_t lift_max_ms;          /**< Default 250. */
	float swipe_start_mm;          /**< Centroid movement that starts the swipe
					*   timer once armed. Default 2.0. */
};

void wp_calib_config_default(struct wp_calib_config *cfg);

enum wp_calib_phase {
	WP_CALIB_IDLE = 0,
	WP_CALIB_HOLDING,
	WP_CALIB_ARMED,
	WP_CALIB_SWIPING,
	WP_CALIB_WAIT_LIFT,
	WP_CALIB_REJECTED,  /**< Session rejected, waiting for all fingers up. */
};

enum wp_calib_event {
	WP_CALIB_EV_NONE = 0,
	WP_CALIB_EV_ARMED,
	WP_CALIB_EV_CANCELLED,  /**< Only emitted if the session had been ARMED. */
	WP_CALIB_EV_COMPLETED,
};

struct wp_calib_result {
	enum wp_calib_event event;
	enum wp_orientation orientation; /**< Valid for WP_CALIB_EV_COMPLETED. */
	float angle_deg;                 /**< Swipe angle per the table above. */
};

struct wp_calib {
	struct wp_calib_config cfg;
	struct wp_pad_geometry geom;     /**< Pad-frame geometry (for mm). */
	enum wp_calib_phase phase;
	int64_t phase_start_us;
	int64_t hold_start_us;
	float anchor_x_mm[WP_TOUCH_MAX_CONTACTS];
	float anchor_y_mm[WP_TOUCH_MAX_CONTACTS];
	uint8_t anchor_id[WP_TOUCH_MAX_CONTACTS];
	uint8_t anchor_count;
	float centroid0_x_mm, centroid0_y_mm;   /**< Centroid when armed. */
	float last_cx_mm, last_cy_mm;
	int64_t swipe_start_us;
	int64_t swipe_end_us;
	float swipe_dx_mm, swipe_dy_mm;
};

void wp_calib_init(struct wp_calib *c, const struct wp_calib_config *cfg,
		   const struct wp_pad_geometry *pad_geom);

void wp_calib_reset(struct wp_calib *c);

/** Feed a pad-frame touch frame. Always fills @p out. */
void wp_calib_update(struct wp_calib *c, const struct wp_touch_frame *pad_frame,
		     struct wp_calib_result *out);

/** Advance time without a new frame (hold/arm timeouts). */
void wp_calib_tick(struct wp_calib *c, int64_t t_us, struct wp_calib_result *out);

/** True while ARMED/SWIPING/WAIT_LIFT: the gesture engine must not scroll. */
bool wp_calib_inhibits_scroll(const struct wp_calib *c);

enum wp_calib_phase wp_calib_phase_get(const struct wp_calib *c);

#ifdef __cplusplus
}
#endif

#endif /* WAFER_PAD_ORIENTATION_CALIB_H_ */
