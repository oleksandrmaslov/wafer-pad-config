/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * orientation_transform: the ONE place where pad-frame quantities are mapped
 * into the user frame for a given discrete orientation (pure C).
 *
 * Every consumer (pointer motion, gesture recognition, scrolling, air mouse
 * axis alignment) MUST use these functions so that all paths agree.
 * Frame and orientation definitions: see <wafer_pad/types.h>.
 *
 *   P -> U delta mapping
 *     WP_ORIENT_0   : ( dx,  dy)
 *     WP_ORIENT_90  : (-dy,  dx)
 *     WP_ORIENT_180 : (-dx, -dy)
 *     WP_ORIENT_270 : ( dy, -dx)
 *
 *   P -> U absolute point mapping (W = max_x, H = max_y of the P frame)
 *     WP_ORIENT_0   : (x,     y)        extent (W, H)
 *     WP_ORIENT_90  : (H - y, x)        extent (H, W)
 *     WP_ORIENT_180 : (W - x, H - y)    extent (W, H)
 *     WP_ORIENT_270 : (y,     W - x)    extent (H, W)
 */

#ifndef WAFER_PAD_ORIENTATION_TRANSFORM_H_
#define WAFER_PAD_ORIENTATION_TRANSFORM_H_

#include <wafer_pad/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Normalize any integer to a valid orientation (modulo 4, negatives ok). */
enum wp_orientation wp_orientation_norm(int quarter_turns);

/** Nearest discrete orientation for an angle in degrees (any range). */
enum wp_orientation wp_orientation_from_deg(float deg);

/** a + b (composition of clockwise rotations). */
enum wp_orientation wp_orientation_add(enum wp_orientation a, enum wp_orientation b);

/** a - b. */
enum wp_orientation wp_orientation_sub(enum wp_orientation a, enum wp_orientation b);

/* ---- 2D touch quantities ------------------------------------------------ */

void wp_xform_delta_i(enum wp_orientation o, int32_t dx, int32_t dy, int32_t *ux, int32_t *uy);
void wp_xform_delta_f(enum wp_orientation o, float dx, float dy, float *ux, float *uy);

/** Inverse mapping U -> P (used by the calibration gesture). */
void wp_xform_delta_inv_f(enum wp_orientation o, float ux, float uy, float *dx, float *dy);

/** Map an absolute P point into U. Coordinates are clamped to the extent. */
void wp_xform_point(enum wp_orientation o, uint16_t max_x, uint16_t max_y, uint16_t x, uint16_t y,
		    uint16_t *ux, uint16_t *uy);

/** Extent of the U frame (swapped for 90/270). */
void wp_xform_extent(enum wp_orientation o, uint16_t max_x, uint16_t max_y, uint16_t *u_max_x,
		     uint16_t *u_max_y);

/** Geometry of the U frame (extent and millimetres swapped for 90/270). */
void wp_xform_geometry(enum wp_orientation o, const struct wp_pad_geometry *pad,
		       struct wp_pad_geometry *user);

/** Transform a whole P frame into U (contacts, extent; ids/strength kept).
 *  @p in and @p out must not alias. */
void wp_xform_frame(enum wp_orientation o, const struct wp_touch_frame *in,
		    struct wp_touch_frame *out);

/** Scroll direction convention. */
struct wp_scroll_config {
	bool natural;    /**< true: content follows the fingers (touchpad style). */
	bool invert_pan; /**< Extra inversion for horizontal pan only. */
};

/**
 * Map a two-finger movement given in the USER frame (mm or any unit, +x right,
 * +y towards user) to wheel/pan amounts in the same unit.
 *   natural = true : wheel = +uy,  pan = -ux
 *   natural = false: wheel = -uy,  pan = +ux
 *   invert_pan flips the sign of pan afterwards.
 * (HID wheel + = scroll up / away from user, AC Pan + = scroll right.)
 */
void wp_scroll_map(const struct wp_scroll_config *cfg, float ux, float uy, float *wheel, float *pan);

/** Convenience: P-frame finger delta -> wheel/pan for orientation @p o. */
void wp_xform_scroll(enum wp_orientation o, const struct wp_scroll_config *cfg, float dx, float dy,
		     float *wheel, float *pan);

/* ---- 3D IMU quantities -------------------------------------------------- */

/** Validate an IMU mount map (signed permutation of 1..3). */
bool wp_imu_mount_valid(const struct wp_imu_mount *m);

/** Rotate an IMU-frame vector into the body frame. @p in and @p out may alias. */
void wp_imu_mount_apply(const struct wp_imu_mount *m, const float in[3], float out[3]);

/**
 * Unit vectors in the BODY frame of the user's "forward" (away from the user)
 * and "right" directions for orientation @p o, assuming the pad lies in its
 * plane (z = 0 component):
 *   0: fwd=(0,+1,0) right=(+1,0,0)     90: fwd=(-1,0,0) right=(0,+1,0)
 *   180: fwd=(0,-1,0) right=(-1,0,0)   270: fwd=(+1,0,0) right=(0,-1,0)
 */
void wp_orientation_axes_body(enum wp_orientation o, float fwd[3], float right[3]);

#ifdef __cplusplus
}
#endif

#endif /* WAFER_PAD_ORIENTATION_TRANSFORM_H_ */
