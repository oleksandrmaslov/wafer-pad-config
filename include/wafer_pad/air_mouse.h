/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Air mouse: gyroscope angular rate -> relative cursor motion (pure C).
 *
 * NO linear displacement is estimated (no double integration of the
 * accelerometer). Cursor motion is proportional to angular velocity:
 *   yaw   (turning left/right)   -> cursor X
 *   pitch (tilting forward edge) -> cursor Y
 *
 * Axes (body frame, see types.h). With fwd/right from
 * wp_orientation_axes_body() for the current orientation:
 *   gravity_compensated = false:
 *     yaw_rate   = gyro . z_B
 *     pitch_rate = gyro . right
 *   gravity_compensated = true (default): with up = normalized low-passed
 *   specific force,
 *     yaw_rate   = gyro . up
 *     pitch_rate = gyro . normalize(fwd x up)   (horizontal "right" axis)
 *   When |fwd x up| is too small (forward edge pointing nearly straight
 *   up/down) only the pitch axis degenerates: yaw stays about `up`, pitch
 *   uses normalize(right - (right . up) up). Without usable gravity
 *   (|up| < 0.5 g, free fall) both fall back to the uncompensated axes.
 * Cursor (HID, +x right, +y down):
 *     dx = -yaw_rate   * gain     (turning left = positive yaw about up)
 *     dy = -pitch_rate * gain     (forward edge up = positive about right)
 *
 * Processing chain per sample: axis projection -> 1-pole low-pass ->
 * soft deadzone (|w| <= dz -> 0, else sign(w)*(|w|-dz)) -> clamp ->
 * gain (counts/deg) -> ramp-in after start -> output (fractional counts).
 */

#ifndef WAFER_PAD_AIR_MOUSE_H_
#define WAFER_PAD_AIR_MOUSE_H_

#include <wafer_pad/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct wp_air_mouse_config {
	float counts_per_deg;       /**< Sensitivity. Default 12. */
	float deadzone_dps;         /**< Soft deadzone. Default 1.5. */
	float lpf_cutoff_hz;        /**< 1-pole low-pass cutoff. Default 15. */
	float max_rate_dps;         /**< Clamp after deadzone. Default 400. */
	uint32_t ramp_in_ms;        /**< Linear gain ramp after start. Default 120. */
	bool gravity_compensated;   /**< Default true. */
	float accel_lp_tau_ms;      /**< Gravity estimate time constant. Default 150. */
	bool invert_x;              /**< User preference. Default false. */
	bool invert_y;              /**< User preference. Default false. */
};

void wp_air_mouse_config_default(struct wp_air_mouse_config *cfg);

struct wp_air_mouse {
	struct wp_air_mouse_config cfg;
	enum wp_orientation orientation;
	bool active;
	int64_t start_us;
	int64_t last_t_us;
	float lp_yaw_dps;
	float lp_pitch_dps;
	bool lp_init;
	float up[3];
	bool up_init;
};

void wp_air_mouse_init(struct wp_air_mouse *am, const struct wp_air_mouse_config *cfg);

/** Set the orientation used for axis alignment. Resets filters. */
void wp_air_mouse_set_orientation(struct wp_air_mouse *am, enum wp_orientation o);

/** Activate (finger clutch pressed). Resets filters and starts the ramp-in. */
void wp_air_mouse_start(struct wp_air_mouse *am, int64_t t_us);

/** Deactivate. Output is exactly zero from now on. */
void wp_air_mouse_stop(struct wp_air_mouse *am);

bool wp_air_mouse_active(const struct wp_air_mouse *am);

/**
 * Feed a body-frame sample (gyro bias-corrected). The gravity estimate is
 * updated even while inactive (so it is ready at start). Writes fractional
 * HID counts to dx/dy; both are 0 when inactive or when !s->gyro_valid.
 */
void wp_air_mouse_update(struct wp_air_mouse *am, const struct wp_imu_sample *s, float *dx,
			 float *dy);

#ifdef __cplusplus
}
#endif

#endif /* WAFER_PAD_AIR_MOUSE_H_ */
