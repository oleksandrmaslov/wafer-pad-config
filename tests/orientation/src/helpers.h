/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared helpers for the orientation test suites (synthetic IMU samples and
 * touch frames).
 */

#ifndef WP_TEST_ORIENTATION_HELPERS_H_
#define WP_TEST_ORIENTATION_HELPERS_H_

#include <math.h>
#include <string.h>

#include <wafer_pad/types.h>
#include <wafer_pad/orientation.h>
#include <wafer_pad/orientation_calib.h>
#include <wafer_pad/orientation_transform.h>

#define MS(x) ((int64_t)(x) * 1000LL)

/** IMU sample period used by the tests. */
#define T_IMU_MS 10

/**
 * Specific force for a pad tilted by @p tilt_deg whose raised (uphill) edge
 * lies at gravity heading @p heading_deg (0 = top edge raised, 90 = left,
 * 180 = bottom, 270 = right; see orientation.h), magnitude @p mag_g * g.
 * uphill unit vector in the pad plane = (-sin h, cos h).
 */
static inline struct wp_imu_sample imu_tilt(int64_t t_us, float tilt_deg, float heading_deg,
					    float mag_g)
{
	struct wp_imu_sample s;
	float t = tilt_deg * WP_DEG_TO_RAD;
	float h = heading_deg * WP_DEG_TO_RAD;
	float g = mag_g * WP_GRAVITY_MS2;

	memset(&s, 0, sizeof(s));
	s.t_us = t_us;
	s.accel[0] = g * sinf(t) * -sinf(h);
	s.accel[1] = g * sinf(t) * cosf(h);
	s.accel[2] = g * cosf(t);
	s.gyro_valid = true;
	return s;
}

static inline struct wp_imu_sample imu_flat(int64_t t_us)
{
	return imu_tilt(t_us, 0.0f, 0.0f, 1.0f);
}

/** Two-finger frame (ids 1 and 2). */
static inline struct wp_touch_frame frame2(int64_t t_us, uint16_t max_x, uint16_t max_y,
					   uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	struct wp_touch_frame f;

	memset(&f, 0, sizeof(f));
	f.t_us = t_us;
	f.max_x = max_x;
	f.max_y = max_y;
	f.count = 2;
	f.contacts[0] = (struct wp_touch_contact){.id = 1, .x = x1, .y = y1, .strength = 500};
	f.contacts[1] = (struct wp_touch_contact){.id = 2, .x = x2, .y = y2, .strength = 500};
	return f;
}

static inline struct wp_touch_frame frame_n(int64_t t_us, uint16_t max_x, uint16_t max_y,
					    uint8_t n, const uint16_t *xy)
{
	struct wp_touch_frame f;

	memset(&f, 0, sizeof(f));
	f.t_us = t_us;
	f.max_x = max_x;
	f.max_y = max_y;
	f.count = n;
	for (uint8_t i = 0; i < n; i++) {
		f.contacts[i] = (struct wp_touch_contact){
			.id = (uint8_t)(i + 1), .x = xy[2 * i], .y = xy[2 * i + 1],
			.strength = 500};
	}
	return f;
}

static inline struct wp_touch_frame frame0(int64_t t_us, uint16_t max_x, uint16_t max_y)
{
	return frame_n(t_us, max_x, max_y, 0, NULL);
}

/**
 * Calibration config used by the phase-machine tests: 2 fingers, 700 ms hold,
 * 15 mm swipe (the pre-v0.1.1 defaults). The mechanism is the same for any
 * finger count; the shipped defaults (3 fingers, 1.5 s, 25 mm) are covered
 * by test_defaults and the router-chain tests in test_calib_chain.c.
 */
static inline void calib_mech_config(struct wp_calib_config *cfg)
{
	wp_calib_config_default(cfg);
	cfg->fingers = 2U;
	cfg->hold_ms = 700U;
	cfg->swipe_min_mm = 15.0f;
}

#endif /* WP_TEST_ORIENTATION_HELPERS_H_ */
