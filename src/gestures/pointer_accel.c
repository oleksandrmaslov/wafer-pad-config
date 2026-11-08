/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pointer acceleration (pure C, host-testable).
 *
 * Piecewise-linear, monotonic, continuous speed -> gain curve:
 *   v <= low_speed_mm_s  : mult = 1
 *   v >= high_speed_mm_s : mult = max_multiplier
 *   between              : linear interpolation
 *   counts = mm * counts_per_mm * sensitivity * mult
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include <wafer_pad/gestures.h>

void wp_pointer_accel_config_default(struct wp_pointer_accel_config *cfg)
{
	cfg->counts_per_mm = 16.0f;
	cfg->sensitivity = 1.0f;
	cfg->low_speed_mm_s = 20.0f;
	cfg->high_speed_mm_s = 200.0f;
	cfg->max_multiplier = 3.0f;
}

float wp_pointer_accel_multiplier(const struct wp_pointer_accel_config *cfg, float speed_mm_s)
{
	float span;

	/* "!(v > low)" also catches NaN: unknown speed -> no acceleration. */
	if (!(speed_mm_s > cfg->low_speed_mm_s)) {
		return 1.0f;
	}
	if (speed_mm_s >= cfg->high_speed_mm_s) {
		return cfg->max_multiplier;
	}
	/* Only reachable when high > low, so span > 0. */
	span = cfg->high_speed_mm_s - cfg->low_speed_mm_s;
	if (!(span > 0.0f)) {
		return cfg->max_multiplier;
	}
	return 1.0f + (cfg->max_multiplier - 1.0f) * ((speed_mm_s - cfg->low_speed_mm_s) / span);
}

void wp_pointer_accel_apply(const struct wp_pointer_accel_config *cfg, float dx_mm, float dy_mm,
			    float dt_s, float *dx_counts, float *dy_counts)
{
	float mult = 1.0f;
	float scale;

	if (dt_s > 0.0f) {
		float dist = sqrtf(dx_mm * dx_mm + dy_mm * dy_mm);

		mult = wp_pointer_accel_multiplier(cfg, dist / dt_s);
	}
	scale = cfg->counts_per_mm * cfg->sensitivity * mult;
	*dx_counts = dx_mm * scale;
	*dy_counts = dy_mm * scale;
}
