/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * orientation_transform: pad frame (P) -> user frame (U) mapping for the four
 * discrete orientations. Pure C, no Zephyr dependencies (host-testable).
 * The mapping tables are documented in <wafer_pad/orientation_transform.h>.
 */

#include <math.h>

#include <wafer_pad/orientation_transform.h>

enum wp_orientation wp_orientation_norm(int quarter_turns)
{
	int m = quarter_turns % WP_ORIENT_COUNT;

	if (m < 0) {
		m += WP_ORIENT_COUNT;
	}
	return (enum wp_orientation)m;
}

enum wp_orientation wp_orientation_from_deg(float deg)
{
	if (!isfinite(deg)) {
		return WP_ORIENT_0;
	}
	/* Reduce first so that the int conversion cannot overflow. */
	float r = fmodf(deg, 360.0f);
	int q = (int)floorf(r / 90.0f + 0.5f);

	return wp_orientation_norm(q);
}

enum wp_orientation wp_orientation_add(enum wp_orientation a, enum wp_orientation b)
{
	return wp_orientation_norm((int)a + (int)b);
}

enum wp_orientation wp_orientation_sub(enum wp_orientation a, enum wp_orientation b)
{
	return wp_orientation_norm((int)a - (int)b);
}

void wp_xform_delta_i(enum wp_orientation o, int32_t dx, int32_t dy, int32_t *ux, int32_t *uy)
{
	int32_t x, y;

	switch (wp_orientation_norm((int)o)) {
	case WP_ORIENT_90:
		x = -dy;
		y = dx;
		break;
	case WP_ORIENT_180:
		x = -dx;
		y = -dy;
		break;
	case WP_ORIENT_270:
		x = dy;
		y = -dx;
		break;
	case WP_ORIENT_0:
	default:
		x = dx;
		y = dy;
		break;
	}
	*ux = x;
	*uy = y;
}

void wp_xform_delta_f(enum wp_orientation o, float dx, float dy, float *ux, float *uy)
{
	float x, y;

	switch (wp_orientation_norm((int)o)) {
	case WP_ORIENT_90:
		x = -dy;
		y = dx;
		break;
	case WP_ORIENT_180:
		x = -dx;
		y = -dy;
		break;
	case WP_ORIENT_270:
		x = dy;
		y = -dx;
		break;
	case WP_ORIENT_0:
	default:
		x = dx;
		y = dy;
		break;
	}
	*ux = x;
	*uy = y;
}

void wp_xform_delta_inv_f(enum wp_orientation o, float ux, float uy, float *dx, float *dy)
{
	/* The inverse of a clockwise rotation by o is a rotation by (4 - o). */
	wp_xform_delta_f(wp_orientation_sub(WP_ORIENT_0, o), ux, uy, dx, dy);
}

void wp_xform_point(enum wp_orientation o, uint16_t max_x, uint16_t max_y, uint16_t x, uint16_t y,
		    uint16_t *ux, uint16_t *uy)
{
	uint16_t px = (x > max_x) ? max_x : x;
	uint16_t py = (y > max_y) ? max_y : y;
	uint16_t rx, ry;

	switch (wp_orientation_norm((int)o)) {
	case WP_ORIENT_90:
		rx = (uint16_t)(max_y - py);
		ry = px;
		break;
	case WP_ORIENT_180:
		rx = (uint16_t)(max_x - px);
		ry = (uint16_t)(max_y - py);
		break;
	case WP_ORIENT_270:
		rx = py;
		ry = (uint16_t)(max_x - px);
		break;
	case WP_ORIENT_0:
	default:
		rx = px;
		ry = py;
		break;
	}
	*ux = rx;
	*uy = ry;
}

void wp_xform_extent(enum wp_orientation o, uint16_t max_x, uint16_t max_y, uint16_t *u_max_x,
		     uint16_t *u_max_y)
{
	enum wp_orientation n = wp_orientation_norm((int)o);

	if (n == WP_ORIENT_90 || n == WP_ORIENT_270) {
		*u_max_x = max_y;
		*u_max_y = max_x;
	} else {
		*u_max_x = max_x;
		*u_max_y = max_y;
	}
}

void wp_xform_geometry(enum wp_orientation o, const struct wp_pad_geometry *pad,
		       struct wp_pad_geometry *user)
{
	struct wp_pad_geometry g = *pad;
	enum wp_orientation n = wp_orientation_norm((int)o);

	if (n == WP_ORIENT_90 || n == WP_ORIENT_270) {
		g.max_x = pad->max_y;
		g.max_y = pad->max_x;
		g.width_mm = pad->height_mm;
		g.height_mm = pad->width_mm;
	}
	*user = g;
}

void wp_xform_frame(enum wp_orientation o, const struct wp_touch_frame *in,
		    struct wp_touch_frame *out)
{
	uint8_t count = (in->count > WP_TOUCH_MAX_CONTACTS) ? WP_TOUCH_MAX_CONTACTS : in->count;

	out->t_us = in->t_us;
	out->count = count;
	wp_xform_extent(o, in->max_x, in->max_y, &out->max_x, &out->max_y);

	for (uint8_t i = 0; i < count; i++) {
		const struct wp_touch_contact *c = &in->contacts[i];
		struct wp_touch_contact *d = &out->contacts[i];

		d->id = c->id;
		d->strength = c->strength;
		d->area = c->area;
		wp_xform_point(o, in->max_x, in->max_y, c->x, c->y, &d->x, &d->y);
	}
	for (uint8_t i = count; i < WP_TOUCH_MAX_CONTACTS; i++) {
		out->contacts[i] = (struct wp_touch_contact){0};
	}
}

void wp_scroll_map(const struct wp_scroll_config *cfg, float ux, float uy, float *wheel, float *pan)
{
	float w, p;

	if (cfg != NULL && !cfg->natural) {
		w = -uy;
		p = ux;
	} else {
		/* Natural (touchpad) scrolling: content follows the fingers.
		 * Fingers towards the user (+uy) pull the content down, i.e. the
		 * view scrolls up (+wheel). Fingers to the right pull the content
		 * right, i.e. the view scrolls left (-pan).
		 */
		w = uy;
		p = -ux;
	}
	if (cfg != NULL && cfg->invert_pan) {
		p = -p;
	}
	*wheel = w;
	*pan = p;
}

void wp_xform_scroll(enum wp_orientation o, const struct wp_scroll_config *cfg, float dx, float dy,
		     float *wheel, float *pan)
{
	float ux, uy;

	wp_xform_delta_f(o, dx, dy, &ux, &uy);
	wp_scroll_map(cfg, ux, uy, wheel, pan);
}

bool wp_imu_mount_valid(const struct wp_imu_mount *m)
{
	bool used[3] = {false, false, false};

	for (int i = 0; i < 3; i++) {
		int v = m->map[i];
		int a = (v < 0) ? -v : v;

		if (a < 1 || a > 3 || used[a - 1]) {
			return false;
		}
		used[a - 1] = true;
	}
	return true;
}

void wp_imu_mount_apply(const struct wp_imu_mount *m, const float in[3], float out[3])
{
	float tmp[3];

	for (int i = 0; i < 3; i++) {
		int v = m->map[i];
		int a = (v < 0) ? -v : v;

		if (a < 1 || a > 3) {
			/* Invalid map: fall back to identity for this axis. */
			tmp[i] = in[i];
			continue;
		}
		tmp[i] = (v < 0) ? -in[a - 1] : in[a - 1];
	}
	out[0] = tmp[0];
	out[1] = tmp[1];
	out[2] = tmp[2];
}

void wp_orientation_axes_body(enum wp_orientation o, float fwd[3], float right[3])
{
	static const float fwd_tab[WP_ORIENT_COUNT][3] = {
		{0.0f, 1.0f, 0.0f},  /* 0:   top edge away    -> fwd = +y_B */
		{-1.0f, 0.0f, 0.0f}, /* 90:  left edge away   -> fwd = -x_B */
		{0.0f, -1.0f, 0.0f}, /* 180: bottom edge away -> fwd = -y_B */
		{1.0f, 0.0f, 0.0f},  /* 270: right edge away  -> fwd = +x_B */
	};
	static const float right_tab[WP_ORIENT_COUNT][3] = {
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{-1.0f, 0.0f, 0.0f},
		{0.0f, -1.0f, 0.0f},
	};
	int n = (int)wp_orientation_norm((int)o);

	for (int i = 0; i < 3; i++) {
		fwd[i] = fwd_tab[n][i];
		right[i] = right_tab[n][i];
	}
}
