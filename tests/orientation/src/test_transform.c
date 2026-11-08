/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * orientation_transform: P -> U mapping for all orientations, scroll mapping,
 * IMU mount and cross-module consistency (body axes, gravity, calibration).
 */

#include <zephyr/ztest.h>

#include "helpers.h"

#define EPS 1e-5f

static const enum wp_orientation all_o[4] = {WP_ORIENT_0, WP_ORIENT_90, WP_ORIENT_180,
					     WP_ORIENT_270};

ZTEST(orient_transform, test_norm_and_from_deg)
{
	zassert_equal(wp_orientation_norm(0), WP_ORIENT_0);
	zassert_equal(wp_orientation_norm(5), WP_ORIENT_90);
	zassert_equal(wp_orientation_norm(-1), WP_ORIENT_270);
	zassert_equal(wp_orientation_norm(-6), WP_ORIENT_180);

	zassert_equal(wp_orientation_from_deg(44.0f), WP_ORIENT_0);
	zassert_equal(wp_orientation_from_deg(46.0f), WP_ORIENT_90);
	zassert_equal(wp_orientation_from_deg(-90.0f), WP_ORIENT_270);
	zassert_equal(wp_orientation_from_deg(359.0f), WP_ORIENT_0);
	zassert_equal(wp_orientation_from_deg(-359.0f), WP_ORIENT_0);
	zassert_equal(wp_orientation_from_deg(180.0f + 720.0f), WP_ORIENT_180);
	zassert_equal(wp_orientation_from_deg(NAN), WP_ORIENT_0);

	zassert_equal(wp_orientation_add(WP_ORIENT_270, WP_ORIENT_180), WP_ORIENT_90);
	zassert_equal(wp_orientation_sub(WP_ORIENT_0, WP_ORIENT_90), WP_ORIENT_270);
	zassert_equal(wp_orientation_deg(WP_ORIENT_270), 270);
}

ZTEST(orient_transform, test_delta_axis_and_diagonal_all_orientations)
{
	/* Expected per types.h: 0:(dx,dy) 90:(-dy,dx) 180:(-dx,-dy) 270:(dy,-dx) */
	static const int32_t in[][2] = {{10, 0}, {0, 10}, {3, 7}, {-5, 2}, {-4, -9}};

	for (size_t i = 0; i < ARRAY_SIZE(in); i++) {
		int32_t dx = in[i][0], dy = in[i][1];
		const int32_t exp[4][2] = {{dx, dy}, {-dy, dx}, {-dx, -dy}, {dy, -dx}};

		for (int o = 0; o < 4; o++) {
			int32_t ux, uy;
			float fx, fy;

			wp_xform_delta_i(all_o[o], dx, dy, &ux, &uy);
			zassert_equal(ux, exp[o][0], "o=%d in=(%d,%d)", o, dx, dy);
			zassert_equal(uy, exp[o][1], "o=%d in=(%d,%d)", o, dx, dy);

			wp_xform_delta_f(all_o[o], (float)dx + 0.25f, (float)dy - 0.5f, &fx, &fy);
			float ex = (o == 0) ? dx + 0.25f
				   : (o == 1) ? -(dy - 0.5f)
				   : (o == 2) ? -(dx + 0.25f)
					      : (dy - 0.5f);
			float ey = (o == 0) ? dy - 0.5f
				   : (o == 1) ? dx + 0.25f
				   : (o == 2) ? -(dy - 0.5f)
					      : -(dx + 0.25f);

			zassert_within(fx, ex, EPS, "o=%d", o);
			zassert_within(fy, ey, EPS, "o=%d", o);
		}
	}
}

ZTEST(orient_transform, test_delta_preserves_length_and_is_rotation)
{
	/* Clockwise rotation of the PAD means the finger motion appears rotated
	 * clockwise in the user frame too (y down): 90 maps P +x to U +y.
	 */
	for (int o = 0; o < 4; o++) {
		float ux, uy;

		wp_xform_delta_f(all_o[o], 3.0f, 4.0f, &ux, &uy);
		zassert_within(sqrtf(ux * ux + uy * uy), 5.0f, EPS);
	}
	/* Composition: rotating by 90 twice equals 180. */
	float ax, ay, bx, by, cx, cy;

	wp_xform_delta_f(WP_ORIENT_90, 2.0f, -7.0f, &ax, &ay);
	wp_xform_delta_f(WP_ORIENT_90, ax, ay, &bx, &by);
	wp_xform_delta_f(WP_ORIENT_180, 2.0f, -7.0f, &cx, &cy);
	zassert_within(bx, cx, EPS);
	zassert_within(by, cy, EPS);
}

ZTEST(orient_transform, test_inverse_round_trip)
{
	static const float in[][2] = {{1.0f, 0.0f}, {0.0f, -1.0f}, {12.5f, -3.25f}, {-7.0f, 9.0f}};

	for (size_t i = 0; i < ARRAY_SIZE(in); i++) {
		for (int o = 0; o < 4; o++) {
			float ux, uy, dx, dy;

			wp_xform_delta_f(all_o[o], in[i][0], in[i][1], &ux, &uy);
			wp_xform_delta_inv_f(all_o[o], ux, uy, &dx, &dy);
			zassert_within(dx, in[i][0], EPS, "o=%d", o);
			zassert_within(dy, in[i][1], EPS, "o=%d", o);

			/* and the other way round */
			wp_xform_delta_inv_f(all_o[o], in[i][0], in[i][1], &dx, &dy);
			wp_xform_delta_f(all_o[o], dx, dy, &ux, &uy);
			zassert_within(ux, in[i][0], EPS, "o=%d", o);
			zassert_within(uy, in[i][1], EPS, "o=%d", o);
		}
	}
}

ZTEST(orient_transform, test_points_corners_extent)
{
	const uint16_t W = 100, H = 60;
	uint16_t ux, uy, ex, ey;

	/* Interior point per the header table. */
	static const uint16_t exp[4][2] = {{10, 20}, {40, 10}, {90, 40}, {20, 90}};

	for (int o = 0; o < 4; o++) {
		wp_xform_point(all_o[o], W, H, 10, 20, &ux, &uy);
		zassert_equal(ux, exp[o][0], "o=%d", o);
		zassert_equal(uy, exp[o][1], "o=%d", o);
	}

	/* The edge that is "away from the user" must map to U y == 0.
	 * 0: top (P y=0); 90: left (P x=0); 180: bottom (P y=H); 270: right (x=W).
	 */
	wp_xform_point(WP_ORIENT_0, W, H, 50, 0, &ux, &uy);
	zassert_equal(uy, 0);
	wp_xform_point(WP_ORIENT_90, W, H, 0, 30, &ux, &uy);
	zassert_equal(uy, 0);
	wp_xform_point(WP_ORIENT_180, W, H, 50, H, &ux, &uy);
	zassert_equal(uy, 0);
	wp_xform_point(WP_ORIENT_270, W, H, W, 30, &ux, &uy);
	zassert_equal(uy, 0);

	/* Corner P(0,0) for 90: on the left (away) edge and the top edge, which
	 * is at the user's right -> U top-right corner.
	 */
	wp_xform_point(WP_ORIENT_90, W, H, 0, 0, &ux, &uy);
	zassert_equal(ux, H);
	zassert_equal(uy, 0);
	wp_xform_point(WP_ORIENT_180, W, H, 0, 0, &ux, &uy);
	zassert_equal(ux, W);
	zassert_equal(uy, H);
	wp_xform_point(WP_ORIENT_270, W, H, 0, 0, &ux, &uy);
	zassert_equal(ux, 0);
	zassert_equal(uy, W);

	/* Clamping of out-of-range input. */
	wp_xform_point(WP_ORIENT_180, W, H, 500, 500, &ux, &uy);
	zassert_equal(ux, 0);
	zassert_equal(uy, 0);

	/* Extent */
	wp_xform_extent(WP_ORIENT_0, W, H, &ex, &ey);
	zassert_true(ex == W && ey == H);
	wp_xform_extent(WP_ORIENT_90, W, H, &ex, &ey);
	zassert_true(ex == H && ey == W);
	wp_xform_extent(WP_ORIENT_180, W, H, &ex, &ey);
	zassert_true(ex == W && ey == H);
	wp_xform_extent(WP_ORIENT_270, W, H, &ex, &ey);
	zassert_true(ex == H && ey == W);
}

ZTEST(orient_transform, test_point_and_delta_mappings_agree)
{
	const uint16_t W = 2047, H = 1535;
	static const uint16_t pts[][2] = {{0, 0}, {100, 1500}, {2047, 7}, {1023, 768}};

	for (int o = 0; o < 4; o++) {
		for (size_t a = 0; a < ARRAY_SIZE(pts); a++) {
			for (size_t b = 0; b < ARRAY_SIZE(pts); b++) {
				uint16_t ax, ay, bx, by;
				int32_t ux, uy;

				wp_xform_point(all_o[o], W, H, pts[a][0], pts[a][1], &ax, &ay);
				wp_xform_point(all_o[o], W, H, pts[b][0], pts[b][1], &bx, &by);
				wp_xform_delta_i(all_o[o], (int32_t)pts[b][0] - pts[a][0],
						 (int32_t)pts[b][1] - pts[a][1], &ux, &uy);
				zassert_equal((int32_t)bx - ax, ux, "o=%d", o);
				zassert_equal((int32_t)by - ay, uy, "o=%d", o);
			}
		}
	}
}

ZTEST(orient_transform, test_geometry)
{
	const struct wp_pad_geometry pad = {
		.max_x = 2047, .max_y = 1535, .width_mm = 65.0f, .height_mm = 49.0f};
	struct wp_pad_geometry u;

	for (int o = 0; o < 4; o++) {
		wp_xform_geometry(all_o[o], &pad, &u);
		bool swap = (o == 1 || o == 3);

		zassert_equal(u.max_x, swap ? pad.max_y : pad.max_x);
		zassert_equal(u.max_y, swap ? pad.max_x : pad.max_y);
		zassert_within(u.width_mm, swap ? pad.height_mm : pad.width_mm, EPS);
		zassert_within(u.height_mm, swap ? pad.width_mm : pad.height_mm, EPS);
	}
}

ZTEST(orient_transform, test_whole_frame)
{
	struct wp_touch_frame in = frame2(MS(42), 100, 60, 10, 20, 70, 5);
	struct wp_touch_frame out;

	in.contacts[0].area = 7;
	in.contacts[1].strength = 1234;

	for (int o = 0; o < 4; o++) {
		memset(&out, 0xA5, sizeof(out));
		wp_xform_frame(all_o[o], &in, &out);

		zassert_equal(out.t_us, MS(42));
		zassert_equal(out.count, 2);
		uint16_t ex, ey;

		wp_xform_extent(all_o[o], 100, 60, &ex, &ey);
		zassert_true(out.max_x == ex && out.max_y == ey);
		for (int i = 0; i < 2; i++) {
			uint16_t px, py;

			wp_xform_point(all_o[o], 100, 60, in.contacts[i].x, in.contacts[i].y, &px,
				       &py);
			zassert_equal(out.contacts[i].x, px);
			zassert_equal(out.contacts[i].y, py);
			zassert_equal(out.contacts[i].id, in.contacts[i].id);
			zassert_equal(out.contacts[i].strength, in.contacts[i].strength);
			zassert_equal(out.contacts[i].area, in.contacts[i].area);
		}
		for (int i = 2; i < WP_TOUCH_MAX_CONTACTS; i++) {
			zassert_equal(out.contacts[i].id, 0);
			zassert_equal(out.contacts[i].strength, 0);
		}
	}

	/* An out-of-range count is clamped. */
	in.count = 9;
	wp_xform_frame(WP_ORIENT_90, &in, &out);
	zassert_equal(out.count, WP_TOUCH_MAX_CONTACTS);
}

ZTEST(orient_transform, test_scroll_all_orientations)
{
	const struct wp_scroll_config natural = {.natural = true, .invert_pan = false};
	const struct wp_scroll_config trad = {.natural = false, .invert_pan = false};
	const struct wp_scroll_config nat_inv = {.natural = true, .invert_pan = true};
	float wheel, pan;

	/* Base mapping in the user frame. */
	wp_scroll_map(&natural, 0.0f, 10.0f, &wheel, &pan);
	zassert_within(wheel, 10.0f, EPS);
	zassert_within(pan, 0.0f, EPS);
	wp_scroll_map(&natural, 10.0f, 0.0f, &wheel, &pan);
	zassert_within(pan, -10.0f, EPS);
	wp_scroll_map(&trad, 0.0f, 10.0f, &wheel, &pan);
	zassert_within(wheel, -10.0f, EPS);
	wp_scroll_map(&trad, 10.0f, 0.0f, &wheel, &pan);
	zassert_within(pan, 10.0f, EPS);
	wp_scroll_map(&nat_inv, 10.0f, 0.0f, &wheel, &pan);
	zassert_within(pan, 10.0f, EPS);
	wp_scroll_map(NULL, 0.0f, 10.0f, &wheel, &pan);
	zassert_within(wheel, 10.0f, EPS, "NULL config = natural");

	/* For every orientation: fingers moving TOWARDS THE USER (U +y) and to
	 * the USER'S RIGHT (U +x) must scroll identically, whatever pad axis
	 * that motion is along.
	 */
	for (int o = 0; o < 4; o++) {
		float dx, dy;

		wp_xform_delta_inv_f(all_o[o], 0.0f, 8.0f, &dx, &dy);
		wp_xform_scroll(all_o[o], &natural, dx, dy, &wheel, &pan);
		zassert_within(wheel, 8.0f, EPS, "o=%d", o);
		zassert_within(pan, 0.0f, EPS, "o=%d", o);
		wp_xform_scroll(all_o[o], &trad, dx, dy, &wheel, &pan);
		zassert_within(wheel, -8.0f, EPS, "o=%d", o);

		wp_xform_delta_inv_f(all_o[o], 6.0f, 0.0f, &dx, &dy);
		wp_xform_scroll(all_o[o], &natural, dx, dy, &wheel, &pan);
		zassert_within(wheel, 0.0f, EPS, "o=%d", o);
		zassert_within(pan, -6.0f, EPS, "o=%d", o);
		wp_xform_scroll(all_o[o], &trad, dx, dy, &wheel, &pan);
		zassert_within(pan, 6.0f, EPS, "o=%d", o);
		wp_xform_scroll(all_o[o], &nat_inv, dx, dy, &wheel, &pan);
		zassert_within(pan, 6.0f, EPS, "o=%d", o);
	}

	/* Concrete case: orientation 90, fingers move towards P bottom edge (+y_P)
	 * = to the user's LEFT -> natural pan is positive (content follows).
	 */
	wp_xform_scroll(WP_ORIENT_90, &natural, 0.0f, 10.0f, &wheel, &pan);
	zassert_within(wheel, 0.0f, EPS);
	zassert_within(pan, 10.0f, EPS);
}

ZTEST(orient_transform, test_imu_mount)
{
	const struct wp_imu_mount ok[] = {
		{{1, 2, 3}}, {{-2, 1, 3}}, {{3, -1, -2}}, {{-1, -2, -3}}};
	const struct wp_imu_mount bad[] = {
		{{1, 1, 3}}, {{0, 2, 3}}, {{4, 2, 3}}, {{1, 2, -2}}, {{-3, 3, 1}}};

	for (size_t i = 0; i < ARRAY_SIZE(ok); i++) {
		zassert_true(wp_imu_mount_valid(&ok[i]), "ok[%u]", (unsigned int)i);
	}
	for (size_t i = 0; i < ARRAY_SIZE(bad); i++) {
		zassert_false(wp_imu_mount_valid(&bad[i]), "bad[%u]", (unsigned int)i);
	}

	const float v[3] = {1.0f, 2.0f, 3.0f};
	float out[3];

	wp_imu_mount_apply(&ok[1], v, out);
	zassert_within(out[0], -2.0f, EPS);
	zassert_within(out[1], 1.0f, EPS);
	zassert_within(out[2], 3.0f, EPS);

	wp_imu_mount_apply(&ok[2], v, out);
	zassert_within(out[0], 3.0f, EPS);
	zassert_within(out[1], -1.0f, EPS);
	zassert_within(out[2], -2.0f, EPS);

	/* In-place (in == out) must not corrupt the permutation. */
	float io[3] = {1.0f, 2.0f, 3.0f};

	wp_imu_mount_apply(&ok[2], io, io);
	zassert_within(io[0], 3.0f, EPS);
	zassert_within(io[1], -1.0f, EPS);
	zassert_within(io[2], -2.0f, EPS);

	/* Invalid entries fall back to identity for that axis. */
	const struct wp_imu_mount z = {{0, -1, 3}};

	wp_imu_mount_apply(&z, v, out);
	zassert_within(out[0], 1.0f, EPS);
	zassert_within(out[1], -1.0f, EPS);
	zassert_within(out[2], 3.0f, EPS);
}

ZTEST(orient_transform, test_axes_body_consistent_with_delta)
{
	for (int o = 0; o < 4; o++) {
		float fwd[3], right[3], ux, uy;

		wp_orientation_axes_body(all_o[o], fwd, right);
		zassert_within(fwd[2], 0.0f, EPS);
		zassert_within(right[2], 0.0f, EPS);
		/* Right-handed B: right x fwd = +z_B (z component == 1). */
		zassert_within(right[0] * fwd[1] - right[1] * fwd[0], 1.0f, EPS, "o=%d", o);

		/* Body -> pad: x_P = x_B, y_P = -y_B. Motion towards the forward
		 * edge must be "away from the user" = U -y.
		 */
		wp_xform_delta_f(all_o[o], fwd[0], -fwd[1], &ux, &uy);
		zassert_within(ux, 0.0f, EPS, "o=%d", o);
		zassert_within(uy, -1.0f, EPS, "o=%d", o);

		wp_xform_delta_f(all_o[o], right[0], -right[1], &ux, &uy);
		zassert_within(ux, 1.0f, EPS, "o=%d", o);
		zassert_within(uy, 0.0f, EPS, "o=%d", o);
	}
}

ZTEST(orient_transform, test_gravity_and_calib_conventions_match_transform)
{
	/* (1) Raising the edge that is "forward" for orientation o makes the
	 * gravity estimator report o.
	 */
	for (int o = 0; o < 4; o++) {
		float fwd[3], right[3];
		struct wp_orient_gravity g;
		struct wp_orient_gravity_result r = {0};
		const float tilt = 30.0f * WP_DEG_TO_RAD;

		wp_orientation_axes_body(all_o[o], fwd, right);
		wp_orient_gravity_init(&g, NULL, WP_ORIENT_0);
		for (int k = 0; k <= 40; k++) {
			struct wp_imu_sample s = {.t_us = MS(k * T_IMU_MS), .gyro_valid = true};

			for (int i = 0; i < 3; i++) {
				s.accel[i] = WP_GRAVITY_MS2 * sinf(tilt) * fwd[i];
			}
			s.accel[2] += WP_GRAVITY_MS2 * cosf(tilt);
			wp_orient_gravity_update(&g, &s, &r);
		}
		zassert_true(r.confirmed, "o=%d", o);
		zassert_equal(r.candidate, all_o[o], "o=%d", o);
		zassert_within(r.heading_deg, (float)wp_orientation_deg(all_o[o]), 0.01f);
	}

	/* (2) A calibration swipe "away from the user" while the pad is in
	 * orientation o is classified as o by the documented angle formula.
	 */
	for (int o = 0; o < 4; o++) {
		float dx, dy;

		wp_xform_delta_inv_f(all_o[o], 0.0f, -1.0f, &dx, &dy);
		float angle = atan2f(-dx, -dy) * WP_RAD_TO_DEG;

		zassert_equal(wp_orientation_from_deg(angle), all_o[o], "o=%d", o);
	}
}

ZTEST_SUITE(orient_transform, NULL, NULL, NULL, NULL, NULL);
