/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * "wp" shell commands (debugging / exhibition support). Built only with
 * CONFIG_SHELL. Every command goes through the public router / platform
 * APIs; requests are queued to the router thread, nothing here touches
 * algorithm state directly. Floats are printed as fixed point (no
 * CONFIG_CBPRINTF_FP_SUPPORT needed).
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <wafer_pad/orientation.h>
#include <wafer_pad/platform.h>
#include <wafer_pad/state_machine.h>
#include <wafer_pad/types.h>

/* Value x10, rounded, for "%d.%d" printing. */
static int tenths(float x)
{
	return (int)(x * 10.0f + (x >= 0.0f ? 0.5f : -0.5f));
}

static void print_fixed1(const struct shell *sh, const char *label, float x, const char *unit)
{
	int v = tenths(x);
	int a = v < 0 ? -v : v;

	shell_print(sh, "%-14s %s%d.%d %s", label, v < 0 ? "-" : "", a / 10, a % 10, unit);
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	struct wp_router_status st;
	int mv;
	int pct;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	wp_router_get_status(&st);

	shell_print(sh, "%-14s %s", "state", wp_state_name((enum wp_state)st.state));
	shell_print(sh, "%-14s %s", "mode", wp_mode_name((enum wp_mode_override)st.mode));
	shell_print(sh, "%-14s %d deg (%s, confidence %d%%)", "orientation", st.orientation_deg,
		    wp_orient_source_name((enum wp_orient_source)st.orientation_source),
		    (int)(st.orientation_confidence * 100.0f + 0.5f));
	print_fixed1(sh, "tilt", st.tilt_deg, "deg");
	print_fixed1(sh, "heading", st.heading_deg, "deg");
	shell_print(sh, "%-14s %u", "contacts", st.contacts);
	shell_print(sh, "%-14s %s", "hid", st.hid_connected ? "connected" : "not connected");
	shell_print(sh, "%-14s %s, gyro %s", "imu", st.imu_present ? "present" : "absent",
		    st.gyro_on ? "on" : "off");
	shell_print(sh, "%-14s touch %u, imu %u, reports %u, drops %u", "counters",
		    st.touch_frames, st.imu_samples, st.reports_sent, st.queue_drops);
	shell_print(sh, "%-14s %u us", "max latency", st.max_latency_us);

	mv = wp_power_battery_mv();
	pct = wp_power_battery_percent();
	if (mv >= 0) {
		/* On USB without a battery VDDH is the charger output: not a SoC. */
		shell_print(sh, "%-14s %d mV (%d%%)", "battery", mv, pct >= 0 ? pct : 0);
	} else {
		shell_print(sh, "%-14s n/a (%d)", "battery", mv);
	}
	return 0;
}

static int cmd_orient(const struct shell *sh, size_t argc, char **argv)
{
	int deg;
	int err;

	ARG_UNUSED(argc);

	if (strcmp(argv[1], "auto") == 0) {
		deg = -1;
	} else {
		char *end = NULL;
		long v = strtol(argv[1], &end, 10);

		if (end == argv[1] || *end != '\0' ||
		    (v != 0 && v != 90 && v != 180 && v != 270)) {
			shell_error(sh, "usage: wp orient <0|90|180|270|auto>");
			return -EINVAL;
		}
		deg = (int)v;
	}

	err = wp_router_request_orientation(deg);
	if (err != 0) {
		shell_error(sh, "request failed (%d)", err);
		return err;
	}
	if (deg < 0) {
		shell_print(sh, "orientation: automatic detection");
	} else {
		shell_print(sh, "orientation forced to %d deg (applied when no finger is down)",
			    deg);
	}
	return 0;
}

static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
	static const char *const names[] = {"auto", "handheld", "trackpad"};
	int mode = -1;
	int err;

	ARG_UNUSED(argc);

	for (int i = 0; i < (int)ARRAY_SIZE(names); i++) {
		if (strcmp(argv[1], names[i]) == 0) {
			mode = i; /* enum wp_mode_override order */
		}
	}
	if (mode < 0) {
		shell_error(sh, "usage: wp mode <auto|handheld|trackpad>");
		return -EINVAL;
	}

	err = wp_router_request_mode(mode);
	if (err != 0) {
		shell_error(sh, "request failed (%d)", err);
		return err;
	}
	shell_print(sh, "mode -> %s", names[mode]);
	return 0;
}

static int cmd_unpair(const struct shell *sh, size_t argc, char **argv)
{
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	err = wp_ble_unpair_all();
	if (err != 0) {
		shell_error(sh, "unpair failed (%d)", err);
		return err;
	}
	shell_print(sh, "all bonds removed, advertising");
	return 0;
}

static int cmd_bias(const struct shell *sh, size_t argc, char **argv)
{
	float b[3];

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!wp_imu_source_get_bias(b)) {
		shell_print(sh, "gyro bias not estimated yet (keep the pad still ~1 s, gyro on)");
		return 0;
	}

	/* rad/s -> milli-degrees per second */
	shell_print(sh, "gyro bias [mdps] x %d  y %d  z %d (body frame)",
		    (int)(b[0] * WP_RAD_TO_DEG * 1000.0f), (int)(b[1] * WP_RAD_TO_DEG * 1000.0f),
		    (int)(b[2] * WP_RAD_TO_DEG * 1000.0f));
	return 0;
}

static int cmd_touch(const struct shell *sh, size_t argc, char **argv)
{
	static uint32_t last_frames;
	static uint32_t last_imu;
	static int64_t last_ms;
	struct wp_router_status st;
	int64_t now = k_uptime_get();
	int64_t dt;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	wp_router_get_status(&st);
	dt = now - last_ms;

	shell_print(sh, "%-14s %u", "contacts", st.contacts);
	shell_print(sh, "%-14s %u", "touch frames", st.touch_frames);
	shell_print(sh, "%-14s %u", "imu samples", st.imu_samples);
	if (last_ms != 0 && dt > 0) {
		shell_print(sh, "%-14s touch %u Hz, imu %u Hz (since last 'wp touch')", "rates",
			    (uint32_t)(((uint64_t)(st.touch_frames - last_frames) * 1000U) /
				       (uint64_t)dt),
			    (uint32_t)(((uint64_t)(st.imu_samples - last_imu) * 1000U) /
				       (uint64_t)dt));
	}
	shell_print(sh, "%-14s %u", "reports sent", st.reports_sent);
	shell_print(sh, "%-14s %u", "queue drops", st.queue_drops);
	shell_print(sh, "%-14s %u us", "max latency", st.max_latency_us);

	last_frames = st.touch_frames;
	last_imu = st.imu_samples;
	last_ms = now;
	return 0;
}

/* Value x100, rounded, printed as [-]a.bc */
static void print_fixed2(const struct shell *sh, const char *label, float x, const char *unit)
{
	int v = (int)(x * 100.0f + (x >= 0.0f ? 0.5f : -0.5f));
	int a = v < 0 ? -v : v;

	shell_print(sh, "%-14s %s%d.%02d %s", label, v < 0 ? "-" : "", a / 100, a % 100, unit);
}

/* Which pad edge (body frame) is raised for a gravity heading (orientation.h). */
static const char *raised_edge(float heading_deg)
{
	static const char *const names[] = {"top (+y)", "left (-x)", "bottom (-y)", "right (+x)"};
	int q = (int)((heading_deg + 45.0f) / 90.0f);

	return names[((q % 4) + 4) % 4];
}

static int cmd_imu(const struct shell *sh, size_t argc, char **argv)
{
	static const char *const ax[] = {"x", "y", "z"};
	struct wp_router_status st;
	char label[16];

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	wp_router_get_status(&st);
	if (!st.imu_present && !st.imu_sample_valid) {
		shell_print(sh, "IMU absent");
		return 0;
	}
	if (!st.imu_sample_valid) {
		shell_print(sh, "no IMU sample yet");
		return 0;
	}

	shell_print(sh, "body frame: +x right edge, +y top edge, +z out of the touch surface");
	shell_print(sh, "flat, face up: accel z ~ +9.81; raise an edge: that axis goes +");
	for (int i = 0; i < 3; i++) {
		snprintk(label, sizeof(label), "accel %s", ax[i]);
		print_fixed2(sh, label, st.accel[i], "m/s^2");
	}
	for (int i = 0; i < 3; i++) {
		snprintk(label, sizeof(label), "gyro %s", ax[i]);
		print_fixed2(sh, label, st.gyro_dps[i], st.gyro_valid ? "dps" : "dps (invalid)");
	}
	shell_print(sh, "%-14s %s", "gyro_valid", st.gyro_valid ? "yes" : "no (off or settling)");
	print_fixed1(sh, "tilt", st.tilt_deg, "deg");
	if (st.gravity_valid) {
		int v = tenths(st.heading_deg);

		shell_print(sh, "%-14s %d.%d deg (raised edge: %s)", "heading", v / 10, v % 10,
			    raised_edge(st.heading_deg));
	} else {
		print_fixed1(sh, "heading", st.heading_deg, "deg (not valid: flat or moving)");
	}
	shell_print(sh, "%-14s %lld ms", "sample age",
		    (long long)(k_uptime_get() - st.imu_t_us / 1000));
	return 0;
}

static int cmd_pair(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t s = 120U;

	if (argc > 1) {
		char *end = NULL;
		long v = strtol(argv[1], &end, 10);

		if (end == argv[1] || *end != '\0' || v < 0 || v > 3600) {
			shell_error(sh, "usage: wp pair [seconds 0..3600]");
			return -EINVAL;
		}
		s = (uint32_t)v;
	}
	(void)wp_ble_open_pairing_window(s);
	if (s == 0U) {
		shell_print(sh, "pairing window closed");
	} else {
		shell_print(sh, "pairing open for %u s (closes after the first bond)", s);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_wp,
	SHELL_CMD(status, NULL, "Device state, orientation, counters, battery", cmd_status),
	SHELL_CMD_ARG(orient, NULL, "Force orientation: <0|90|180|270|auto>", cmd_orient, 2, 0),
	SHELL_CMD_ARG(mode, NULL, "Mode override: <auto|handheld|trackpad>", cmd_mode, 2, 0),
	SHELL_CMD(unpair, NULL, "Remove all BLE bonds and restart advertising", cmd_unpair),
	SHELL_CMD(bias, NULL, "Print the gyroscope bias estimate", cmd_bias),
	SHELL_CMD(touch, NULL, "Touch / IMU frame counters and rates", cmd_touch),
	SHELL_CMD(imu, NULL, "Latest body-frame IMU sample, tilt and gravity heading "
			     "(verify the IMU mounting: tilt each edge up)", cmd_imu),
	SHELL_CMD_ARG(pair, NULL, "Open the BLE pairing window: [seconds, default 120]",
		      cmd_pair, 1, 1),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(wp, &sub_wp, "Wafer Pad commands", NULL);
