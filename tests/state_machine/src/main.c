/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for src/state/state_machine.c.
 */

#include <math.h>
#include <string.h>

#include <zephyr/ztest.h>

#include <wafer_pad/imu_algo.h>
#include <wafer_pad/state_machine.h>

#define TICK_MS 10U

static struct wp_sm sm;
static struct wp_sm_config cfg;
static struct wp_sm_inputs in;
static struct wp_sm_outputs out;
static int64_t now_us;
static int changes; /* state changes seen by run_ms() */

/* ---- input helpers ------------------------------------------------------ */

static void motion_none(void)
{
	memset(&in.motion, 0, sizeof(in.motion));
	in.motion_valid = false;
}

/** Valid IMU data, nothing notable (neither still, moving nor tremor). */
static void motion_neutral(void)
{
	memset(&in.motion, 0, sizeof(in.motion));
	in.motion_valid = true;
	in.motion.valid = true;
	in.motion.face_up = true;
}

static void motion_moving(void)
{
	motion_neutral();
	in.motion.moving = true;
	in.motion.gyro_mag_dps = 60.0f;
}

static void motion_held(void)
{
	motion_neutral();
	in.motion.hand_tremor = true;
	in.motion.tremor_from_gyro = true;
	in.motion.tremor_dps = 1.0f;
}

/** Table-like stillness (still_ms left 0: the machine tracks the duration). */
static void motion_still(void)
{
	motion_neutral();
	in.motion.still = true;
}

static void touch(uint8_t contacts, bool still, uint32_t still_ms)
{
	in.contacts = contacts;
	in.single_still = still;
	in.single_still_ms = still_ms;
}

/* ---- driving helpers ---------------------------------------------------- */

static void tick(void)
{
	now_us += (int64_t)TICK_MS * 1000;
	in.t_us = now_us;
	if (in.single_still) {
		in.single_still_ms += TICK_MS;
	}
	wp_sm_update(&sm, &in, &out);
	if (out.state_changed) {
		changes++;
	}
}

/** Run updates for @p ms; stop early (returns true) when @p stop_at is reached. */
static bool run_until(uint32_t ms, enum wp_state stop_at)
{
	for (uint32_t t = 0; t < ms; t += TICK_MS) {
		tick();
		if (out.state == stop_at) {
			return true;
		}
	}
	return false;
}

static void run_ms(uint32_t ms)
{
	for (uint32_t t = 0; t < ms; t += TICK_MS) {
		tick();
	}
}

static void ev(enum wp_sm_event e)
{
	wp_sm_event(&sm, e, now_us);
}

static int64_t ms_since(int64_t t0)
{
	return (now_us - t0) / 1000;
}

static void start_with(const struct wp_sm_config *c)
{
	now_us = 0;
	changes = 0;
	memset(&in, 0, sizeof(in));
	memset(&out, 0, sizeof(out));
	wp_sm_init(&sm, c, now_us);
	motion_still();
}

/** BOOT -> INITIALIZING -> TRACKPAD. */
static void boot_to_trackpad(void)
{
	ev(WP_SM_EV_INIT_START);
	tick();
	zassert_equal(out.state, WP_STATE_INITIALIZING);
	ev(WP_SM_EV_INIT_OK);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	zassert_true(out.state_changed);
	zassert_equal(out.prev_state, WP_STATE_INITIALIZING);
	ev(WP_SM_EV_BLE_CONNECTED);
}

/** TRACKPAD -> PICKUP_CANDIDATE -> HANDHELD by continuous motion. */
static void to_handheld(void)
{
	motion_moving();
	tick();
	zassert_equal(out.state, WP_STATE_PICKUP_CANDIDATE);
	zassert_true(run_until(1000, WP_STATE_HANDHELD));
	motion_held();
}

static void to_air_mouse(void)
{
	to_handheld();
	touch(1, true, 0);
	zassert_true(run_until(1000, WP_STATE_AIR_MOUSE));
}

static void before(void *f)
{
	ARG_UNUSED(f);
	wp_sm_config_default(&cfg);
	start_with(&cfg);
}

/* ---- tests -------------------------------------------------------------- */

ZTEST(state_machine, test_config_defaults)
{
	struct wp_sm_config c;

	wp_sm_config_default(&c);
	zassert_equal(c.pickup_confirm_ms, 400U);
	zassert_equal(c.pickup_settle_ms, 150U);
	zassert_equal(c.pickup_timeout_ms, 1500U);
	zassert_equal(c.putdown_still_ms, 300U);
	zassert_equal(c.handheld_putdown_ms, 1200U);
	zassert_true(c.air_mouse_enabled);
	zassert_equal(c.air_hold_ms, 300U);
	zassert_equal(c.idle_timeout_ms, 30000U);
	zassert_false(c.sleep_enabled);
	zassert_equal(c.sleep_timeout_ms, 900000U);
	zassert_equal(c.disconnected_sleep_ms, 0U);
	zassert_equal(c.calib_timeout_ms, 5000U);
	zassert_equal(c.error_retry_ms, 500U);
	zassert_equal(c.error_retry_max_ms, 8000U);
	zassert_false(c.gyro_always_on);
}

ZTEST(state_machine, test_names)
{
	zassert_str_equal(wp_state_name(WP_STATE_BOOT), "BOOT");
	zassert_str_equal(wp_state_name(WP_STATE_PICKUP_CANDIDATE), "PICKUP_CANDIDATE");
	zassert_str_equal(wp_state_name(WP_STATE_AIR_MOUSE), "AIR_MOUSE");
	zassert_str_equal(wp_state_name(WP_STATE_ERROR_RECOVERY), "ERROR_RECOVERY");
	zassert_str_equal(wp_state_name(WP_STATE_COUNT), "UNKNOWN");
	for (int s = 0; s < WP_STATE_COUNT; s++) {
		zassert_not_null(wp_state_name((enum wp_state)s));
		zassert_true(strcmp(wp_state_name((enum wp_state)s), "UNKNOWN") != 0);
	}
	zassert_str_equal(wp_mode_name(WP_MODE_AUTO), "AUTO");
	zassert_str_equal(wp_mode_name(WP_MODE_FORCE_HANDHELD), "FORCE_HANDHELD");
	zassert_str_equal(wp_mode_name(WP_MODE_FORCE_TRACKPAD), "FORCE_TRACKPAD");
	zassert_str_equal(wp_mode_name((enum wp_mode_override)7), "UNKNOWN");
}

ZTEST(state_machine, test_boot_sequence_and_change_reporting)
{
	tick();
	zassert_equal(out.state, WP_STATE_BOOT);
	zassert_false(out.state_changed);

	/* INIT_OK before INIT_START is not in the table: ignored. */
	ev(WP_SM_EV_INIT_OK);
	tick();
	zassert_equal(out.state, WP_STATE_BOOT);

	ev(WP_SM_EV_INIT_START);
	zassert_equal(wp_sm_state(&sm), WP_STATE_INITIALIZING); /* applied immediately */
	tick();
	zassert_true(out.state_changed);
	zassert_equal(out.prev_state, WP_STATE_BOOT);
	tick();
	zassert_false(out.state_changed, "reported exactly once");
	zassert_equal(out.prev_state, WP_STATE_BOOT);

	ev(WP_SM_EV_INIT_OK);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	zassert_true(out.state_changed);
	zassert_equal(out.prev_state, WP_STATE_INITIALIZING);
	run_ms(100);
	zassert_false(out.state_changed);
	zassert_equal(changes, 2);

	/* A -> B -> A between two updates is no reported change. */
	ev(WP_SM_EV_CALIB_ARMED);
	ev(WP_SM_EV_CALIB_CANCEL);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	zassert_false(out.state_changed);
	zassert_equal(out.prev_state, WP_STATE_INITIALIZING);

	/* Two events between updates: one change, prev = last reported state. */
	start_with(&cfg);
	ev(WP_SM_EV_INIT_START);
	ev(WP_SM_EV_INIT_OK);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	zassert_true(out.state_changed);
	zassert_equal(out.prev_state, WP_STATE_BOOT);
}

ZTEST(state_machine, test_init_fail_error_recovery_backoff)
{
	ev(WP_SM_EV_INIT_START);
	tick();
	ev(WP_SM_EV_INIT_FAIL);
	int64_t t0 = now_us;

	tick();
	zassert_equal(out.state, WP_STATE_ERROR_RECOVERY);
	zassert_true(out.request_reinit, "pulse on entry");
	tick();
	zassert_false(out.request_reinit, "one-shot");

	/* Record pulse times over 30 s. */
	int64_t pulses[16];
	int n = 0;

	for (int i = 0; i < 3000; i++) {
		tick();
		if (out.request_reinit && n < 16) {
			pulses[n++] = ms_since(t0); /* t0 = entry (event) time */
		}
	}
	/* Intervals 500, 1000, 2000, 4000, 8000, 8000, 8000 (capped). */
	static const int64_t expect[] = {500, 1500, 3500, 7500, 15500, 23500};

	zassert_true(n >= (int)ARRAY_SIZE(expect), "only %d pulses", n);
	for (size_t i = 0; i < ARRAY_SIZE(expect); i++) {
		zassert_within(pulses[i], expect[i], TICK_MS, "pulse %d at %lld", (int)i,
			       (long long)pulses[i]);
	}
	zassert_equal(n, (int)ARRAY_SIZE(expect), "no extra pulses");

	/* Further INIT_FAIL / ERROR in recovery do not restart the backoff. */
	ev(WP_SM_EV_INIT_FAIL);
	ev(WP_SM_EV_ERROR);
	tick();
	zassert_false(out.request_reinit);
	zassert_false(out.state_changed);

	ev(WP_SM_EV_RECOVERED);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	zassert_false(out.request_reinit);
	run_ms(20000);
	zassert_false(out.request_reinit);

	/* EV_ERROR from operation: backoff restarts at error_retry_ms. */
	ev(WP_SM_EV_ERROR);
	t0 = now_us;
	tick();
	zassert_equal(out.state, WP_STATE_ERROR_RECOVERY);
	zassert_true(out.request_reinit);
	n = 0;
	for (int i = 0; i < 160; i++) {
		tick();
		if (out.request_reinit && n < 16) {
			pulses[n++] = ms_since(t0);
		}
	}
	zassert_equal(n, 2);
	zassert_within(pulses[0], 500, TICK_MS);
	zassert_within(pulses[1], 1500, TICK_MS);

	/* INIT_OK also leaves recovery. */
	ev(WP_SM_EV_INIT_OK);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
}

ZTEST(state_machine, test_error_from_every_operational_state)
{
	static const enum wp_state targets[] = {
		WP_STATE_TRACKPAD, WP_STATE_PICKUP_CANDIDATE, WP_STATE_HANDHELD,
		WP_STATE_AIR_MOUSE, WP_STATE_IDLE, WP_STATE_CALIBRATING,
	};

	for (size_t i = 0; i < ARRAY_SIZE(targets); i++) {
		start_with(&cfg);
		boot_to_trackpad();
		switch (targets[i]) {
		case WP_STATE_PICKUP_CANDIDATE:
			motion_moving();
			tick();
			break;
		case WP_STATE_HANDHELD:
			to_handheld();
			break;
		case WP_STATE_AIR_MOUSE:
			to_air_mouse();
			break;
		case WP_STATE_IDLE:
			zassert_true(run_until(31000, WP_STATE_IDLE));
			break;
		case WP_STATE_CALIBRATING:
			ev(WP_SM_EV_CALIB_ARMED);
			tick();
			break;
		default:
			break;
		}
		zassert_equal(out.state, targets[i]);
		ev(WP_SM_EV_ERROR);
		tick();
		zassert_equal(out.state, WP_STATE_ERROR_RECOVERY, "from %s",
			      wp_state_name(targets[i]));
		zassert_true(out.request_reinit);
	}

	/* EV_ERROR while initializing counts as an init failure. */
	start_with(&cfg);
	ev(WP_SM_EV_INIT_START);
	ev(WP_SM_EV_ERROR);
	tick();
	zassert_equal(out.state, WP_STATE_ERROR_RECOVERY);

	/* ... but BOOT ignores it. */
	start_with(&cfg);
	ev(WP_SM_EV_ERROR);
	tick();
	zassert_equal(out.state, WP_STATE_BOOT);
}

ZTEST(state_machine, test_pickup_to_handheld_by_tremor)
{
	boot_to_trackpad();
	run_ms(100);

	motion_moving();
	tick();
	zassert_equal(out.state, WP_STATE_PICKUP_CANDIDATE);
	zassert_true(out.gyro_needed);
	int64_t t0 = now_us;

	/* Held in a hand: tremor, no further big motion. */
	motion_held();
	zassert_true(run_until(1000, WP_STATE_HANDHELD));
	zassert_true(out.state_changed);
	zassert_equal(out.prev_state, WP_STATE_PICKUP_CANDIDATE);
	zassert_within(ms_since(t0), 400, TICK_MS, "confirmed after %lld ms",
		       (long long)ms_since(t0));
}

ZTEST(state_machine, test_pickup_to_handheld_by_continued_motion)
{
	boot_to_trackpad();
	motion_moving();
	tick();
	int64_t t0 = now_us;

	zassert_true(run_until(1000, WP_STATE_HANDHELD));
	zassert_within(ms_since(t0), 400, TICK_MS);
}

/* ---- real motion chain: wp_motion_update() -> wp_sm_update() ----------- */

#define CHAIN_DT_US 9615 /* 104 Hz */

static struct wp_motion chain_m;
static bool chain_handheld_seen;
static int64_t chain_gyro_req_us;
static uint32_t chain_rng;

static float chain_rnd(void)
{
	chain_rng = chain_rng * 1664525u + 1013904223u;
	return ((float)(chain_rng >> 8) / (float)(1u << 24)) * 2.0f - 1.0f;
}

static void chain_init(void)
{
	wp_motion_init(&chain_m, NULL);
	chain_handheld_seen = false;
	chain_gyro_req_us = -1;
	chain_rng = 4242u;
}

/*
 * One IMU sample (accel in g, physical rate in dps). The gyroscope follows
 * the platform: powered while the machine asks for it, data valid 100 ms
 * after power-up (turn-on + settle). The SM is updated on every sample, as
 * the router does (housekeeping after every message).
 */
static void chain_sample(const float acc_g[3], const float gyro_dps[3])
{
	struct wp_imu_sample s = {0};

	now_us += CHAIN_DT_US;
	if (out.gyro_needed) {
		if (chain_gyro_req_us < 0) {
			chain_gyro_req_us = now_us;
		}
	} else {
		chain_gyro_req_us = -1;
	}
	s.t_us = now_us;
	s.gyro_valid = chain_gyro_req_us >= 0 && (now_us - chain_gyro_req_us) >= 100000;
	for (int i = 0; i < 3; i++) {
		s.accel[i] = acc_g[i] * WP_GRAVITY_MS2;
		s.gyro[i] = gyro_dps[i] * WP_DEG_TO_RAD;
	}
	wp_motion_update(&chain_m, &s, &in.motion);
	in.motion_valid = true;
	in.t_us = now_us;
	if (in.single_still) {
		in.single_still_ms += CHAIN_DT_US / 1000;
	}
	wp_sm_update(&sm, &in, &out);
	if (out.state_changed) {
		changes++;
	}
	if (out.state == WP_STATE_HANDHELD) {
		chain_handheld_seen = true;
	}
}

/* Lying on the table: 1 mg accel noise; gyro noise 0.05 dps plus a 0.4 dps
 * residual bias error on z (a DC offset is not hand tremor). */
static void chain_table(int n, float gyro_noise_dps)
{
	for (int i = 0; i < n; i++) {
		const float a[3] = {0.001f * chain_rnd(), 0.001f * chain_rnd(),
				    1.0f + 0.001f * chain_rnd()};
		const float g[3] = {gyro_noise_dps * chain_rnd(), gyro_noise_dps * chain_rnd(),
				    0.4f + gyro_noise_dps * chain_rnd()};

		chain_sample(a, g);
	}
}

ZTEST(state_machine, test_bump_returns_to_trackpad)
{
	/* ALG-3: knocks of 1-5 samples (+-0.15 g) on the pad lying on the
	 * table with a resting finger never reach HANDHELD / AIR_MOUSE. */
	static const int knock_len[] = {1, 2, 3, 5};

	for (size_t k = 0; k < ARRAY_SIZE(knock_len); k++) {
		start_with(&cfg);
		boot_to_trackpad();
		chain_init();
		touch(1, true, 500);
		chain_table(104, 0.05f);
		zassert_equal(out.state, WP_STATE_TRACKPAD);
		changes = 0;

		for (int j = 0; j < knock_len[k]; j++) {
			const float a[3] = {(j & 1) ? -0.15f : 0.15f, 0.0f, 1.0f};
			const float g[3] = {0.0f, 0.0f, 0.4f};

			chain_sample(a, g);
		}
		zassert_equal(out.state, WP_STATE_PICKUP_CANDIDATE, "knock %d", knock_len[k]);
		chain_table(2 * 104, 0.05f);
		zassert_false(chain_handheld_seen, "knock of %d samples promoted to HANDHELD",
			      knock_len[k]);
		zassert_equal(out.state, WP_STATE_TRACKPAD, "knock %d: %s", knock_len[k],
			      wp_state_name(out.state));
		zassert_equal(changes, 2, "knock %d", knock_len[k]); /* PICKUP, TRACKPAD */
	}
}

ZTEST(state_machine, test_real_pickup_reaches_handheld)
{
	/* The other side of ALG-3: an actual pickup still confirms. */
	boot_to_trackpad();
	chain_init();
	chain_table(104, 0.05f);

	/* Lift: 400 ms of 2 Hz motion (0.12 g, 30 dps). */
	for (int i = 0; i < 42; i++) {
		float ph = 2.0f * WP_PI * 2.0f * (float)i * CHAIN_DT_US / 1e6f;
		const float a[3] = {0.12f * sinf(ph), 0.05f * cosf(ph), 1.0f};
		const float g[3] = {30.0f * cosf(ph), 5.0f, 0.0f};

		chain_sample(a, g);
	}
	/* Held: 9 Hz tremor 0.6 dps, 4 mg accel jitter. */
	for (int i = 0; i < 104; i++) {
		float ph = 2.0f * WP_PI * 9.0f * (float)i * CHAIN_DT_US / 1e6f;
		const float a[3] = {0.004f * sinf(ph), 0.004f * chain_rnd(), 1.0f};
		const float g[3] = {0.6f * sinf(ph), 0.6f * cosf(ph), 0.1f};

		chain_sample(a, g);
	}
	zassert_true(chain_handheld_seen, "pickup not confirmed");
	zassert_equal(out.state, WP_STATE_HANDHELD);
}

ZTEST(state_machine, test_air_mouse_steady_aim_with_tremor_stays)
{
	/* ALG-4: holding the cursor steady in AIR_MOUSE (|w| < still threshold,
	 * hand tremor present) is not a put-down. */
	boot_to_trackpad();
	to_air_mouse();
	chain_init();
	chain_gyro_req_us = now_us - 1000000; /* gyro already running */
	changes = 0;

	for (int i = 0; i < 3 * 104; i++) {
		float ph = 2.0f * WP_PI * 9.0f * (float)i * CHAIN_DT_US / 1e6f;
		const float a[3] = {0.003f * chain_rnd(), 0.003f * chain_rnd(),
				    1.0f + 0.003f * chain_rnd()};
		const float g[3] = {0.5f * sinf(ph), 0.5f * cosf(ph), 0.5f * sinf(ph + 1.0f)};

		chain_sample(a, g);
		zassert_equal(out.state, WP_STATE_AIR_MOUSE, "dropped at %d ms (still %d tremor %d)",
			      i * CHAIN_DT_US / 1000, (int)in.motion.still,
			      (int)in.motion.hand_tremor);
	}
	zassert_true(in.motion.still, "test premise: |w| below the still threshold");
	zassert_true(in.motion.hand_tremor);
	zassert_equal(changes, 0);

	/* Laid on the table (0.08 dps noise): put down within ~1.5 s. */
	int64_t t0 = now_us;

	for (int i = 0; i < 3 * 104 && out.state != WP_STATE_TRACKPAD; i++) {
		chain_table(1, 0.08f);
	}
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	zassert_true(ms_since(t0) < 2000, "put-down took %lld ms", (long long)ms_since(t0));
}

ZTEST(state_machine, test_bump_uses_reported_still_ms)
{
	boot_to_trackpad();
	motion_moving();
	tick();
	/* The motion module reports a stillness that already lasts 300 ms. */
	motion_still();
	in.motion.still_ms = 300;
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
}

ZTEST(state_machine, test_pickup_timeout)
{
	boot_to_trackpad();
	motion_moving();
	tick();
	int64_t t0 = now_us;

	/* Ambiguous: neither still, nor tremor, nor moving. */
	motion_neutral();
	zassert_true(run_until(3000, WP_STATE_TRACKPAD));
	zassert_within(ms_since(t0), 1500, TICK_MS);
}

ZTEST(state_machine, test_air_mouse_engage_and_disengage)
{
	boot_to_trackpad();
	to_handheld();

	/* Moving finger: no air mouse. */
	touch(1, false, 0);
	run_ms(1000);
	zassert_equal(out.state, WP_STATE_HANDHELD);

	/* Two still fingers: no air mouse. */
	touch(2, true, 0);
	run_ms(1000);
	zassert_equal(out.state, WP_STATE_HANDHELD);

	/* One still finger for air_hold_ms. */
	touch(1, true, 0);
	run_ms(280);
	zassert_equal(out.state, WP_STATE_HANDHELD);
	zassert_true(run_until(100, WP_STATE_AIR_MOUSE));
	zassert_true(in.single_still_ms >= 300U);
	zassert_true(out.air_mouse_active);
	zassert_false(out.pointer_from_touch);
	zassert_true(out.gyro_needed);

	/* Stays engaged while the finger is down and the hand moves. */
	motion_moving();
	run_ms(2000);
	zassert_equal(out.state, WP_STATE_AIR_MOUSE);

	/* Lift: back to HANDHELD. */
	touch(0, false, 0);
	tick();
	zassert_equal(out.state, WP_STATE_HANDHELD);
	zassert_true(out.state_changed);
	zassert_false(out.air_mouse_active);
	zassert_true(out.pointer_from_touch);

	/* Re-engage, then a second finger disengages too. */
	motion_held();
	touch(1, true, 0);
	zassert_true(run_until(1000, WP_STATE_AIR_MOUSE));
	touch(2, false, 0);
	tick();
	zassert_equal(out.state, WP_STATE_HANDHELD);
}

ZTEST(state_machine, test_air_mouse_disabled)
{
	cfg.air_mouse_enabled = false;
	start_with(&cfg);
	boot_to_trackpad();
	to_handheld();
	touch(1, true, 0);
	run_ms(3000);
	zassert_equal(out.state, WP_STATE_HANDHELD);
}

ZTEST(state_machine, test_put_down_from_handheld_and_air_mouse)
{
	boot_to_trackpad();
	to_handheld();

	/* Lying still but not long enough. */
	motion_still();
	run_ms(1000);
	zassert_equal(out.state, WP_STATE_HANDHELD);
	/* Stillness interrupted resets the timer. */
	motion_held();
	tick();
	motion_still();
	int64_t t0 = now_us;

	zassert_true(run_until(3000, WP_STATE_TRACKPAD));
	zassert_within(ms_since(t0), 1200, 2 * TICK_MS);
	zassert_equal(out.prev_state, WP_STATE_HANDHELD);

	/* Tilted-but-held (no stillness) never counts as put down. */
	to_handheld();
	in.motion.tilt_deg = 0.0f; /* horizontal */
	run_ms(5000);
	zassert_equal(out.state, WP_STATE_HANDHELD);

	/* AIR_MOUSE put down with the finger still resting. */
	touch(1, true, 0);
	zassert_true(run_until(1000, WP_STATE_AIR_MOUSE));
	motion_still();
	zassert_true(run_until(3000, WP_STATE_TRACKPAD));
	zassert_equal(out.prev_state, WP_STATE_AIR_MOUSE);
}

ZTEST(state_machine, test_idle_and_wake)
{
	boot_to_trackpad();
	motion_still();
	touch(0, false, 0);
	int64_t t0 = now_us;

	zassert_true(run_until(40000, WP_STATE_IDLE));
	zassert_within(ms_since(t0), 30000, TICK_MS);
	zassert_true(out.imu_low_rate);
	zassert_true(out.touch_low_power);
	zassert_false(out.ble_low_latency);

	/* Touch wakes. */
	touch(1, false, 0);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	zassert_false(out.touch_low_power);

	/* Touch activity defers idle; idle counts from the last activity. */
	run_ms(20000);
	touch(0, false, 0);
	t0 = now_us;
	zassert_true(run_until(40000, WP_STATE_IDLE));
	zassert_within(ms_since(t0), 30000, 2 * TICK_MS);

	/* Motion wakes as well (goes through TRACKPAD first). */
	motion_moving();
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	tick();
	zassert_equal(out.state, WP_STATE_PICKUP_CANDIDATE);
}

ZTEST(state_machine, test_sleep_disabled_never_sleeps)
{
	boot_to_trackpad();
	ev(WP_SM_EV_BLE_DISCONNECTED);
	motion_still();
	zassert_true(run_until(40000, WP_STATE_IDLE));
	run_ms(2000000);
	zassert_equal(out.state, WP_STATE_IDLE);
	zassert_false(out.request_sleep);
}

ZTEST(state_machine, test_sleep_enabled_inactivity)
{
	cfg.sleep_enabled = true;
	cfg.sleep_timeout_ms = 60000U;
	start_with(&cfg);
	boot_to_trackpad();
	zassert_true(run_until(40000, WP_STATE_IDLE));
	int64_t t0 = now_us;

	zassert_true(run_until(70000, WP_STATE_SLEEP));
	zassert_within(ms_since(t0), 60000, TICK_MS);
	zassert_true(out.request_sleep);
	zassert_false(out.gyro_needed);

	/* Terminal: neither activity nor events leave SLEEP. */
	touch(2, false, 0);
	motion_moving();
	ev(WP_SM_EV_ERROR);
	ev(WP_SM_EV_MODE_TOGGLE);
	ev(WP_SM_EV_CALIB_ARMED);
	run_ms(1000);
	zassert_equal(out.state, WP_STATE_SLEEP);
	zassert_true(out.request_sleep);
	zassert_equal(out.mode, WP_MODE_AUTO);
}

ZTEST(state_machine, test_ble_disconnected_sleep)
{
	cfg.sleep_enabled = true;
	cfg.disconnected_sleep_ms = 5000U;
	start_with(&cfg);
	boot_to_trackpad(); /* connected */

	zassert_true(run_until(40000, WP_STATE_IDLE));
	run_ms(10000);
	zassert_equal(out.state, WP_STATE_IDLE, "connected: no disconnected sleep");

	ev(WP_SM_EV_BLE_DISCONNECTED);
	int64_t t0 = now_us;

	run_ms(3000);
	ev(WP_SM_EV_BLE_DISCONNECTED); /* repeated event does not restart the timer */
	zassert_true(run_until(10000, WP_STATE_SLEEP));
	zassert_within(ms_since(t0), 5000, TICK_MS);

	/* Same with sleep disabled: never. */
	cfg.sleep_enabled = false;
	start_with(&cfg);
	boot_to_trackpad();
	ev(WP_SM_EV_BLE_DISCONNECTED);
	zassert_true(run_until(40000, WP_STATE_IDLE));
	run_ms(20000);
	zassert_equal(out.state, WP_STATE_IDLE);

	/* disconnected_sleep_ms = 0 means never. */
	cfg.sleep_enabled = true;
	cfg.disconnected_sleep_ms = 0U;
	start_with(&cfg);
	boot_to_trackpad();
	ev(WP_SM_EV_BLE_DISCONNECTED);
	zassert_true(run_until(40000, WP_STATE_IDLE));
	run_ms(60000);
	zassert_equal(out.state, WP_STATE_IDLE);
}

ZTEST(state_machine, test_calibration_enter_and_return)
{
	boot_to_trackpad();

	/* From TRACKPAD, DONE returns to TRACKPAD. */
	ev(WP_SM_EV_CALIB_ARMED);
	tick();
	zassert_equal(out.state, WP_STATE_CALIBRATING);
	zassert_false(out.orientation_auto_allowed);
	zassert_true(out.ble_low_latency);
	ev(WP_SM_EV_CALIB_DONE);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);

	/* Timeout returns too. */
	ev(WP_SM_EV_CALIB_ARMED);
	tick();
	int64_t t0 = now_us;

	zassert_true(run_until(6000, WP_STATE_TRACKPAD));
	zassert_within(ms_since(t0), 5000, TICK_MS);

	/* From HANDHELD, CANCEL returns to HANDHELD. */
	to_handheld();
	ev(WP_SM_EV_CALIB_ARMED);
	tick();
	zassert_equal(out.state, WP_STATE_CALIBRATING);
	ev(WP_SM_EV_CALIB_CANCEL);
	tick();
	zassert_equal(out.state, WP_STATE_HANDHELD);

	/* From AIR_MOUSE, DONE returns to HANDHELD (not AIR_MOUSE). */
	touch(1, true, 0);
	zassert_true(run_until(1000, WP_STATE_AIR_MOUSE));
	ev(WP_SM_EV_CALIB_ARMED);
	tick();
	zassert_equal(out.state, WP_STATE_CALIBRATING);
	zassert_false(out.air_mouse_active);
	ev(WP_SM_EV_CALIB_DONE);
	tick();
	zassert_equal(out.state, WP_STATE_HANDHELD);
	zassert_equal(out.prev_state, WP_STATE_CALIBRATING);

	/* Not armable from PICKUP_CANDIDATE or IDLE. */
	touch(0, false, 0);
	motion_still();
	zassert_true(run_until(3000, WP_STATE_TRACKPAD));
	motion_moving();
	tick();
	zassert_equal(out.state, WP_STATE_PICKUP_CANDIDATE);
	ev(WP_SM_EV_CALIB_ARMED);
	tick();
	zassert_not_equal(out.state, WP_STATE_CALIBRATING);
	motion_still();
	zassert_true(run_until(40000, WP_STATE_IDLE));
	ev(WP_SM_EV_CALIB_ARMED);
	tick();
	zassert_equal(out.state, WP_STATE_IDLE);
}

/*
 * S1 regression: CALIBRATING entered from HANDHELD / AIR_MOUSE keeps the gyro
 * powered and long press disallowed, and accel-only stillness during the
 * calibration (gyro samples not valid) must not count towards the put-down,
 * or the device falls back to TRACKPAD as soon as the calibration ends.
 */
ZTEST(state_machine, test_calibration_keeps_handheld_context)
{
	boot_to_trackpad();

	/* From TRACKPAD: trackpad context, no gyro. */
	ev(WP_SM_EV_CALIB_ARMED);
	tick();
	zassert_equal(out.state, WP_STATE_CALIBRATING);
	zassert_false(out.gyro_needed);
	zassert_true(out.long_press_allowed);
	ev(WP_SM_EV_CALIB_CANCEL);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);

	/* From HANDHELD and from AIR_MOUSE: handheld context. */
	to_handheld();
	ev(WP_SM_EV_CALIB_ARMED);
	tick();
	zassert_equal(out.state, WP_STATE_CALIBRATING);
	zassert_true(out.gyro_needed, "gyro powered off for calibration");
	zassert_false(out.long_press_allowed);
	ev(WP_SM_EV_CALIB_CANCEL);
	tick();
	zassert_equal(out.state, WP_STATE_HANDHELD);
	zassert_true(out.gyro_needed);

	touch(1, true, 0);
	zassert_true(run_until(1000, WP_STATE_AIR_MOUSE));
	ev(WP_SM_EV_CALIB_ARMED);
	tick();
	zassert_equal(out.state, WP_STATE_CALIBRATING);
	zassert_true(out.gyro_needed, "gyro powered off for calibration");
	zassert_false(out.long_press_allowed);

	/* Gyro samples not valid for the whole calibration (accel-only data,
	 * tremor_from_gyro == false) while the accelerometer reads "still":
	 * well over handheld_putdown_ms. */
	touch(3, false, 0);
	motion_still();
	run_ms(cfg.handheld_putdown_ms + 800U);
	zassert_equal(out.state, WP_STATE_CALIBRATING);
	ev(WP_SM_EV_CALIB_DONE);
	tick();
	zassert_equal(out.state, WP_STATE_HANDHELD);

	/* Right after the return the gyro may still be settling (accel-only):
	 * the device must stay HANDHELD, not be "put down" by stillness that
	 * was accumulated on accel-only data during the calibration. */
	touch(0, false, 0);
	run_ms(200);
	zassert_equal(out.state, WP_STATE_HANDHELD, "put down by accel-only stillness");
	motion_held();
	run_ms(cfg.handheld_putdown_ms * 2U);
	zassert_equal(out.state, WP_STATE_HANDHELD);

	/* Gyro-based stillness during the calibration is real evidence: after
	 * the return the put-down follows immediately. */
	ev(WP_SM_EV_CALIB_ARMED);
	tick();
	zassert_equal(out.state, WP_STATE_CALIBRATING);
	motion_still();
	in.motion.tremor_from_gyro = true;
	run_ms(cfg.handheld_putdown_ms + 100U);
	ev(WP_SM_EV_CALIB_DONE);
	tick();
	zassert_equal(out.state, WP_STATE_HANDHELD);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
}

ZTEST(state_machine, test_mode_override_cycle)
{
	boot_to_trackpad();
	zassert_equal(out.mode, WP_MODE_AUTO);
	zassert_true(out.long_press_allowed);

	/* AUTO -> FORCE_HANDHELD: TRACKPAD becomes HANDHELD immediately. */
	ev(WP_SM_EV_MODE_TOGGLE);
	zassert_equal(wp_sm_state(&sm), WP_STATE_HANDHELD);
	tick();
	zassert_equal(out.mode, WP_MODE_FORCE_HANDHELD);
	zassert_equal(out.state, WP_STATE_HANDHELD);
	zassert_true(out.state_changed);
	zassert_false(out.long_press_allowed);
	zassert_true(out.gyro_needed);

	/* Put-down disabled: lying still for a long time stays HANDHELD. */
	motion_still();
	run_ms(10000);
	zassert_equal(out.state, WP_STATE_HANDHELD);

	/* Air mouse still works, and its put-down is disabled as well. */
	motion_held();
	touch(1, true, 0);
	zassert_true(run_until(1000, WP_STATE_AIR_MOUSE));
	motion_still();
	run_ms(5000);
	zassert_equal(out.state, WP_STATE_AIR_MOUSE);
	touch(0, false, 0);
	tick();
	zassert_equal(out.state, WP_STATE_HANDHELD);

	/* Calibration from HANDHELD returns to HANDHELD; long press stays blocked. */
	ev(WP_SM_EV_CALIB_ARMED);
	tick();
	zassert_equal(out.state, WP_STATE_CALIBRATING);
	zassert_false(out.long_press_allowed, "FORCE_HANDHELD blocks long press everywhere");
	ev(WP_SM_EV_CALIB_DONE);
	tick();
	zassert_equal(out.state, WP_STATE_HANDHELD);

	/* FORCE_HANDHELD -> FORCE_TRACKPAD: HANDHELD returns to TRACKPAD. */
	ev(WP_SM_EV_MODE_TOGGLE);
	tick();
	zassert_equal(out.mode, WP_MODE_FORCE_TRACKPAD);
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	zassert_true(out.long_press_allowed);

	/* Pickup detection disabled. */
	motion_moving();
	run_ms(2000);
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	zassert_false(out.gyro_needed);

	/* FORCE_TRACKPAD -> AUTO: pickup works again. */
	ev(WP_SM_EV_MODE_TOGGLE);
	tick();
	zassert_equal(out.mode, WP_MODE_AUTO);
	tick();
	zassert_equal(out.state, WP_STATE_PICKUP_CANDIDATE);

	/* FORCE_HANDHELD from PICKUP_CANDIDATE -> HANDHELD. */
	ev(WP_SM_EV_MODE_TOGGLE);
	tick();
	zassert_equal(out.state, WP_STATE_HANDHELD);

	/* FORCE_TRACKPAD from AIR_MOUSE -> TRACKPAD. */
	motion_held();
	touch(1, true, 0);
	zassert_true(run_until(1000, WP_STATE_AIR_MOUSE));
	ev(WP_SM_EV_MODE_TOGGLE);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	zassert_equal(out.prev_state, WP_STATE_AIR_MOUSE);

	/* FORCE_TRACKPAD from PICKUP_CANDIDATE: cycle AUTO, pick up, toggle twice. */
	ev(WP_SM_EV_MODE_TOGGLE); /* AUTO */
	touch(0, false, 0);
	motion_moving();
	tick();
	tick();
	zassert_equal(out.state, WP_STATE_PICKUP_CANDIDATE);
	ev(WP_SM_EV_MODE_TOGGLE); /* FORCE_HANDHELD */
	ev(WP_SM_EV_MODE_TOGGLE); /* FORCE_TRACKPAD */
	tick();
	zassert_equal(out.mode, WP_MODE_FORCE_TRACKPAD);
	zassert_equal(out.state, WP_STATE_TRACKPAD);

	/* FORCE_HANDHELD from IDLE: wake goes to TRACKPAD, then HANDHELD. */
	ev(WP_SM_EV_MODE_TOGGLE); /* AUTO */
	motion_still();
	zassert_true(run_until(40000, WP_STATE_IDLE));
	ev(WP_SM_EV_MODE_TOGGLE); /* FORCE_HANDHELD */
	tick();
	zassert_equal(out.state, WP_STATE_IDLE);
	zassert_false(out.long_press_allowed);
	touch(1, false, 0);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	tick();
	zassert_equal(out.state, WP_STATE_HANDHELD);

	/* Mode switched to FORCE_HANDHELD during a calibration armed in TRACKPAD:
	 * the return to TRACKPAD is followed by the forced HANDHELD.
	 */
	ev(WP_SM_EV_MODE_TOGGLE); /* FORCE_TRACKPAD */
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	ev(WP_SM_EV_MODE_TOGGLE); /* AUTO */
	ev(WP_SM_EV_CALIB_ARMED);
	tick();
	zassert_equal(out.state, WP_STATE_CALIBRATING);
	ev(WP_SM_EV_MODE_TOGGLE); /* FORCE_HANDHELD */
	tick();
	zassert_equal(out.state, WP_STATE_CALIBRATING);
	ev(WP_SM_EV_CALIB_DONE);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	tick();
	zassert_equal(out.state, WP_STATE_HANDHELD);
}

ZTEST(state_machine, test_no_imu_plain_trackpad)
{
	motion_none();
	boot_to_trackpad();

	/* Bogus motion fields without motion_valid are ignored. */
	in.motion.moving = true;
	in.motion.hand_tremor = true;
	run_ms(2000);
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	zassert_false(out.gyro_needed);
	zassert_true(out.pointer_from_touch);
	zassert_true(out.long_press_allowed);

	/* motion_valid with motion.valid == false is not usable either. */
	in.motion_valid = true;
	in.motion.valid = false;
	run_ms(1000);
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	motion_none();

	/* Touch keeps it active; idle without touch; touch wakes. */
	touch(1, false, 0);
	run_ms(40000);
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	touch(0, false, 0);
	zassert_true(run_until(31000, WP_STATE_IDLE));
	touch(1, false, 0);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	touch(1, true, 500);
	run_ms(1000);
	zassert_equal(out.state, WP_STATE_TRACKPAD, "no air mouse without IMU");

	/* IMU lost while handheld / in air mouse / pickup: falls back. */
	touch(0, false, 0);
	to_handheld();
	motion_none();
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);

	to_air_mouse();
	motion_none();
	tick();
	zassert_equal(out.state, WP_STATE_HANDHELD);
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);

	touch(0, false, 0);
	motion_moving();
	tick();
	zassert_equal(out.state, WP_STATE_PICKUP_CANDIDATE);
	motion_none();
	tick();
	zassert_equal(out.state, WP_STATE_TRACKPAD);
}

struct flags {
	enum wp_state st;
	bool gyro, low_rate, touch_lp, ptr_touch, air, long_press, orient, ble_ll, sleep;
};

static void check_flags(const struct flags *f)
{
	const char *n = wp_state_name(f->st);

	zassert_equal(out.state, f->st, "expected %s got %s", n, wp_state_name(out.state));
	zassert_equal(out.gyro_needed, f->gyro, "%s gyro_needed", n);
	zassert_equal(out.imu_low_rate, f->low_rate, "%s imu_low_rate", n);
	zassert_equal(out.touch_low_power, f->touch_lp, "%s touch_low_power", n);
	zassert_equal(out.pointer_from_touch, f->ptr_touch, "%s pointer_from_touch", n);
	zassert_equal(out.air_mouse_active, f->air, "%s air_mouse_active", n);
	zassert_equal(out.long_press_allowed, f->long_press, "%s long_press_allowed", n);
	zassert_equal(out.orientation_auto_allowed, f->orient, "%s orientation_auto", n);
	zassert_equal(out.ble_low_latency, f->ble_ll, "%s ble_low_latency", n);
	zassert_equal(out.request_sleep, f->sleep, "%s request_sleep", n);
}

ZTEST(state_machine, test_output_flags_per_state)
{
	/*                     st                         gyro   lowr   tlp    ptr    air
	 *                     long   orient ble_ll sleep
	 */
	static const struct flags tab[] = {
		{WP_STATE_BOOT, false, false, false, true, false, true, false, false, false},
		{WP_STATE_INITIALIZING, false, false, false, true, false, true, false, false,
		 false},
		{WP_STATE_TRACKPAD, false, false, false, true, false, true, true, true, false},
		{WP_STATE_PICKUP_CANDIDATE, true, false, false, true, false, false, true, true,
		 false},
		{WP_STATE_HANDHELD, true, false, false, true, false, false, true, true, false},
		{WP_STATE_AIR_MOUSE, true, false, false, false, true, false, false, true, false},
		/* Entered from AIR_MOUSE: keeps the handheld context (S1). */
		{WP_STATE_CALIBRATING, true, false, false, true, false, false, false, true, false},
		{WP_STATE_IDLE, false, true, true, true, false, true, true, false, false},
		{WP_STATE_SLEEP, false, false, false, true, false, true, true, false, true},
		{WP_STATE_ERROR_RECOVERY, false, false, false, true, false, true, false, false,
		 false},
	};

	cfg.sleep_enabled = true;
	cfg.sleep_timeout_ms = 1000U;
	start_with(&cfg);

	tick();
	check_flags(&tab[0]);
	ev(WP_SM_EV_INIT_START);
	tick();
	check_flags(&tab[1]);
	ev(WP_SM_EV_INIT_OK);
	tick();
	check_flags(&tab[2]);
	motion_moving();
	tick();
	check_flags(&tab[3]);
	zassert_true(run_until(1000, WP_STATE_HANDHELD));
	check_flags(&tab[4]);
	motion_held();
	touch(1, true, 0);
	zassert_true(run_until(1000, WP_STATE_AIR_MOUSE));
	check_flags(&tab[5]);
	ev(WP_SM_EV_CALIB_ARMED);
	tick();
	check_flags(&tab[6]);
	ev(WP_SM_EV_CALIB_DONE);
	touch(0, false, 0);
	motion_still();
	zassert_true(run_until(3000, WP_STATE_TRACKPAD));
	zassert_true(run_until(40000, WP_STATE_IDLE));
	check_flags(&tab[7]);
	zassert_true(run_until(2000, WP_STATE_SLEEP));
	check_flags(&tab[8]);

	start_with(&cfg);
	ev(WP_SM_EV_INIT_START);
	ev(WP_SM_EV_INIT_FAIL);
	tick();
	check_flags(&tab[9]);
	zassert_true(out.request_reinit);
}

ZTEST(state_machine, test_gyro_always_on)
{
	cfg.gyro_always_on = true;
	cfg.sleep_enabled = true;
	cfg.sleep_timeout_ms = 1000U;
	start_with(&cfg);
	boot_to_trackpad();
	zassert_true(out.gyro_needed);
	zassert_true(run_until(40000, WP_STATE_IDLE));
	zassert_true(out.gyro_needed);
	zassert_true(run_until(2000, WP_STATE_SLEEP));
	zassert_false(out.gyro_needed, "never in SLEEP");
}

ZTEST(state_machine, test_robustness)
{
	/* NULL inputs: evaluated as "no contacts, no IMU" at the current time. */
	boot_to_trackpad();
	wp_sm_update(&sm, NULL, &out);
	zassert_equal(out.state, WP_STATE_TRACKPAD);
	wp_sm_update(&sm, &in, NULL);
	wp_sm_update(NULL, &in, &out);
	zassert_equal(out.state, WP_STATE_BOOT);
	wp_sm_event(NULL, WP_SM_EV_INIT_OK, 0);

	/* Time going backwards does not trigger timeouts or underflow. */
	start_with(&cfg);
	boot_to_trackpad();
	motion_moving();
	tick();
	zassert_equal(out.state, WP_STATE_PICKUP_CANDIDATE);
	motion_neutral();
	in.t_us = now_us - 1000000;
	wp_sm_update(&sm, &in, &out);
	zassert_equal(out.state, WP_STATE_PICKUP_CANDIDATE);

	/* NULL config = defaults. */
	wp_sm_init(&sm, NULL, 0);
	zassert_equal(sm.cfg.idle_timeout_ms, 30000U);
	zassert_equal(wp_sm_state(&sm), WP_STATE_BOOT);
}

ZTEST_SUITE(state_machine, NULL, NULL, before, NULL, NULL);
