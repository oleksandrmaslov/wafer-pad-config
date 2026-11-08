/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Status LED (devicetree alias led0), fully non-blocking.
 *
 * Two layers driven by one k_work_delayable on the system work queue:
 *   base pattern (repeats until changed):
 *     OFF, ADVERTISING (200/800 ms), CONNECTED (40 ms blip every 3 s),
 *     CALIB_ARMED (80/80 ms), AIR_MOUSE (solid), ERROR (double blink)
 *   one-shot overlay (plays once, then the base pattern resumes):
 *     CALIB_DONE  arg = orientation in DEGREES -> arg/90 + 1 blinks
 *     MODE_CHANGE arg = enum wp_mode_override  -> arg + 1 blinks
 * Requesting the one-shot that is already playing (same pattern and arg)
 * does not restart it. WP_LED_OFF is applied synchronously (sleep path).
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>

#include <wafer_pad/platform.h>

LOG_MODULE_REGISTER(wp_led, CONFIG_WAFER_PAD_LOG_LEVEL);

#if defined(CONFIG_WAFER_PAD_LED) && DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(led0))

#define BLINK_MS      150
#define LEAD_IN_MS    250
#define TAIL_MS       400

/* Alternating ON, OFF durations in ms, starting with ON. */
static const uint16_t seq_advertising[] = {200, 800};
static const uint16_t seq_connected[] = {40, 2960};
static const uint16_t seq_calib_armed[] = {80, 80};
static const uint16_t seq_error[] = {100, 100, 100, 700};

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static struct k_work_delayable led_work;
static struct k_spinlock led_lock;
static bool led_ready;

static enum wp_led_pattern base_pattern = WP_LED_OFF;
static uint8_t base_step;

static bool ov_active;
static enum wp_led_pattern ov_pattern;
static int ov_arg;
static uint8_t ov_blinks;
static uint8_t ov_step;

static size_t base_seq(enum wp_led_pattern p, const uint16_t **seq)
{
	switch (p) {
	case WP_LED_ADVERTISING:
		*seq = seq_advertising;
		return ARRAY_SIZE(seq_advertising);
	case WP_LED_CONNECTED:
		*seq = seq_connected;
		return ARRAY_SIZE(seq_connected);
	case WP_LED_CALIB_ARMED:
		*seq = seq_calib_armed;
		return ARRAY_SIZE(seq_calib_armed);
	case WP_LED_ERROR:
		*seq = seq_error;
		return ARRAY_SIZE(seq_error);
	default:
		*seq = NULL;
		return 0; /* OFF / AIR_MOUSE: static level */
	}
}

static bool is_one_shot(enum wp_led_pattern p)
{
	return p == WP_LED_CALIB_DONE || p == WP_LED_MODE_CHANGE;
}

static uint8_t one_shot_blinks(enum wp_led_pattern p, int arg)
{
	int n = (p == WP_LED_CALIB_DONE) ? (arg / 90) + 1 : arg + 1;

	return (uint8_t)CLAMP(n, 1, 4);
}

static void led_work_handler(struct k_work *work)
{
	bool on = false;
	int32_t delay_ms = -1;
	bool done = false;

	ARG_UNUSED(work);

	k_spinlock_key_t key = k_spin_lock(&led_lock);

	if (ov_active) {
		uint8_t total = (uint8_t)(1U + 2U * ov_blinks);

		if (ov_step == 0U) {
			on = false; /* separate the blinks from the base pattern */
			delay_ms = LEAD_IN_MS;
			ov_step++;
			done = true;
		} else if (ov_step < total) {
			uint8_t idx = ov_step - 1U;

			on = (idx % 2U) == 0U;
			delay_ms = (idx == (uint8_t)(total - 2U)) ? TAIL_MS : BLINK_MS;
			ov_step++;
			done = true;
		} else {
			ov_active = false;
			base_step = 0U;
		}
	}

	if (!done) {
		const uint16_t *seq;
		size_t n = base_seq(base_pattern, &seq);

		if (n == 0U) {
			on = base_pattern == WP_LED_AIR_MOUSE;
		} else {
			if (base_step >= n) {
				base_step = 0U;
			}
			on = (base_step % 2U) == 0U;
			delay_ms = seq[base_step];
			base_step = (uint8_t)((base_step + 1U) % n);
		}
	}

	k_spin_unlock(&led_lock, key);

	(void)gpio_pin_set_dt(&led, on ? 1 : 0);
	if (delay_ms >= 0) {
		(void)k_work_reschedule(&led_work, K_MSEC(delay_ms));
	}
}

int wp_led_init(void)
{
	int err;

	if (led_ready) {
		return 0;
	}
	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("LED GPIO not ready");
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		LOG_ERR("LED configure failed (%d)", err);
		return err;
	}
	k_work_init_delayable(&led_work, led_work_handler);
	led_ready = true;
	return 0;
}

void wp_led_set(enum wp_led_pattern pattern, int arg)
{
	bool kick = false;
	bool off_now = false;

	if (!led_ready || (unsigned int)pattern > (unsigned int)WP_LED_MODE_CHANGE) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&led_lock);

	if (is_one_shot(pattern)) {
		if (!(ov_active && ov_pattern == pattern && ov_arg == arg)) {
			ov_active = true;
			ov_pattern = pattern;
			ov_arg = arg;
			ov_blinks = one_shot_blinks(pattern, arg);
			ov_step = 0U;
			kick = true;
		}
	} else if (pattern == WP_LED_OFF) {
		base_pattern = WP_LED_OFF;
		base_step = 0U;
		ov_active = false;
		off_now = true;
	} else if (pattern != base_pattern) {
		base_pattern = pattern;
		base_step = 0U;
		kick = !ov_active; /* a running one-shot finishes first */
	}

	k_spin_unlock(&led_lock, key);

	if (off_now) {
		(void)k_work_cancel_delayable(&led_work);
		(void)gpio_pin_set_dt(&led, 0);
	} else if (kick) {
		(void)k_work_reschedule(&led_work, K_NO_WAIT);
	}
}

#else /* LED disabled or no led0 alias */

int wp_led_init(void)
{
	return IS_ENABLED(CONFIG_WAFER_PAD_LED) ? -ENODEV : -ENOTSUP;
}

void wp_led_set(enum wp_led_pattern pattern, int arg)
{
	ARG_UNUSED(pattern);
	ARG_UNUSED(arg);
}

#endif
