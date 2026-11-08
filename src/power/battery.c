/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Battery measurement: nRF52840 SAADC on VDDH/5 (channel from the
 * zephyr,user io-channels property in boards/nice_nano_v2.overlay).
 *
 * The nice!nano v2 runs the SoC in high-voltage mode from BAT+, so VDDH is
 * the battery voltage. adc_raw_to_millivolts_dt() yields VDDH/5 in mV; the
 * result is multiplied by 5.
 *
 * NOTE: on USB power WITHOUT a battery, VDDH is the charger output (about
 * 4.2 V or more): the reading then shows ~100 % and is meaningless.
 *
 * The ADC channel is set up lazily on first use; a mutex serializes callers
 * (periodic work in power.c, "wp status" from the shell).
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <wafer_pad/platform.h>

LOG_MODULE_REGISTER(wp_battery, CONFIG_WAFER_PAD_LOG_LEVEL);

#define VDDH_DIVIDER 5

/*
 * Approximate LiPo open-circuit discharge curve at light load
 * (piecewise linear, descending mV). 4200 mV = 100 %, 3300 mV = 0 %.
 */
struct lipo_point {
	int16_t mv;
	uint8_t pct;
};

static const struct lipo_point lipo_curve[] = {
	{4200, 100}, {4100, 90}, {4000, 79}, {3900, 62}, {3800, 42},
	{3700, 23},  {3600, 10}, {3500, 4},  {3400, 1},  {3300, 0},
};

static int lipo_percent(int mv)
{
	if (mv >= lipo_curve[0].mv) {
		return 100;
	}
	for (size_t i = 1; i < ARRAY_SIZE(lipo_curve); i++) {
		const struct lipo_point *hi = &lipo_curve[i - 1];
		const struct lipo_point *lo = &lipo_curve[i];

		if (mv >= lo->mv) {
			int span_mv = hi->mv - lo->mv;
			int span_pct = hi->pct - lo->pct;

			/* span_mv > 0 by construction of the table */
			return lo->pct + ((mv - lo->mv) * span_pct + span_mv / 2) / span_mv;
		}
	}
	return 0;
}

#if defined(CONFIG_WAFER_PAD_BATTERY) && DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)

static const struct adc_dt_spec vbat_chan = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

K_MUTEX_DEFINE(bat_lock);
static bool bat_setup_done;
static bool bat_calibrated;

static int battery_setup_locked(void)
{
	int err;

	if (bat_setup_done) {
		return 0;
	}
	if (!adc_is_ready_dt(&vbat_chan)) {
		LOG_ERR("ADC %s not ready", vbat_chan.dev->name);
		return -ENODEV;
	}
	err = adc_channel_setup_dt(&vbat_chan);
	if (err < 0) {
		LOG_ERR("ADC channel %d setup failed (%d)", vbat_chan.channel_id, err);
		return err;
	}
	bat_setup_done = true;
	return 0;
}

int wp_power_battery_mv(void)
{
	int16_t sample = 0;
	int32_t val;
	struct adc_sequence seq = {
		.buffer = &sample,
		.buffer_size = sizeof(sample),
	};
	int err;

	(void)k_mutex_lock(&bat_lock, K_FOREVER);

	err = battery_setup_locked();
	if (err != 0) {
		goto out;
	}

	err = adc_sequence_init_dt(&vbat_chan, &seq);
	if (err < 0) {
		goto out;
	}
	/* SAADC offset calibration once, on the first conversion. */
	seq.calibrate = !bat_calibrated;

	err = adc_read_dt(&vbat_chan, &seq);
	if (err < 0) {
		LOG_WRN("ADC read failed (%d)", err);
		goto out;
	}
	bat_calibrated = true;

	val = MAX((int32_t)sample, 0);
	err = adc_raw_to_millivolts_dt(&vbat_chan, &val);
	if (err < 0) {
		goto out;
	}
	err = (int)(val * VDDH_DIVIDER);

out:
	(void)k_mutex_unlock(&bat_lock);
	return err;
}

int wp_power_battery_percent(void)
{
	int mv = wp_power_battery_mv();

	if (mv < 0) {
		return mv;
	}
	return lipo_percent(mv);
}

#else /* battery measurement disabled or no zephyr,user io-channels */

int wp_power_battery_mv(void)
{
	return -ENOTSUP;
}

int wp_power_battery_percent(void)
{
	ARG_UNUSED(lipo_percent);
	return -ENOTSUP;
}

#endif
