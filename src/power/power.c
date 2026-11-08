/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Power management: reset-cause logging, periodic battery reporting and
 * System OFF entry.
 *
 * WHEN to sleep is decided by the state machine (sleep_enabled and its
 * timeouts); this module only EXECUTES the request (CONFIG_WAFER_PAD_SLEEP
 * merely makes it possible). wp_power_enter_sleep() is called by the router
 * from its control work queue, never from the router thread.
 *
 * The P0.13 sensor rail (3V3 header pin, regulator-always-on) is NEVER
 * switched off here: GPIO output latches are retained in System OFF, so the
 * TPS65 stays powered and can wake the MCU through RDY.
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_HWINFO)
#include <zephyr/drivers/hwinfo.h>
#endif
#if defined(CONFIG_POWEROFF)
#include <zephyr/sys/poweroff.h>
#endif

#include <wafer_pad/platform.h>

LOG_MODULE_REGISTER(wp_power, CONFIG_WAFER_PAD_LOG_LEVEL);

/* Lets the deferred logger drain before System OFF (work queue context). */
#define SLEEP_LOG_FLUSH_MS 100
/* First battery reading after boot (lets BLE come up first). */
#define BATTERY_FIRST_DELAY_S 2

#if defined(CONFIG_WAFER_PAD_BATTERY)
static struct k_work_delayable bat_work;
static int bat_last_pct = -1;

static void bat_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	int mv = wp_power_battery_mv();
	int pct = wp_power_battery_percent();

	if (mv >= 0 && pct >= 0) {
		LOG_DBG("battery %d mV, %d %%", mv, pct);
		if (pct != bat_last_pct) {
			int err = wp_ble_set_battery_level((uint8_t)pct);

			/* Retry on the next period if BAS is not ready yet. */
			bat_last_pct = (err == 0) ? pct : -1;
		}
	} else {
		LOG_DBG("battery not measurable (%d)", mv);
	}

	(void)k_work_schedule(dwork, K_SECONDS(CONFIG_WAFER_PAD_BATTERY_INTERVAL_S));
}
#endif /* CONFIG_WAFER_PAD_BATTERY */

static void log_reset_cause(void)
{
#if defined(CONFIG_HWINFO)
	static const struct {
		uint32_t bit;
		const char *name;
	} causes[] = {
		{RESET_PIN, "pin"},
		{RESET_SOFTWARE, "software"},
		{RESET_BROWNOUT, "brownout"},
		{RESET_POR, "power-on"},
		{RESET_WATCHDOG, "watchdog"},
		{RESET_DEBUG, "debug"},
		{RESET_LOW_POWER_WAKE, "wake from System OFF"},
		{RESET_CPU_LOCKUP, "CPU lockup"},
	};
	uint32_t cause = 0;
	int err = hwinfo_get_reset_cause(&cause);

	if (err != 0) {
		LOG_DBG("reset cause unavailable (%d)", err);
		return;
	}

	if (cause == 0U) {
		LOG_INF("reset cause: power-on (no flags)");
	}
	for (size_t i = 0; i < ARRAY_SIZE(causes); i++) {
		if ((cause & causes[i].bit) != 0U) {
			LOG_INF("reset cause: %s", causes[i].name);
		}
	}
	if ((cause & RESET_LOW_POWER_WAKE) != 0U) {
		LOG_INF("woke up from System OFF");
	}

	(void)hwinfo_clear_reset_cause();
#endif
}

int wp_power_init(void)
{
	log_reset_cause();

#if defined(CONFIG_WAFER_PAD_BATTERY)
	k_work_init_delayable(&bat_work, bat_work_handler);
	(void)k_work_schedule(&bat_work, K_SECONDS(BATTERY_FIRST_DELAY_S));
#endif

	LOG_INF("power: sleep %s, battery %s",
		IS_ENABLED(CONFIG_WAFER_PAD_SLEEP) ? "available" : "disabled",
		IS_ENABLED(CONFIG_WAFER_PAD_BATTERY) ? "monitored" : "off");
	return 0;
}

int wp_power_enter_sleep(void)
{
#if defined(CONFIG_WAFER_PAD_SLEEP) && defined(CONFIG_POWEROFF)
	int sources = 0;
	int err;

	err = wp_touch_source_prepare_wakeup();
	if (err == 0) {
		sources++;
	} else if (err != -ENOTSUP) {
		LOG_ERR("touch wake-up not armed (%d)", err);
	}

	err = wp_imu_source_prepare_wakeup();
	if (err == 0) {
		sources++;
	} else if (err != -ENOTSUP) {
		LOG_WRN("IMU wake-up not armed (%d)", err);
	}

	if (sources == 0) {
		/* Without a wake source only the reset button would revive the
		 * pad: refuse (the router keeps the device running). */
		LOG_ERR("no wake-up source armed: not entering System OFF");
		/* Undo the preparation: the IMU's prepare_wakeup() suspends its
		 * sampling thread and powers the sensor down even when it returns
		 * an error, and only reinit() resumes it. Restore the normal touch
		 * profile too (best effort; the router re-applies power modes).
		 */
		(void)wp_imu_source_reinit();
		(void)wp_touch_source_set_low_power(false);
		return -ENOTSUP;
	}

#if defined(CONFIG_WAFER_PAD_BATTERY)
	(void)k_work_cancel_delayable(&bat_work);
#endif
	wp_led_set(WP_LED_OFF, 0);

	LOG_INF("System OFF (%d wake source(s))", sources);
	k_msleep(SLEEP_LOG_FLUSH_MS);

	/*
	 * Arm the wake pins LAST, with interrupts locked (sys_poweroff() locks
	 * them again and never returns): a touch window serviced during the
	 * sleep above has masked the RDY sense, and the IMU INT1 level
	 * interrupt has no consumer, so with interrupts enabled an early INT1
	 * would make the GPIOTE ISR re-fire forever. A level that is already
	 * active simply wakes the SoC from System OFF at once (a reboot).
	 */
	(void)irq_lock();
	(void)wp_touch_source_arm_wake_pin();
	wp_imu_source_arm_wake_pin();
	sys_poweroff();
#else
	LOG_WRN("sleep requested but CONFIG_WAFER_PAD_SLEEP is disabled");
	return -ENOTSUP;
#endif
}
