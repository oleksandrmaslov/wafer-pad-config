/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wafer Pad entry point: brings the subsystems up in dependency order and
 * returns. All work is done by the router thread, the sensor threads, the
 * BLE stack and the system work queue.
 *
 * The init results of the sensors are posted to the router (which drives
 * the state machine INITIALIZING -> TRACKPAD / ERROR_RECOVERY). The touch
 * source posts its own status (main only reports an init failure); the IMU
 * source posts the same status main does, which the router handles
 * idempotently.
 * The trackpad works without the IMU (motion features invalid).
 *
 * A failed HID (BLE) bring-up cannot be retried in place (bt_enable() is not
 * re-entrant after a half-way failure), so the pad reboots after a short
 * delay, at most WP_HID_FAIL_REBOOTS times in a row (counter kept in
 * no-init RAM across warm resets); after that it keeps running with the
 * error LED so the cause can be read on the shell.
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/version.h>

#include <wafer_pad/platform.h>

LOG_MODULE_REGISTER(wp_main, CONFIG_WAFER_PAD_LOG_LEVEL);

#if __has_include(<app_version.h>)
#include <app_version.h>
#define WP_FW_VERSION APP_VERSION_STRING
#else
#define WP_FW_VERSION "0.1.0"
#endif

#define WP_HID_FAIL_REBOOTS     3
#define WP_HID_FAIL_REBOOT_S    5
#define WP_HID_FAIL_MAGIC       0x57504846U /* "WPHF" */

/* Survives warm resets (sys_reboot); random after power-on -> magic check. */
static __noinit struct {
	uint32_t magic;
	uint32_t count;
} hid_fail;

static void hid_fail_reboot_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_ERR("rebooting to recover the HID transport");
	sys_reboot(SYS_REBOOT_COLD);
}

static K_WORK_DELAYABLE_DEFINE(hid_fail_reboot_work, hid_fail_reboot_fn);

static void hid_init_result(int err)
{
	if (hid_fail.magic != WP_HID_FAIL_MAGIC) {
		hid_fail.magic = WP_HID_FAIL_MAGIC;
		hid_fail.count = 0U;
	}
	if (err == 0) {
		hid_fail.count = 0U;
		return;
	}

	LOG_ERR("HID init failed (%d)", err);
	wp_led_set(WP_LED_ERROR, 0);
	if (hid_fail.count >= WP_HID_FAIL_REBOOTS) {
		LOG_ERR("HID init failed %u times in a row: not rebooting again",
			hid_fail.count);
		return;
	}
	hid_fail.count++;
	(void)k_work_schedule(&hid_fail_reboot_work, K_SECONDS(WP_HID_FAIL_REBOOT_S));
}

int main(void)
{
	int err;

	LOG_INF("Wafer Pad firmware v%s (Zephyr %s)", WP_FW_VERSION, KERNEL_VERSION_STRING);

	err = wp_led_init();
	if (err != 0) {
		LOG_WRN("status LED unavailable (%d)", err);
	}

	/* The router must exist before any producer starts posting. */
	err = wp_router_init();
	if (err != 0) {
		LOG_ERR("router init failed (%d)", err);
		wp_led_set(WP_LED_ERROR, 0);
		return err;
	}

	err = wp_hid_init();
	hid_init_result(err);

	/* On success the touch source reports the controller state itself: OK
	 * or FATAL now if the asynchronous bring-up already finished, otherwise
	 * RESET+OK / FATAL from the driver callback later. Posting OK here would
	 * mask a FATAL (the router's latch keeps the newest status).
	 */
	err = wp_touch_source_init();
	if (err != 0) {
		LOG_ERR("touch init failed (%d): state machine will retry", err);
		(void)wp_router_post_touch_status(WP_TOUCH_STATUS_FATAL);
	}

	err = wp_imu_source_init();
	if (err == -ENODEV) {
		LOG_INF("no IMU: trackpad-only operation");
		(void)wp_router_post_imu_status(WP_IMU_STATUS_ABSENT);
	} else if (err != 0) {
		LOG_WRN("IMU init failed (%d): retrying in the background", err);
		(void)wp_router_post_imu_status(WP_IMU_STATUS_ERROR);
	} else {
		(void)wp_router_post_imu_status(WP_IMU_STATUS_OK);
	}

	err = wp_power_init();
	if (err != 0) {
		LOG_WRN("power init failed (%d)", err);
	}

	LOG_INF("init complete");
	return 0;
}
