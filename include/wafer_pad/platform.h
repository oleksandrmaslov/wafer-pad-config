/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform (Zephyr-side) subsystem interfaces.
 *
 * Data flow (all producers post into ONE router thread, which owns every
 * algorithm instance, so algorithms need no locking):
 *
 *   IQS5xx driver --frame cb--> touch source --P frame--+
 *   lsm6dso (sensor API) --> imu source --body sample---+--> event router
 *   (future) keyboard halves over BLE --key event-------+      |
 *                                                             v
 *              state machine, orientation, calibration, gestures, air mouse
 *                                                             |
 *                                                     HID report composer
 *                                                             |
 *                                     HID transport (BLE HOG; USB optional)
 *
 * All functions return 0 or a negative errno unless stated otherwise.
 */

#ifndef WAFER_PAD_PLATFORM_H_
#define WAFER_PAD_PLATFORM_H_

#include <wafer_pad/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Event router (src/router)                                                 */
/* ------------------------------------------------------------------------ */

enum wp_touch_status {
	WP_TOUCH_STATUS_OK = 0,     /**< (Re)initialized and streaming. */
	WP_TOUCH_STATUS_RESET,      /**< Controller reset seen: all contacts lost. */
	WP_TOUCH_STATUS_ERROR,      /**< Communication failing, recovery running. */
	WP_TOUCH_STATUS_FATAL,      /**< Gave up; needs re-init from the state machine. */
};

enum wp_imu_status {
	WP_IMU_STATUS_OK = 0,
	WP_IMU_STATUS_ERROR,
	WP_IMU_STATUS_ABSENT,       /**< Not in devicetree or not responding at boot. */
};

/** Create the router thread. Must be called before any producer starts. */
int wp_router_init(void);

/** Post a PAD-frame touch frame (mount correction already applied).
 *  Safe from any thread (not ISR). Never blocks; returns -ENOMSG if full. */
int wp_router_post_touch(const struct wp_touch_frame *pad_frame);
int wp_router_post_touch_status(enum wp_touch_status status);

/** Post a body-frame IMU sample (gyro bias-corrected). Never blocks. */
int wp_router_post_imu(const struct wp_imu_sample *sample);
int wp_router_post_imu_status(enum wp_imu_status status);

/** Transport connection state changed (connected = ready for reports). */
int wp_router_post_hid_connected(bool connected);

/** Transport can accept the next report (e.g. notification sent). */
int wp_router_post_hid_ready(void);

/** FUTURE keyboard producer entry point. Returns -ENOTSUP in v0.1. */
int wp_router_post_key(const struct wp_key_event *ev);

/** Debug / shell: request an orientation (-1 = back to automatic). */
int wp_router_request_orientation(int deg);

/** Debug / shell: cycle or set the mode override (0 auto, 1 handheld, 2 trackpad). */
int wp_router_request_mode(int mode);

struct k_work_q;

/** Application control work queue (preemptible, below the router and the
 *  sensor threads). Runs sensor power-mode changes, re-initialization,
 *  connection-parameter requests, settings writes and System OFF entry, so
 *  that none of this blocks the system work queue the Bluetooth host needs
 *  for TX completions. Valid after wp_router_init(). */
struct k_work_q *wp_router_ctl_queue(void);

struct wp_router_status {
	int state;                  /**< enum wp_state */
	int mode;                   /**< enum wp_mode_override */
	int orientation_deg;
	int orientation_source;     /**< enum wp_orient_source */
	float orientation_confidence;
	float tilt_deg;
	float heading_deg;
	uint8_t contacts;
	bool hid_connected;
	bool imu_present;
	bool gyro_on;
	uint32_t touch_frames;
	uint32_t imu_samples;
	uint32_t reports_sent;
	uint32_t queue_drops;
	uint32_t max_latency_us;    /**< Max frame-timestamp -> report-send latency. */
	/* Latest body-frame IMU sample seen by the router (shell "wp imu"). */
	bool imu_sample_valid;      /**< At least one sample received. */
	int64_t imu_t_us;           /**< Its timestamp (k_uptime based). */
	float accel[3];             /**< m/s^2, body frame (specific force, +z up when flat). */
	float gyro_dps[3];          /**< deg/s, body frame, bias corrected. */
	bool gyro_valid;
	bool gravity_valid;         /**< tilt/heading from a valid gravity sample. */
};

void wp_router_get_status(struct wp_router_status *st);

/* ------------------------------------------------------------------------ */
/* Touch source (src/touch) — wraps the controller driver                    */
/* ------------------------------------------------------------------------ */

/** Bind to DT alias `touch0`, reset + initialize the controller, start posting
 *  frames to the router. Returns -ENODEV if the alias/device is missing. */
int wp_touch_source_init(void);

/** Pad-frame geometry (after the mount correction). */
int wp_touch_source_get_geometry(struct wp_pad_geometry *pad_geom);

/** Enter/leave the controller's low-power reporting mode. */
int wp_touch_source_set_low_power(bool low_power);

/** Hardware reset + re-initialization (used by ERROR_RECOVERY). */
int wp_touch_source_reinit(void);

/** Prepare the controller to wake the MCU from System OFF (RDY). */
int wp_touch_source_prepare_wakeup(void);

/** Final step before sys_poweroff(), called with interrupts LOCKED: re-apply
 *  the RDY level sense (a RDY interrupt serviced since prepare_wakeup()
 *  masks it). Never blocks. No-op unless prepare_wakeup() armed the wake-up. */
int wp_touch_source_arm_wake_pin(void);

/* ------------------------------------------------------------------------ */
/* IMU source (src/imu) — wraps the Zephyr sensor API (st,lsm6dso)           */
/* ------------------------------------------------------------------------ */

/** Bind to DT alias `motion0`, configure ranges/ODR, start sampling and
 *  posting body-frame samples. Returns -ENODEV if absent (device optional). */
int wp_imu_source_init(void);

bool wp_imu_source_present(void);

/** Power the gyroscope up/down (power gating). */
int wp_imu_source_set_gyro(bool enabled);

/** Low data rate for IDLE. */
int wp_imu_source_set_low_rate(bool low_rate);

/** Current gyro bias estimate (rad/s, body frame); returns false if invalid. */
bool wp_imu_source_get_bias(float bias[3]);

/** Re-initialize after an error. */
int wp_imu_source_reinit(void);

/** Prepare the IMU to wake the MCU from System OFF (INT1), if wired. The
 *  INT1 pin interrupt itself is NOT enabled here (see arm_wake_pin). */
int wp_imu_source_prepare_wakeup(void);

/** Final step before sys_poweroff(), called with interrupts LOCKED: enable
 *  the INT1 level sense as wake source. Never blocks. No-op unless
 *  prepare_wakeup() armed the IMU wake-up. */
void wp_imu_source_arm_wake_pin(void);

/** Allow (default) or forbid gyro bias learning. The router forbids it in
 *  AIR_MOUSE, where a steadily aimed rotation must never be learnt as bias. */
void wp_imu_source_set_bias_learning(bool allowed);

/* ------------------------------------------------------------------------ */
/* HID transport (src/hid)                                                   */
/* ------------------------------------------------------------------------ */

struct wp_hid_transport_api {
	const char *name;
	int (*init)(void);
	/** Connected, subscribed and able to accept a report right now. */
	bool (*is_ready)(void);
	int (*send_mouse)(const struct wp_mouse_report *r);
	/** Incremented on every transition to "ready" (new link, CCC
	 *  re-enabled, ...). */
	uint32_t (*link_gen)(void);
};

/** Initialize all enabled transports (BLE always; USB if configured). */
int wp_hid_init(void);

/** True if the active transport can accept a report now. */
bool wp_hid_is_ready(void);

/** Send to the active transport (USB preferred when connected, else BLE). */
int wp_hid_send_mouse(const struct wp_mouse_report *r);

/** Changes whenever any transport became ready again since the last call
 *  could have observed it: the host may have missed reports, so the full
 *  button state must be re-sent. */
uint32_t wp_hid_link_gen(void);

/* ------------------------------------------------------------------------ */
/* BLE (src/hid/ble*.c)                                                      */
/* ------------------------------------------------------------------------ */

bool wp_ble_is_connected(void);

/** Request short (true) or power-saving (false) connection parameters. */
int wp_ble_set_low_latency(bool low_latency);

/** Remove all bonds and restart advertising. */
int wp_ble_unpair_all(void);

/** Accept new pairings for @p seconds (0 closes the window). Pairing is also
 *  accepted while no bond exists and for CONFIG_WAFER_PAD_BLE_PAIRING_WINDOW_S
 *  after boot; otherwise only bonded hosts can (re)connect. */
int wp_ble_open_pairing_window(uint32_t seconds);

/** Seconds left in the pairing window (0 = closed). */
uint32_t wp_ble_pairing_window_left(void);

/** Update the Battery Service level (0..100). */
int wp_ble_set_battery_level(uint8_t percent);

/* ------------------------------------------------------------------------ */
/* Power (src/power)                                                         */
/* ------------------------------------------------------------------------ */

int wp_power_init(void);

/** Configure wake sources and enter System OFF. Does not return on success. */
int wp_power_enter_sleep(void);

/** Battery voltage in mV, or negative errno if not measurable. */
int wp_power_battery_mv(void);

/** Battery state of charge estimate 0..100, or negative errno. */
int wp_power_battery_percent(void);

/* ------------------------------------------------------------------------ */
/* Status LED (src/power/led.c)                                              */
/* ------------------------------------------------------------------------ */

enum wp_led_pattern {
	WP_LED_OFF = 0,
	WP_LED_ADVERTISING,   /**< slow blink */
	WP_LED_CONNECTED,     /**< short blip every few seconds */
	WP_LED_CALIB_ARMED,   /**< fast blink */
	WP_LED_CALIB_DONE,    /**< N blinks = orientation/90 + 1 */
	WP_LED_AIR_MOUSE,     /**< solid on */
	WP_LED_ERROR,         /**< double blink */
	WP_LED_MODE_CHANGE,   /**< N blinks = mode + 1 */
};

int wp_led_init(void);
void wp_led_set(enum wp_led_pattern pattern, int arg);

#ifdef __cplusplus
}
#endif

#endif /* WAFER_PAD_PLATFORM_H_ */
