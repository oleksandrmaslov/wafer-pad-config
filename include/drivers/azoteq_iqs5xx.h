/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public API of the out-of-tree Azoteq IQS5xx-B000 driver (azoteq,iqs5xx).
 *
 * The driver owns all I2C traffic to the controller. It only talks to the
 * device inside RDY communication windows during normal operation (the
 * controller clock-stretches the SHARED bus when addressed outside a window,
 * which would stall the IMU). Forced communication is limited to
 * initialization, recovery and the optional stale-contact watchdog.
 *
 * Frames are delivered from the driver's work queue context; the callback
 * must be short and must not block (copy the frame and return).
 */

#ifndef WAFER_PAD_DRIVERS_AZOTEQ_IQS5XX_H_
#define WAFER_PAD_DRIVERS_AZOTEQ_IQS5XX_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IQS5XX_MAX_FINGERS 5

/** Product numbers (register 0x0000). */
#define IQS5XX_PRODUCT_IQS550 40
#define IQS5XX_PRODUCT_IQS572 58
#define IQS5XX_PRODUCT_IQS525 52

/** System Info 1 (iqs5xx_frame.sys_info1) flags that invalidate a frame's XY
 *  data: TOO_MANY_FINGERS clears it (DS §5.3), PALM_DETECT suppresses it
 *  (DS §5.5). Research notes iqs5xx.md §4.3 (System Info 1) and §6.2. */
#define IQS5XX_SYS_INFO1_TOO_MANY_FINGERS (1U << 2)
#define IQS5XX_SYS_INFO1_PALM_DETECT      (1U << 1)

/** Coordinate the controller reserves for slots that are not in touch
 *  (research notes iqs5xx.md §6.2; single source, Linux driver comment). */
#define IQS5XX_COORD_NONE 0xFFFFU

/** One finger slot. The slot index is the controller's tracking id; the slot
 *  is active iff strength != 0 (coordinates of inactive slots are invalid;
 *  a slot reporting IQS5XX_COORD_NONE is never a valid contact). */
struct iqs5xx_finger {
	uint16_t x;
	uint16_t y;
	uint16_t strength;
	uint8_t area;
};

/** One coherent snapshot read in a single burst (0x000C..0x0038). */
struct iqs5xx_frame {
	int64_t timestamp_us;   /**< k_uptime-based time of the RDY interrupt. */
	uint8_t prev_cycle_ms;
	uint8_t gesture_events0;
	uint8_t gesture_events1;
	uint8_t sys_info0;
	uint8_t sys_info1;
	uint8_t num_fingers;
	int16_t rel_x;
	int16_t rel_y;
	struct iqs5xx_finger fingers[IQS5XX_MAX_FINGERS];
};

enum iqs5xx_event {
	IQS5XX_EVT_FRAME = 0,   /**< New frame (frame != NULL). */
	IQS5XX_EVT_RESET,       /**< Controller reset detected (SHOW_RESET) and the
				 *   driver re-applied its configuration. All
				 *   contacts must be considered lifted. */
	IQS5XX_EVT_ERROR,       /**< I2C errors; automatic recovery in progress. */
	IQS5XX_EVT_RECOVERED,   /**< Recovery succeeded. */
	IQS5XX_EVT_FATAL,       /**< Recovery failed; iqs5xx_reinit() required. */
};

typedef void (*iqs5xx_callback_t)(const struct device *dev, enum iqs5xx_event evt,
				  const struct iqs5xx_frame *frame, void *user_data);

struct iqs5xx_info {
	uint16_t product_number;
	uint16_t project_number;
	uint8_t version_major;
	uint8_t version_minor;
	uint16_t resolution_x;  /**< X resolution register (0x066E). */
	uint16_t resolution_y;  /**< Y resolution register (0x0670). */
	uint8_t i2c_timeout_ms; /**< 0x058A, logged for diagnostics. */
};

struct iqs5xx_stats {
	uint32_t frames;
	uint32_t i2c_errors;
	uint32_t device_resets;   /**< SHOW_RESET seen at runtime. */
	uint32_t recoveries;
	uint32_t forced_reads;    /**< Reads outside a RDY window (watchdog). */
};

/** Register the frame/event callback (one consumer). NULL unregisters. */
int iqs5xx_set_callback(const struct device *dev, iqs5xx_callback_t cb, void *user_data);

int iqs5xx_get_info(const struct device *dev, struct iqs5xx_info *info);

int iqs5xx_get_stats(const struct device *dev, struct iqs5xx_stats *stats);

/** Hardware reset (NRST if available, else software reset) and full
 *  re-initialization. Blocks for up to a few hundred milliseconds. */
int iqs5xx_reinit(const struct device *dev);

/** Low-power profile: event mode and slower report rates (true), or the
 *  normal streaming profile (false). */
int iqs5xx_set_low_power(const struct device *dev, bool low_power);

/** Configure the controller so that a touch raises RDY while the MCU is in
 *  System OFF (event mode, low-power rates) and leave RDY as a wake source. */
int iqs5xx_prepare_wakeup(const struct device *dev);

/** Last step before sys_poweroff(), callable with interrupts LOCKED (takes
 *  no lock, never sleeps): re-enable the RDY level sense that a serviced
 *  window may have masked. Returns -EAGAIN if prepare_wakeup() was not done
 *  (or was undone by set_low_power()). */
int iqs5xx_arm_wake_pin(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* WAFER_PAD_DRIVERS_AZOTEQ_IQS5XX_H_ */
