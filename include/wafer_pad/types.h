/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Common, hardware-independent data types shared by every Wafer Pad module.
 *
 * This header MUST stay free of Zephyr includes: the algorithm modules that
 * use it (orientation, gestures, air mouse, state machine, HID composer) are
 * unit-tested on the host (native_sim) without any hardware.
 *
 * ---------------------------------------------------------------------------
 * Coordinate frames (authoritative definition, see docs/architecture.md)
 * ---------------------------------------------------------------------------
 *
 * Sensor frame S (2D)
 *   Raw coordinates as reported by the touch controller (IQS5xx counts).
 *
 * Pad frame P (2D)
 *   S after the FIXED mounting correction (swap / invert, struct
 *   wp_touch_mount).  Viewed from the touch surface: +x points to the pad's
 *   right edge, +y points to the pad's bottom edge.  Origin = top-left corner.
 *   The edge at y == 0 is the pad's REFERENCE ("top") edge.
 *   Units: controller counts, 0..max_x / 0..max_y.
 *
 * Body frame B (3D, right-handed, used for IMU data)
 *   x_B = +x_P (towards right edge)
 *   y_B = -y_P (towards the reference/top edge)
 *   z_B = out of the touch surface (towards the finger)
 *   The IMU's own axes are mapped into B by struct wp_imu_mount.
 *   Accelerometer values are specific force: at rest, face up, a = (0,0,+g).
 *   Gyroscope values follow the right-hand rule around the B axes.
 *
 * User frame U (2D)
 *   What the person holding the pad perceives: +x = user's right,
 *   +y = towards the user ("down", same as HID +Y).  U = R(orientation) * P.
 *
 * Orientation (enum wp_orientation)
 *   Clockwise rotation of the pad, viewed from above the touch surface,
 *   relative to the reference posture in which the reference (top) edge
 *   faces AWAY from the user.
 *     WP_ORIENT_0   : top    edge (P y==0)    away from user
 *     WP_ORIENT_90  : left   edge (P x==0)    away from user
 *     WP_ORIENT_180 : bottom edge (P y==max)  away from user
 *     WP_ORIENT_270 : right  edge (P x==max)  away from user
 *   Delta mapping P -> U:
 *     0: ( dx,  dy)   90: (-dy,  dx)   180: (-dx, -dy)   270: ( dy, -dx)
 */

#ifndef WAFER_PAD_TYPES_H_
#define WAFER_PAD_TYPES_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WP_BIT(n) (1UL << (n))

#define WP_US_PER_MS   1000LL
#define WP_GRAVITY_MS2 9.80665f
#define WP_PI          3.14159265358979f
#define WP_DEG_TO_RAD  (WP_PI / 180.0f)
#define WP_RAD_TO_DEG  (180.0f / WP_PI)

/* ------------------------------------------------------------------------ */
/* Orientation                                                              */
/* ------------------------------------------------------------------------ */

enum wp_orientation {
	WP_ORIENT_0 = 0,
	WP_ORIENT_90 = 1,
	WP_ORIENT_180 = 2,
	WP_ORIENT_270 = 3,
};

#define WP_ORIENT_COUNT 4

/** Orientation in degrees (0, 90, 180, 270). */
static inline int wp_orientation_deg(enum wp_orientation o)
{
	return (int)o * 90;
}

/* ------------------------------------------------------------------------ */
/* Touch                                                                    */
/* ------------------------------------------------------------------------ */

/** Physical description of the touch surface in a given frame. */
struct wp_pad_geometry {
	uint16_t max_x;   /**< Largest x coordinate (counts). */
	uint16_t max_y;   /**< Largest y coordinate (counts). */
	float width_mm;   /**< Physical extent along x. */
	float height_mm;  /**< Physical extent along y. */
};

/** Fixed mounting correction applied by the touch producer: S -> P. */
struct wp_touch_mount {
	bool swap_xy;   /**< Applied first: exchange x and y (and max_x/max_y). */
	bool invert_x;  /**< Then: x = max_x - x. */
	bool invert_y;  /**< Then: y = max_y - y. */
};

#define WP_TOUCH_MAX_CONTACTS 5

/** One finger contact. */
struct wp_touch_contact {
	uint8_t id;        /**< Stable tracking id while the finger stays down. */
	uint16_t x;        /**< Position in the frame's coordinate system. */
	uint16_t y;
	uint16_t strength; /**< Controller touch strength (0 = no touch). */
	uint8_t area;      /**< Controller touch area/size (0 if unknown). */
};

/**
 * Complete snapshot of all fingers at one instant ("normalized touch frame").
 * Produced by the touch source, independent of the controller type.
 * contacts[0..count-1] are valid, ordered by ascending id.
 */
struct wp_touch_frame {
	int64_t t_us;      /**< Capture timestamp (monotonic microseconds). */
	uint16_t max_x;    /**< Coordinate extent of this frame. */
	uint16_t max_y;
	uint8_t count;     /**< Number of valid contacts. */
	struct wp_touch_contact contacts[WP_TOUCH_MAX_CONTACTS];
};

enum wp_touch_event_type {
	WP_TOUCH_DOWN = 0, /**< Finger appeared. */
	WP_TOUCH_MOVE,     /**< Finger moved (only emitted if position changed). */
	WP_TOUCH_UP,       /**< Finger lifted (or contact lost / sensor reset). */
};

/** Normalized per-finger event ("TouchEvent"). Derived from frames. */
struct wp_touch_event {
	enum wp_touch_event_type type;
	uint8_t id;
	uint16_t x;        /**< Current (DOWN/MOVE) or last known (UP) position. */
	uint16_t y;
	int32_t dx;        /**< Movement since previous event of this id; 0 for DOWN/UP. */
	int32_t dy;
	int64_t t_us;
};

/* ------------------------------------------------------------------------ */
/* IMU                                                                      */
/* ------------------------------------------------------------------------ */

/** One IMU sample, already rotated into the body frame B ("ImuSample"). */
struct wp_imu_sample {
	int64_t t_us;      /**< Capture timestamp (same clock as touch frames). */
	float accel[3];    /**< Specific force, m/s^2, body frame. */
	float gyro[3];     /**< Angular rate, rad/s, body frame, bias-corrected. */
	bool gyro_valid;   /**< false while the gyroscope is powered down. */
	bool restart;      /**< First sample after a sensor reconfiguration (data
			    *   rate change, re-init): filters that compare against
			    *   earlier samples restart from this one. */
};

/** Mapping IMU axes -> body axes. body[i] = sign(map[i]) * imu[|map[i]| - 1].
 *  Valid entries are +-1, +-2, +-3 and the absolute values must be a
 *  permutation of {1,2,3}. Identity = {1, 2, 3}. */
struct wp_imu_mount {
	int8_t map[3];
};

/* ------------------------------------------------------------------------ */
/* HID                                                                      */
/* ------------------------------------------------------------------------ */

#define WP_BTN_LEFT    WP_BIT(0)
#define WP_BTN_RIGHT   WP_BIT(1)
#define WP_BTN_MIDDLE  WP_BIT(2)
#define WP_BTN_BACK    WP_BIT(3)
#define WP_BTN_FORWARD WP_BIT(4)
#define WP_BTN_ALL     0x1Fu

/** Logical mouse input report (packed to bytes by hid_report.h). */
struct wp_mouse_report {
	uint8_t buttons;   /**< WP_BTN_* bitmask, current pressed state. */
	int16_t dx;        /**< Relative X, HID units, +right.  Range -32767..32767. */
	int16_t dy;        /**< Relative Y, HID units, +down.   Range -32767..32767. */
	int8_t wheel;      /**< Vertical wheel detents, +up (away from user). -127..127. */
	int8_t pan;        /**< Horizontal AC Pan, +right. -127..127. */
};

/* ------------------------------------------------------------------------ */
/* Keyboard (FUTURE: Wafer split keyboard halves, not implemented in v0.1)  */
/* ------------------------------------------------------------------------ */

enum wp_key_source {
	WP_KEY_SRC_LOCAL = 0,
	WP_KEY_SRC_LEFT_HALF,
	WP_KEY_SRC_RIGHT_HALF,
};

/** Key event reserved for the future keyboard producer. */
struct wp_key_event {
	enum wp_key_source source;
	uint16_t usage_page; /**< HID usage page (0x07 keyboard, 0x0C consumer). */
	uint16_t usage;      /**< HID usage id. */
	bool pressed;
	int64_t t_us;
};

#ifdef __cplusplus
}
#endif

#endif /* WAFER_PAD_TYPES_H_ */
