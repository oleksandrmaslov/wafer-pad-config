/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Automatic orientation detection (pure C, host-testable).
 *
 * PHYSICAL LIMITATION: a 6-axis IMU has no absolute yaw reference. When the
 * pad lies (nearly) flat, gravity carries NO information about which edge
 * faces the user. The estimator therefore:
 *   A) derives orientation from the gravity projection ONLY when the pad is
 *      tilted enough, assuming the conventional posture "raised edge points
 *      away from the user";
 *   B) otherwise keeps (locks) the last confident orientation, never changes
 *      it during an active touch gesture, and rejects samples taken while the
 *      pad is being accelerated (lifted / shaken);
 *   C) accepts an explicit manual orientation from the calibration gesture
 *      (see orientation_calib.h);
 *   D) optionally tracks relative rotation about the pad normal with the
 *      gyroscope after a confident fix (experimental, default OFF).
 *
 * Gravity heading convention (body frame B, see types.h):
 *   uphill direction u = (a_x, a_y) (projection of specific force onto pad)
 *   heading = atan2(-a_x, a_y) in degrees, wrapped to [0, 360)
 *   uphill = +y_B (top edge raised)   -> heading   0 -> WP_ORIENT_0
 *   uphill = -x_B (left edge raised)  -> heading  90 -> WP_ORIENT_90
 *   uphill = -y_B (bottom edge raised)-> heading 180 -> WP_ORIENT_180
 *   uphill = +x_B (right edge raised) -> heading 270 -> WP_ORIENT_270
 */

#ifndef WAFER_PAD_ORIENTATION_H_
#define WAFER_PAD_ORIENTATION_H_

#include <wafer_pad/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Algorithm A: gravity-based estimator                                      */
/* ------------------------------------------------------------------------ */

struct wp_orient_gravity_config {
	float min_tilt_deg;      /**< Below this the pad counts as flat. Default 20. */
	float max_tilt_deg;      /**< Above this (near vertical / face down) reject. Default 75. */
	float max_mag_dev_g;     /**< Max | |a|/g - 1 |. Default 0.12. */
	float hysteresis_deg;    /**< A NEW orientation is only a candidate when the
				  *   heading is within (45 - hysteresis) deg of its
				  *   centre. Default 15. */
	float max_gyro_dps;      /**< Reject while rotating faster (if gyro valid). Default 20. */
	uint32_t confirm_ms;     /**< Candidate must persist this long. Default 300. */
	uint32_t confirm_ms_no_gyro; /**< Same, while the gyro is off (no rate check).
				      *   Default 800. */
	float max_tilt_change_deg;   /**< The tilt must stay within this of its value
				      *   when the candidate timer started, else the
				      *   timer restarts: a pick-up / put-down sweep is
				      *   not a posture. Default 6. */
	float min_confidence;    /**< Required confidence to confirm, 0..1. Default 0.6. */
	float lpf_tau_ms;        /**< Gravity low-pass time constant. Default 50. */
};

/** Fill @p cfg with the documented defaults. */
void wp_orient_gravity_config_default(struct wp_orient_gravity_config *cfg);

struct wp_orient_gravity_result {
	bool valid;                    /**< Sample usable (magnitude, tilt range, rate ok). */
	float tilt_deg;                /**< Angle between pad normal and vertical. */
	float heading_deg;             /**< [0,360), meaningful only if tilt >= min. */
	enum wp_orientation candidate; /**< Orientation currently suggested. */
	float confidence;              /**< 0..1 for the candidate. */
	bool confirmed;                /**< candidate stable >= confirm_ms with conf >= min. */
};

struct wp_orient_gravity {
	struct wp_orient_gravity_config cfg;
	float g_lp[3];                 /**< Low-passed specific force. */
	bool g_init;
	int64_t last_t_us;
	enum wp_orientation reference; /**< Orientation the hysteresis is relative to. */
	bool have_candidate;
	enum wp_orientation candidate;
	int64_t candidate_since_us;
	float heading_ref_deg;         /**< Heading when the candidate started (stability). */
	float tilt_ref_deg;            /**< Tilt when the candidate timer (re)started. */
};

void wp_orient_gravity_init(struct wp_orient_gravity *g, const struct wp_orient_gravity_config *cfg,
			    enum wp_orientation reference);

/** Forget filter state and candidate (e.g. after an orientation change). */
void wp_orient_gravity_reset(struct wp_orient_gravity *g, enum wp_orientation reference);

/** Feed one body-frame sample. Always fills @p out. */
void wp_orient_gravity_update(struct wp_orient_gravity *g, const struct wp_imu_sample *s,
			      struct wp_orient_gravity_result *out);

/* ------------------------------------------------------------------------ */
/* Algorithm D: gyro-assisted relative rotation (experimental)               */
/* ------------------------------------------------------------------------ */

struct wp_orient_gyro_config {
	bool enabled;            /**< Default false for the exhibition build. */
	float step_deg;          /**< Accumulated yaw needed for a 90 deg step. Default 70. */
	float min_rate_dps;      /**< Ignore yaw rate below this (drift). Default 8. */
	uint32_t max_window_ms;  /**< Accumulation forgotten after this. Default 1500. */
	float max_tilt_deg;      /**< Only integrate while |tilt| below this. Default 50. */
};

void wp_orient_gyro_config_default(struct wp_orient_gyro_config *cfg);

/* ------------------------------------------------------------------------ */
/* Orientation manager: A + B + C + D                                        */
/* ------------------------------------------------------------------------ */

enum wp_orient_source {
	WP_ORIENT_SRC_DEFAULT = 0, /**< Boot default, never confirmed. */
	WP_ORIENT_SRC_GRAVITY,     /**< Algorithm A. */
	WP_ORIENT_SRC_MANUAL,      /**< Algorithm C. */
	WP_ORIENT_SRC_GYRO,        /**< Algorithm D. */
	WP_ORIENT_SRC_FORCED,      /**< Set via shell/config. */
};

enum wp_manual_lock_mode {
	WP_MANUAL_LOCK_NONE = 0,     /**< Auto detection may override immediately. */
	WP_MANUAL_LOCK_TIMED,        /**< Auto disabled for manual_lock_ms. */
	WP_MANUAL_LOCK_UNTIL_RECAL,  /**< Auto disabled until next manual calibration
				      *   or wp_orient_mgr_set_auto(true). */
};

struct wp_orient_mgr_config {
	struct wp_orient_gravity_config gravity;
	struct wp_orient_gyro_config gyro;
	bool auto_enabled;                  /**< Algorithm A active. Default true. */
	enum wp_manual_lock_mode manual_lock_mode; /**< Default WP_MANUAL_LOCK_TIMED. */
	uint32_t manual_lock_ms;            /**< Default 30000. */
	uint32_t touch_release_settle_ms;   /**< After last finger up, wait this long
					     *   before applying a deferred change.
					     *   Default 150. */
};

void wp_orient_mgr_config_default(struct wp_orient_mgr_config *cfg);

enum wp_orient_block_reason {
	WP_ORIENT_BLOCK_NONE = 0,
	WP_ORIENT_BLOCK_TOUCH,      /**< Deferred: gesture active. */
	WP_ORIENT_BLOCK_MANUAL_LOCK,
	WP_ORIENT_BLOCK_AUTO_OFF,
	WP_ORIENT_BLOCK_EXTERNAL,   /**< Caller disallowed changes (state machine). */
};

struct wp_orient_status {
	enum wp_orientation current;
	enum wp_orient_source source;
	float confidence;              /**< Confidence of the last accepted fix. */
	bool pending;                  /**< A confirmed change is waiting. */
	enum wp_orientation pending_orientation;
	enum wp_orient_block_reason block;
	struct wp_orient_gravity_result last_gravity;
};

struct wp_orient_mgr {
	struct wp_orient_mgr_config cfg;
	struct wp_orient_gravity gravity;
	enum wp_orientation current;
	enum wp_orient_source source;
	float confidence;
	bool pending;
	enum wp_orientation pending_orientation;
	enum wp_orient_source pending_source;
	bool changed;                  /**< Latched until wp_orient_mgr_poll_change(). */
	uint8_t contacts;
	int64_t last_touch_up_us;
	bool external_allow;
	bool manual_lock_active;
	int64_t manual_lock_until_us;
	int64_t now_us;
	/* gyro assist */
	bool gyro_ref_valid;
	float gyro_accum_deg;
	int64_t gyro_window_start_us;
	struct wp_orient_gravity_result last_gravity;
	enum wp_orient_block_reason block;
};

void wp_orient_mgr_init(struct wp_orient_mgr *m, const struct wp_orient_mgr_config *cfg,
			enum wp_orientation initial);

/** Feed a body-frame IMU sample (drives A and D and time). */
void wp_orient_mgr_on_imu(struct wp_orient_mgr *m, const struct wp_imu_sample *s);

/** Report the number of fingers currently down (drives B). */
void wp_orient_mgr_on_touch(struct wp_orient_mgr *m, uint8_t contacts, int64_t t_us);

/** Advance time without new samples (applies deferred changes, lock expiry). */
void wp_orient_mgr_tick(struct wp_orient_mgr *m, int64_t t_us);

/** Allow/disallow automatic changes (state machine gate, e.g. in AIR_MOUSE). */
void wp_orient_mgr_allow_auto(struct wp_orient_mgr *m, bool allow);

/** Enable/disable algorithm A at runtime (shell / settings). */
void wp_orient_mgr_set_auto(struct wp_orient_mgr *m, bool enabled);

/** Algorithm C result: manual orientation. Applied immediately if no finger is
 *  down, otherwise as soon as all fingers are lifted. Starts the manual lock. */
void wp_orient_mgr_set_manual(struct wp_orient_mgr *m, enum wp_orientation o, int64_t t_us);

/** Force an orientation (shell / persisted setting). Applied like manual. */
void wp_orient_mgr_force(struct wp_orient_mgr *m, enum wp_orientation o, int64_t t_us);

/**
 * Returns true exactly once after the active orientation changed. The caller
 * MUST then reset accumulated cursor offsets, filters and gesture state
 * (gesture engine, HID composer remainders, air mouse filters).
 */
bool wp_orient_mgr_poll_change(struct wp_orient_mgr *m, enum wp_orientation *new_o);

enum wp_orientation wp_orient_mgr_get(const struct wp_orient_mgr *m);

void wp_orient_mgr_get_status(const struct wp_orient_mgr *m, struct wp_orient_status *st);

const char *wp_orient_source_name(enum wp_orient_source src);

#ifdef __cplusplus
}
#endif

#endif /* WAFER_PAD_ORIENTATION_H_ */
