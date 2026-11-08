/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMU signal processing helpers (pure C, host-testable):
 *   - gyroscope bias estimation while stationary
 *   - motion features used by the state machine (still / moving / tremor)
 *
 * All inputs are BODY-frame values (see types.h). Gyro inputs to the bias
 * estimator are RAW (not yet bias corrected), in rad/s.
 */

#ifndef WAFER_PAD_IMU_ALGO_H_
#define WAFER_PAD_IMU_ALGO_H_

#include <wafer_pad/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Gyroscope bias estimator                                                  */
/* ------------------------------------------------------------------------ */

/*
 * The bias is learnt only while the device is really at rest (on a table):
 *   - |raw - bias| < still_gyro_dps (1 dps): a slow deliberate rotation of a
 *     few dps (air-mouse aiming) is never absorbed into the bias;
 *   - gyro AC noise RMS (|raw - lp(raw)|, 500 ms low-pass) < max_noise_dps:
 *     a hand always shows tremor (> 0.35 dps), a table only sensor noise;
 *   - dynamic acceleration < still_accel_dev_g.
 * Re-learning: if the device is quiet (noise gate, accel jitter RMS below
 * relearn_accel_rms_g) for relearn_ms but the rate stays more than
 * still_gyro_dps away from the bias, the bias is wrong (e.g. a stale
 * persisted value) and is replaced by the low-passed rate.
 */
struct wp_gyro_bias_config {
	float still_gyro_dps;     /**< |raw - bias| below this counts as still. Default 1.0. */
	float still_accel_dev_g;  /**< |a| change vs. low-pass below this. Default 0.03. */
	uint32_t still_min_ms;    /**< Stillness needed before (re)estimating. Default 800. */
	float alpha;              /**< EMA factor per sample once still. Default 0.02. */
	float max_bias_dps;       /**< Reject estimates above this. Default 10. */
	float max_noise_dps;      /**< Gyro AC noise RMS above this is not at rest. Default 0.15. */
	uint32_t relearn_ms;      /**< Quiet time before a wrong bias is replaced. Default 5000. */
	float relearn_accel_rms_g;/**< Accel jitter RMS limit for re-learning. Default 0.002. */
};

void wp_gyro_bias_config_default(struct wp_gyro_bias_config *cfg);

struct wp_gyro_bias {
	struct wp_gyro_bias_config cfg;
	float bias[3];            /**< rad/s */
	bool valid;               /**< At least one full stationary estimate done (or restored). */
	bool still;
	int64_t still_since_us;
	float acc_lp[3];
	bool acc_lp_init;
	int64_t last_t_us;
	/* accumulation for the first estimate */
	float sum[3];
	uint32_t n;
	/* rest detection */
	float g_lp[3];            /**< Low-passed raw gyro (rad/s). */
	bool g_lp_init;
	float noise_dps2;         /**< EMA of |raw - g_lp|^2 (dps^2). */
	float jitter_g2;          /**< EMA of the dynamic acceleration^2 (g^2). */
	bool quiet;
	int64_t quiet_since_us;
};

void wp_gyro_bias_init(struct wp_gyro_bias *b, const struct wp_gyro_bias_config *cfg);

/** Restore a previously persisted bias (rad/s); marks it valid. */
void wp_gyro_bias_set(struct wp_gyro_bias *b, const float bias[3]);

/**
 * Feed one raw sample. Returns true while the device is considered still.
 * The first estimate is the mean over the first still_min_ms of stillness;
 * afterwards the bias tracks slowly with an EMA while still.
 * Call wp_gyro_bias_restart() after the gyro was powered down/up.
 */
bool wp_gyro_bias_update(struct wp_gyro_bias *b, int64_t t_us, const float raw_gyro[3],
			 const float accel[3]);

/** Reset stillness tracking (not the bias itself), e.g. after gyro power-up. */
void wp_gyro_bias_restart(struct wp_gyro_bias *b);

/** out = raw - bias (zero bias if not yet valid). @p raw and @p out may alias. */
void wp_gyro_bias_apply(const struct wp_gyro_bias *b, const float raw[3], float out[3]);

bool wp_gyro_bias_valid(const struct wp_gyro_bias *b);

/* ------------------------------------------------------------------------ */
/* Motion features                                                           */
/* ------------------------------------------------------------------------ */

struct wp_motion_config {
	float still_gyro_dps;      /**< Gyro magnitude below this may be still. Default 1.5. */
	float still_accel_dev_g;   /**< Dynamic accel below this may be still. Default 0.015. */
	uint32_t still_window_ms;  /**< Continuous stillness needed for `still`. Default 300. */
	float move_gyro_dps;       /**< Gyro magnitude above this = moving. Default 20. */
	float move_accel_dev_g;    /**< Dynamic accel above this = moving. Default 0.06. */
	float tremor_tau_ms;       /**< EMA time constant for tremor RMS. Default 250. */
	float tremor_hp_tau_ms;    /**< Gyro tremor is the AC part |w - lp(w)| with this
				    *   low-pass time constant, so a residual gyro bias
				    *   or a slow steady rotation is not "tremor".
				    *   Default 500. */
	float accel_lp_tau_ms;     /**< Baseline (gravity) low-pass. Default 400. */
	float handheld_tremor_dps; /**< Gyro tremor RMS above this suggests a hand. Default 0.35. */
	float handheld_accel_jitter_g; /**< Accel jitter RMS above this suggests a hand
					*   (used when gyro is off). Default 0.004. */
};

void wp_motion_config_default(struct wp_motion_config *cfg);

struct wp_motion_features {
	bool valid;
	bool still;              /**< Table-like stillness for >= still_window_ms. */
	bool moving;             /**< Clear deliberate motion in this sample. */
	bool hand_tremor;        /**< Tremor level consistent with being held. */
	float gyro_mag_dps;      /**< |gyro| (0 if gyro invalid). */
	float accel_dev_g;       /**< |a - a_lp| / g (dynamic acceleration). */
	float tremor_dps;        /**< RMS of the gyro AC part |w - lp(w)| (EMA). 0 if
				  *   gyro invalid. */
	float accel_jitter_g;    /**< RMS of dynamic acceleration (EMA). */
	bool tremor_from_gyro;   /**< hand_tremor was derived from the gyroscope (not
				  *   from the accel-jitter fallback used while the
				  *   gyro is off). Equals the sample's gyro_valid
				  *   whatever hand_tremor is, so false means the
				  *   features are accel-only. */
	float tilt_deg;          /**< Angle between pad normal (z_B) and vertical. */
	bool face_up;            /**< a_z > 0 */
	uint32_t still_ms;       /**< Duration of current stillness (0 if not still). */
};

struct wp_motion {
	struct wp_motion_config cfg;
	float acc_lp[3];
	bool init;
	int64_t last_t_us;
	int64_t still_candidate_since_us;
	bool still_candidate;
	float tremor_ms2;        /**< EMA of |w - lp(w)|^2 (dps^2). */
	float gyro_lp[3];        /**< Low-passed gyro (dps) for the tremor AC part. */
	bool gyro_lp_init;
	float jitter_ms2;        /**< EMA of accel_dev^2 (g^2). */
};

void wp_motion_init(struct wp_motion *m, const struct wp_motion_config *cfg);
void wp_motion_reset(struct wp_motion *m);
void wp_motion_update(struct wp_motion *m, const struct wp_imu_sample *s,
		      struct wp_motion_features *out);

/* ------------------------------------------------------------------------ */
/* Sensor plausibility watchdog                                              */
/* ------------------------------------------------------------------------ */

/*
 * A silent sensor reset (supply glitch) leaves the LSM6DSOX in power-down:
 * reads keep succeeding but return zeros, so no read error ever triggers a
 * re-init. Readings that are physically implausible for MORE than dead_ms
 * flag the sensor as dead:
 *   - |accel| < min_accel_g: only possible in free fall (a drop from 1.2 m
 *     lasts ~0.5 s), never while the device rests or is held;
 *   - gyro exactly (0, 0, 0) while the gyro is on and settled: a live gyro
 *     always shows a few LSB of noise and zero-rate offset.
 */
struct wp_imu_plaus_config {
	float min_accel_g;   /**< |accel| below this is implausible. Default 0.2. */
	uint32_t dead_ms;    /**< Implausible for longer than this = dead. 0 disables
			      *   the watchdog. Default 500. */
};

void wp_imu_plaus_config_default(struct wp_imu_plaus_config *cfg);

#define WP_IMU_PLAUS_ACCEL_DEAD WP_BIT(0) /**< |accel| < min_accel_g too long. */
#define WP_IMU_PLAUS_GYRO_DEAD  WP_BIT(1) /**< gyro exactly 0,0,0 too long. */

struct wp_imu_plaus {
	struct wp_imu_plaus_config cfg;
	bool accel_bad;              /**< Current sample implausible. */
	int64_t accel_bad_since_us;
	bool gyro_bad;
	int64_t gyro_bad_since_us;
};

void wp_imu_plaus_init(struct wp_imu_plaus *p, const struct wp_imu_plaus_config *cfg);

/** Forget the timers (after a re-init or a pause in sampling). */
void wp_imu_plaus_reset(struct wp_imu_plaus *p);

/**
 * Feed one sample. @p accel_ms2 in m/s^2 (any frame). @p gyro_raw: the gyro
 * reading BEFORE bias correction (any unit / frame), or NULL while the gyro
 * is off or still settling (this also restarts the gyro timer).
 * @return WP_IMU_PLAUS_* flags of readings implausible for more than
 *         dead_ms (0: plausible, or not for long enough yet).
 */
uint32_t wp_imu_plaus_update(struct wp_imu_plaus *p, int64_t t_us, const float accel_ms2[3],
			     const float *gyro_raw);

#ifdef __cplusplus
}
#endif

#endif /* WAFER_PAD_IMU_ALGO_H_ */
