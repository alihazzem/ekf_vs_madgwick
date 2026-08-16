/**
 * @file config_ekf.h
 * @brief Filter selection and tuning parameters for the EKF and Madgwick filters.
 * Two independently-tuned parameter sets — one per ROBOT_MODE.
 *
 * Included automatically by app_config.h — do not include directly.
 */
#ifndef APP_CONFIG_EKF_H
#define APP_CONFIG_EKF_H

/* ====== ENABLE/DISABLE FILTERS ====== */
#define RUN_MADGWICK 0
#define RUN_EKF      1

/* ====== MADGWICK PARAMS ====== */
#define MADGWICK_BETA            1.0f   /* final/steady-state beta */
#define MADGWICK_BETA_START      0.5f   /* initial beta for fast convergence */
#define MADGWICK_BETA_DECAY_S    2.0f   /* seconds to ramp from BETA_START to BETA */
#define MADGWICK_ZETA            0.0f   /* gyro bias gain; disabled */
#define MADGWICK_BETA_MOTION_K   5.0f   /* motion-adaptive k */
#define MADGWICK_BETA_MIN        0.0f   /* beta floor — pure gyro during violent motion */
#define MADGWICK_ACCEL_REJECT_EN 1
#define MADGWICK_ACCEL_MIN_G     0.3f
#define MADGWICK_ACCEL_MAX_G     4.0f

/* ====== EKF PARAMS ======
 * Tune at runtime with: EKF TUNE <sigma_gyro> <sigma_bias> <sigma_accel> <sigma_mag> <r_adapt_k>
 */
#if ROBOT_MODE == ROBOT_MODE_LEG
/* ---------- LEG MODE ----------
 * Physical profile: slow, deliberate swings; severe footstep impact spikes.
 * Goal: maximum smoothness and immunity to heel-strike noise.
 */
#define EKF_SIGMA_GYRO        0.005f   /* trust gyro heavily — slow leg segments */
#define EKF_SIGMA_BIAS        6.3e-5f  /* bias random-walk rad/s^2/sqrt(Hz) */
#define EKF_SIGMA_ACCEL       0.8f     /* high — footstep impacts are severe */
#define EKF_SIGMA_MAG         8.0f     /* stability-first */
#define EKF_R_ADAPT_K         500.0f   /* aggressively reject accel during swings */
#define EKF_BIAS_MAX_DEV_G    0.40f    /* freeze bias update during impacts */
#define EKF_MAG_NIS_GATE      20.0f
#define EKF_ACCEL_NIS_GATE    35.0f    /* wide — heel-strike briefly shifts true a */
#define EKF_MAG_RESIDUAL_MAX  1.5f
#define EKF_P0                1.0f
#define EKF_P0_BIAS           1e-4f
#define EKF_ACCEL_REJECT_EN   1
#define EKF_ACCEL_MIN_G       0.3f     /* reject free-fall / sensor fault */
#define EKF_ACCEL_MAX_G       5.5f     /* reject heel-strike impact spikes */
#define EKF_ACCEL_TIMEOUT_S   0.0f     /* instant recovery */
#define EKF_BIAS_CLAMP_RAD_S  0.05f    /* ~2.9 deg/s — leg bias barely drifts */
#define EKF_MAG_DQ_CLAMP      0.015f
#define EKF_CONVERGENCE_TRACE 0.05f
#define EKF_MAX_TRACE         50.0f

#else
/* ---------- ARM MODE ----------
 * Physical profile: fast, intentional gestures; wrist snaps; no ground impact.
 * Goal: responsive tracking of rapid direction changes with quick snap-back.
 */
#define EKF_SIGMA_GYRO        0.01f    /* slightly more uncertainty → faster accel corrections */
#define EKF_SIGMA_BIAS        6.3e-5f
#define EKF_SIGMA_ACCEL       0.3f     /* trust accel more — arm gestures are intentional */
#define EKF_SIGMA_MAG         8.0f
#define EKF_R_ADAPT_K         300.0f   /* moderate rejection */
#define EKF_BIAS_MAX_DEV_G    0.35f
#define EKF_MAG_NIS_GATE      20.0f
#define EKF_ACCEL_NIS_GATE    25.0f    /* tighter — arm has cleaner motion */
#define EKF_MAG_RESIDUAL_MAX  1.5f
#define EKF_P0                1.0f
#define EKF_P0_BIAS           1e-4f
#define EKF_ACCEL_REJECT_EN   1
#define EKF_ACCEL_MIN_G       0.3f
#define EKF_ACCEL_MAX_G       6.0f     /* wrist snaps can briefly exceed 5.5g */
#define EKF_ACCEL_TIMEOUT_S   0.0f     /* instant recovery */
#define EKF_BIAS_CLAMP_RAD_S  0.08f    /* wider — arm mounting angle varies more */
#define EKF_MAG_DQ_CLAMP      0.015f
#define EKF_CONVERGENCE_TRACE 0.05f
#define EKF_MAX_TRACE         50.0f
#endif /* ROBOT_MODE */

#endif /* APP_CONFIG_EKF_H */
