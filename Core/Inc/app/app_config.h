#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ====== SAMPLE RATE ====== */
#define IMU_FS_HZ 100.0f
#define IMU_DT_S (1.0f / IMU_FS_HZ)

/* ====== ENABLE/DISABLE FILTERS ====== */
#define RUN_MADGWICK 1
#define RUN_EKF 1

/* ====== LOGGING ====== */
#define LOG_UART 1 // 1 = UART, 0 = SWO (later)
#define LOG_HEADER_ONCE 1

/* ====== MADGWICK PARAM ====== */
#define MADGWICK_BETA 0.08f        // final/steady-state beta
#define MADGWICK_BETA_START 0.5f   // initial beta for fast convergence
#define MADGWICK_BETA_DECAY_S 2.0f // seconds to ramp from BETA_START down to BETA
#define MADGWICK_ZETA 0.015f       // gyro bias gain; set to 0.0f to disable
#define MADGWICK_BETA_MOTION_K 10.0f // motion-adaptive k: beta_eff /= (1 + k * dev^2)
#define MADGWICK_BETA_MIN      0.01f // beta floor — prevents pure gyro integration

/* ====== EKF PARAMS ======
 * Physical noise densities from MPU6050 datasheet, conservative starting values.
 * Tune at runtime with: EKF TUNE <sigma_gyro> <sigma_bias> <sigma_accel> <r_adapt_k>
 */
#define EKF_SIGMA_GYRO 0.01f  /* gyro noise density  rad/s/sqrt(Hz)  */
#define EKF_SIGMA_BIAS 2e-5f  /* bias random-walk    rad/s^2/sqrt(Hz) */
#define EKF_SIGMA_ACCEL 0.05f /* accel noise density g/sqrt(Hz)       */
#define EKF_R_ADAPT_K 200.0f   /* adaptive-R steepness (higher -> faster trust drop during motion) */
#define EKF_P0 1.0f           /* initial P diagonal  (high = uncertain at start -> fast convergence) */

/* ====== MPU6050 SETTINGS ====== */
#define MPU6050_ADDR_7BIT 0x68 // AD0=0 -> 0x68, AD0=1 -> 0x69

#define MADGWICK_ACCEL_REJECT_EN 1
#define MADGWICK_ACCEL_MIN_G 0.85f
#define MADGWICK_ACCEL_MAX_G 1.15f

/* ====== SENSOR -> BODY AXIS REMAPPING ======
 * Derived from 3-pose calibration on the current board mounting.
 * To change the physical orientation, edit ONLY these six macros.
 *
 *  Inputs  : ax_s, ay_s, az_s  — sensor-frame accel  (g)
 *            wx_s, wy_s, wz_s  — sensor-frame gyro   (rad/s)
 *  Outputs : ax_g, ay_g, az_g  — body-frame  accel   (g)
 *            wx,   wy,   wz    — body-frame  gyro    (rad/s)
 *
 * Current mapping (verified on hardware):
 *   ax_g =  -ay_s    (sensor Y  -> body X, negated)
 *   ay_g =  -az_s    (sensor Z  -> body Y, negated)
 *   az_g =  +ax_s    (sensor X  -> body Z, unchanged)
 *   wx   =  -wy_s    (sensor wy -> body wx, negated)
 *   wy   =  -wz_s    (sensor wz -> body wy, negated)
 *   wz   =  +wx_s    (sensor wx -> body wz, unchanged)
 */
#define REMAP_AX_G(ax_s, ay_s, az_s)  (-(ay_s))
#define REMAP_AY_G(ax_s, ay_s, az_s)  (-(az_s))
#define REMAP_AZ_G(ax_s, ay_s, az_s)  ( (ax_s))

#define REMAP_WX(wx_s, wy_s, wz_s)    (-(wy_s))
#define REMAP_WY(wx_s, wy_s, wz_s)    (-(wz_s))
#define REMAP_WZ(wx_s, wy_s, wz_s)    ( (wx_s))

#endif // APP_CONFIG_H
