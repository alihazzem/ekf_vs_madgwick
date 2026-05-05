#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ====== SENSOR SELECT ======
 * 1 = GY-91 module (MPU-9255 accel/gyro + AK8963 magnetometer).
 * 0 = legacy MPU-6050 (accel/gyro only, no magnetometer).
 * BMP280 on the GY-91 board is intentionally ignored in both cases.
 */
#define SENSOR_GY91 1

/* ====== MPU-92xx SETTINGS (active when SENSOR_GY91 = 1) ====== */
#define MPU9255_ADDR_7BIT 0x68 /* SAO/SDO low → 0x68, high → 0x69 */

/* Choose which MPU-92xx variant to accept at init. */
#define MPU92XX_VARIANT_AUTO 0
#define MPU92XX_VARIANT_9255 1
#define MPU92XX_VARIANT_9250 2
#define MPU92XX_VARIANT MPU92XX_VARIANT_AUTO

/* ====== AK8963 SETTINGS (active when SENSOR_GY91 = 1) ====== */
#define AK8963_ADDR_7BIT 0x0C /* Fixed address, not user-configurable */

/* ====== SAMPLE RATE ====== */
#define IMU_FS_HZ 200.0f
#define IMU_DT_S (1.0f / IMU_FS_HZ)

/* ====== ENABLE/DISABLE FILTERS ====== */
#define RUN_MADGWICK 1
#define RUN_EKF 1

/* ====== LOGGING ====== */
#define LOG_UART 1 // 1 = UART, 0 = SWO (later)
#define LOG_HEADER_ONCE 1

/* ====== MADGWICK PARAM ====== */
#define MADGWICK_BETA 1.0f       // final/steady-state beta (increased for better tracking)
#define MADGWICK_BETA_START 0.5f // initial beta for fast convergence
#define MADGWICK_BETA_DECAY_S \
  2.0f                     // seconds to ramp from BETA_START down to BETA
#define MADGWICK_ZETA 0.0f // gyro bias gain; disabled to prevent divergence
#define MADGWICK_BETA_MOTION_K 5.0f  // motion-adaptive k: aggressively reduces beta during dynamic motion
#define MADGWICK_BETA_MIN 0.0f // beta floor — allows pure gyro integration during violent motion

/* ====== EKF PARAMS ======
 * Physical noise densities from MPU6050 datasheet, conservative starting
 * values. Tune at runtime with: EKF TUNE <sigma_gyro> <sigma_bias>
 * <sigma_accel> <sigma_mag> <r_adapt_k>
 */
#define EKF_SIGMA_GYRO 0.01f      /* gyro noise density  rad/s/sqrt(Hz)  */
#define EKF_SIGMA_BIAS 1e-6f      /* bias random-walk    rad/s^2/sqrt(Hz) */
#define EKF_SIGMA_ACCEL 0.05f     /* accel noise density g/sqrt(Hz)       */
#define EKF_SIGMA_MAG 8.0f        /* stability-first mag trust (slower yaw lock) */
#define EKF_R_ADAPT_K 1000.0f     /* adaptive-R steepness: heavily down-weights lateral accelerations */
#define EKF_BIAS_MAX_DEV_G 0.15f  /* freeze bias update if |a|-1g exceeds this */
#define EKF_MAG_NIS_GATE 20.0f    /* mag NIS gate (higher = fewer rejects) */
#define EKF_MAG_RESIDUAL_MAX 1.5f /* max |y| before residual is scaled */

#define EKF_P0                                                                \
  1.0f /* initial P diagonal  (high = uncertain at start -> fast convergence) \
        */

#define EKF_ACCEL_REJECT_EN 1
#define EKF_ACCEL_MIN_G 0.3f     /* reject if |a| < 0.3 g (free-fall / sensor fault) */
#define EKF_ACCEL_MAX_G 4.0f     /* reject if |a| > 4.0 g (impact spikes)            */
#define EKF_ACCEL_TIMEOUT_S 0.0f /* 0.0s = instant recovery when acceleration stops (no starvation) */

/* ====== MPU6050 SETTINGS ====== */
#define MPU6050_ADDR_7BIT 0x68 // AD0=0 -> 0x68, AD0=1 -> 0x69

#define MADGWICK_ACCEL_REJECT_EN 1
#define MADGWICK_ACCEL_MIN_G 0.3f /* reject if |a| < 0.8 g */
#define MADGWICK_ACCEL_MAX_G 4.0f /* reject if |a| > 1.2 g */

/* ====== ACCELEROMETER BIAS CALIBRATION ======
 * Measured per-axis zero-g offsets in raw LSB (±2 g range, 16384 LSB/g).
 * At flat rest the raw readings should be [0, 0, +16384]; subtract these
 * offsets to remove the DC bias before the scale conversion.
 *
 * To re-derive: place board FLAT AND STILL, capture ~500 samples, compute:
 *   bias_x = mean(ax_raw)
 *   bias_y = mean(ay_raw)
 *   bias_z = mean(az_raw) - 16384
 */
#define ACCEL_BIAS_X (-1300) /* LSB, mean ax_raw when flat    */
#define ACCEL_BIAS_Y (160)   /* LSB, mean ay_raw when flat    */
#define ACCEL_BIAS_Z (10976) /* LSB, mean(az_raw) - 16384     */

/* ====== SENSOR -> BODY AXIS REMAPPING ======
 * Derived from 3-pose calibration on the current board mounting.
 * To change the physical orientation, edit ONLY these six macros.
 *
 *  Inputs  : ax_s, ay_s, az_s  — sensor-frame accel  (g)
 *            wx_s, wy_s, wz_s  — sensor-frame gyro   (rad/s)
 *  Outputs : ax_g, ay_g, az_g  — body-frame  accel   (g)
 *            wx,   wy,   wz    — body-frame  gyro    (rad/s)
 *
 * Default mapping (no remap):
 *   ax_g = +ax_s
 *   ay_g = +ay_s
 *   az_g = +az_s
 *   wx   = +wx_s
 *   wy   = +wy_s
 *   wz   = +wz_s
 */
#define REMAP_AX_G(ax_s, ay_s, az_s) ((ax_s))
#define REMAP_AY_G(ax_s, ay_s, az_s) ((ay_s))
#define REMAP_AZ_G(ax_s, ay_s, az_s) ((az_s))

#define REMAP_WX(wx_s, wy_s, wz_s) ((wx_s))
#define REMAP_WY(wx_s, wy_s, wz_s) ((wy_s))
#define REMAP_WZ(wx_s, wy_s, wz_s) ((wz_s))

/* ====== AK8963 → BODY AXIS REMAP (active when SENSOR_GY91 = 1) ======
 * The AK8963 die inside the MPU-9255 is physically mounted at a different
 * orientation than the accel/gyro die (documented in MPU-9255 datasheet §4.2):
 *
 *   AK8963 +X  ≡  accel/gyro +Y   (swap X↔Y)
 *   AK8963 +Y  ≡  accel/gyro +X   (swap X↔Y)
 *   AK8963 +Z  ≡  accel/gyro -Z   (negate Z)
 *
 * These macros map raw AK8963 sensor-frame counts into the body frame that
 * is consistent with the accel/gyro remap above.  Edit ONLY these three
 * macros if the board is mounted differently.
 *
 * Inputs : mx_s, my_s, mz_s — AK8963 sensor-frame raw 16-bit counts
 * Output : body-frame counts (before ASA correction and µT scaling)
 */
#define REMAP_MX(mx_s, my_s, mz_s) (+(my_s))
#define REMAP_MY(mx_s, my_s, mz_s) (+(mx_s))
#define REMAP_MZ(mx_s, my_s, mz_s) (-(mz_s))

/* Scale factor: 0.15 µT per LSB in 16-bit mode (AK8963 datasheet Table 3) */
#define AK8963_UT_PER_LSB 0.15f

/* ====== MAGNETOMETER CALIBRATION (active when SENSOR_GY91 = 1) ======
 * Generated by scripts/capture_mag_cal.py — paste the printed block here
 * and reflash.  Until calibrated the defaults are b=0, S=I (no correction).
 *
 * Online formula applied in imu_app.c:
 *   m_cal = S × (m_body − b)
 *
 * where m_body is the body-frame µT vector (after remap + ASA correction).
 */
#define MAG_CAL_BX 10.8608f
#define MAG_CAL_BY -6.4249f
#define MAG_CAL_BZ 35.0075f

#define MAG_CAL_S11 1.1278f
#define MAG_CAL_S12 0.0452f
#define MAG_CAL_S13 0.0391f
#define MAG_CAL_S21 0.0452f
#define MAG_CAL_S22 1.0887f
#define MAG_CAL_S23 -0.0243f
#define MAG_CAL_S31 0.0391f
#define MAG_CAL_S32 -0.0243f
#define MAG_CAL_S33 0.9704f

#endif // APP_CONFIG_H
