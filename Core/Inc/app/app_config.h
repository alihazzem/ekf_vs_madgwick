#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ====== SENSOR SELECT ======
 * 1 = GY-91 module (MPU-9255 accel/gyro + AK8963 magnetometer).
 * 0 = legacy MPU-6050 (accel/gyro only, no magnetometer).
 * BMP280 on the GY-91 board is intentionally ignored in both cases.
 */
#define SENSOR_GY91 0

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
#define RUN_MADGWICK 0
#define RUN_EKF 1

/* ====== CONVERGENCE TEST (FAULT INJECTION) ======
 * Set to 1 to force filters to start at identity quaternion (0,0,0),
 * bypassing accel/mag alignment on the first sample. Use for thesis tests
 * only, then set back to 0 for normal operation.
 */
#define FAULT_INJECT_IDENTITY_INIT 0

/* ====== LOGGING ====== */
#define LOG_UART 1 // 1 = UART, 0 = SWO (later)
#define LOG_HEADER_ONCE 1

/* ====== MADGWICK PARAM ====== */
#define MADGWICK_BETA 1.0f       // final/steady-state beta (increased for better tracking)
#define MADGWICK_BETA_START 0.5f // initial beta for fast convergence
#define MADGWICK_BETA_DECAY_S \
  2.0f                              // seconds to ramp from BETA_START down to BETA
#define MADGWICK_ZETA 0.0f          // gyro bias gain; disabled to prevent divergence
#define MADGWICK_BETA_MOTION_K 5.0f // motion-adaptive k: aggressively reduces beta during dynamic motion
#define MADGWICK_BETA_MIN 0.0f      // beta floor — allows pure gyro integration during violent motion

/* ====== EKF PARAMS ======
 * Physical noise densities from MPU6050 datasheet, conservative starting
 * values. Tune at runtime with: EKF TUNE <sigma_gyro> <sigma_bias>
 * <sigma_accel> <sigma_mag> <r_adapt_k>
 */
#define EKF_SIGMA_GYRO 0.002f     /* gyro noise density  rad/s/sqrt(Hz)  */
#define EKF_SIGMA_BIAS 6.3e-5f    /* bias random-walk    rad/s^2/sqrt(Hz) */
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
#define NUM_IMUS 1          // Set to 1 for single IMU, 2 for dual IMU setup
#define MPU6050_ADDR_0 0x68 // IMU 0: AD0=GND → 0x68
#define MPU6050_ADDR_1 0x69 // IMU 1: AD0=VCC → 0x69

/* ====== DC MOTOR COUNT ======
 * 1 = gripper only  (TIM4 CH1=PB6, CH2=PB7)
 * 2 = gripper + second motor  (adds TIM4 CH3=PB8, CH4=PB9)
 * The second motor always mirrors the gripper at 50% PWM.
 */
#define NUM_DC_MOTORS 1

/* ====== IMU 1 RAW ACCEL MODE ======
 * Only meaningful when NUM_IMUS = 2.
 * 0 = IMU 1 uses the same filter pipeline as IMU 0 (EKF / Madgwick).
 * 1 = IMU 1 uses raw accelerometer tilt only (no gyro, no filter).
 *     IMU 0 always keeps the full filter. Both motors wait for IMU 0
 *     calibration to finish before moving (existing safety behaviour).
 *     Re-zero button captures a raw pitch/roll offset for IMU 1.
 */
#define IMU1_RAW_ACCEL 0

#define MADGWICK_ACCEL_REJECT_EN 1
#define MADGWICK_ACCEL_MIN_G 0.3f /* reject if |a| < 0.3 g */
#define MADGWICK_ACCEL_MAX_G 4.0f /* reject if |a| > 4.0 g */

/* ====== ACCELEROMETER BIAS CALIBRATION ======
 * Measured per-axis zero-g offsets in raw LSB (±2 g range, 16384 LSB/g).
 * At flat rest the raw readings should be [0, 0, +16384]; subtract these
 * offsets to remove the DC bias before the scale conversion.
 *
 * To re-derive: place board FLAT AND STILL, then run via CLI:
 *   MPU CAL ACCEL 3000
 * The CLI prints per-IMU offsets — paste them into the macros below.
 *
 * IMU 0 (addr 0x68):
 *   bias_x = mean(ax_raw)
 *   bias_y = mean(ay_raw)
 *   bias_z = mean(az_raw) - 16384
 *
 * IMU 1 (addr 0x69):
 *   bias_x = mean(ax_raw)
 *   bias_y = mean(ay_raw)
 *   bias_z = mean(az_raw) - 16384
 */
#define ACCEL_BIAS_X_0 (107) /* IMU 0: calibrated 2025-05-25 */
#define ACCEL_BIAS_Y_0 (-967)
#define ACCEL_BIAS_Z_0 (-408)
#define ACCEL_BIAS_X_1 (-841) /* IMU 1: calibrated 2025-05-25 */
#define ACCEL_BIAS_Y_1 (278)
#define ACCEL_BIAS_Z_1 (-358)

/* ====== SENSOR -> BODY AXIS REMAPPING ======
 * Derived from 3-pose calibration on the current board mounting.
 * To change the physical orientation, edit ONLY these six macros.
 *
 *  Inputs  : ax_s, ay_s, az_s  — sensor-frame accel  (g)
 *            wx_s, wy_s, wz_s  — sensor-frame gyro   (rad/s)
 *  Outputs : ax_g, ay_g, az_g  — body-frame  accel   (g)
 *            wx,   wy,   wz    — body-frame  gyro    (rad/s)
 *
 * ====== ACCEL/GYRO -> BODY AXIS REMAP ======
 * Default orientation: identity mapping (no sign flips or swaps).
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
 * With identity accel/gyro remap, the body-frame mapping is:
 *   Body X = +(my_s)
 *   Body Y = +(mx_s)
 *   Body Z = -(mz_s)
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

/* ====== EMG MOTOR FUSION ====== */
#define EMG_SPEED_TIMEOUT_MS 500 /* stop motors if no valid EMG packet for 500 ms */

#endif // APP_CONFIG_H
