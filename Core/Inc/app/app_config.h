#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ====== ROBOT MODE ======
 * Choose the active tracking target to prevent CPU overload and hardware conflicts.
 * 1 = ROBOT_MODE_ARM : Servos and Gripper active, tracks arm kinematics.
 * 2 = ROBOT_MODE_LEG : Stepper motor active via I2C3, tracks knee gait.
 */
#define ROBOT_MODE_ARM 1
#define ROBOT_MODE_LEG 2
#define ROBOT_MODE ROBOT_MODE_LEG

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
 * Two independently-tuned sets of parameters — one per ROBOT_MODE.
 * Tune at runtime with: EKF TUNE <sigma_gyro> <sigma_bias> <sigma_accel> <sigma_mag> <r_adapt_k>
 */
#if ROBOT_MODE == ROBOT_MODE_LEG
/* ---------- LEG MODE ----------
 * Physical profile: slow, deliberate swings; severe footstep impact spikes.
 * Goal: maximum smoothness and immunity to heel-strike noise.
 *
 * Trust gyro HEAVILY (low sigma_gyro) — the EKF rides the gyro integration
 * almost exclusively, giving buttery-smooth angles.
 * Trust accel VERY LITTLE (high sigma_accel + high R_ADAPT_K) — footstep
 * impacts produce multi-g spikes that would corrupt the pitch if allowed in.
 * Wide NIS gate — a heel-strike briefly changes the true acceleration vector,
 * so we give the filter room to reject it rather than chase it.
 */
#define EKF_SIGMA_GYRO       0.005f   /* trust gyro heavily — slow leg segments */
#define EKF_SIGMA_BIAS       6.3e-5f  /* bias random-walk rad/s^2/sqrt(Hz) */
#define EKF_SIGMA_ACCEL      0.8f     /* high — footstep impacts are severe */
#define EKF_SIGMA_MAG        8.0f     /* stability-first (no mag on MPU-6500) */
#define EKF_R_ADAPT_K        500.0f   /* aggressively reject accel during swings */
#define EKF_BIAS_MAX_DEV_G   0.40f    /* freeze bias update during impacts */
#define EKF_MAG_NIS_GATE     20.0f    /* mag NIS gate */
#define EKF_ACCEL_NIS_GATE   35.0f    /* wide — heel-strike briefly shifts true a */
#define EKF_MAG_RESIDUAL_MAX 1.5f
#define EKF_P0               1.0f
#define EKF_P0_BIAS          1e-4f
#define EKF_ACCEL_REJECT_EN  1
#define EKF_ACCEL_MIN_G      0.3f     /* reject free-fall / sensor fault */
#define EKF_ACCEL_MAX_G      5.5f     /* reject heel-strike impact spikes */
#define EKF_ACCEL_TIMEOUT_S  0.0f     /* instant recovery after impact */
#define EKF_BIAS_CLAMP_RAD_S 0.05f    /* ~2.9 deg/s — leg bias barely drifts */
#define EKF_MAG_DQ_CLAMP     0.015f
#define EKF_CONVERGENCE_TRACE 0.05f
#define EKF_MAX_TRACE         50.0f

#else
/* ---------- ARM MODE ----------
 * Physical profile: fast, intentional gestures; wrist snaps; no ground impact.
 * Goal: responsive tracking of rapid direction changes with quick snap-back.
 *
 * Trust gyro less strictly (slightly higher sigma_gyro) — the wrist changes
 * direction abruptly, and allowing slightly higher gyro uncertainty lets the
 * accel correction snap the quaternion back to the right attitude faster.
 * Trust accel more (lower sigma_accel) — arm accelerations are intentional
 * gestures, not destructive impact spikes; accel helps snap back quickly.
 * Lower R_ADAPT_K — arm dynamics are fast but deliberate, so we don't reject
 * them as aggressively as leg footstep shocks.
 * Wider bias clamp — arm can be mounted at more varied angles, so the gyro
 * bias estimate needs more headroom to converge from any starting position.
 */
#define EKF_SIGMA_GYRO       0.01f    /* slightly more uncertainty → faster accel corrections */
#define EKF_SIGMA_BIAS       6.3e-5f  /* bias random-walk rad/s^2/sqrt(Hz) */
#define EKF_SIGMA_ACCEL      0.3f     /* trust accel more — arm gestures are intentional */
#define EKF_SIGMA_MAG        8.0f     /* stability-first mag trust */
#define EKF_R_ADAPT_K        300.0f   /* moderate rejection — arm is dynamic but not spiky */
#define EKF_BIAS_MAX_DEV_G   0.35f    /* tighter freeze threshold for cleaner arm motion */
#define EKF_MAG_NIS_GATE     20.0f    /* mag NIS gate */
#define EKF_ACCEL_NIS_GATE   25.0f    /* tighter — arm has cleaner, more predictable motion */
#define EKF_MAG_RESIDUAL_MAX 1.5f
#define EKF_P0               1.0f
#define EKF_P0_BIAS          1e-4f
#define EKF_ACCEL_REJECT_EN  1
#define EKF_ACCEL_MIN_G      0.3f     /* reject free-fall / sensor fault */
#define EKF_ACCEL_MAX_G      6.0f     /* wrist snaps can briefly exceed 5.5g */
#define EKF_ACCEL_TIMEOUT_S  0.0f     /* instant recovery */
#define EKF_BIAS_CLAMP_RAD_S 0.08f    /* wider — arm mounting angle varies more */
#define EKF_MAG_DQ_CLAMP     0.015f
#define EKF_CONVERGENCE_TRACE 0.05f
#define EKF_MAX_TRACE         50.0f
#endif /* ROBOT_MODE */


/* ====== MPU6050 SETTINGS ====== */
#define NUM_IMUS 2         // Set to 1 for single IMU, 2 for dual IMU setup
#define MPU6050_ADDR_0 0x68 // IMU 0: AD0=GND → 0x68
#define MPU6050_ADDR_1 0x69 // IMU 1: AD0=VCC → 0x69

/* ====== PER-IMU HARDWARE CONFIG (LEG MODE ONLY) ======
 * In LEG mode the two IMUs are on different body segments with different
 * motion profiles, so we configure them independently for best resolution.
 *
 * IMU 0 = THIGH: slower rotation, no ground impact → tightest range, lowest noise BW
 * IMU 1 = SHIN:  faster rotation, ground impact     → wider accel range, higher BW
 *
 * Gyro register values:  0x00=±250dps(131 LSB/dps), 0x08=±500dps(65.5), 0x10=±1000dps(32.8)
 * Accel register values: 0x00=±2g(16384 LSB/g),     0x08=±4g(8192),     0x10=±8g(4096)
 * DLPF CONFIG values:    0x02=98Hz BW,               0x03=42Hz BW
 *
 * In ARM mode mpu6050_init_200hz() is used for both IMUs (±1000dps, ±8g, 98Hz) — unchanged.
 */
#if ROBOT_MODE == ROBOT_MODE_LEG
  /* IMU 0 — Thigh: ±500dps, ±4g, 42Hz DLPF */
  #define LEG_IMU0_SMPLRT_DIV       0x04u   /* 200 Hz output rate */
  #define LEG_IMU0_DLPF_CFG         0x03u   /* 42 Hz bandwidth — filters strap vibration */
  #define LEG_IMU0_GYRO_FS          0x08u   /* ±500 dps */
  #define LEG_IMU0_ACCEL_FS         0x08u   /* ±4 g    */
  #define LEG_IMU0_GYRO_LSB_PER_DPS 65.5f   /* 65.5 LSB / (°/s) at ±500 dps */
  #define LEG_IMU0_ACC_LSB_PER_G    8192.0f /* 8192 LSB / g at ±4 g          */

  /* IMU 1 — Shin: ±500dps, ±8g, 98Hz DLPF */
  #define LEG_IMU1_SMPLRT_DIV       0x04u   /* 200 Hz output rate */
  #define LEG_IMU1_DLPF_CFG         0x02u   /* 98 Hz bandwidth — preserves fast knee flexion */
  #define LEG_IMU1_GYRO_FS          0x08u   /* ±500 dps */
  #define LEG_IMU1_ACCEL_FS         0x10u   /* ±8 g (handles footstep impact shocks) */
  #define LEG_IMU1_GYRO_LSB_PER_DPS 65.5f   /* 65.5 LSB / (°/s) at ±500 dps */
  #define LEG_IMU1_ACC_LSB_PER_G    4096.0f /* 4096 LSB / g at ±8 g          */
#endif /* ROBOT_MODE_LEG */

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
 * Measured per-axis zero-g offsets in raw LSB.
 * At flat rest the true Z reading is +1g; we subtract the actual +1g LSB value
 * internally, leaving just the raw offset which is subtracted here.
 *
 * To re-derive: place board FLAT AND STILL, then run via CLI:
 *   MPU CAL ACCEL 3000
 */
#if ROBOT_MODE == ROBOT_MODE_LEG
/* LEG MODE: IMU0=±4g, IMU1=±8g */
#define ACCEL_BIAS_X_0 (-65) 
#define ACCEL_BIAS_Y_0 (-2)
#define ACCEL_BIAS_Z_0 (219)

#define ACCEL_BIAS_X_1 (354)
#define ACCEL_BIAS_Y_1 (99)
#define ACCEL_BIAS_Z_1 (-236)

#else
/* ARM MODE: Both IMUs at ±8g */
#define ACCEL_BIAS_X_0 (-36)
#define ACCEL_BIAS_Y_0 (0)
#define ACCEL_BIAS_Z_0 (112)

#define ACCEL_BIAS_X_1 (361)
#define ACCEL_BIAS_Y_1 (109)
#define ACCEL_BIAS_Z_1 (-236)
#endif

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
