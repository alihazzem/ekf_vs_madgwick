/**
 * @file config_imu.h
 * @brief IMU hardware configuration: sampling, I2C addresses,
 *        per-IMU register settings, and sensor feature flags.
 *
 * Included automatically by app_config.h — do not include directly.
 */
#ifndef APP_CONFIG_IMU_H
#define APP_CONFIG_IMU_H

/* ====== SAMPLE RATE ====== */
#define IMU_FS_HZ 200.0f
#define IMU_DT_S  (1.0f / IMU_FS_HZ)

/* ====== IMU COUNT & I2C ADDRESSES ====== */
#if ROBOT_MODE == ROBOT_MODE_ARM
  #define NUM_IMUS     2
#elif ROBOT_MODE == ROBOT_MODE_LEG
  #if NUM_LEGS == 2
    #define NUM_IMUS   4
  #else
    #define NUM_IMUS   2
  #endif
#elif ROBOT_MODE == ROBOT_MODE_FULL
  #define NUM_IMUS     ( (NUM_LEGS * 2) + NUM_ARMS )
#endif

#define MPU6050_ADDR_0  0x68   /* IMU 0: AD0 = GND */
#define MPU6050_ADDR_1  0x69   /* IMU 1: AD0 = VCC */

/* ====== IMU 1 RAW ACCEL MODE ======
 * Only meaningful when NUM_IMUS = 2.
 * 0 = IMU 1 uses the same EKF/Madgwick pipeline as IMU 0.
 * 1 = IMU 1 uses raw accelerometer tilt only (no gyro, no filter).
 */
#define IMU1_RAW_ACCEL 0

/* ====== BYPASS IMU FILTER ======
 * 0 = normal operation (EKF / Madgwick output drives servos).
 * 1 = raw accelerometer tilt used directly (useful for debugging filter impact).
 */
#define BYPASS_IMU_FILTER 0

/* ====== CONVERGENCE TEST (FAULT INJECTION) ======
 * Set to 1 to force filters to start at identity quaternion,
 * bypassing accel/mag alignment on the first sample. Thesis use only.
 */
#define FAULT_INJECT_IDENTITY_INIT 0

/* ====== LOGGING ====== */
#define LOG_UART       1  /* 1 = UART, 0 = SWO */
#define LOG_HEADER_ONCE 1

/* ====== MPU-9255 SETTINGS (active when SENSOR_GY91 = 1) ====== */
#define MPU9255_ADDR_7BIT    0x68
#define MPU92XX_VARIANT_AUTO 0
#define MPU92XX_VARIANT_9255 1
#define MPU92XX_VARIANT_9250 2
#define MPU92XX_VARIANT      MPU92XX_VARIANT_AUTO

/* ====== AK8963 SETTINGS (active when SENSOR_GY91 = 1) ====== */
#define AK8963_ADDR_7BIT 0x0C  /* Fixed address */

/* ====== PER-IMU HARDWARE CONFIG (LEG MODE ONLY) ======
 * IMU 0 = THIGH: slow rotation, no impact  → tight range, low noise BW
 * IMU 1 = SHIN:  fast rotation, impact      → wider accel range, higher BW
 *
 * Gyro register:  0x00=±250dps, 0x08=±500dps, 0x10=±1000dps
 * Accel register: 0x00=±2g,     0x08=±4g,     0x10=±8g
 * DLPF CONFIG:    0x02=98Hz BW, 0x03=42Hz BW
 *
 * In ARM mode mpu6050_init_200hz() is used for both IMUs (±1000dps, ±8g, 98Hz).
 */
#if ROBOT_MODE == ROBOT_MODE_LEG || ROBOT_MODE == ROBOT_MODE_FULL
  /* IMU 0 — Leg 1 Thigh */
  #define LEG_IMU0_SMPLRT_DIV       0x04u
  #define LEG_IMU0_DLPF_CFG         0x03u   /* 42 Hz BW */
  #define LEG_IMU0_GYRO_FS          0x08u   /* ±500 dps */
  #define LEG_IMU0_ACCEL_FS         0x08u   /* ±4 g */
  #define LEG_IMU0_GYRO_LSB_PER_DPS 65.5f
  #define LEG_IMU0_ACC_LSB_PER_G    8192.0f

  /* IMU 1 — Leg 1 Shin */
  #define LEG_IMU1_SMPLRT_DIV       0x04u
  #define LEG_IMU1_DLPF_CFG         0x02u   /* 98 Hz BW */
  #define LEG_IMU1_GYRO_FS          0x08u   /* ±500 dps */
  #define LEG_IMU1_ACCEL_FS         0x10u   /* ±8 g */
  #define LEG_IMU1_GYRO_LSB_PER_DPS 65.5f
  #define LEG_IMU1_ACC_LSB_PER_G    4096.0f

  /* IMU 2 — Leg 2 Thigh (or Arm 1) */
  #define LEG_IMU2_SMPLRT_DIV       0x04u
  #define LEG_IMU2_DLPF_CFG         0x03u   /* 42 Hz BW */
  #define LEG_IMU2_GYRO_FS          0x08u   /* ±500 dps */
  #define LEG_IMU2_ACCEL_FS         0x08u   /* ±4 g */
  #define LEG_IMU2_GYRO_LSB_PER_DPS 65.5f
  #define LEG_IMU2_ACC_LSB_PER_G    8192.0f

  /* IMU 3 — Leg 2 Shin (or Arm 2) */
  #define LEG_IMU3_SMPLRT_DIV       0x04u
  #define LEG_IMU3_DLPF_CFG         0x02u   /* 98 Hz BW */
  #define LEG_IMU3_GYRO_FS          0x08u   /* ±500 dps */
  #define LEG_IMU3_ACCEL_FS         0x10u   /* ±8 g */
  #define LEG_IMU3_GYRO_LSB_PER_DPS 65.5f
  #define LEG_IMU3_ACC_LSB_PER_G    4096.0f
#endif

#endif /* APP_CONFIG_IMU_H */
