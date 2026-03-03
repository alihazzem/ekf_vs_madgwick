#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ====== SAMPLE RATE ====== */
#define IMU_FS_HZ 100.0f
#define IMU_DT_S (1.0f / IMU_FS_HZ)

/* ====== ENABLE/DISABLE FILTERS ====== */
#define RUN_MADGWICK 1
#define RUN_EKF 0

/* ====== LOGGING ====== */
#define LOG_UART 1 // 1 = UART, 0 = SWO (later)
#define LOG_HEADER_ONCE 1

/* ====== MADGWICK PARAM ====== */
#define MADGWICK_BETA 0.08f        // final/steady-state beta
#define MADGWICK_BETA_START 0.5f   // initial beta for fast convergence
#define MADGWICK_BETA_DECAY_S 2.0f // seconds to ramp from BETA_START down to BETA
#define MADGWICK_ZETA 0.015f       // gyro bias gain; set to 0.0f to disable

/* ====== MPU6050 SETTINGS ====== */
#define MPU6050_ADDR_7BIT 0x68 // AD0=0 -> 0x68, AD0=1 -> 0x69

#define MADGWICK_ACCEL_REJECT_EN 1
#define MADGWICK_ACCEL_MIN_G 0.85f
#define MADGWICK_ACCEL_MAX_G 1.15f

/* ====== AXIS REMAP (we will fill exact mapping later) ======
   Put +1 or -1 and select axes mapping in code after we confirm your final Arduino remap.
*/

#endif // APP_CONFIG_H
