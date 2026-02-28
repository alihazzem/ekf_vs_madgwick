#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ====== SAMPLE RATE ====== */
#define IMU_FS_HZ          100.0f
#define IMU_DT_S           (1.0f / IMU_FS_HZ)

/* ====== ENABLE/DISABLE FILTERS ====== */
#define RUN_MADGWICK       1
#define RUN_EKF            1

/* ====== LOGGING ====== */
#define LOG_UART           1    // 1 = UART, 0 = SWO (later)
#define LOG_HEADER_ONCE    1

/* ====== MADGWICK PARAM ====== */
#define MADGWICK_BETA      0.08f  // start value (we tune later)

/* ====== MPU6050 SETTINGS ====== */
#define MPU6050_ADDR_7BIT  0x68   // AD0=0 -> 0x68, AD0=1 -> 0x69

/* ====== AXIS REMAP (we will fill exact mapping later) ======
   Put +1 or -1 and select axes mapping in code after we confirm your final Arduino remap.
*/

#endif // APP_CONFIG_H
