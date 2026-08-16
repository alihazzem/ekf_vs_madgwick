#ifndef APP_CONFIG_FULL_H
#define APP_CONFIG_FULL_H

/**
 * @file config_full.h
 * @brief Configuration parameters specific to the Full Robot Mode (Leg + Arm).
 *
 * All definitions in this file are only relevant when ROBOT_MODE == ROBOT_MODE_FULL.
 */

#if ROBOT_MODE == ROBOT_MODE_FULL

/* ====== HARDWARE CONFIGURATION ====== */
/**
 * Number of active legs.
 * In FULL mode, typically 1 Leg (IMU 0 and 1 on I2C1).
 */
#define NUM_LEGS 1

/**
 * Number of active arms.
 * 1 = Single arm on I2C3 (IMU 2) - Uses 2 servos (TIM3 CH1/CH2)
 * 2 = Dual arm on I2C3 (IMU 2 and 3) - Uses 4 servos (TIM3 CH1-4)
 */
#define NUM_ARMS 1

#endif /* ROBOT_MODE_FULL */
#endif /* APP_CONFIG_FULL_H */
