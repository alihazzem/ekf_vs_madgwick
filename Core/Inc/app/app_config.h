#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* =====================================================================
 * app_config.h — Master configuration umbrella
 *
 * This file contains ONLY the top-level mode selectors. All detailed
 * configuration lives in the sub-headers below, which are included
 * automatically. Do not add new #defines here; add them to the
 * appropriate sub-header instead.
 *
 * ROBOT_MODE   : Selects the active hardware and control pipeline.
 *   1 = ROBOT_MODE_ARM  — Servos + gripper + arm kinematics tracking.
 *   2 = ROBOT_MODE_LEG  — ESP32 telemetry + knee gait analysis.
 *   3 = ROBOT_MODE_FULL — Both Leg telemetry and Arm servos simultaneously.
 *
 * SENSOR_GY91  : Selects the IMU hardware variant.
 *   0 = MPU-6050 (accel/gyro only — default).
 *   1 = GY-91 module (MPU-9255 accel/gyro + AK8963 magnetometer).
 * ===================================================================== */

#define ROBOT_MODE_ARM  1
#define ROBOT_MODE_LEG  2
#define ROBOT_MODE_FULL 3
#define ROBOT_MODE      ROBOT_MODE_FULL

#define SENSOR_GY91 0

/* ── Sub-headers (order matters: IMU before EKF, EKF before CAL) ──── */
#include "app/config_imu.h"
#include "app/config_ekf.h"
#include "app/config_cal.h"
#include "app/config_arm.h"
#include "app/config_leg.h"
#include "app/config_full.h"

#endif /* APP_CONFIG_H */
