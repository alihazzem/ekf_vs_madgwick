/**
 * @file config_arm.h
 * @brief ARM-mode specific configuration: servo control, gripper, and EMG.
 *
 * Included automatically by app_config.h — do not include directly.
 * All definitions in this file are only relevant when ROBOT_MODE == ROBOT_MODE_ARM.
 */
#ifndef APP_CONFIG_ARM_H
#define APP_CONFIG_ARM_H

/* ====== SERVO DIRECTION ======
 * Set to +1.0f or -1.0f to invert the pitch servo mapping.
 * -1.0f = forward tilt drives the servo in the "positive" direction.
 */
#define PITCH_SERVO_DIR (-1.0f)

/* ====== DC MOTOR COUNT ======
 * 1 = gripper only  (TIM4 CH1=PB6, CH2=PB7)
 * 2 = gripper + second motor (adds TIM4 CH3=PB8, CH4=PB9)
 */
#define NUM_DC_MOTORS 1

/* ====== EMG MOTOR FUSION ====== */
#define EMG_SPEED_TIMEOUT_MS 500  /* stop motors if no valid EMG packet for 500 ms */

#endif /* APP_CONFIG_ARM_H */
