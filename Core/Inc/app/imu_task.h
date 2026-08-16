/**
 * @file imu_task.h
 * @brief FreeRTOS task handling IMU sampling, kinematics, and motor control.
 */
#ifndef APP_IMU_TASK_H
#define APP_IMU_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The main IMU task entry point. 
 *        Spawns from main.c osThreadNew().
 */
void imu_task_fn(void *arg);

/**
 * @brief Global flag to request a re-zeroing of the IMU orientation.
 *        Set by CLI or button press; cleared by the IMU task.
 */
extern volatile int g_rezero_requested;

#if STACK_TUNING_MODE
#include "FreeRTOS.h"
#include "portmacro.h" /* UBaseType_t */
extern volatile UBaseType_t s_imu_hwm;
#endif

#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_TASK_H */
