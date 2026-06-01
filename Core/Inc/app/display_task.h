/**
 * @file display_task.h
 * @brief RTOS display task, SystemState_t IPC struct, and queue handle.
 *
 * Architecture overview:
 *
 *   [IMU Task] ──(osMessageQueuePut, timeout=0)──► [displayQueueHandle]
 *                                                           │
 *                                             (osMessageQueueGet, blocking)
 *                                                           ▼
 *                                                  [Display Task (Low)]
 *                                                           │
 *                                             (ssd1306_flush via hi2c2)
 *                                                           ▼
 *                                                      [OLED Display]
 *
 * The zero-timeout PUT means the IMU task NEVER blocks to wait for the
 * display task. If the queue is already full (display task is still
 * rendering the previous frame) the new message is silently dropped.
 * This guarantees the 200 Hz real-time loop is never impacted by the
 * 10 Hz display refresh.
 */
#ifndef APP_DISPLAY_TASK_H
#define APP_DISPLAY_TASK_H

#include "cmsis_os.h"
#include <stdint.h>

/* ── System Status Enum ───────────────────────────────────────────────────── */
typedef enum {
    SYS_STATUS_CALIBRATING = 0,  /**< IMU warmup / gyro calibration running */
    SYS_STATUS_RUNNING,          /**< Calibration done, servos active        */
    SYS_STATUS_REZEROED,         /**< Neutral pose just re-captured (1 sec)  */
} SystemStatus_t;

/* ── IPC Data Struct ──────────────────────────────────────────────────────── */
/**
 * @brief Snapshot of system state sent from IMU task to display task.
 *
 * Kept small (20 bytes) so it copies in one word-aligned burst.
 */
typedef struct {
    float          pitch_0;           /**< IMU 0: Pitch angle in degrees (relative)  */
    float          roll_0;            /**< IMU 0: Roll angle in degrees  (relative)  */
    uint32_t       servo_pitch_us_0;  /**< IMU 0: Pitch servo pulse width in µs      */
    uint32_t       servo_roll_us_0;   /**< IMU 0: Roll servo pulse width in µs       */
    float          pitch_1;           /**< IMU 1: Pitch angle in degrees (relative)  */
    float          roll_1;            /**< IMU 1: Roll angle in degrees  (relative)  */
    uint32_t       servo_pitch_us_1;  /**< IMU 1: Pitch servo pulse width in µs      */
    uint32_t       servo_roll_us_1;   /**< IMU 1: Roll servo pulse width in µs       */
    SystemStatus_t status;          /**< Current system status              */
} SystemState_t;

/* ── RTOS Handle (defined in display_task.c) ─────────────────────────────── */
extern osMessageQueueId_t displayQueueHandle;

/* ── Task Entry Point ─────────────────────────────────────────────────────── */
/**
 * @brief FreeRTOS task function for the OLED display.
 *        Spawn with osPriorityLow and at least 512-word stack.
 */
void display_task_fn(void *arg);

#endif /* APP_DISPLAY_TASK_H */
