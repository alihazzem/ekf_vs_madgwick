/**
 * @file gait_app.h
 * @brief Leg gait analysis: tare calibration, angle smoothing, and
 *        STANCE/SWING phase detection.
 *
 * This module consumes raw EKF quaternions from imu_app and produces
 * tared, smoothed joint angles consumed by display_task and main.c.
 *
 * Call gait_app_update() once per IMU sample (from imu_app_step).
 * Call gait_app_tare() to set the current pose as the zero reference.
 */
#ifndef APP_GAIT_APP_H
#define APP_GAIT_APP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float tare_thigh;
    float tare_shin;

    float thigh_pitch;
    float shin_pitch;
    float knee_angle;

    float smooth_thigh;
    float smooth_shin;
    bool  ema_init;

    int     gait_phase;
    float   last_hs_s;
    float   last_to_s;
    float   gyro_sma;
    float   swing_max_knee;
    uint8_t mid_swing_armed;
    float   stride_time;
    int     event_flag;

    bool     auto_tare_done;
    float    last_thigh_sample;
    float    last_shin_sample;
    uint32_t stable_start_ms;
} GaitState_t;

/**
 * @brief Initialize a GaitState_t structure to its default values.
 */
void gait_app_init_state(GaitState_t *state);

/**
 * @brief Update gait state from the latest EKF quaternions.
 *
 * @param state  Pointer to the leg's state structure.
 * @param t_sec  Elapsed time in seconds (used for stride timing).
 * @param gy_shin_raw  Raw gyro Y reading from the shin IMU (LSB).
 * @param imu_thigh_idx  Index of the thigh IMU (e.g. 0 or 2).
 * @param imu_shin_idx   Index of the shin IMU (e.g. 1 or 3).
 */
void gait_app_update(GaitState_t *state, float t_sec, int16_t gy_shin_raw, uint8_t imu_thigh_idx, uint8_t imu_shin_idx);

/**
 * @brief Capture the current pose as the zero-reference (tare).
 */
void gait_app_tare(GaitState_t *state, uint8_t imu_thigh_idx, uint8_t imu_shin_idx);

/**
 * @brief Auto-tare check: call once per sample while EKF is warming up.
 */
void gait_app_auto_tare_check(GaitState_t *state, uint8_t imu_thigh_idx, uint8_t imu_shin_idx);

#ifdef __cplusplus
}
#endif

#endif /* APP_GAIT_APP_H */
