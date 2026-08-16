#include "app/gait_app.h"
#include "app/imu_app.h"      /* imu_app_get_ekf() public API */
#include "app/app_config.h"
#include "app/imu_types.h"    /* Attitude_t */

#include "stm32f4xx_hal.h"    /* HAL_GetTick */
#include <math.h>             /* atan2f, roundf, fabsf */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>           /* memset */
#include "utils/math3d.h"

/* ── Public API ──────────────────────────────────────────────────────────── */

void gait_app_init_state(GaitState_t *state)
{
    memset(state, 0, sizeof(GaitState_t));
    state->last_thigh_sample = -9999.0f;
    state->last_shin_sample  = -9999.0f;
}

void gait_app_tare(GaitState_t *state, uint8_t imu_thigh_idx, uint8_t imu_shin_idx)
{
    Attitude_t att0, att1;
    if (imu_app_get_ekf(imu_thigh_idx, &att0) && imu_app_get_ekf(imu_shin_idx, &att1)) {
        float q0[4] = {att0.q0, att0.q1, att0.q2, att0.q3};
        float q1[4] = {att1.q0, att1.q1, att1.q2, att1.q3};
        state->tare_thigh = math3d_gravity_pitch(q0);
        state->tare_shin  = math3d_gravity_pitch(q1);
        /* Reset EMA so it seeds cleanly from the new zero reference */
        state->ema_init = false;
    }
}

void gait_app_auto_tare_check(GaitState_t *state, uint8_t imu_thigh_idx, uint8_t imu_shin_idx)
{
    if (state->auto_tare_done) return;

    Attitude_t att0, att1;
    if (!imu_app_get_ekf(imu_thigh_idx, &att0) || !imu_app_get_ekf(imu_shin_idx, &att1)) return;

    float q0[4] = {att0.q0, att0.q1, att0.q2, att0.q3};
    float q1[4] = {att1.q0, att1.q1, att1.q2, att1.q3};

    float cur_thigh = math3d_gravity_pitch(q0);
    float cur_shin  = math3d_gravity_pitch(q1);

    float thigh_delta = fabsf(cur_thigh - state->last_thigh_sample);
    float shin_delta  = fabsf(cur_shin  - state->last_shin_sample);

    state->last_thigh_sample = cur_thigh;
    state->last_shin_sample  = cur_shin;

    bool is_stable = (thigh_delta < 0.5f) && (shin_delta < 0.5f)
                     && (state->last_thigh_sample != -9999.0f);

    uint32_t now_ms = HAL_GetTick();
    if (!is_stable) {
        state->stable_start_ms = now_ms;
    } else if ((now_ms - state->stable_start_ms) > 5000) {
        gait_app_tare(state, imu_thigh_idx, imu_shin_idx);
        state->auto_tare_done = true;
    }
}

void gait_app_update(GaitState_t *state, float t_sec, int16_t gy_shin_raw, uint8_t imu_thigh_idx, uint8_t imu_shin_idx)
{
    if (!state->auto_tare_done) {
        state->thigh_pitch = 0.0f;
        state->shin_pitch  = 0.0f;
        state->knee_angle  = 0.0f;
        return;
    }

    Attitude_t att0, att1;
    if (!imu_app_get_ekf(imu_thigh_idx, &att0) || !imu_app_get_ekf(imu_shin_idx, &att1)) return;

    /* ── 1. Raw tared pitch angles from gravity vector ────────────────── */
    float q0[4] = {att0.q0, att0.q1, att0.q2, att0.q3};
    float q1[4] = {att1.q0, att1.q1, att1.q2, att1.q3};

    float raw_thigh = math3d_gravity_pitch(q0);
    float raw_shin  = math3d_gravity_pitch(q1);

    float t_pitch = -(raw_thigh - state->tare_thigh);
    float s_pitch = -(raw_shin  - state->tare_shin);

    /* ── 2. EMA smoothing (alpha=0.08 for maximum smoothness) ─────────── */
    if (!state->ema_init) {
        state->smooth_thigh = t_pitch;
        state->smooth_shin  = s_pitch;
        state->ema_init = true;
    } else {
        const float alpha = 0.08f;
        state->smooth_thigh = state->smooth_thigh * (1.0f - alpha) + t_pitch * alpha;
        state->smooth_shin  = state->smooth_shin  * (1.0f - alpha) + s_pitch * alpha;
    }

    /* ── 3. Quantise to 0.1° and apply 0.1° deadband ──────────────────── */
    float q_thigh = roundf(state->smooth_thigh * 10.0f) / 10.0f;
    float q_shin  = roundf(state->smooth_shin  * 10.0f) / 10.0f;

    if (fabsf(q_thigh - state->thigh_pitch) >= 0.1f) state->thigh_pitch = q_thigh;
    if (fabsf(q_shin  - state->shin_pitch)  >= 0.1f) state->shin_pitch  = q_shin;

    /* ── 4. Knee angle = Thigh − Shin (clamp to ≥ 0) ──────────────────── */
    state->knee_angle = state->thigh_pitch - state->shin_pitch;
    if (state->knee_angle < 0.0f) state->knee_angle = 0.0f;

    /* ── 5. STANCE / SWING state machine ──────────────────────────────── */
    state->gyro_sma = state->gyro_sma * 0.8f + (float)gy_shin_raw * 0.2f;
    state->event_flag = 0;

    if (state->gait_phase == 0) { /* STANCE */
        if ((t_sec - state->last_hs_s) > 0.300f) {
            if (state->knee_angle > 15.0f && state->gyro_sma > 1000.0f) {
                state->gait_phase    = 1;  /* → SWING */
                state->last_to_s     = t_sec;
                state->event_flag    = 2;  /* Toe-Off */
                state->swing_max_knee = state->knee_angle;
            }
        }
    } else { /* SWING */
        if (state->knee_angle > state->swing_max_knee)
            state->swing_max_knee = state->knee_angle;

        if (!state->mid_swing_armed
            && (t_sec - state->last_to_s) > 0.300f
            && state->knee_angle < (state->swing_max_knee * 0.8f)) {
            state->mid_swing_armed = 1;
            state->event_flag = 3; /* Mid-Swing armed */
        }

        if (state->mid_swing_armed
            && state->knee_angle < 25.0f
            && state->gyro_sma < 0.0f) {
            state->gait_phase      = 0; /* → STANCE */
            state->mid_swing_armed = 0;
            state->event_flag      = 1; /* Heel-Strike */
            state->stride_time     = t_sec - state->last_hs_s;
            state->last_hs_s       = t_sec;
        }
    }
}
