#pragma once

#ifdef __cplusplus
extern "C" {
#endif

float math3d_sqrtf(float x);
float math3d_inv_sqrtf(float x);

void math3d_quat_normalize(float *q0, float *q1, float *q2, float *q3);

void math3d_quat_to_euler_deg(float q0, float q1, float q2, float q3,
                             float *roll_deg, float *pitch_deg, float *yaw_deg);

void math3d_quat_multiply(const float qa[4], const float qb[4], float q_out[4]);

void math3d_quat_conjugate(const float q[4], float q_out[4]);

#ifdef __cplusplus
}
#endif
