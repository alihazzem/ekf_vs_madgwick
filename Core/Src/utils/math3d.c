#include "utils/math3d.h"
#include <math.h>

#ifndef MATH3D_EPS
#define MATH3D_EPS (1.0e-12f)
#endif

float math3d_sqrtf(float x)
{
  return sqrtf(x);
}

float math3d_inv_sqrtf(float x)
{
  if (x <= MATH3D_EPS) return 0.0f;
  return 1.0f / sqrtf(x);
}

void math3d_quat_normalize(float *q0, float *q1, float *q2, float *q3)
{
  float n2 = (*q0)*(*q0) + (*q1)*(*q1) + (*q2)*(*q2) + (*q3)*(*q3);
  if (n2 <= MATH3D_EPS) { *q0 = 1.0f; *q1 = *q2 = *q3 = 0.0f; return; }
  float inv = math3d_inv_sqrtf(n2);
  *q0 *= inv; *q1 *= inv; *q2 *= inv; *q3 *= inv;
}

static float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void math3d_quat_to_euler_deg(float q0, float q1, float q2, float q3,
                             float *roll_deg, float *pitch_deg, float *yaw_deg)
{
  float sinr_cosp = 2.0f * (q0*q1 + q2*q3);
  float cosr_cosp = 1.0f - 2.0f * (q1*q1 + q2*q2);
  float roll = atan2f(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (q0*q2 - q3*q1);
  sinp = clampf(sinp, -1.0f, 1.0f);
  float pitch = asinf(sinp);

  float siny_cosp = 2.0f * (q0*q3 + q1*q2);
  float cosy_cosp = 1.0f - 2.0f * (q2*q2 + q3*q3);
  float yaw = atan2f(siny_cosp, cosy_cosp);

  const float RAD2DEG = 57.29577951308232f;
  if (roll_deg)  *roll_deg  = roll  * RAD2DEG;
  if (pitch_deg) *pitch_deg = pitch * RAD2DEG;
  if (yaw_deg)   *yaw_deg   = yaw   * RAD2DEG;
}
