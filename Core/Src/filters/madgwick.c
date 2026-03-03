#include "filters/madgwick.h"
#include "utils/math3d.h"

#ifndef MADGWICK_EPS
#define MADGWICK_EPS (1.0e-9f)
#endif

void madgwick_init(madgwick_t *m, float beta)
{
  if (!m) return;
  m->q0 = 1.0f; m->q1 = 0.0f; m->q2 = 0.0f; m->q3 = 0.0f;
  m->beta = beta;

  m->accel_reject_en = false;
  m->accel_reject_min_g = 0.85f;
  m->accel_reject_max_g = 1.15f;
}

void madgwick_reset(madgwick_t *m)
{
  if (!m) return;
  m->q0 = 1.0f; m->q1 = 0.0f; m->q2 = 0.0f; m->q3 = 0.0f;
}

void madgwick_set_beta(madgwick_t *m, float beta)
{
  if (!m) return;
  m->beta = beta;
}

float madgwick_get_beta(const madgwick_t *m)
{
  if (!m) return 0.0f;
  return m->beta;
}

void madgwick_set_accel_reject(madgwick_t *m, bool en, float min_g, float max_g)
{
  if (!m) return;
  m->accel_reject_en = en;
  m->accel_reject_min_g = min_g;
  m->accel_reject_max_g = max_g;
}

static void integrate_gyro_only(madgwick_t *m, float wx, float wy, float wz, float dt_s)
{
  float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;

  float qDot0 = 0.5f * (-q1*wx - q2*wy - q3*wz);
  float qDot1 = 0.5f * ( q0*wx + q2*wz - q3*wy);
  float qDot2 = 0.5f * ( q0*wy - q1*wz + q3*wx);
  float qDot3 = 0.5f * ( q0*wz + q1*wy - q2*wx);

  q0 += qDot0 * dt_s;
  q1 += qDot1 * dt_s;
  q2 += qDot2 * dt_s;
  q3 += qDot3 * dt_s;

  math3d_quat_normalize(&q0, &q1, &q2, &q3);
  m->q0 = q0; m->q1 = q1; m->q2 = q2; m->q3 = q3;
}

void madgwick_update_imu(madgwick_t *m,
                         float wx, float wy, float wz,
                         float ax_g, float ay_g, float az_g,
                         float dt_s)
{
  if (!m) return;
  if (dt_s <= 0.0f) return;

  // accel magnitude check (optional)
  if (m->accel_reject_en) {
    float an = math3d_sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    if (!(an >= m->accel_reject_min_g && an <= m->accel_reject_max_g)) {
      integrate_gyro_only(m, wx, wy, wz, dt_s);
      return;
    }
  }

  // normalize accel
  float ax = ax_g, ay = ay_g, az = az_g;
  float a2 = ax*ax + ay*ay + az*az;

  if (a2 < MADGWICK_EPS) {
    integrate_gyro_only(m, wx, wy, wz, dt_s);
    return;
  }

  float inva = math3d_inv_sqrtf(a2);
  ax *= inva; ay *= inva; az *= inva;

  float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;

  // auxiliaries
  const float _2q0 = 2.0f*q0;
  const float _2q1 = 2.0f*q1;
  const float _2q2 = 2.0f*q2;
  const float _2q3 = 2.0f*q3;
  const float _4q0 = 4.0f*q0;
  const float _4q1 = 4.0f*q1;
  const float _4q2 = 4.0f*q2;
  const float _8q1 = 8.0f*q1;
  const float _8q2 = 8.0f*q2;

  const float q0q0 = q0*q0;
  const float q1q1 = q1*q1;
  const float q2q2 = q2*q2;
  const float q3q3 = q3*q3;

  // gradient step
  float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
  float s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
  float s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
  float s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

  float s2n = s0*s0 + s1*s1 + s2*s2 + s3*s3;
  if (s2n > MADGWICK_EPS) {
    float invs = math3d_inv_sqrtf(s2n);
    s0 *= invs; s1 *= invs; s2 *= invs; s3 *= invs;
  } else {
    s0 = s1 = s2 = s3 = 0.0f;
  }

  // qDot
  float qDot0 = 0.5f * (-q1*wx - q2*wy - q3*wz) - m->beta*s0;
  float qDot1 = 0.5f * ( q0*wx + q2*wz - q3*wy) - m->beta*s1;
  float qDot2 = 0.5f * ( q0*wy - q1*wz + q3*wx) - m->beta*s2;
  float qDot3 = 0.5f * ( q0*wz + q1*wy - q2*wx) - m->beta*s3;

  // integrate
  q0 += qDot0 * dt_s;
  q1 += qDot1 * dt_s;
  q2 += qDot2 * dt_s;
  q3 += qDot3 * dt_s;

  math3d_quat_normalize(&q0, &q1, &q2, &q3);
  m->q0 = q0; m->q1 = q1; m->q2 = q2; m->q3 = q3;
}
