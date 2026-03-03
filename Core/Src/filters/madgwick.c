#include "filters/madgwick.h"
#include "utils/math3d.h"
#include <math.h>

#ifndef MADGWICK_EPS
#define MADGWICK_EPS (1.0e-9f)
#endif

void madgwick_init(madgwick_t *m, float beta)
{
  if (!m)
    return;
  m->q0 = 1.0f;
  m->q1 = 0.0f;
  m->q2 = 0.0f;
  m->q3 = 0.0f;
  m->beta = beta;

  m->accel_reject_en = false;
  m->accel_reject_min_g = 0.85f;
  m->accel_reject_max_g = 1.15f;

  // gyro bias — disabled until madgwick_set_bias_gain() is called
  m->gbx = 0.0f;
  m->gby = 0.0f;
  m->gbz = 0.0f;
  m->zeta = 0.0f;

  // adaptive beta — disabled by default (same start as final)
  m->elapsed_s = 0.0f;
  m->beta_start = beta;   // identical => no ramp effect
  m->beta_decay_s = 0.0f; // 0 => disabled
}

void madgwick_reset(madgwick_t *m)
{
  if (!m)
    return;
  m->q0 = 1.0f;
  m->q1 = 0.0f;
  m->q2 = 0.0f;
  m->q3 = 0.0f;
  // clear learned bias so the filter starts fresh
  m->gbx = 0.0f;
  m->gby = 0.0f;
  m->gbz = 0.0f;
  // restart adaptive-beta ramp
  m->elapsed_s = 0.0f;
}

void madgwick_set_beta(madgwick_t *m, float beta)
{
  if (!m)
    return;
  m->beta = beta;
}

float madgwick_get_beta(const madgwick_t *m)
{
  if (!m)
    return 0.0f;
  return m->beta;
}

void madgwick_set_accel_reject(madgwick_t *m, bool en, float min_g, float max_g)
{
  if (!m)
    return;
  m->accel_reject_en = en;
  m->accel_reject_min_g = min_g;
  m->accel_reject_max_g = max_g;
}

void madgwick_set_bias_gain(madgwick_t *m, float zeta)
{
  if (!m)
    return;
  m->zeta = zeta;
}

void madgwick_set_adaptive_beta(madgwick_t *m, float beta_start, float beta_decay_s)
{
  if (!m)
    return;
  m->beta_start = beta_start;
  m->beta_decay_s = beta_decay_s;
}

void madgwick_init_from_accel(madgwick_t *m, float ax, float ay, float az)
{
  if (!m)
    return;

  // need a valid accel vector
  float a2 = ax * ax + ay * ay + az * az;
  if (a2 < MADGWICK_EPS)
    return;

  float inv = math3d_inv_sqrtf(a2);
  ax *= inv;
  ay *= inv;
  az *= inv;

  // roll and pitch from gravity direction (yaw stays 0 — no magnetometer)
  float roll = atan2f(ay, az);
  float pitch = atan2f(-ax, math3d_sqrtf(ay * ay + az * az));

  // roll/pitch/yaw=0 to quaternion (ZYX convention, cy=1, sy=0)
  float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);

  m->q0 = cr * cp;
  m->q1 = sr * cp;
  m->q2 = cr * sp;
  m->q3 = -sr * sp;

  // restart adaptive-beta ramp from this point
  m->elapsed_s = 0.0f;
  // NOTE: intentionally keep learned bias (gbx/gby/gbz) — don't throw it away
}

static void integrate_gyro_only(madgwick_t *m, float wx, float wy, float wz, float dt_s)
{
  float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;

  float qDot0 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
  float qDot1 = 0.5f * (q0 * wx + q2 * wz - q3 * wy);
  float qDot2 = 0.5f * (q0 * wy - q1 * wz + q3 * wx);
  float qDot3 = 0.5f * (q0 * wz + q1 * wy - q2 * wx);

  q0 += qDot0 * dt_s;
  q1 += qDot1 * dt_s;
  q2 += qDot2 * dt_s;
  q3 += qDot3 * dt_s;

  math3d_quat_normalize(&q0, &q1, &q2, &q3);
  m->q0 = q0;
  m->q1 = q1;
  m->q2 = q2;
  m->q3 = q3;
}

void madgwick_update_imu(madgwick_t *m,
                         float wx, float wy, float wz,
                         float ax_g, float ay_g, float az_g,
                         float dt_s)
{
  if (!m)
    return;
  if (dt_s <= 0.0f)
    return;

  // ---- 1. advance elapsed time (always, every call) ----
  m->elapsed_s += dt_s;

  // ---- 2. adaptive beta ----
  float beta_eff;
  if (m->beta_decay_s > 0.0f && m->elapsed_s < m->beta_decay_s)
  {
    float t = m->elapsed_s / m->beta_decay_s;
    beta_eff = m->beta_start + (m->beta - m->beta_start) * t;
  }
  else
  {
    beta_eff = m->beta;
  }

  // ---- 3. gyro bias correction ----
  wx -= m->gbx;
  wy -= m->gby;
  wz -= m->gbz;

  // ---- 4. accel magnitude check (optional) ----
  if (m->accel_reject_en)
  {
    float an = math3d_sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    if (!(an >= m->accel_reject_min_g && an <= m->accel_reject_max_g))
    {
      integrate_gyro_only(m, wx, wy, wz, dt_s);
      return;
    }
  }

  // ---- 5. normalize accel ----
  float ax = ax_g, ay = ay_g, az = az_g;
  float a2 = ax * ax + ay * ay + az * az;

  if (a2 < MADGWICK_EPS)
  {
    integrate_gyro_only(m, wx, wy, wz, dt_s);
    return;
  }

  float inva = math3d_inv_sqrtf(a2);
  ax *= inva;
  ay *= inva;
  az *= inva;

  float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;

  // auxiliaries
  const float _2q0 = 2.0f * q0;
  const float _2q1 = 2.0f * q1;
  const float _2q2 = 2.0f * q2;
  const float _2q3 = 2.0f * q3;
  const float _4q0 = 4.0f * q0;
  const float _4q1 = 4.0f * q1;
  const float _4q2 = 4.0f * q2;
  const float _8q1 = 8.0f * q1;
  const float _8q2 = 8.0f * q2;

  const float q0q0 = q0 * q0;
  const float q1q1 = q1 * q1;
  const float q2q2 = q2 * q2;
  const float q3q3 = q3 * q3;

  // ---- 6. gradient step ----
  float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
  float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
  float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
  float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

  float s2n = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
  if (s2n > MADGWICK_EPS)
  {
    float invs = math3d_inv_sqrtf(s2n);
    s0 *= invs;
    s1 *= invs;
    s2 *= invs;
    s3 *= invs;
  }
  else
  {
    s0 = s1 = s2 = s3 = 0.0f;
  }

  // ---- 7. gyro bias update (uses normalised gradient, only when accel contributes) ----
  if (m->zeta > 0.0f)
  {
    m->gbx += 2.0f * dt_s * m->zeta * (q0 * s1 - q1 * s0 + q2 * s3 - q3 * s2);
    m->gby += 2.0f * dt_s * m->zeta * (q0 * s2 - q1 * s3 - q2 * s0 + q3 * s1);
    m->gbz += 2.0f * dt_s * m->zeta * (q0 * s3 + q1 * s2 - q2 * s1 - q3 * s0);
  }

  // ---- 8. qDot with effective (possibly ramping) beta ----
  float qDot0 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz) - beta_eff * s0;
  float qDot1 = 0.5f * (q0 * wx + q2 * wz - q3 * wy) - beta_eff * s1;
  float qDot2 = 0.5f * (q0 * wy - q1 * wz + q3 * wx) - beta_eff * s2;
  float qDot3 = 0.5f * (q0 * wz + q1 * wy - q2 * wx) - beta_eff * s3;

  // ---- 9. integrate + normalize ----
  q0 += qDot0 * dt_s;
  q1 += qDot1 * dt_s;
  q2 += qDot2 * dt_s;
  q3 += qDot3 * dt_s;

  math3d_quat_normalize(&q0, &q1, &q2, &q3);
  m->q0 = q0;
  m->q1 = q1;
  m->q2 = q2;
  m->q3 = q3;
}
