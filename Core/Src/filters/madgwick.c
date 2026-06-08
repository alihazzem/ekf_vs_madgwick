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

  // motion-adaptive beta — disabled by default
  m->beta_motion_k = 0.0f;
  m->beta_min = 0.0f;
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

void madgwick_set_motion_gain(madgwick_t *m, float motion_k, float beta_min)
{
  if (!m)
    return;
  m->beta_motion_k = motion_k;
  m->beta_min = beta_min;
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

void madgwick_init_from_marg(madgwick_t *m,
                             float ax, float ay, float az,
                             float mx, float my, float mz)
{
  if (!m)
    return;

  // 1) roll/pitch from accel (yaw=0 baseline)
  madgwick_init_from_accel(m, ax, ay, az);

  // 2) yaw from normalized magnetometer
  float m2 = mx * mx + my * my + mz * mz;
  if (m2 < MADGWICK_EPS)
    return;

  float invm = math3d_inv_sqrtf(m2);
  mx *= invm;
  my *= invm;
  mz *= invm;

  float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;

  // De-rotate mag into earth frame for yaw solve (same convention as EKF init).
  float hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) + mz * (q1 * q3 + q0 * q2));
  float hy = 2.0f * (mx * (q1 * q2 + q0 * q3) + my * (0.5f - q1 * q1 - q3 * q3) + mz * (q2 * q3 - q0 * q1));

  float yaw = atan2f(-hy, hx);
  float cy = cosf(0.5f * yaw);
  float sy = sinf(0.5f * yaw);

  // Pre-multiply by pure-yaw quaternion [cy, 0, 0, sy]
  float t0 = cy * q0 - sy * q3;
  float t1 = cy * q1 - sy * q2;
  float t2 = cy * q2 + sy * q1;
  float t3 = cy * q3 + sy * q0;

  math3d_quat_normalize(&t0, &t1, &t2, &t3);
  m->q0 = t0;
  m->q1 = t1;
  m->q2 = t2;
  m->q3 = t3;

  // Restart adaptive-beta ramp from this aligned state.
  m->elapsed_s = 0.0f;
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

  // ---- 4. accel magnitude (computed once — reused below) ----
  float a2 = ax_g * ax_g + ay_g * ay_g + az_g * az_g;
  float a_mag = math3d_sqrtf(a2);

  // ----  motion-adaptive beta: reduce correction gain during dynamics ----
  // beta_eff /= (1 + k * (|a| - 1)^2)   Mirror of EKF's adaptive R.
  if (m->beta_motion_k > 0.0f)
  {
    float dev = a_mag - 1.0f;
    beta_eff /= (1.0f + m->beta_motion_k * dev * dev);
  }
  // beta floor: prevent the filter from becoming a pure gyro integrator
  if (beta_eff < m->beta_min)
    beta_eff = m->beta_min;

  // ---- 5. hard reject — uses precomputed a_mag ----
  if (m->accel_reject_en)
  {
    if (!(a_mag >= m->accel_reject_min_g && a_mag <= m->accel_reject_max_g))
    {
      integrate_gyro_only(m, wx, wy, wz, dt_s);
      return;
    }
  }

  // ---- 6. normalize accel — uses precomputed a2 ----
  float ax = ax_g, ay = ay_g, az = az_g;

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

  // ---- 7. gradient step ----
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

  // ---- 8. gyro bias update (uses normalised gradient, only when accel contributes) ----
  if (m->zeta > 0.0f)
  {
    m->gbx += 2.0f * dt_s * m->zeta * (q0 * s1 - q1 * s0 + q2 * s3 - q3 * s2);
    m->gby += 2.0f * dt_s * m->zeta * (q0 * s2 - q1 * s3 - q2 * s0 + q3 * s1);
    m->gbz += 2.0f * dt_s * m->zeta * (q0 * s3 + q1 * s2 - q2 * s1 - q3 * s0);
  }

  // ---- 9. qDot with effective (possibly ramping) beta ----
  float qDot0 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz) - beta_eff * s0;
  float qDot1 = 0.5f * (q0 * wx + q2 * wz - q3 * wy) - beta_eff * s1;
  float qDot2 = 0.5f * (q0 * wy - q1 * wz + q3 * wx) - beta_eff * s2;
  float qDot3 = 0.5f * (q0 * wz + q1 * wy - q2 * wx) - beta_eff * s3;

  // ---- 10. integrate + normalize ----
  q0 += qDot0 * dt_s;
  q1 += qDot1 * dt_s;
  q2 += qDot2 * dt_s;
  q3 += qDot3 * dt_s;

  math3d_quat_normalize(&q0, &q1, &q2, &q3);
  m->q0 = q0;
  m->q1 = q1;
  m->q2 = q2;
  m->q3 = q3;
  m->last_beta_eff = beta_eff;  /* cache for external diagnostics */
}

void madgwick_update_marg(madgwick_t *m,
                          float wx, float wy, float wz,
                          float ax_g, float ay_g, float az_g,
                          float mx, float my, float mz,
                          float dt_s)
{
  if (!m || dt_s <= 0.0f)
    return;

  // Fallback to IMU mode if mag data is zero/invalid
  float m2 = mx * mx + my * my + mz * mz;
  if (m2 < MADGWICK_EPS)
  {
    madgwick_update_imu(m, wx, wy, wz, ax_g, ay_g, az_g, dt_s);
    return;
  }

  // ---- 1. advance elapsed time ----
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

  // ---- 3. gyro bias ---
  wx -= m->gbx;
  wy -= m->gby;
  wz -= m->gbz;

  // ---- 4. accel motion adaptation ----
  float a2 = ax_g * ax_g + ay_g * ay_g + az_g * az_g;
  float a_mag = math3d_sqrtf(a2);

  if (m->beta_motion_k > 0.0f)
  {
    float dev = a_mag - 1.0f;
    beta_eff /= (1.0f + m->beta_motion_k * dev * dev);
  }
  if (beta_eff < m->beta_min)
    beta_eff = m->beta_min;

  if (m->accel_reject_en && !(a_mag >= m->accel_reject_min_g && a_mag <= m->accel_reject_max_g))
  {
    integrate_gyro_only(m, wx, wy, wz, dt_s);
    return;
  }

  if (a2 < MADGWICK_EPS)
  {
    integrate_gyro_only(m, wx, wy, wz, dt_s);
    return;
  }

  // normalize accel and mag
  float inva = math3d_inv_sqrtf(a2);
  float ax = ax_g * inva;
  float ay = ay_g * inva;
  float az = az_g * inva;

  float invm = math3d_inv_sqrtf(m2);
  mx *= invm;
  my *= invm;
  mz *= invm;

  float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;

  // Aux variables
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;

  // Reference direction of Earth's magnetic field
  float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
  float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
  float hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
  float bx = math3d_sqrtf(hx * hx + hy * hy);
  float bz = hz;

  float _2q0 = 2.0f * q0;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2bx = 2.0f * bx;
  float _2bz = 2.0f * bz;
  float _4bx = 4.0f * bx;
  float _4bz = 4.0f * bz;

  // Gradient computation
  float s0 = -_2q2 * (2.0f * q1q3 - 2.0f * q0q2 - ax) + _2q1 * (2.0f * q0q1 + 2.0f * q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  float s1 = _2q3 * (2.0f * q1q3 - 2.0f * q0q2 - ax) + _2q0 * (2.0f * q0q1 + 2.0f * q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  float s2 = -_2q0 * (2.0f * q1q3 - 2.0f * q0q2 - ax) + _2q3 * (2.0f * q0q1 + 2.0f * q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  float s3 = _2q1 * (2.0f * q1q3 - 2.0f * q0q2 - ax) + _2q2 * (2.0f * q0q1 + 2.0f * q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

  float s2n = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
  if (s2n > MADGWICK_EPS)
  {
    float invs = math3d_inv_sqrtf(s2n);
    s0 *= invs;
    s1 *= invs;
    s2 *= invs;
    s3 *= invs;
  }

  // gyro bias
  if (m->zeta > 0.0f)
  {
    m->gbx += 2.0f * dt_s * m->zeta * (q0 * s1 - q1 * s0 + q2 * s3 - q3 * s2);
    m->gby += 2.0f * dt_s * m->zeta * (q0 * s2 - q1 * s3 - q2 * s0 + q3 * s1);
    m->gbz += 2.0f * dt_s * m->zeta * (q0 * s3 + q1 * s2 - q2 * s1 - q3 * s0);
  }

  // qDot
  float qDot0 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz) - beta_eff * s0;
  float qDot1 = 0.5f * (q0 * wx + q2 * wz - q3 * wy) - beta_eff * s1;
  float qDot2 = 0.5f * (q0 * wy - q1 * wz + q3 * wx) - beta_eff * s2;
  float qDot3 = 0.5f * (q0 * wz + q1 * wy - q2 * wx) - beta_eff * s3;

  // Integrate
  q0 += qDot0 * dt_s;
  q1 += qDot1 * dt_s;
  q2 += qDot2 * dt_s;
  q3 += qDot3 * dt_s;

  math3d_quat_normalize(&q0, &q1, &q2, &q3);
  m->q0 = q0;
  m->q1 = q1;
  m->q2 = q2;
  m->q3 = q3;
  m->last_beta_eff = beta_eff;  /* cache for external diagnostics */
}

float madgwick_get_beta_eff(const madgwick_t *m)
{
  if (!m)
    return 0.0f;
  return m->last_beta_eff;
}
