#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  // quaternion (w,x,y,z)
  float q0, q1, q2, q3;

  // algorithm gain
  float beta;

  // optional accel rejection based on |a| in g
  bool  accel_reject_en;
  float accel_reject_min_g;
  float accel_reject_max_g;
} madgwick_t;

void  madgwick_init(madgwick_t *m, float beta);
void  madgwick_reset(madgwick_t *m);

void  madgwick_set_beta(madgwick_t *m, float beta);
float madgwick_get_beta(const madgwick_t *m);

void  madgwick_set_accel_reject(madgwick_t *m, bool en, float min_g, float max_g);

/**
 * Update (IMU-only, no magnetometer)
 * gyro: rad/s
 * accel: g (doesn't need to be normalized)
 * dt: seconds
 */
void madgwick_update_imu(madgwick_t *m,
                         float wx, float wy, float wz,
                         float ax_g, float ay_g, float az_g,
                         float dt_s);

#ifdef __cplusplus
}
#endif
