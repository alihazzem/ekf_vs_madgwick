#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    // quaternion (w,x,y,z)
    float q0, q1, q2, q3;

    // algorithm gain (final / steady-state value)
    float beta;

    // optional accel rejection based on |a| in g
    bool accel_reject_en;
    float accel_reject_min_g;
    float accel_reject_max_g;

    // online gyro bias estimation (rad/s)
    float gbx, gby, gbz; // estimated bias, accumulated over time
    float zeta;          // bias gain — set to 0 to disable

    // adaptive beta: ramps from beta_start down to beta over beta_decay_s seconds
    float elapsed_s;    // time since last init or reset
    float beta_start;   // starting beta (set equal to beta to disable adaptive)
    float beta_decay_s; // ramp duration in seconds (0 to disable)

    // motion-adaptive beta: scales beta down when |a| deviates from 1g
    float beta_motion_k; // beta_eff /= (1 + k * dev^2), dev = |a_mag - 1|
    float beta_min;      // floor: beta_eff never drops below this value
  } madgwick_t;

  void madgwick_init(madgwick_t *m, float beta);
  void madgwick_reset(madgwick_t *m);

  void madgwick_set_beta(madgwick_t *m, float beta);
  float madgwick_get_beta(const madgwick_t *m);

  void madgwick_set_accel_reject(madgwick_t *m, bool en, float min_g, float max_g);

  // Online gyro bias gain (call after madgwick_init; 0.0 = disabled)
  void madgwick_set_bias_gain(madgwick_t *m, float zeta);

  // Adaptive beta: ramps from beta_start to m->beta over beta_decay_s seconds
  void madgwick_set_adaptive_beta(madgwick_t *m, float beta_start, float beta_decay_s);

  // Motion-adaptive beta + floor: beta_eff is scaled down as |a| deviates from 1g,
  // and is clamped to beta_min to prevent the filter going fully gyro-only.
  // motion_k = 0 disables the scaling; beta_min = 0 disables the floor.
  void madgwick_set_motion_gain(madgwick_t *m, float motion_k, float beta_min);

  // Initialise quaternion from a single accel reading (sets roll/pitch, yaw=0).
  // Call this before the first madgwick_update_imu to skip the cold-start convergence.
  void madgwick_init_from_accel(madgwick_t *m, float ax, float ay, float az);

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
