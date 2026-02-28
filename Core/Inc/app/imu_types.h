#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========= RAW SENSOR ========= */
typedef struct
{
    int16_t ax, ay, az;   // accel raw
    int16_t gx, gy, gz;   // gyro raw
    int16_t temp;         // optional
} ImuRaw_t;

/* ========= PROCESSED SENSOR (shared input to both filters) ========= */
typedef struct
{
    uint32_t t_us;        // timestamp (microseconds)
    float dt;             // seconds

    // Accel in g (or m/s^2 if you prefer later). Start with g to match your MATLAB pipeline.
    float ax_g, ay_g, az_g;

    // Gyro in rad/s (best for integration)
    float wx, wy, wz;

    // Useful diagnostics
    float acc_norm;       // sqrt(ax^2+ay^2+az^2) in g
} ImuData_t;

/* ========= ATTITUDE OUTPUT ========= */
typedef struct
{
    // Quaternion (w, x, y, z)
    float q0, q1, q2, q3;

    // Euler angles in degrees (for logging only)
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} Attitude_t;

/* ========= FILTER OUTPUT PACK ========= */
typedef struct
{
    Attitude_t madgwick;
    Attitude_t ekf;

    // EKF-specific accel usage weight (0..1) (if you implement weighting)
    float ekf_acc_used;

} FusionOut_t;

#ifdef __cplusplus
}
#endif

#endif // IMU_TYPES_H
