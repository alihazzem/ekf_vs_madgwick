# Madgwick Review & EKF Implementation Plan

---

## Part 1 — Current Madgwick Implementation Review

### What's Good

| Aspect | Detail |
|--------|--------|
| **Gradient math** | Correct IMU-only gradient-descent step matching the Madgwick 2011 paper |
| **Gyro-only fallback** | `integrate_gyro_only()` cleanly handles the case when accel is rejected or near-zero |
| **Accel rejection** | Magnitude window (0.85–1.15 g) rejects linear acceleration — prevents tilt errors during motion |
| **Normalization** | Both accel input and gradient vector are normalized before use, with epsilon guards |
| **Measured dt** | Uses DWT cycle counter for real dt rather than assuming fixed 10 ms — critical for accuracy |
| **Clean API** | Init / reset / set_beta / set_accel_reject are properly separated and NULL-safe |

### Suggested Enhancements

#### 1. ✅ DONE — Online Gyro Bias Estimation

The current calibration (`imu_app_cal_gyro`) is a one-shot blocking average at startup. Gyro bias drifts with temperature. The original Madgwick paper includes an optional **online bias estimator** using the gradient direction:

```c
// Add to madgwick_t:
float gbx, gby, gbz;   // estimated gyro bias (rad/s)
float zeta;             // bias drift gain (typically 0.015)

// Inside madgwick_update_imu(), after computing normalized s0–s3:
m->gbx += 2.0f * dt_s * m->zeta * (q0*s1 - q1*s0 + q2*s3 - q3*s2);
m->gby += 2.0f * dt_s * m->zeta * (q0*s2 - q1*s3 - q2*s0 + q3*s1);
m->gbz += 2.0f * dt_s * m->zeta * (q0*s3 + q1*s2 - q2*s1 - q3*s0);

// Subtract bias from gyro before computing qDot:
wx -= m->gbx;
wy -= m->gby;
wz -= m->gbz;
```

This continuously tracks slow drift without requiring the user to re-calibrate.

#### 2. ✅ DONE — Gravity-Based Initial Alignment

`madgwick_reset()` sets q = [1, 0, 0, 0] (identity = board flat). If the board is tilted when streaming starts, Madgwick must converge from a wrong initial guess, which can take several seconds.

Add:
```c
void madgwick_init_from_accel(madgwick_t *m, float ax, float ay, float az);
```

This computes the initial roll/pitch from the first accel sample so the filter starts already aligned. Yaw remains unknown (no magnetometer), but roll/pitch are immediate.

Algorithm:
```
pitch = atan2(-ax, sqrt(ay*ay + az*az))
roll  = atan2(ay, az)
q     = euler_to_quaternion(roll, pitch, 0)
```

#### 3. ✅ DONE — Adaptive Beta

Fixed beta = 0.08 means slow initial convergence. A common enhancement:

```c
// Start with high beta for fast initial alignment
float beta_start = 0.5f;
float beta_final = 0.08f;
float beta_decay_time = 2.0f;  // seconds

// In update:
float t = elapsed_since_init;
float beta = (t < beta_decay_time)
    ? beta_start + (beta_final - beta_start) * (t / beta_decay_time)
    : beta_final;
```

This gives fast convergence in the first ~2 seconds, then settles to low-noise steady-state.

#### 4. 🔧 PENDING — Convergence Metric Export

Currently `s_mad_valid` is just a boolean. Exporting the gradient magnitude (`sqrt(s0² + s1² + s2² + s3²)` before normalization) gives a continuous convergence health metric — useful when comparing against the EKF's covariance trace.

#### 5. ❌ MOOT — `_2q3` Claimed Unused

`_2q3` was initially flagged as unused, but it **is** used in the `s1` and `s2` gradient terms. No change needed.

---

## Part 2 — EKF Implementation Plan

### Theory

The EKF (Extended Kalman Filter) estimates attitude as a quaternion, using:

- **State vector (x):** 4-element unit quaternion `[q0, q1, q2, q3]`
- **Predict (gyro):** Quaternion kinematics driven by gyro measurements
- **Update (accel):** Corrects using gravity direction observed by accelerometer

The key equations:

$$
\text{Predict: } q_{k+1} = \left(I + \frac{\Delta t}{2}\Omega(\omega)\right) q_k
$$

$$
\text{Covariance: } P_{k+1} = F \cdot P_k \cdot F^T + Q
$$

$$
\text{Update: } K = P H^T (H P H^T + R)^{-1}
$$

$$
q = q + K \cdot (z - h(q)), \quad \text{then normalize}
$$

$$
P = (I - KH) P
$$

Where:
- $\Omega(\omega)$ is the 4×4 skew-symmetric matrix from gyro rates
- $h(q) = R(q)^T [0,0,1]^T$ is the predicted gravity in body frame
- $z$ is the normalized accelerometer reading
- $H$ is the 3×4 Jacobian of $h(q)$ with respect to $q$

### EKF vs Madgwick Comparison

| Aspect | Madgwick | EKF |
|--------|----------|-----|
| Tuning knob | Single `beta` | `sigma_gyro`, `sigma_accel` (physical units) |
| Accel weighting | Binary (reject or full weight) | **Continuous** — Kalman gain adapts automatically |
| CPU cost | ~150 multiplies | ~400–600 multiplies + one 3×3 inverse |
| Memory | 7 floats | ~80 floats (state + P matrix + temporaries) |
| Convergence | Gradient descent — can oscillate if beta too high | Optimal in the MMSE sense for Gaussian noise |
| Implementation complexity | Simple | Moderate (Jacobians, matrix ops) |

At 100 Hz on an 84 MHz Cortex-M4F with hardware FPU, both fit comfortably within the 10 ms budget. Expect the EKF to use ~100–200 µs per step.

### Step-by-Step Implementation

#### Step 1 — Define the EKF Struct (`ekf.h`)

```c
#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // State: unit quaternion [w, x, y, z]
    float q[4];

    // Error covariance (4×4, symmetric — store full for simplicity)
    float P[4][4];

    // Tunables
    float sigma_gyro;    // gyro noise (rad/s/√Hz)
    float sigma_accel;   // accel noise (g/√Hz)

    // Accel rejection (same idea as Madgwick)
    bool  accel_reject_en;
    float accel_reject_min_g;
    float accel_reject_max_g;
} ekf_attitude_t;

void  ekf_init(ekf_attitude_t *e, float sigma_gyro, float sigma_accel);
void  ekf_reset(ekf_attitude_t *e);

void  ekf_predict(ekf_attitude_t *e,
                  float wx, float wy, float wz,
                  float dt_s);

void  ekf_update_accel(ekf_attitude_t *e,
                       float ax_g, float ay_g, float az_g);

void  ekf_step(ekf_attitude_t *e,
               float wx, float wy, float wz,
               float ax_g, float ay_g, float az_g,
               float dt_s);

void  ekf_set_accel_reject(ekf_attitude_t *e, bool en, float min_g, float max_g);

#ifdef __cplusplus
}
#endif
```

#### Step 2 — Predict Step (`ekf.c`)

This is the gyro-driven time-update:

```c
void ekf_predict(ekf_attitude_t *e, float wx, float wy, float wz, float dt_s)
{
    float q0 = e->q[0], q1 = e->q[1], q2 = e->q[2], q3 = e->q[3];

    // 1. Build state transition matrix F = I + 0.5*dt*Omega(w)
    //
    //  Omega(w) = [  0  -wx  -wy  -wz ]
    //             [ wx   0   wz  -wy ]
    //             [ wy  -wz   0   wx ]
    //             [ wz   wy  -wx   0 ]
    //
    //  F[i][j] = delta_ij + 0.5 * dt * Omega[i][j]

    float hdt = 0.5f * dt_s;
    float F[4][4] = {
        { 1.0f,     -hdt*wx,  -hdt*wy,  -hdt*wz },
        { hdt*wx,    1.0f,     hdt*wz,  -hdt*wy },
        { hdt*wy,   -hdt*wz,   1.0f,     hdt*wx },
        { hdt*wz,    hdt*wy,  -hdt*wx,   1.0f   }
    };

    // 2. Propagate state: q_new = F * q_old
    float qn[4] = {0};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            qn[i] += F[i][j] * e->q[j];

    // Normalize
    float norm = sqrtf(qn[0]*qn[0] + qn[1]*qn[1] + qn[2]*qn[2] + qn[3]*qn[3]);
    if (norm > 1e-9f) {
        float inv = 1.0f / norm;
        for (int i = 0; i < 4; i++) e->q[i] = qn[i] * inv;
    }

    // 3. Process noise: Q = sigma_gyro^2 * dt^2 * (G * G^T)
    //    G = dq/dw * 0.5, simplified for quaternion kinematics
    //    For simplicity, use Q = sigma_g^2 * dt * I_4x4 (diagonal approx)
    float q_var = e->sigma_gyro * e->sigma_gyro * dt_s;

    // 4. Covariance: P = F * P * F^T + Q
    float FP[4][4] = {0};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                FP[i][j] += F[i][k] * e->P[k][j];

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++)
                sum += FP[i][k] * F[j][k];   // F^T
            e->P[i][j] = sum + ((i == j) ? q_var : 0.0f);
        }
}
```

#### Step 3 — Update Step (Accelerometer Correction)

This is the measurement-update where we correct the gyro-only prediction using gravity:

```c
void ekf_update_accel(ekf_attitude_t *e, float ax_g, float ay_g, float az_g)
{
    // 1. Accel rejection check (same as Madgwick)
    float a_mag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    if (e->accel_reject_en) {
        if (a_mag < e->accel_reject_min_g || a_mag > e->accel_reject_max_g)
            return;  // skip update, keep predict-only
    }

    // 2. Normalize accel
    if (a_mag < 1e-9f) return;
    float inv = 1.0f / a_mag;
    float ax = ax_g * inv, ay = ay_g * inv, az = az_g * inv;

    float q0 = e->q[0], q1 = e->q[1], q2 = e->q[2], q3 = e->q[3];

    // 3. Predicted gravity in body frame: h(q) = R(q)^T * [0,0,1]
    float hx = 2.0f * (q1*q3 - q0*q2);
    float hy = 2.0f * (q2*q3 + q0*q1);
    float hz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 4. Innovation: y = z - h(q)
    float y[3] = { ax - hx, ay - hy, az - hz };

    // 5. Jacobian H (3×4): dh/dq
    float H[3][4] = {
        { -2*q2,  2*q3, -2*q0,  2*q1 },
        {  2*q1,  2*q0,  2*q3,  2*q2 },
        {  2*q0, -2*q1, -2*q2,  2*q3 }
    };

    // 6. S = H * P * H^T + R   (3×3)
    //    K = P * H^T * S^{-1}   (4×3)
    //    q = q + K * y
    //    P = (I - K*H) * P
    //
    // (implement with inline 3×3 inverse — see math helpers below)
}
```

#### Step 4 — Math Helpers Needed (`math3d.c`)

```c
// Analytic 3×3 inverse (Cramer's rule — always 3×3, never larger)
bool math3d_mat3_inv(const float A[3][3], float Ainv[3][3]);

// Matrix multiply helpers (small fixed-size, fully unrolled)
void math3d_mat4x4_mult(const float A[4][4], const float B[4][4], float C[4][4]);
void math3d_mat4x3_mult_3x4(const float A[4][3], const float B[3][4], float C[4][4]);
```

The 3×3 inverse via Cramer's rule is ~30 multiplies and 1 division — very efficient on M4F.

#### Step 5 — Wire Into `imu_app.c`

Mirror the existing Madgwick wiring:

```c
#include "filters/ekf.h"

static ekf_attitude_t s_ekf;
static Attitude_t     s_ekf_att;
static uint8_t        s_ekf_valid = 0;

// In imu_app_init():
#if RUN_EKF
    ekf_init(&s_ekf, EKF_SIGMA_GYRO, EKF_SIGMA_ACCEL);
    ekf_set_accel_reject(&s_ekf, MADGWICK_ACCEL_REJECT_EN,
                         MADGWICK_ACCEL_MIN_G, MADGWICK_ACCEL_MAX_G);
#endif

// In imu_app_poll(), after the Madgwick block:
#if RUN_EKF
    if (dt_s > 0.0f) {
        ekf_step(&s_ekf, wx, wy, wz, ax_g, ay_g, az_g, dt_s);

        math3d_quat_to_euler_deg(s_ekf.q[0], s_ekf.q[1], s_ekf.q[2], s_ekf.q[3],
                                 &s_ekf_att.roll_deg, &s_ekf_att.pitch_deg, &s_ekf_att.yaw_deg);
        s_ekf_valid = 1;
    }
#endif
```

#### Step 6 — Add CLI Commands (`cli_app.c`)

```
EKF SHOW        — print quaternion + Euler angles
EKF RESET       — reset to identity quaternion, reset P
EKF TUNE <sg> <sa> — set sigma_gyro and sigma_accel at runtime
EKF DIAG        — print trace(P) as convergence metric
```

#### Step 7 — Add Config Defaults (`app_config.h`)

```c
#define EKF_SIGMA_GYRO     0.01f     // rad/s — MPU6050 typical
#define EKF_SIGMA_ACCEL    0.05f     // g     — MPU6050 typical
```

### Initial Tuning Values

| Parameter | Value | Source |
|-----------|-------|--------|
| `sigma_gyro` | 0.01 rad/s | MPU6050 datasheet: gyro noise density ~0.005 °/s/√Hz at ±250 dps. Use 0.01 as conservative starting point. |
| `sigma_accel` | 0.05 g | MPU6050 datasheet: accel noise ~400 µg/√Hz at ±2g. Use 0.05 as conservative starting point. |
| `accel_reject` | 0.85–1.15 g | Same window as Madgwick for fair comparison |

### Recommended Build Order

| Order | Task | Test |
|-------|------|------|
| 1 | `ekf.h` struct + `ekf_init` / `ekf_reset` | Compiles, q = [1,0,0,0], P = I |
| 2 | `ekf_predict` (gyro only) | Keep board still → quaternion should barely move. Rotate → quaternion drifts (no correction yet) |
| 3 | `ekf_update_accel` + 3×3 inverse | Tilt board → roll/pitch track correctly. Hold at angle → steady output |
| 4 | Wire into `imu_app.c` behind `#if RUN_EKF` | `EKF SHOW` prints angles alongside `MAD SHOW` |
| 5 | CLI commands | `EKF RESET`, `EKF TUNE`, `EKF DIAG` all work |
| 6 | Side-by-side comparison | Stream both filters, compare convergence speed, noise, and tilt accuracy |

### CPU Budget Estimate (84 MHz Cortex-M4F)

| Operation | Approx. cycles | Approx. µs |
|-----------|---------------|-------------|
| Predict (F build + mat mult + normalize) | 800–1200 | 10–15 |
| Update (H, S, S⁻¹, K, q update, P update) | 2000–4000 | 25–50 |
| **Total EKF per step** | **~3000–5000** | **~35–60** |
| Madgwick per step (for reference) | ~800–1200 | ~10–15 |
| Available budget at 100 Hz | 840,000 | 10,000 |

Both filters combined use < 1% of the available CPU budget.
