# Madgwick Filter — Algorithm Details & Tuning

## Overview

The project implements the **IMU-only** variant of Sebastian Madgwick's gradient-descent orientation filter (no magnetometer). The filter fuses gyroscope (angular rate) and accelerometer (gravity direction) data to estimate a quaternion representing the sensor's orientation relative to the world frame.

**Reference:** Madgwick, S.O.H., Harrison, A.J.L., Vaidyanathan, R. — *"An efficient orientation filter for inertial and inertial/magnetic measurement units"* (2011).

---

## Algorithm Summary

The filter maintains a unit quaternion $ \mathbf{q} = [q_0, q_1, q_2, q_3] $ (scalar-first convention) representing the rotation from the world frame to the body frame.

### Step 1 — Gyroscope Integration (Prediction)

The quaternion derivative from angular velocity $ \boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z] $ (rad/s) is:

$$
\dot{\mathbf{q}}_\omega = \frac{1}{2} \mathbf{q} \otimes [0, \omega_x, \omega_y, \omega_z]
$$

Expanded:

$$
\begin{aligned}
\dot{q}_0 &= \tfrac{1}{2}(-q_1 \omega_x - q_2 \omega_y - q_3 \omega_z) \\
\dot{q}_1 &= \tfrac{1}{2}(q_0 \omega_x + q_2 \omega_z - q_3 \omega_y) \\
\dot{q}_2 &= \tfrac{1}{2}(q_0 \omega_y - q_1 \omega_z + q_3 \omega_x) \\
\dot{q}_3 &= \tfrac{1}{2}(q_0 \omega_z + q_1 \omega_y - q_2 \omega_x)
\end{aligned}
$$

### Step 2 — Accelerometer Correction (Gradient Descent)

The accelerometer measures gravity in the body frame. The expected gravity in the body frame, given quaternion $ \mathbf{q} $, is obtained by rotating $ [0, 0, 1]^T $ by $ \mathbf{q}^{-1} $. The error function measures how far the measured accel direction is from predicted gravity.

The gradient of this error with respect to the quaternion components produces a correction vector $ \mathbf{s} = [s_0, s_1, s_2, s_3] $, which is normalized.

### Step 3 — Fusion

The gyro-predicted rate is blended with the gradient correction:

$$
\dot{\mathbf{q}} = \dot{\mathbf{q}}_\omega - \beta \cdot \hat{\mathbf{s}}
$$

Where $ \beta $ is the algorithm gain parameter. The quaternion is then integrated:

$$
\mathbf{q}_{t+1} = \mathbf{q}_t + \dot{\mathbf{q}} \cdot \Delta t
$$

And re-normalized to unit length.

---

## Implementation Details

### Source File: `Core/Src/filters/madgwick.c`

The implementation closely follows the original Madgwick paper with these characteristics:

1. **Variable dt:** Each call to `madgwick_update_imu()` takes the actual measured `dt_s` (from DWT cycle counter), not a fixed constant. This makes the filter robust to timing jitter.

2. **Accel normalization:** The accelerometer input can be in any magnitude (g-units); it is normalized to unit length before computing the gradient.

3. **Degenerate input guard:** If the accel magnitude is near zero (free-fall), only gyro integration runs.

4. **Gradient normalization:** The step vector $ \mathbf{s} $ is normalized before application to ensure the correction magnitude is controlled solely by $ \beta $.

### Accelerometer Rejection

An optional feature (enabled by default) rejects the accelerometer correction when the measured acceleration norm falls outside an acceptable range around 1 g:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MADGWICK_ACCEL_REJECT_EN` | `1` | Enable feature |
| `MADGWICK_ACCEL_MIN_G` | `0.85` | Lower norm threshold |
| `MADGWICK_ACCEL_MAX_G` | `1.15` | Upper norm threshold |

When $ \| \mathbf{a} \| \notin [0.85, 1.15] $ g, the filter falls back to **gyro-only integration** for that sample. This prevents linear acceleration (hand motion, vibration) from corrupting the gravity estimate.

---

## Tuning the Beta Parameter

Beta ($ \beta $) is the **sole tuning knob** of the Madgwick filter. It controls the trade-off between:

| Low beta (e.g., 0.01) | High beta (e.g., 0.5) |
|------------------------|----------------------|
| Trust gyroscope more | Trust accelerometer more |
| Slow convergence to true gravity | Fast convergence |
| Smooth, low-noise output | Noisy, jerky output |
| Accumulates gyro drift over time | Resists drift |

### Recommended Procedure

1. Start with `beta = 0.1` and enable streaming:
   ```
   MAD BETA 0.1
   MPU STREAM ON
   ```
2. Tilt the board slowly and check `MAD SHOW` — angles should track physical motion.
3. Set the board flat and watch for drift over 30–60 seconds.
4. If drifty → increase beta slightly.
5. If too noisy / vibration-sensitive → decrease beta.
6. Typical good values: **0.02 – 0.15** for MPU6050 at 100 Hz.

You can change beta at any time without resetting the filter:
```
MAD BETA 0.04
```

### Theoretical Optimal Beta

From the Madgwick paper, the optimal beta relates to gyroscope measurement error:

$$
\beta_{\text{opt}} = \sqrt{\frac{3}{4}} \cdot \sigma_\omega
$$

Where $ \sigma_\omega $ is the gyroscope noise density in rad/s. For the MPU6050 at ±250 °/s, typical noise density is ~0.005 °/s/√Hz, giving:

$$
\beta_{\text{opt}} \approx 0.866 \times 0.005 \times \frac{\pi}{180} \approx 0.00008 \text{ rad/s}
$$

In practice, higher values (0.01–0.1) work better because they account for unmodeled gyro bias drift.

---

## Quaternion to Euler Conversion

The output quaternion is converted to aerospace-convention Euler angles (ZYX intrinsic rotation) using `math3d_quat_to_euler_deg()`:

$$
\begin{aligned}
\text{roll}  &= \text{atan2}(2(q_0 q_1 + q_2 q_3),\; 1 - 2(q_1^2 + q_2^2)) \\
\text{pitch} &= \text{asin}(\text{clamp}(2(q_0 q_2 - q_3 q_1),\; -1, 1)) \\
\text{yaw}   &= \text{atan2}(2(q_0 q_3 + q_1 q_2),\; 1 - 2(q_2^2 + q_3^2))
\end{aligned}
$$

**Note:** Euler angles are for logging/display only. The quaternion is the primary representation used internally.

---

## Comparison with EKF (Planned)

Once the EKF is implemented, the two filters can be compared on:

| Criterion | Madgwick | EKF |
|-----------|----------|-----|
| CPU cost | Very low (no matrix ops) | Higher (matrix multiply, invert) |
| Tuning parameters | 1 (beta) | Many (Q, R matrices) |
| Convergence speed | Fast | Depends on initialization |
| Steady-state accuracy | Good | Potentially optimal |
| Dynamic motion handling | Accel rejection heuristic | Formal noise model |
| Code complexity | ~150 lines | ~300+ lines expected |

The `RUN_MADGWICK` and `RUN_EKF` flags in `app_config.h` allow enabling both filters simultaneously for side-by-side comparison using the same sensor data.
