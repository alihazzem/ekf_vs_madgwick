# Signal Processing Pipeline and Filter Design - EKF vs Madgwick on STM32F411

This document describes the exact implementation used in this project for both the Madgwick filter and the 7-state EKF. The emphasis is on embedded constraints, the sensor configuration, and the concrete gating and tuning logic used in the firmware.

## 0. Scope, hardware, and notation

- Target MCU: STM32F411.
- Sensor stack: GY-91 (MPU-9255 accel/gyro + AK8963 magnetometer) when SENSOR_GY91=1, otherwise MPU6050 (accel/gyro only).
- Nominal sample rate: IMU_FS_HZ = 200.0 Hz. Each filter step uses the measured dt from the DWT cycle counter (84 MHz), not a fixed 1/IMU_FS_HZ.
- Units: accel in g, gyro in rad/s, mag in uT after ASA and calibration.
- Frames:
    - sensor frame (s): raw device axes
    - body frame (b): after REMAP_* macros in app_config.h
    - earth frame (n): z aligned with gravity; yaw defined by magnetometer when available
- Quaternion convention: q = [q0, q1, q2, q3] (scalar first). The code uses v_b = R(q)^T v_n, so the predicted gravity vector is h(q) = R(q)^T [0,0,1]^T.
- Euler output: ZYX (yaw, pitch, roll). Euler angles are for logging only; gimbal lock occurs near pitch = +/-90 deg.

## 1. Madgwick orientation filter (IMU and MARG)

### 1.1 State and gyro propagation

The state is the unit quaternion q. The gyro-driven derivative is:

```
q_dot_gyro = 0.5 * quat_mul(q, [0, wx, wy, wz])
```

Integration is first-order Euler:

```
q_{k+1} = normalize(q_k + q_dot * dt)
```

The implementation subtracts the estimated gyro bias before propagation:

```
wx <- wx - gbx
wy <- wy - gby
wz <- wz - gbz
```

### 1.2 Accelerometer correction as a gradient descent

The predicted gravity direction in the body frame is:

```
g_b(q) = [ 2*(q1*q3 - q0*q2),
                     2*(q2*q3 + q0*q1),
                     q0^2 - q1^2 - q2^2 + q3^2 ]
```

After normalizing the measured accelerometer a_hat, the objective is:

```
f(q) = g_b(q) - a_hat
```

The gradient step is:

```
s = J^T f
q_dot = q_dot_gyro - beta_eff * s
```

The implementation uses the closed-form s0..s3 expressions (computed from q and a_hat), normalizes s, then applies the correction. If the accel magnitude is invalid, the update falls back to pure gyro integration.

### 1.3 Adaptive beta and hard reject

The effective beta combines a startup ramp, motion-adaptive scaling, and a floor:

```
beta_ramp = beta_start + (beta - beta_start) * clamp(t / beta_decay_s, 0, 1)
beta_eff  = beta_ramp / (1 + beta_motion_k * (|a| - 1)^2)
beta_eff  = max(beta_eff, beta_min)
```

A hard reject window provides a binary safety gate:

```
if accel_reject_en and (|a| < min_g or |a| > max_g):
    integrate gyro only
```

In the current configuration the window is [0.3 g, 4.0 g].

### 1.4 Gyro bias estimation (optional)

An integral feedback term is supported:

```
b_dot = 2 * zeta * [ q0*s1 - q1*s0 + q2*s3 - q3*s2, ... ]
```

In the current configuration MADGWICK_ZETA = 0.0, so this bias update is disabled to avoid divergence during aggressive motion.

### 1.5 Magnetometer (MARG) mode

When SENSOR_GY91=1 and the calibrated magnetometer norm is in a safe range (5 to 100 uT), the filter uses madgwick_update_marg:

- The objective function is extended with the magnetic field term.
- The same adaptive beta logic applies.
- If mag is invalid, it falls back to IMU mode.

This provides yaw stabilization without changing the IMU path.

## 2. Extended Kalman filter (7-state, IMU + optional MARG)

### 2.1 State and process model

```
x = [q0, q1, q2, q3, bx, by, bz]^T
```

Quaternion kinematics and bias random walk:

```
q_dot = 0.5 * Omega(omega - b) * q
b_dot = w_b
```

where:

```
Omega(w) = [ 0,  -wx, -wy, -wz
                         wx,  0,   wz, -wy
                         wy, -wz,  0,   wx
                         wz,  wy, -wx,  0  ]
```

The discrete Jacobian is:

```
F = [ I4 + 0.5*dt*Omega(w_c),  -0.5*dt*Xi(q)
            0_{3x4}               ,  I3           ]
```

with:

```
Xi(q) = [ -q1, -q2, -q3
                     q0, -q3,  q2
                     q3,  q0, -q1
                    -q2,  q1,  q0 ]
```

### 2.2 Predict step

- Bias-corrected gyro: w_c = w_raw - b.
- Quaternion propagation: q_new = F_qq * q, then normalize.
- Covariance propagation: P = F * P * F^T + Q.

Process noise is diagonal:

```
Q_qq = sigma_gyro^2 * dt * I4
Q_bb = sigma_bias^2 * dt * I3
```

This is a deliberate conservative approximation of the exact dt^2 scaling. It keeps P from collapsing too quickly on embedded hardware.

### 2.3 Accelerometer update

Measurement model (gravity in body frame):

```
h(q) = [ 2*(q1*q3 - q0*q2),
                 2*(q2*q3 + q0*q1),
                 q0^2 - q1^2 - q2^2 + q3^2 ]
```

The innovation is:

```
y = a_hat - h(q) ,  a_hat = a / |a|
```

The analytic Jacobian H (3x7) is used (bias columns are zero).

Adaptive measurement noise:

```
R_eff = sigma_accel^2 * (1 + r_adapt_k * (|a| - 1)^2) * I3
```

Additional safeguards:

- Hard reject: if accel_reject_en is true and |a| is outside [min_g, max_g], the accel update is skipped. A timeout can delay re-acceptance when needed.
- Bias freeze: if |a| deviates by more than EKF_BIAS_MAX_DEV_G, the bias rows of K are zeroed so dynamic acceleration does not corrupt bias.
- Bias clamp: bx/by/bz are clamped to +/-0.05 rad/s.

Covariance update uses the Joseph form to preserve positive semi-definiteness:

```
P = (I - K H) P (I - K H)^T + K R K^T
```

The 3x3 S inverse uses an analytic Cramer rule. P is symmetrized every 64 accel updates to avoid drift from floating-point asymmetry.

### 2.4 Magnetometer update (MARG)

When the calibrated magnetometer is valid, ekf7_update_mag performs:

- Normalize mag and bootstrap the earth-frame reference m_ref_n on first use.
- Predict body-frame mag from q and m_ref_n.
- Numeric Jacobian for robustness against algebraic errors.
- NIS gate with EKF_MAG_NIS_GATE.
- Residual scaling (EKF_MAG_RESIDUAL_MAX) and dq clamping (+/-0.015) to prevent large jumps.
- Bias rows of K are forced to zero (mag does not update gyro bias).
- Joseph covariance update and periodic symmetrization.

If mag is invalid, ekf7_step (IMU-only) is used and yaw is left to gyro integration.

## 3. Key differences between the two filters

| Aspect | Madgwick | EKF |
|---|---|---|
| State | Quaternion only (4 floats) | Quaternion + gyro bias (7 states) + 7x7 covariance |
| Update mechanism | Gradient descent | EKF predict/update with linearization |
| Bias handling | Optional integral feedback (zeta) | Explicit bias states, freeze logic, and clamp |
| Accel trust | beta schedule + hard reject | Adaptive R + hard reject + bias freeze |
| Mag support | MARG mode when mag valid | Dedicated mag update with NIS gate and numeric Jacobian |
| Covariance | N/A | Joseph update + symmetry enforcement |
| CPU cost | Low (scalar ops) | Higher (matrix ops, 3x3 inverse) |
| Outputs | Quaternion -> Euler | Quaternion -> Euler, P available via ekf7_trace_P |

## 4. Sensor pipeline and preprocessing

### 4.1 Data acquisition

Each imu_app_step() iteration (called by the scheduler):

1. Captures the current cycle count (DWT) and computes dt from the previous sample.
2. Reads accel/gyro from MPU-9255 (or MPU6050).
3. Optionally reads AK8963 magnetometer (best-effort, may be invalid for a sample).

The magnetometer path is only compiled when SENSOR_GY91=1.

### 4.2 Raw to physical units

Accel (g), gyro (rad/s):

```
ax_s = (ax_raw - ACCEL_BIAS_X) / 16384
ay_s = (ay_raw - ACCEL_BIAS_Y) / 16384
az_s = (az_raw - ACCEL_BIAS_Z) / 16384

wx_s = (gx_raw - gx_off) / 131 * (pi/180)
wy_s = (gy_raw - gy_off) / 131 * (pi/180)
wz_s = (gz_raw - gz_off) / 131 * (pi/180)
```

- ACCEL_BIAS_* are compile-time offsets from a flat, static calibration.
- gx_off/gy_off/gz_off are runtime offsets from a blocking gyro calibration routine.

### 4.3 Axis remapping

Sensor axes are mapped into the body frame using the macros:

```
ax_g = REMAP_AX_G(ax_s, ay_s, az_s)
ay_g = REMAP_AY_G(ax_s, ay_s, az_s)
az_g = REMAP_AZ_G(ax_s, ay_s, az_s)

wx   = REMAP_WX(wx_s, wy_s, wz_s)
wy   = REMAP_WY(wx_s, wy_s, wz_s)
wz   = REMAP_WZ(wx_s, wy_s, wz_s)
```

The default mapping is identity, but the macros allow any mounting orientation without modifying filter code.

### 4.4 Magnetometer preprocessing (GY-91 only)

- Raw AK8963 counts are remapped to the body frame.
- ASA sensitivity adjustment is applied.
- Units are scaled to uT using AK8963_UT_PER_LSB.
- Hard/soft-iron calibration is applied:

```
m_cal = S * (m_body - b)
```

Magnetometer samples are considered valid only if the norm is within 5 to 100 uT.

### 4.5 First-sample alignment

On the first valid sample, both filters initialize roll and pitch from the accelerometer. If a valid magnetometer is present, yaw is also initialized:

- Madgwick: madgwick_init_from_accel or madgwick_init_from_marg
- EKF: ekf7_init_from_accel or ekf7_init_from_marg

## 5. End-to-end pipeline (per sample)

```
Timer/scheduler tick (IMU_FS_HZ)
    -> imu_app_step()
         -> Read accel/gyro (+ mag if available)
         -> Compute dt from DWT cycles
         -> Bias correction + unit conversion
         -> Axis remap (sensor -> body)
         -> Init alignment (first sample only)
         -> Madgwick update (IMU or MARG)
         -> EKF update (predict + accel + optional mag)
         -> Quaternion to Euler (ZYX)
         -> CSV logging (decimated)
```

## 6. Numerical and real-time considerations

- All filters run in float32.
- Large EKF scratch matrices are static to avoid stack overflow on the 1 KB default stack.
- 3x3 inverses use analytic Cramer rule to avoid heavy linear algebra libraries.
- Quaternion and measurement vectors are normalized at every step.
- P is symmetrized periodically to counter floating-point asymmetry.
- The integration dt is measured at cycle resolution to avoid timing quantization errors.

## 7. Configuration reference (from app_config.h)

### Sensor and sampling

| Macro | Value | Description |
|---|---|---|
| SENSOR_GY91 | 1 | 1 = GY-91 (MPU-9255 + AK8963), 0 = MPU6050 |
| IMU_FS_HZ | 200.0 | Nominal sample rate (Hz) |
| RUN_MADGWICK | 1 | Enable Madgwick |
| RUN_EKF | 1 | Enable EKF |

### Madgwick parameters

| Macro | Value | Description |
|---|---|---|
| MADGWICK_BETA | 1.0 | Steady-state correction gain |
| MADGWICK_BETA_START | 0.5 | Initial beta for fast convergence |
| MADGWICK_BETA_DECAY_S | 2.0 | Ramp duration (s) |
| MADGWICK_ZETA | 0.0 | Gyro bias gain (disabled) |
| MADGWICK_BETA_MOTION_K | 5.0 | Motion-adaptive scaling |
| MADGWICK_BETA_MIN | 0.0 | Beta floor |
| MADGWICK_ACCEL_REJECT_EN | 1 | Hard reject enable |
| MADGWICK_ACCEL_MIN_G | 0.3 | Reject if |a| < 0.3 g |
| MADGWICK_ACCEL_MAX_G | 4.0 | Reject if |a| > 4.0 g |

### EKF parameters

| Macro | Value | Description |
|---|---|---|
| EKF_SIGMA_GYRO | 0.01 | Gyro noise density (rad/s/sqrt(Hz)) |
| EKF_SIGMA_BIAS | 1e-6 | Bias random walk (rad/s^2/sqrt(Hz)) |
| EKF_SIGMA_ACCEL | 0.05 | Accel noise density (g/sqrt(Hz)) |
| EKF_SIGMA_MAG | 8.0 | Magnetometer noise (uT) |
| EKF_R_ADAPT_K | 1000.0 | Adaptive-R steepness |
| EKF_BIAS_MAX_DEV_G | 0.15 | Freeze bias update if |a|-1 > this |
| EKF_MAG_NIS_GATE | 20.0 | Mag NIS gate |
| EKF_MAG_RESIDUAL_MAX | 1.5 | Mag residual clamp |
| EKF_P0 | 1.0 | Initial P diagonal |
| EKF_ACCEL_REJECT_EN | 1 | Hard reject enable |
| EKF_ACCEL_MIN_G | 0.3 | Reject if |a| < 0.3 g |
| EKF_ACCEL_MAX_G | 4.0 | Reject if |a| > 4.0 g |
| EKF_ACCEL_TIMEOUT_S | 0.0 | Accel re-accept delay |

## 8. Output and logging

The UART CSV line (decimated by s_print_div) is:

```
D,t_ms,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,
mad_roll_mdeg,mad_pitch_mdeg,mad_yaw_mdeg,mad_us,
ekf_roll_mdeg,ekf_pitch_mdeg,ekf_yaw_mdeg,ekf_us
```

Notes:

- Euler outputs are in milli-degrees.
- Pitch is negated when printed to match the desired visualization convention.
- EKF trace(P) and bias are available via getters (ekf7_trace_P, ekf7_get_bias) but are not part of the default CSV.

## 9. Assumptions and limitations

- If magnetometer data is missing or invalid, yaw is unobservable and will drift.
- Accel-based correction assumes gravity dominates; dynamic linear acceleration is treated as measurement noise (adaptive R or hard reject).
- Bias estimation is only active in the EKF by default (Madgwick zeta is disabled).
- Euler angles are only for visualization and are not used internally.
