# Signal Processing Pipeline & Filter Design — EKF vs Madgwick on STM32F411

## 1. How Each Filter Works

### 1A. Madgwick Orientation Filter

The Madgwick filter is a **gradient-descent-based complementary filter**. It fuses gyroscope and accelerometer data to estimate a unit quaternion `q = [q0, q1, q2, q3]` representing the board's 3D orientation.

**Core idea:** At every time step, two independent orientation estimates compete:

1. **Gyroscope integration** (fast, but drifts over time) — gives the quaternion rate of change:

```
q_dot_gyro = 0.5 * q ⊗ ω
```

where `ω = [0, wx, wy, wz]` is the bias-corrected angular velocity.

2. **Accelerometer correction** (noisy instant-by-instant, but zero mean drift) — the filter asks: *"Given my current quaternion estimate, where should gravity point in the body frame?"* It then computes the **gradient of the error** between the predicted gravity direction and the measured accelerometer vector, and steps the quaternion in the direction that minimizes that error.

**The gradient step** (see `Core/Src/filters/madgwick.c`):

The objective function being minimized is:

```
f(q, a_hat) = q* ⊗ g ⊗ q − a_hat
```

where `g = [0,0,0,1]` (gravity in earth frame) and `a_hat` is the normalized accelerometer measurement. The Jacobian `J` of this function with respect to `q` gives a gradient direction:

```
∇f = Jᵀ f
```

The filter normalizes this gradient `ŝ = ∇f / ‖∇f‖` and subtracts it from the gyro-driven quaternion derivative, weighted by `β`:

```
q_dot = q_dot_gyro − β · ŝ
```

The parameter **β** controls the tradeoff:
- Larger β → more trust in accelerometer → less drift, but more vibration sensitivity
- Smaller β → more trust in gyroscope → smoother but more drift

Your implementation adds several advanced features on top of the basic algorithm:

| Feature | Parameter | What it does |
|---|---|---|
| **Adaptive beta ramp** | `beta_start=0.5`, `beta_decay_s=2.0` | Starts with high β for fast initial convergence, linearly decays to steady-state `β=0.08` over 2 seconds |
| **Motion-adaptive beta** | `beta_motion_k=10.0`, `beta_min=0.01` | Reduces β when `‖a‖ ≠ 1g` (dynamic motion): `β_eff = β / (1 + k(‖a‖−1)²)`, floored at `beta_min` |
| **Online gyro bias estimation** | `zeta=0.015` | Integrates the gradient error to slowly estimate and subtract gyro bias: `b_dot = 2ζ(q* ⊗ ŝ)` |
| **Hard accel rejection** | `[0.85g, 1.15g]` | If `‖a‖` is outside this window, skip accel correction entirely and integrate gyro only |

---

### 1B. Extended Kalman Filter (7-State)

The EKF is a **statistically optimal recursive estimator**. It maintains a **7-element state vector** and a **7×7 covariance matrix P** that tracks uncertainty:

```
x = [q0, q1, q2, q3, bx, by, bz]ᵀ
```

where `q0..q3` is the orientation quaternion and `bx, by, bz` is the gyro bias (rad/s).

The EKF operates in two alternating phases every sample:

#### Predict Step (`Core/Src/filters/ekf.c`) — Gyro-Driven Time Update

1. **Bias correction:** Subtract estimated bias from raw gyro: `ω_c = ω_raw − b`

2. **State propagation:** Apply the quaternion kinematic equation via the state-transition matrix F:

```
F = [ I₄ + (dt/2)·Ω(ω_c)  |  −(dt/2)·Ξ(q) ]
    [       0₃ₓ₄           |       I₃        ]
```

where `Ω(ω)` is the 4×4 skew-symmetric gyro matrix and `Ξ(q)` maps gyro perturbations to quaternion-rate perturbations. The new quaternion is `q_new = F_qq · q`, then renormalized.

3. **Covariance propagation:**

```
P⁻ = F · P · Fᵀ + Q
```

where Q is diagonal process noise:
- `Q[0..3] = σ_gyro² · dt` — quaternion uncertainty grows with gyro noise
- `Q[4..6] = σ_bias² · dt` — bias drifts as a random walk

#### Update Step (`Core/Src/filters/ekf.c`) — Accelerometer Measurement Correction

1. **Measurement model:** The expected gravity in body frame given the current quaternion:

```
h(q) = Rᵀ(q) · [0, 0, 1]ᵀ = [ 2(q1·q3 − q0·q2) ]
                               [ 2(q2·q3 + q0·q1) ]
                               [ q0²− q1²− q2²+ q3² ]
```

2. **Innovation:** `y = a_normalized − h(q)` — the discrepancy between what the accelerometer reads and what the filter predicts.

3. **Jacobian H (3×7):** Analytically derived partial derivatives `∂h/∂x`. The last 3 columns (bias) are zero because gravity prediction does not directly depend on bias.

4. **Adaptive measurement noise R:**

```
R_eff = σ_accel² · (1 + k · (‖a‖ − 1)²) · I₃
```

This is the key design: when the accelerometer magnitude deviates from 1g (meaning the board is accelerating, not just measuring gravity), the measurement noise inflates **continuously and smoothly**. With `k = 200`, even a 0.1g deviation doubles R, heavily discounting the accelerometer. This is more elegant than a binary hard-reject threshold.

5. **Standard Kalman equations:**
   - `S = H·P⁻·Hᵀ + R_eff` — innovation covariance (3×3), inverted analytically via Cramer's rule
   - `K = P⁻·Hᵀ·S⁻¹` — Kalman gain (7×3)
   - `x⁺ = x⁻ + K·y` — state correction
   - `P⁺ = (I − K·H)·P⁻` — covariance shrink

6. **Maintenance:** Quaternion is renormalized after update. Every 64 updates, P is symmetrized (`P = (P + Pᵀ)/2`) to prevent numerical drift from accumulating.

---

### Key Differences Between the Two Filters

| Aspect | Madgwick | EKF |
|---|---|---|
| **State** | Quaternion only (4 floats) | Quaternion + bias (7 floats) + 7×7 covariance |
| **Bias tracking** | Heuristic integral of gradient error (ζ) | Statistically optimal via Kalman gain into bias states |
| **Accel trust** | β parameter (heuristic, motion-adaptive) | Adaptive R based on covariance propagation (optimal) |
| **Convergence** | Controlled by β schedule | Controlled by initial P₀ (high = uncertain = fast) |
| **CPU cost** | ~tens of µs (scalar operations only) | ~hundreds of µs (7×7 matrix multiplications) |
| **Memory** | ~50 bytes | ~250+ bytes (state + 7×7 P matrix + static scratch) |
| **Tuning** | 1 main knob (β) + optional extras | 5 noise parameters (σ_gyro, σ_bias, σ_accel, k, P₀) |

---

## 2. How the Data Is Filtered

### Raw Sensor → Physical Units

The MPU6050 is configured at 100 Hz with DLPF at mode 3 (~42 Hz bandwidth). A burst I2C read of 14 bytes (`ACCEL_XOUT_H` through `GYRO_ZOUT_L`) is performed every tick. The raw 16-bit values are converted:

- **Accelerometer:** `raw_int16 ÷ 16384 = g` (±2g full-scale, `AFS_SEL=0`)
- **Gyroscope:** `(raw_int16 − calibration_offset) ÷ 131 × π/180 = rad/s` (±250°/s full-scale, `FS_SEL=0`)

A static gyro offset (`s_gx_off`, `s_gy_off`, `s_gz_off`) is subtracted before conversion — a simple at-rest calibration to remove constant bias from the raw counts.

### Axis Remapping

The sensor frame does not match the desired body frame. Six macros in `Core/Inc/app/app_config.h` remap axes:

```
Body X  = −Sensor Y        Body ωx = −Sensor ωy
Body Y  = −Sensor Z        Body ωy = −Sensor ωz
Body Z  = +Sensor X        Body ωz = +Sensor ωx
```

### Gravity Alignment (First Sample)

On the very first valid sample, both filters call their `init_from_accel()` function. This computes roll and pitch from the gravity vector:

```
roll  = atan2(ay, az)
pitch = atan2(−ax, sqrt(ay² + az²))
```

and converts to a quaternion (with yaw = 0, since there is no magnetometer). This means the filters start with correct roll/pitch immediately rather than needing seconds to converge from the identity quaternion.

### Per-Sample Filter Execution

Both filters receive the **same** body-frame `(wx, wy, wz, ax_g, ay_g, az_g)` and the **measured** dt (computed from DWT cycle counter, not assumed to be exactly 10 ms). Each filter then:

1. Uses the gyroscope to propagate the quaternion forward in time (prediction)
2. Uses the accelerometer to correct drift (correction/update)
3. Outputs a quaternion which is converted to Euler angles (roll, pitch, yaw in degrees) via the ZYX convention in `math3d_quat_to_euler_deg()`

The Madgwick filter does both in a single fused step (`madgwick_update_imu`). The EKF does them sequentially (`ekf7_predict` → `ekf7_update_accel`, wrapped in `ekf7_step`).

### What "Filtering" Actually Means Here

Both filters perform **sensor fusion**, not simple low-pass filtering:

- The **gyroscope** provides high-frequency orientation changes (responsive but drifting)
- The **accelerometer** provides a low-frequency gravity reference (stable long-term but noisy short-term)
- The filters **optimally blend** these two: trusting gyro for fast motions, trusting accel for steady-state

During dynamic motion (when `‖a‖ ≠ 1g`), both filters automatically reduce accelerometer influence — the Madgwick via motion-adaptive β, the EKF via adaptive R.

---

## 3. The Full Pipeline

Complete data flow from silicon to serial output, executed at 100 Hz:

```
┌─────────────────────────────────────────────────────────────┐
│  TIM interrupt (100 Hz)  →  sets s_tick_due = true          │
└───────────────────────────────┬─────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│  imu_app_poll()  (main super-loop)                          │
│                                                             │
│  1. Consume tick flag                                       │
│  2. Capture DWT cycle timestamp → compute dt (seconds)      │
│  3. I2C burst read: 14 bytes from MPU6050                   │
│     (ax, ay, az, temp, gx, gy, gz — big-endian int16)       │
└───────────────────────────────┬─────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│  Unit Conversion                                            │
│                                                             │
│  accel : raw / 16384               →  g                     │
│  gyro  : (raw − offset) / 131 × π/180  →  rad/s            │
└───────────────────────────────┬─────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│  Axis Remapping  (sensor frame → body frame)                │
│                                                             │
│  ax_body = −ay_sensor    wx_body = −wy_sensor               │
│  ay_body = −az_sensor    wy_body = −wz_sensor               │
│  az_body = +ax_sensor    wz_body = +wx_sensor               │
└───────────────────────────────┬─────────────────────────────┘
                                │
                     ┌──────────┴──────────┐
                     │   First sample?      │
                     │  → init_from_accel() │
                     │   (gravity align)    │
                     └──────────┬──────────┘
                                │
              ┌─────────────────┴─────────────────┐
              ▼                                   ▼
┌──────────────────────────┐    ┌──────────────────────────────┐
│   MADGWICK FILTER         │    │   EXTENDED KALMAN FILTER      │
│                           │    │                               │
│ 1. Compute effective β:   │    │  PREDICT:                     │
│    - adaptive ramp        │    │  1. subtract bias from gyro   │
│    - motion scaling       │    │  2. build 7×7 Jacobian F      │
│    - floor clamp          │    │  3. propagate q via F         │
│ 2. Hard-reject check      │    │  4. P = F·P·Fᵀ + Q            │
│    (‖a‖ ∈ [0.85, 1.15]?) │    │                               │
│ 3. Normalize accel        │    │  UPDATE:                      │
│ 4. Gradient descent:      │    │  5. hard-reject check         │
│    - compute ∇f           │    │  6. normalize accel           │
│    - normalize gradient   │    │  7. h(q) = predicted gravity  │
│ 5. Update gyro bias       │    │  8. innovation y = a − h(q)   │
│    (ζ integration)        │    │  9. adaptive R from ‖a‖       │
│ 6. Fuse:                  │    │  10. S = H·P·Hᵀ + R           │
│    q_dot = q_dot_gyro     │    │  11. K = P·Hᵀ·S⁻¹             │
│          − β_eff · ŝ      │    │  12. x += K·y                 │
│ 7. Integrate + normalize  │    │  13. P = (I − KH)·P           │
│                            │    │  14. renormalize q            │
│ Output : quaternion        │    │                               │
│ Cost   : ~tens µs          │    │  Output : quaternion + bias   │
└─────────────┬──────────────┘    │  Cost   : ~hundreds µs        │
              │                   └───────────────┬───────────────┘
              │                                   │
              └─────────────┬─────────────────────┘
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  Quaternion → Euler Conversion  (ZYX convention)            │
│                                                             │
│  roll  = atan2( 2(q0·q1 + q2·q3),  1 − 2(q1² + q2²) )    │
│  pitch = asin ( clamp(2(q0·q2 − q3·q1)) )                  │
│  yaw   = atan2( 2(q0·q3 + q1·q2),  1 − 2(q2² + q3²) )    │
│                                                             │
│  × 57.296  →  degrees                                       │
└───────────────────────────────┬─────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│  UART CSV Output  (decimated by s_print_div)                │
│                                                             │
│  D, t_ms,                                                   │
│  ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw,           │
│  mad_roll, mad_pitch, mad_yaw, mad_cpu_µs,                  │
│  ekf_roll, ekf_pitch, ekf_yaw, trace(P), ekf_cpu_µs,       │
│  bias_x, bias_y, bias_z                                     │
│                                                             │
│  → captured by Python scripts → plotted in MATLAB           │
└─────────────────────────────────────────────────────────────┘
```

### Timing & Scheduling

- A **hardware timer** fires at 100 Hz, calling `imu_app_on_100hz_tick()` which sets a flag.
- The **main super-loop** calls `imu_app_poll()`, which checks the flag, reads the sensor, runs both filters, and optionally prints.
- The actual dt is measured using the ARM **DWT cycle counter** (`timebase_cycles()`), not assumed — this handles any jitter in the cooperative scheduler.
- If the main loop is too slow and misses a tick, the overrun is counted (`s_missed`) and the sample is skipped rather than doubled.

### Stack Safety

The EKF needs large temporary matrices (7×7 = 196 floats each). On the STM32F411 with a 1 KB default stack, these are declared as **file-scope static arrays** (`s_F`, `s_FP`, `s_HP`, `s_PHt`, `s_K`) rather than stack-local. This is safe because the system is single-threaded cooperative — predict and update never run concurrently.

---

## 4. Configuration Reference

All tunable parameters live in `Core/Inc/app/app_config.h`.

### Sample Rate & Feature Switches

| Macro | Value | Description |
|---|---|---|
| `IMU_FS_HZ` | `100.0` | Sensor sample rate (Hz) |
| `RUN_MADGWICK` | `1` | Enable/disable Madgwick filter |
| `RUN_EKF` | `1` | Enable/disable EKF |

### Madgwick Parameters

| Macro | Value | Description |
|---|---|---|
| `MADGWICK_BETA` | `0.08` | Steady-state correction gain |
| `MADGWICK_BETA_START` | `0.5` | Initial gain (fast convergence) |
| `MADGWICK_BETA_DECAY_S` | `2.0` | Ramp duration (seconds) |
| `MADGWICK_ZETA` | `0.015` | Gyro bias tracking gain |
| `MADGWICK_BETA_MOTION_K` | `10.0` | Motion-adaptive scaling factor |
| `MADGWICK_BETA_MIN` | `0.01` | Minimum effective β (floor) |
| `MADGWICK_ACCEL_MIN_G` | `0.85` | Hard-reject lower bound (g) |
| `MADGWICK_ACCEL_MAX_G` | `1.15` | Hard-reject upper bound (g) |

### EKF Parameters

| Macro | Value | Description |
|---|---|---|
| `EKF_SIGMA_GYRO` | `0.01` | Gyro noise density (rad/s/√Hz) |
| `EKF_SIGMA_BIAS` | `2e-5` | Bias random-walk (rad/s²/√Hz) |
| `EKF_SIGMA_ACCEL` | `0.05` | Accel noise density (g/√Hz) |
| `EKF_R_ADAPT_K` | `200.0` | Adaptive-R steepness coefficient |
| `EKF_P0` | `1.0` | Initial P diagonal (higher = faster cold-start convergence) |
