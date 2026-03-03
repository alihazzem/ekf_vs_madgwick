# Data Flow & Timing

This document traces the complete signal path from sensor hardware to attitude output, and explains all timing relationships.

---

## Signal Chain

```
 ┌─────────────┐
 │  MPU6050     │  Physical sensor on I2C1 bus
 │  (6-axis)    │  Accel ±2g, Gyro ±250°/s, 100 Hz internal rate
 └──────┬───────┘
        │  I2C burst read (14 bytes, ~400 kHz)
        ▼
 ┌─────────────────────────────────────────────────────────────┐
 │  mpu6050_read_raw()                                         │
 │  → big-endian decode → mpu6050_raw_t (ax,ay,az,gx,gy,gz)  │
 └──────┬──────────────────────────────────────────────────────┘
        │
        ▼
 ┌─────────────────────────────────────────────────────────────┐
 │  Unit Conversion (imu_app_poll)                             │
 │  ┌───────────────────────────────┐                          │
 │  │ Accel: raw / 16384 → g       │                          │
 │  │ Gyro:  (raw − offset) / 131  │                          │
 │  │        × π/180 → rad/s       │                          │
 │  └───────────────────────────────┘                          │
 └──────┬──────────────────────────────────────────────────────┘
        │
        ▼
 ┌─────────────────────────────────────────────────────────────┐
 │  Axis Remap (sensor frame → body frame)                     │
 │  ┌───────────────────────────────┐                          │
 │  │ Permutation:                  │                          │
 │  │   X_body = −Z_sensor          │                          │
 │  │   Y_body = −Y_sensor          │  (then X↔Y swap)        │
 │  │   Z_body =  X_sensor          │                          │
 │  └───────────────────────────────┘                          │
 │  Applied identically to both accel and gyro                 │
 └──────┬──────────────────────────────────────────────────────┘
        │
        ▼
 ┌─────────────────────────────────────────────────────────────┐
 │  Madgwick Filter (madgwick_update_imu)                      │
 │  ┌───────────────────────────────┐                          │
 │  │ Inputs:  wx,wy,wz (rad/s)    │                          │
 │  │          ax,ay,az (g)         │                          │
 │  │          dt (measured, sec)   │                          │
 │  │                               │                          │
 │  │ Steps:                        │                          │
 │  │  1. Accel norm check          │                          │
 │  │  2. Normalize accel           │                          │
 │  │  3. Compute gradient s[]      │                          │
 │  │  4. Normalize gradient        │                          │
 │  │  5. qDot = gyro − β·s        │                          │
 │  │  6. q += qDot · dt            │                          │
 │  │  7. Normalize quaternion      │                          │
 │  └───────────────────────────────┘                          │
 └──────┬──────────────────────────────────────────────────────┘
        │
        ▼
 ┌─────────────────────────────────────────────────────────────┐
 │  Euler Conversion (math3d_quat_to_euler_deg)                │
 │  → roll_deg, pitch_deg, yaw_deg  (for display/logging)     │
 └──────┬──────────────────────────────────────────────────────┘
        │
        ▼
 ┌─────────────────────────────────────────────────────────────┐
 │  Output Storage (s_mad_att)                                 │
 │  → queried by "MAD SHOW" CLI command or external consumer  │
 └─────────────────────────────────────────────────────────────┘
```

---

## Timing Architecture

### Clock Sources

| Clock | Source | Resolution | Used For |
|-------|--------|------------|----------|
| **SysTick** | 84 MHz / 1000 | 1 ms | `HAL_GetTick()`, delays, uptime |
| **TIM2** | 84 MHz, PSC=8399, ARR=99 | 10 ms (100 Hz) | IMU sample trigger |
| **DWT CYCCNT** | 84 MHz | ~12 ns | Precise dt measurement, service time |

### Sample Trigger Flow

```
  TIM2 overflow (100 Hz)
      │
      ▼
  HAL_TIM_PeriodElapsedCallback()
      │
      ▼
  imu_app_on_100hz_tick()
      ├── s_tick_count++
      ├── if (!s_stream_en) → return
      ├── if (s_tick_due already set) → s_missed++, return  [overrun!]
      └── s_tick_due = true
      
  ─── interrupt returns ───

  main loop (cooperative)
      │
      ▼
  imu_app_poll()
      ├── if (!s_stream_en || !s_tick_due || !hi2c) → return
      ├── s_tick_due = false       ← consume the tick
      ├── now_cyc = DWT->CYCCNT   ← timestamp
      ├── dt = (now_cyc − prev_cyc) / 84 → microseconds → seconds
      ├── mpu6050_read_raw()       ← I2C burst (~300 µs at 400 kHz)
      ├── convert + remap          ← ~few µs (all float, FPU)
      ├── madgwick_update_imu()    ← ~10-30 µs (quaternion math, FPU)
      ├── euler conversion         ← ~5 µs
      └── update stats (dt, rate window, service time)
```

### Timing Budget (approximate at 84 MHz)

| Phase | Duration | Notes |
|-------|----------|-------|
| I2C burst read (14 bytes @ 400 kHz) | ~300 µs | Dominates the cycle |
| Unit conversion + axis remap | ~2 µs | Simple float divides, FPU |
| Madgwick filter update | ~15–30 µs | Quaternion ops, 1 sqrt, FPU |
| Quat → Euler | ~5 µs | atan2, asin |
| Stats bookkeeping | ~1 µs | Integer ops |
| **Total service time** | **~350 µs** | Well within 10 ms budget |
| Available headroom | ~9,650 µs | For EKF, logging, etc. |

### dt Measurement

Instead of assuming a constant dt = 10 ms, the firmware measures the **actual** elapsed time between consecutive successful samples using DWT CYCCNT:

```c
uint32_t now_cyc  = timebase_cycles();      // current cycle count
uint32_t dt_cyc   = now_cyc - prev_cyc;     // wrap-safe subtraction
uint32_t dt_us    = dt_cyc / 84;            // cycles → microseconds
float    dt_s     = (float)dt_us * 1e-6f;   // → seconds (passed to filter)
```

This correctly handles jitter from ISR latency, I2C bus contention, or interrupted main-loop polling.

### Rate Measurement

The actual sample rate is measured using a 1-second sliding window:

1. At each sample, increment `s_win_samples`
2. When the window elapsed time ≥ 1,000,000 µs:
   - Compute `rate_mhz = (samples × 10⁹) / elapsed_us` (milli-Hz)
   - Reset window counters

Stored as integer milli-Hz to avoid float-printf overhead on the CLI.

---

## Missed Tick Detection

If the main loop doesn't service `s_tick_due` before the next TIM2 interrupt:

1. `imu_app_on_100hz_tick()` sees `s_tick_due` is still `true`
2. Increments `s_missed` counter
3. Records `s_last_miss_tick` for debugging
4. Does **not** double-set the flag (so only one sample fires when poll runs)

Missed ticks indicate either:
- Main loop is blocked (e.g., CLI UART TX taking too long)
- I2C read is stuck (sensor issue, bus error)
- Other ISR with higher priority delaying execution

The `MPU STATS` command reports `missed` count and `miss_ppm` (parts per million of ticks missed).

---

## Data Protection (ISR ↔ Main Loop)

The firmware uses **volatile flags** and brief **`__disable_irq()` / `__enable_irq()` critical sections** to avoid data races:

| Data | Writer | Reader | Protection |
|------|--------|--------|------------|
| `s_tick_due` | TIM2 ISR | main loop (poll) | `volatile bool` — single-byte atomic on Cortex-M4 |
| `s_stream_en` | CLI (main loop) | TIM2 ISR | `volatile bool` — single-byte atomic |
| `s_mad_att` | imu_app_poll() | CLI MAD SHOW | `__disable_irq()` during copy (prevents tearing) |
| UART RX bytes | UART ISR | main loop (poll) | Lock-free ring buffer (single-producer/single-consumer) |
| Stats snapshot | imu_app_poll() | CLI STATS | `__disable_irq()` during snapshot |

---

## Future: EKF Integration Point

When the EKF is implemented, it will receive the **same** converted and remapped sensor data as the Madgwick filter. The integration point in `imu_app_poll()` is:

```c
#if RUN_EKF
    if (dt_s > 0.0f) {
        ekf_update(&s_ekf, wx, wy, wz, ax_g, ay_g, az_g, dt_s);
        // copy to s_ekf_att ...
    }
#endif
```

Both filters will run on every sample if both `RUN_MADGWICK` and `RUN_EKF` are set to 1, enabling direct comparison with identical inputs.
