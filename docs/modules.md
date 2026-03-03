# Module Reference

This document describes every source module in the project, its purpose, and its public API.

---

## Application Layer (`Core/Src/app/`)

### `imu_app.c` / `imu_app.h`

The **central application module** that orchestrates the entire IMU pipeline.

**Responsibilities:**
- Reads raw data from the MPU6050 on each 100 Hz tick
- Converts raw sensor data to physical units (g for accel, rad/s for gyro)
- Applies gyro bias calibration offsets
- Remaps sensor axes to the desired body frame
- Feeds corrected data into the Madgwick filter
- Tracks real-time statistics (sample rate, dt jitter, missed ticks, service time)

**Key API:**

| Function | Description |
|----------|-------------|
| `imu_app_init(hi2c)` | Initialize with I2C handle; sets up Madgwick filter with default beta |
| `imu_app_on_100hz_tick()` | Called from TIM2 ISR; sets `tick_due` flag (or increments missed counter) |
| `imu_app_poll()` | Called from main loop; performs I2C read + filter update if tick is due |
| `imu_app_stream_set(en)` | Enable/disable IMU streaming |
| `imu_app_stream_get()` | Query streaming state |
| `imu_app_set_print_div(n)` | Print raw data every N samples (0 = off) |
| `imu_app_stats_reset()` | Clear all counters and timing stats |
| `imu_app_get_stats(out)` | Snapshot all stats into `imu_stats_t` struct |
| `imu_app_get_rate_hz()` | Get measured sample rate as float (Hz) |
| `imu_app_get_rate_mhz()` | Get measured sample rate in milli-Hz (integer, avoids float printf) |
| `imu_app_get_madgwick(out)` | Copy latest Madgwick attitude (quaternion + Euler) to caller |
| `imu_app_madgwick_reset()` | Reset quaternion to identity `[1, 0, 0, 0]` |
| `imu_app_madgwick_set_beta(beta)` | Change Madgwick gain at runtime |
| `imu_app_madgwick_get_beta()` | Read current beta |
| `imu_app_cal_gyro(duration_ms)` | Blocking gyro calibration (averages raw gyro while stationary) |
| `imu_app_cal_get(gx, gy, gz)` | Read current gyro offsets (raw LSB) |
| `imu_app_cal_clear()` | Zero out gyro offsets |

**Statistics tracked (`imu_stats_t`):**

| Field | Description |
|-------|-------------|
| `ticks` | Total TIM2 ticks since reset |
| `samples` | Successful MPU6050 reads |
| `missed` | Ticks where previous sample wasn't processed in time |
| `dt_min_us / dt_avg_us / dt_max_us` | Inter-sample timing (microseconds) |
| `svc_last_us / svc_max_us` | Service time per sample (I2C read + filter) |
| `rate_hz` | Measured sample rate over a 1-second sliding window |

**Axis Remap Logic:**

The sensor-frame to body-frame mapping applies a specific permutation and sign flips derived from a 3-pose calibration procedure:
```
Body X = −(sensor Z)  → then swapped with Y
Body Y = −(sensor Y)  → then swapped with X
Body Z =  (sensor X)
```
The same transform is applied to both accelerometer and gyroscope channels.

---

### `cli_app.c` / `cli_app.h`

A **text-based command parser** for the UART CLI. Receives complete lines from `uart_cli.c`, tokenizes them, and dispatches to the appropriate handler.

**Key API:**

| Function | Description |
|----------|-------------|
| `app_cli_print_banner()` | Print startup banner with project name |
| `app_cli_handle_line(line)` | Parse and dispatch a single CLI command string |

See [CLI Command Reference](cli_reference.md) for the full command list.

**Parser design:**
- Input is trimmed, uppercased, and tokenized into up to 8 space-separated tokens
- Numeric arguments support decimal, hex with `0x` prefix, or bare hex digits
- All commands are case-insensitive

---

### `app_imu_task.c` / `app_imu_task.h`

Currently a **stub** — reserved for a possible RTOS-based task wrapper if the project moves from bare-metal super-loop to FreeRTOS.

---

## Driver Layer (`Core/Src/drivers/`)

### `mpu6050.c` / `mpu6050.h`

**MPU6050 6-axis IMU driver** with a minimal, clean API built on top of `i2c_reg`.

**Configuration applied by `mpu6050_init_100hz()`:**

| Register | Value | Meaning |
|----------|-------|---------|
| `PWR_MGMT_1` | `0x01` | Wake up, clock source = PLL with Gyro X |
| `SMPLRT_DIV` | `9` | Sample rate = 1000/(1+9) = **100 Hz** |
| `CONFIG` | `0x03` | DLPF bandwidth ~44 Hz (accel) / ~42 Hz (gyro) |
| `GYRO_CONFIG` | `0x00` | Full-scale = **±250 °/s** (sensitivity: 131 LSB/°/s) |
| `ACCEL_CONFIG` | `0x00` | Full-scale = **±2 g** (sensitivity: 16384 LSB/g) |

**Key API:**

| Function | Description |
|----------|-------------|
| `mpu6050_whoami(hi2c, addr7, &id)` | Read WHO_AM_I register (expect 0x68) |
| `mpu6050_init_100hz(hi2c, addr7, &cfg)` | Full initialization + readback |
| `mpu6050_read_cfg(hi2c, addr7, &cfg)` | Read current register configuration |
| `mpu6050_read_raw(hi2c, addr7, &raw)` | Burst-read 14 bytes (accel + temp + gyro) |

**Data struct (`mpu6050_raw_t`):** `ax, ay, az, temp, gx, gy, gz` — all `int16_t`, big-endian decoded.

---

### `i2c_reg.c` / `i2c_reg.h`

Thin wrapper around STM32 HAL I2C memory-mapped read/write operations.

| Function | Description |
|----------|-------------|
| `i2c_scan(hi2c, found[], max, &count)` | Scan bus 0x03–0x77, return list of responding addresses |
| `i2c_read_reg(hi2c, addr7, reg, buf, len, timeout)` | Read `len` bytes starting at `reg` |
| `i2c_write_reg(hi2c, addr7, reg, val, timeout)` | Write single byte `val` to `reg` |

All functions return `i2c_reg_status_t` (`OK`, `TIMEOUT`, `ERROR`).

---

### `uart_cli.c` / `uart_cli.h`

**Interrupt-driven UART receive + blocking transmit** implementing a line-based CLI.

**Architecture:**
1. `HAL_UART_RxCpltCallback` → `uart_cli_on_rx_byte()` pushes byte into ring buffer, re-arms RX interrupt
2. `uart_cli_poll()` (called from main loop) drains ring buffer one byte at a time, assembles a line
3. On `\n`, the assembled line is dispatched to `app_cli_handle_line()`
4. Pressing `!` at any time immediately stops IMU streaming (emergency stop)
5. Backspace (`0x7F` / `0x08`) is handled for interactive editing

**TX functions:** `uart_cli_send(str)` and `uart_cli_sendf(fmt, ...)` use blocking `HAL_UART_Transmit`.

**Buffer sizes:** RX ring buffer = 512 bytes, line buffer = 128 chars.

---

### `uart_logger.c` / `uart_logger.h`

**Stub** — reserved for structured/timestamped logging (e.g., CSV output on a separate UART or SWO trace).

---

## Filter Layer (`Core/Src/filters/`)

### `madgwick.c` / `madgwick.h`

Implementation of **Sebastian Madgwick's gradient-descent orientation filter** (IMU-only variant, no magnetometer).

See [Madgwick Filter Details](madgwick_filter.md) for the full algorithm writeup.

**Key API:**

| Function | Description |
|----------|-------------|
| `madgwick_init(m, beta)` | Set quaternion to identity, store gain parameter |
| `madgwick_reset(m)` | Reset quaternion to `[1, 0, 0, 0]` |
| `madgwick_set_beta(m, beta)` | Change filter gain |
| `madgwick_get_beta(m)` | Read current gain |
| `madgwick_set_accel_reject(m, en, min_g, max_g)` | Configure accelerometer magnitude rejection |
| `madgwick_update_imu(m, wx, wy, wz, ax, ay, az, dt)` | Run one filter step (gyro in rad/s, accel in g) |

**State (`madgwick_t`):** quaternion `(q0, q1, q2, q3)`, `beta`, accel rejection settings.

---

### `ekf.c` / `ekf.h`

**Stub** — placeholder for Extended Kalman Filter implementation. Both files contain only the include guard.

---

## Utility Layer (`Core/Src/utils/`)

### `math3d.c` / `math3d.h`

Small math helper library for 3D orientation work.

| Function | Description |
|----------|-------------|
| `math3d_sqrtf(x)` | Wrapper around `sqrtf()` |
| `math3d_inv_sqrtf(x)` | Safe `1/sqrt(x)` with epsilon guard (returns 0 for near-zero input) |
| `math3d_quat_normalize(q0..q3)` | In-place quaternion normalization |
| `math3d_quat_to_euler_deg(q0..q3, &roll, &pitch, &yaw)` | Quaternion → Euler angles (degrees), with gimbal-lock-safe asin clamping |

**Euler convention:** ZYX intrinsic (aerospace convention): yaw → pitch → roll.

---

### `timebase.c` / `timebase.h`

High-resolution timestamp utility using the ARM Cortex-M4 **DWT cycle counter** (CYCCNT).

| Function | Description |
|----------|-------------|
| `timebase_init()` | Enable DWT CYCCNT (set `TRCENA` + `CYCCNTENA` bits) |
| `timebase_cycles()` | Read raw 32-bit cycle counter (wraps every ~51 seconds at 84 MHz) |
| `timebase_cycles_to_us(cyc)` | Convert cycle delta to microseconds (divides by 84) |

Used by `imu_app.c` to measure true inter-sample `dt` and service time.

---

### `ringbuf.c` / `ringbuf.h`

**Lock-free single-producer / single-consumer ring buffer** for byte streams.

| Function | Description |
|----------|-------------|
| `rb_init(rb, storage, size)` | Initialize with caller-provided storage array |
| `rb_push(rb, byte)` | Enqueue one byte (returns -1 if full) |
| `rb_pop(rb, &byte)` | Dequeue one byte (returns -1 if empty) |

Used by `uart_cli.c` to pass bytes from the UART RX ISR to the main-loop line parser without a critical section.

---

## Shared Data Types (`Core/Inc/app/imu_types.h`)

| Type | Fields | Purpose |
|------|--------|---------|
| `ImuRaw_t` | `ax, ay, az, gx, gy, gz, temp` (int16) | Raw sensor register values |
| `ImuData_t` | `t_us, dt, ax_g, ay_g, az_g, wx, wy, wz, acc_norm` (float) | Processed sensor data in physical units |
| `Attitude_t` | `q0..q3, roll_deg, pitch_deg, yaw_deg` (float) | Orientation output from a filter |
| `FusionOut_t` | `madgwick, ekf` (Attitude_t), `ekf_acc_used` | Combined output from both filters |

---

## Configuration (`Core/Inc/app/app_config.h`)

Compile-time constants that control the entire system:

| Define | Default | Description |
|--------|---------|-------------|
| `IMU_FS_HZ` | `100.0f` | Target sample rate (Hz) |
| `IMU_DT_S` | `0.01f` | Nominal dt (1/Fs) |
| `RUN_MADGWICK` | `1` | Enable Madgwick filter (preprocessor gate) |
| `RUN_EKF` | `0` | Enable EKF filter (preprocessor gate) |
| `LOG_UART` | `1` | Use UART for logging (vs SWO) |
| `LOG_HEADER_ONCE` | `1` | Print CSV header once at start |
| `MADGWICK_BETA` | `0.08f` | Default Madgwick gain |
| `MPU6050_ADDR_7BIT` | `0x68` | I2C 7-bit address (AD0=0) |
| `MADGWICK_ACCEL_REJECT_EN` | `1` | Enable accel magnitude rejection |
| `MADGWICK_ACCEL_MIN_G` | `0.85f` | Lower bound for accel norm acceptance |
| `MADGWICK_ACCEL_MAX_G` | `1.15f` | Upper bound for accel norm acceptance |
