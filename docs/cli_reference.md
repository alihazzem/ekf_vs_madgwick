# CLI Command Reference

All commands are **case-insensitive**. The CLI runs over **USART2 at 115200 baud (8N1)**. Type commands and press Enter.

**Emergency stop:** Press `!` at any time to immediately stop IMU streaming and raw printing.

---

## General Commands

| Command | Response | Description |
|---------|----------|-------------|
| `HELP` | Command list | Print all available commands |
| `PING` | `PONG` | Connectivity check |
| `STATUS` | System info | Show uptime, UART config, I2C config, streaming state, print divider |

---

## I2C Commands

### `I2C SCAN`

Scan the I2C1 bus from address 0x03 to 0x77. Reports all responding device addresses.

**Example:**
```
> I2C SCAN
found=1
0x68
```

### `I2C R <addr> <reg> [len]`

Read `len` bytes (default 1) starting at register `reg` from device at `addr`.

- `addr` and `reg` accept decimal, hex with `0x` prefix, or bare hex digits.
- Maximum length: 32 bytes.

**Example:**
```
> I2C R 68 75
0x75: 68
```

### `I2C W <addr> <reg> <val>`

Write a single byte `val` to register `reg` at device `addr`.

**Example:**
```
> I2C W 68 6B 01
ok
```

---

## MPU6050 Commands

### `MPU WHOAMI`

Read the WHO_AM_I register (0x75). Expected value for MPU6050: `0x68`.

```
> MPU WHOAMI
MPU addr=0x68 WHO_AM_I=0x68
```

### `MPU INIT`

Perform full MPU6050 initialization for 100 Hz operation:
- Wake device (PLL with Gyro X reference)
- Set sample rate divider = 9 → 100 Hz
- DLPF bandwidth ~44 Hz
- Gyro ±250 °/s, Accel ±2 g

```
> MPU INIT
mpu init ok
```

### `MPU CFG`

Read and display current MPU6050 register configuration.

```
> MPU CFG
WHOAMI=0x68 PWR=0x01 DIV=9 CFG=0x03 GYRO=0x00 ACC=0x00
```

### `MPU READ`

Perform a single burst-read of all 7 sensor values (accel XYZ, temp, gyro XYZ) and display as raw integers.

```
> MPU READ
ax=124 ay=-312 az=16412 temp=2340 gx=-5 gy=12 gz=-2
```

### `MPU STREAM ON` / `MPU STREAM OFF`

Enable or disable the 100 Hz continuous IMU sampling loop. When enabled, TIM2 triggers periodic reads and the Madgwick filter runs on each sample.

```
> MPU STREAM ON
stream on
> MPU STREAM OFF
stream off
```

### `MPU PRINT <N>`

Set the raw-data print divider. When streaming is on, raw sensor values are printed every N samples. Set to 0 to disable printing (filter still runs silently).

```
> MPU PRINT 10
ok
```
This prints raw data every 10th sample (i.e., at 10 Hz if sampling at 100 Hz).

### `MPU RATE`

Display the measured sample rate (averaged over a 1-second window). Shown as integer + 3 decimal places to avoid float printf.

```
> MPU RATE
rate_hz=100.000
```

### `MPU STATS`

Display detailed timing and performance statistics:
- Elapsed time since last reset
- Total ticks, samples, and missed ticks
- Measured rate (Hz)
- Inter-sample dt statistics (min / avg / max, in microseconds)
- Service time per sample (last / max, in microseconds)
- Miss rate in parts-per-million (ppm)

```
> MPU STATS
elapsed_ms=5000 stream=1 tick_due=0 print_div=0
ticks=500 samples=500 missed=0
rate_hz=100.00 dt_us(min/avg/max)=9980/10000/10020
svc_us(last/max)=650/720 last_miss_tick=0
miss_ppm=0
```

### `MPU STATS RESET`

Reset all statistics counters and timing accumulators.

```
> MPU STATS RESET
ok
```

---

## Gyro Calibration Commands

### `MPU CAL GYRO <ms>`

Perform a **blocking** gyro calibration. The sensor must be **completely still** during this period. Reads gyro samples rapidly for `<ms>` milliseconds and averages them to compute raw LSB offsets.

Minimum duration: 200 ms. Recommended: 2000–5000 ms.

```
> MPU CAL GYRO 3000
Calibrating gyro... keep still
done. offsets=(-12 5 -3)
```

These offsets are subtracted from all subsequent gyro readings before unit conversion.

### `MPU CAL SHOW`

Display the current gyro calibration offsets (raw LSB).

```
> MPU CAL SHOW
gyro_off_raw=(-12 5 -3)
```

### `MPU CAL CLEAR`

Zero out the gyro offsets (revert to uncalibrated).

```
> MPU CAL CLEAR
gyro offsets cleared
```

---

## Madgwick Filter Commands

### `MAD SHOW`

Display the current Madgwick filter output:
- Quaternion (scaled ×10⁶ to avoid float printf)
- Roll, pitch, yaw in milli-degrees

```
> MAD SHOW
q_1e6=(999998 -1234 567 -89)
rpy_mdeg=(-1412 648 -102)
```

To convert: divide q values by 1,000,000 and rpy values by 1,000 to get degrees.

### `MAD BETA <value>`

Change the Madgwick filter gain parameter at runtime. Accepts float values. Valid range: (0, 5].

- **Lower beta** (e.g., 0.01): trusts gyro more, slower convergence, smoother
- **Higher beta** (e.g., 0.5): trusts accel more, faster convergence, noisier

```
> MAD BETA 0.04
ok beta=0.0400
```

### `MAD RESET`

Reset the Madgwick quaternion to identity `[1, 0, 0, 0]`. Useful when changing orientation or after calibration.

```
> MAD RESET
ok
```

---

## Typical Workflow

```
1.  PING                    # verify UART connection
2.  I2C SCAN                # confirm MPU6050 is detected at 0x68
3.  MPU INIT                # configure sensor for 100 Hz
4.  MPU CFG                 # verify configuration
5.  MPU READ                # single-shot read to confirm data
6.  MPU CAL GYRO 3000       # calibrate gyro (keep sensor still!)
7.  MPU CAL SHOW            # verify offsets
8.  MAD BETA 0.08           # set desired Madgwick gain
9.  MPU STREAM ON           # start 100 Hz sampling + filtering
10. MAD SHOW                # check attitude estimate
11. MPU RATE                # verify actual sample rate
12. MPU STATS               # check timing health
13. !                       # emergency stop streaming
14. MPU STREAM OFF          # or graceful stop
```
