# EKF vs Madgwick Filter Comparison — STM32F411 + MPU6050

## Project Overview

This is a **bare-metal STM32F411CEU6** firmware project that reads 6-axis IMU data (accelerometer + gyroscope) from an **MPU6050** sensor over I2C and estimates orientation (roll, pitch, yaw) using sensor-fusion filters. The project is designed to compare two attitude estimation algorithms side-by-side:

| Filter | Status | Description |
|--------|--------|-------------|
| **Madgwick** | ✅ Implemented | Gradient-descent complementary filter using quaternion representation |
| **EKF** | 🚧 Placeholder | Extended Kalman Filter (headers and source stubs created, not yet implemented) |

The firmware exposes a **UART-based interactive CLI** (command-line interface) at 115200 baud, allowing real-time control, calibration, diagnostics, and data streaming — making it ideal for development, tuning, and live comparison.

---

## Hardware

| Component | Detail |
|-----------|--------|
| **MCU** | STM32F411CEU6 (ARM Cortex-M4F, 84 MHz, FPU) |
| **IMU Sensor** | MPU6050 (InvenSense 6-axis, I2C address 0x68) |
| **I2C Bus** | I2C1 — PB8 (SCL) / PB9 (SDA), 400 kHz Fast-Mode |
| **UART** | USART2, 115200 baud, 8N1 — used for CLI and data output |
| **Crystal** | 25 MHz HSE, PLL → 84 MHz SYSCLK |
| **Timer** | TIM2 — generates a precise 100 Hz interrupt for IMU sampling |

### Clock Tree

```
HSE (25 MHz) → PLL (PLLM=25, PLLN=336, PLLP=4) → SYSCLK = 84 MHz
  ├── AHB  = 84 MHz (÷1)
  ├── APB1 = 42 MHz (÷2)  ← I2C1, TIM2
  └── APB2 = 84 MHz (÷1)  ← USART2
```

### TIM2 Configuration (100 Hz Tick)

```
Timer clock = 84 MHz
Prescaler   = 8399  → tick = 84 MHz / 8400 = 10 kHz
Period      = 99    → overflow = 10 kHz / 100 = 100 Hz
```

---

## Software Architecture

### Layer Diagram

```
┌──────────────────────────────────────────────────────────────┐
│                        main.c (super-loop)                   │
│    ┌────────────────┐   ┌──────────────────┐                 │
│    │  uart_cli_poll │   │  imu_app_poll    │   (cooperative) │
│    └───────┬────────┘   └────────┬─────────┘                 │
│            │                     │                            │
├────────────┼─────────────────────┼────────────────────────────┤
│  APP LAYER │                     │                            │
│   cli_app.c│  ◄── parses CLI ──►│  imu_app.c                 │
│            │      commands       │  (sensor read, axis remap, │
│            │                     │   filter update, stats)    │
├────────────┼─────────────────────┼────────────────────────────┤
│  FILTERS   │                     │                            │
│            │               madgwick.c   ekf.c (stub)          │
├────────────┼─────────────────────┼────────────────────────────┤
│  DRIVERS   │                     │                            │
│   uart_cli.c  ──  ringbuf.c     │  mpu6050.c  ── i2c_reg.c  │
├────────────┼─────────────────────┼────────────────────────────┤
│  UTILS     │                     │                            │
│          timebase.c       math3d.c       ringbuf.c            │
├───────────────────────────────────────────────────────────────┤
│  HAL       │  STM32F4xx HAL (I2C, UART, TIM, GPIO, RCC)      │
└───────────────────────────────────────────────────────────────┘
```

### Execution Flow

1. **Boot** — `main()` initializes HAL, clocks, GPIO, I2C1, TIM2, USART2.
2. **Init** — `uart_cli_init()` arms UART RX interrupt; `timebase_init()` enables DWT cycle counter; `imu_app_init()` initializes Madgwick filter state.
3. **Super-loop** — `while(1)` alternates between:
   - `uart_cli_poll()` — drains the RX ring buffer, assembles lines, dispatches to `app_cli_handle_line()`.
   - `imu_app_poll()` — if `s_tick_due` flag is set (by TIM2 ISR), reads MPU6050, converts units, remaps axes, runs the Madgwick filter, updates stats.
4. **TIM2 ISR** — fires at 100 Hz, calls `imu_app_on_100hz_tick()` which sets `s_tick_due = true`. If the previous tick wasn't serviced yet, it increments a missed-tick counter.
5. **UART RX ISR** — `HAL_UART_RxCpltCallback()` pushes each received byte into a ring buffer via `uart_cli_on_rx_byte()`.

---

## Directory Structure

```
ekf_madgwick_comparsion/
├── Core/
│   ├── Inc/                        # All header files
│   │   ├── main.h
│   │   ├── stm32f4xx_hal_conf.h
│   │   ├── stm32f4xx_it.h
│   │   ├── app/                    # Application-layer headers
│   │   │   ├── app_config.h        # Compile-time settings (sample rate, filter enables, beta, etc.)
│   │   │   ├── app_imu_task.h      # (stub — not used yet)
│   │   │   ├── cli_app.h           # CLI command handler interface
│   │   │   ├── imu_app.h           # IMU application API (init, poll, stats, Madgwick control, cal)
│   │   │   └── imu_types.h         # Shared data types: ImuRaw_t, ImuData_t, Attitude_t, FusionOut_t
│   │   ├── drivers/                # Hardware driver headers
│   │   │   ├── i2c_reg.h           # I2C register read/write/scan utilities
│   │   │   ├── mpu6050.h           # MPU6050 driver (whoami, init, read_raw, read_cfg)
│   │   │   ├── uart_cli.h          # UART CLI TX/RX + polling interface
│   │   │   └── uart_logger.h       # (stub — reserved for future structured logging)
│   │   ├── filters/                # Filter algorithm headers
│   │   │   ├── ekf.h               # (stub — EKF not yet implemented)
│   │   │   └── madgwick.h          # Madgwick filter types + API
│   │   └── utils/                  # Utility headers
│   │       ├── math3d.h            # sqrt, inv_sqrt, quaternion normalize, quat→euler
│   │       ├── ringbuf.h           # Lock-free single-producer/single-consumer ring buffer
│   │       └── timebase.h          # DWT cycle counter (microsecond-resolution timestamps)
│   ├── Src/                        # All source files (mirrors Inc/ structure)
│   │   ├── main.c                  # Entry point, peripheral init, super-loop, ISR callbacks
│   │   ├── app/
│   │   │   ├── app_imu_task.c      # (stub)
│   │   │   ├── cli_app.c           # CLI command parser and dispatcher (~450 lines)
│   │   │   └── imu_app.c           # Core IMU pipeline (read→convert→remap→filter→stats, ~410 lines)
│   │   ├── drivers/
│   │   │   ├── i2c_reg.c           # I2C HAL wrappers (scan, read_reg, write_reg)
│   │   │   ├── mpu6050.c           # MPU6050 initialization (100 Hz, ±2g, ±250 dps) + burst read
│   │   │   ├── uart_cli.c          # UART TX (blocking), RX (interrupt→ringbuf→line assembly)
│   │   │   └── uart_logger.c       # (stub)
│   │   ├── filters/
│   │   │   ├── ekf.c               # (stub)
│   │   │   └── madgwick.c          # Madgwick gradient-descent IMU filter (~150 lines)
│   │   └── utils/
│   │       ├── math3d.c            # Math helpers (sqrtf wrappers, quat normalize, quat→euler)
│   │       ├── ringbuf.c           # Ring buffer implementation
│   │       └── timebase.c          # DWT CYCCNT init + cycle→microsecond conversion
│   └── Startup/
│       └── startup_stm32f411ceux.s # ARM vector table + reset handler (assembly)
├── Drivers/                        # ST-provided libraries
│   ├── CMSIS/                      # ARM CMSIS core headers
│   └── STM32F4xx_HAL_Driver/       # STM32F4 HAL source and headers
├── Debug/                          # Build output (makefile, .o, .list, .map)
├── docs/                           # ← You are here — project documentation
├── ekf_madgwick_comparsion.ioc     # STM32CubeMX project file
├── STM32F411CEUX_FLASH.ld          # Linker script (flash)
└── STM32F411CEUX_RAM.ld            # Linker script (RAM)
```

---

## Module Reference

Detailed documentation for each module is available in:

- [Module Reference](modules.md) — per-file API descriptions
- [CLI Command Reference](cli_reference.md) — all supported serial commands
- [Madgwick Filter](madgwick_filter.md) — algorithm details and tuning guide
- [Data Flow & Timing](dataflow.md) — signal chain from sensor to output
