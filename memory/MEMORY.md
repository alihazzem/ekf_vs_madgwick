# Project Memory — ekf_madgwick_comparsion

## Project Goal
EMG–IMU sensor fusion system: combine Myo armband 8-channel surface EMG with MPU6050 orientation (from STM32F411) for context-aware arm motion interpretation.

---

## Hardware
| Component | Role |
|-----------|------|
| STM32F411CEU6 (Black Pill) | Runs Madgwick + EKF at 100 Hz, streams via USART2 |
| MPU6050 | IMU on STM32 via I2C1 (PB8/PB9) |
| Myo Armband | 8-channel surface EMG at ~200 Hz via BLE |
| Myo USB Dongle | BLE bridge to PC |
| PC | Acquisition, logging, visualization |

USART2: PA2 TX / PA3 RX, 115200 baud, 8N1. SDK DLL: `scripts/myo64.dll`.

---

## What Is Done and Verified Working

### STM32 Firmware (`Core/`)
- Madgwick filter + 7-state EKF running at 100 Hz
- CLI over USART2: `MPU INIT`, `MPU CAL GYRO`, `MPU STREAM ON/OFF`, `MPU PRINT <div>`
- Data lines prefixed `D,` with 19 fields (raw IMU + Madgwick angles + EKF angles + bias)

### Python Scripts (`scripts/`)
- `capture_ekf.py` — captures IMU stream to CSV (existing, working)
- `capture_mad.py` — Madgwick-only capture (existing, working)
- `capture_emg.py` — captures raw Myo 8-ch EMG to CSV (~200 Hz, verified with armband)
- `process_emg.py` — offline: DC removal, causal moving-RMS envelope, MAV/RMS/ZC/WL features
- `capture_fusion.py` — **captures EMG + IMU simultaneously**, shared `pc_t_ms` clock, writes two CSVs

### MATLAB Scripts (`matlab/`)
- `plot_emg.m` — 8-channel raw EMG visualization (verified working)
- `plot_emg_features.m` — 4-panel MAV/RMS/ZC/WL + per-channel bar chart (verified working)
- `plot_comparison.m`, `plot_ekf.m`, `plot_mad.m` — IMU filter plots (existing, working)

### Data (`emg_data/`)
- `at_rest.csv` — rest-state EMG capture
- `emg_capture_test.csv` — test capture
- `emg_capture_test_envelope.csv` — envelope output from process_emg.py
- `emg_capture_test_features.csv` — feature output from process_emg.py

---

## Current Position in Project

**EMG acquisition and offline processing are complete and verified.**

**Next immediate step: test `capture_fusion.py`.**
- Run it, do a 30–60 s capture with fist clenches + wrist movement
- Confirm both streams land in `emg_data/fusion_emg_*.csv` and `fusion_imu_*.csv`
- Verify `pc_t_ms` is monotonic and rates are correct (~200 Hz EMG, ~100 Hz IMU)
- Confirm no BLE/serial threading crashes

Do NOT write `plot_fusion.m` or any downstream code until `capture_fusion.py` is verified working.

---

## Planned Next Stages (after fusion capture verified)
1. `plot_fusion.m` — MATLAB offline fusion analysis (align streams, overlay EMG envelope with IMU angles)
2. PC-to-STM32 UART bridge (`E,` packet protocol, Section 15 of acquisition doc)
3. STM32 firmware additions: `emg_parse_line()`, EMG ring buffer, `E,` detection before CLI dispatch
4. On-device EMG + IMU fusion

---

## Key File Paths
| File | Purpose |
|------|---------|
| `scripts/capture_fusion.py` | Simultaneous EMG+IMU capture |
| `scripts/process_emg.py` | Offline EMG feature extraction |
| `scripts/capture_emg.py` | Standalone EMG capture |
| `matlab/plot_emg_features.m` | Feature visualization |
| `docs/myo_emg_acquisition_module.md` | Full acquisition module spec + UART bridge protocol |
| `Core/Src/app/cli_app.c` | STM32 CLI handler (future: add `E,` detection) |
| `Core/Src/app/imu_app.c` | STM32 IMU + filter loop |

---

## EMG CSV Formats
- Raw: `t_ms, emg0..emg7`
- Envelope: `t_ms, env0..env7`
- Features: `t_ms, mav0..mav7, rms0..rms7, zc0..zc7, wl0..wl7`
- Fusion EMG: `pc_t_ms, emg_t_ms, emg0..emg7`
- Fusion IMU: `pc_t_ms, t_ms, ax_raw..gz_raw, mad_roll..mad_us, ekf_roll..ekf_us, bx..bz_uradps`

## UART Bridge Protocol (specified, not yet implemented)
Packet: `E,<t_ms>,<emg0>..<emg7>\n`
Bandwidth note: 200 Hz EMG stream uses ~52% of 115200 baud — needs baud increase or rate reduction.
