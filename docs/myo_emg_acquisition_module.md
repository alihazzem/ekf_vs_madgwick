# Myo Armband EMG Data Acquisition Module

## EMG–IMU Sensor Fusion Project
### Signal Processing and Sensor Fusion for Context-Aware Arm Motion Interpretation

---

# 1. Purpose of This Module

This document describes the integration of the Myo Armband with the development PC and the data acquisition pipeline for EMG signals.

The purpose of this stage is to establish a reliable EMG data source that will later be combined with IMU orientation data from the MPU6050 (already running on the STM32F411) for multi-sensor fusion experiments.

This module focuses on:

- establishing BLE communication with the Myo armband via the Myo SDK and myo-python
- acquiring 8-channel EMG data streams at ~200 Hz on the PC
- verifying signal integrity with rest-state and contraction tests
- logging EMG signals to CSV for offline analysis
- preparing the EMG data pipeline for the PC-to-STM32 UART bridge (future stage)

This stage does **not** perform sensor fusion. It only establishes the EMG acquisition pipeline.

---

# 2. Hardware Used

| Component | Purpose |
|-----------|---------|
| Myo Armband | 8-channel EMG sensor device (worn on forearm) |
| Myo USB Dongle | Bluetooth Low Energy communication bridge |
| Development PC | BLE host, EMG acquisition, logging, and visualization |
| STM32F411CEU6 | Existing IMU platform — fusion target in a later stage |
| MPU6050 | Already connected to STM32 via I2C1 (PB8/PB9) |

The Myo armband provides 8 EMG channels positioned circumferentially around the forearm. Only the EMG channels are used — the Myo's built-in IMU (accelerometer, gyroscope, magnetometer) is ignored because the MPU6050 on the STM32 is the primary orientation source.

---

# 3. Myo Armband Specifications

| Parameter | Value |
|-----------|-------|
| EMG channels | 8 (dry surface electrodes) |
| EMG sampling rate | ~200 Hz per channel |
| EMG data type | signed 8-bit integer per channel per sample |
| Communication | Bluetooth Low Energy (BLE) via proprietary GATT profile |
| Onboard IMU | accelerometer + gyroscope + magnetometer (not used) |
| Battery | rechargeable lithium-ion |
| Wake gesture | double-tap on forearm |

Each EMG sample is a vector of 8 values:

```
[ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7]
```

Values range approximately from -128 to +127 (signed 8-bit).

---

# 4. Communication Architecture

```
Myo Armband ──BLE──► Myo USB Dongle ──USB──► PC (myo-python)
                                               │
                                         EMG CSV logging
                                         Signal visualization
                                               │
                                         (future stage)
                                               │
                                         USART2 ──UART──► STM32F411
                                                          (fusion with MPU6050)
```

During this phase the PC is responsible for:

- BLE connection management via Myo Connect + myo-python
- EMG signal acquisition at ~200 Hz
- CSV data logging
- signal visualization with Matplotlib

The STM32 is not involved in this stage. The existing IMU firmware continues to run independently. The UART bridge that forwards EMG data to the STM32 is specified in Section 15 but implemented in a later stage.

---

# 5. Software Dependencies

### PC-Side Software

| Software | Version | Purpose |
|----------|---------|---------|
| Myo Connect | 1.0.1 (last official release) | Background service that manages BLE link to Myo dongle |
| Myo SDK | 0.9.0 | Shared library (`myo.dll` / `myo.framework`) required by myo-python |
| Python | 3.10+ | EMG acquisition scripts |
| myo-python | 1.0.5 | Python bindings for the Myo SDK |
| NumPy | latest | signal processing and array operations |
| Matplotlib | latest | real-time and offline visualization |
| PySerial | latest | reserved for future UART bridge to STM32 |

### Myo Connect and SDK Setup

The Myo SDK is **discontinued** but still functional. The required files are:

1. **Myo Connect** application — must be running in the system tray before any script starts
2. **myo.dll** (Windows) or **libmyo.dylib** (macOS) — must be on the system PATH or in the script's working directory

Verify the SDK DLL is accessible:

```python
import myo
myo.init()   # raises RuntimeError if myo.dll is not found
```

If `myo.init()` fails, set the SDK path explicitly:

```python
myo.init(sdk_path="C:/path/to/myo-sdk/bin")
```

---

# 6. Python Environment Setup

```bash
python --version        # verify Python 3.10+

pip install myo-python  # Myo SDK Python bindings
pip install numpy       # signal processing
pip install matplotlib  # visualization
pip install pyserial    # for future UART bridge stage
```

Verify myo-python installation:

```python
import myo
print(myo.__version__)   # should print version string
```

---

# 7. Myo Device Initialization Procedure

1. Plug the Myo USB dongle into a USB port on the PC.
2. Launch **Myo Connect** — confirm it appears in the system tray.
3. Put on the Myo armband and perform the **double-tap wake gesture**.
4. Wait for the Myo LED to pulse blue (searching) then solid blue (connected to Myo Connect).
5. Run the Python acquisition script.

Expected console output on successful connection:

```
Myo connected
EMG streaming enabled
```

If the armband is not detected within 10 seconds, verify:
- Myo Connect is running
- the USB dongle is inserted
- the armband is charged and awake (blue LED visible)

---

# 8. EMG Data Format

Each callback delivers one EMG frame containing 8 signed integer values:

```
[ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7]
```

| Property | Value |
|----------|-------|
| Channels | 8 (circumferential electrode array) |
| Data type | signed 8-bit integer (-128 to +127) |
| Sampling rate | ~200 Hz (approximately 5 ms between frames) |
| Frequency content | 20–450 Hz (raw surface EMG bandwidth) |
| Signal characteristic | noisy, bipolar, amplitude proportional to muscle contraction force |

The channel ordering maps to physical electrode positions around the forearm. Channel 0 is near the Myo logo indicator.

---

# 9. EMG Acquisition Script

```python
"""
capture_emg.py — Acquire EMG data from the Myo armband and save to CSV.

Usage
-----
  python capture_emg.py                    # default output name
  python capture_emg.py my_emg_test        # custom CSV name -> my_emg_test.csv

Dependencies
------------
  pip install myo-python numpy
  Myo Connect must be running.
"""

import sys
import csv
import time
import myo
from myo import Hub, DeviceListener
from datetime import datetime

CSV_HEADER = [
    "t_ms",
    "emg0", "emg1", "emg2", "emg3",
    "emg4", "emg5", "emg6", "emg7",
]


class EmgListener(DeviceListener):

    def __init__(self, writer, counter):
        super().__init__()
        self.writer = writer
        self.counter = counter
        self.t_start = None

    def on_connected(self, event):
        print("Myo connected")
        event.device.stream_emg(True)
        print("EMG streaming enabled")

    def on_disconnected(self, event):
        print("Myo disconnected")

    def on_emg(self, event):
        if self.t_start is None:
            self.t_start = event.timestamp

        t_ms = int((event.timestamp - self.t_start) / 1000)  # us -> ms
        row = [t_ms] + list(event.emg)
        self.writer.writerow(row)
        self.counter[0] += 1

        if self.counter[0] % 200 == 0:
            print(f"  [{self.counter[0]} samples | "
                  f"{self.counter[0] / 200.0:.1f} s @ ~200 Hz]      ",
                  end="\r", flush=True)


def main():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if len(sys.argv) > 1:
        base_name = sys.argv[1].removesuffix(".csv")
        out_file = f"{base_name}.csv"
    else:
        out_file = f"emg_capture_{timestamp}.csv"

    print(f"Output  : {out_file}")
    print("Waiting for Myo connection ...")
    print("Press Enter to stop.\n")

    f = open(out_file, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(CSV_HEADER)

    counter = [0]

    myo.init()
    hub = Hub()

    listener = EmgListener(writer, counter)

    with hub.run_in_background(listener):
        try:
            input()
        except KeyboardInterrupt:
            pass

    f.close()
    print(f"\nDone. {counter[0]} samples saved to {out_file}")


if __name__ == "__main__":
    main()
```

This script follows the same CSV-logging and progress-reporting conventions as the existing `capture_ekf.py` and `capture_mad.py` scripts but runs as a standalone BLE acquisition tool independent of the serial capture toolchain.

---

# 10. CSV Output Format

Output file: `emg_capture_YYYYMMDD_HHMMSS.csv`

```csv
t_ms,emg0,emg1,emg2,emg3,emg4,emg5,emg6,emg7
0,-12,4,8,-3,6,-1,7,2
5,-11,5,9,-2,5,-2,6,1
10,-14,3,7,-4,7,0,8,3
```

| Column | Description |
|--------|-------------|
| `t_ms` | elapsed time in milliseconds since first sample |
| `emg0` – `emg7` | 8-channel EMG values (signed 8-bit) |

---

# 11. Signal Visualization

### Offline Plotting (after capture)

```python
import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt("emg_capture.csv", delimiter=",", skiprows=1)
t_s = data[:, 0] / 1000.0   # ms -> seconds

fig, axes = plt.subplots(8, 1, figsize=(12, 10), sharex=True)
for ch in range(8):
    axes[ch].plot(t_s, data[:, ch + 1], linewidth=0.5)
    axes[ch].set_ylabel(f"Ch {ch}")
    axes[ch].set_ylim(-128, 127)

axes[-1].set_xlabel("Time (s)")
fig.suptitle("Myo EMG — 8 Channels")
plt.tight_layout()
plt.show()
```

### MATLAB Plotting (optional)

```matlab
data = readmatrix('emg_capture.csv');
t_s = data(:,1) / 1000;

figure;
for ch = 1:8
    subplot(8,1,ch);
    plot(t_s, data(:,ch+1), 'LineWidth', 0.5);
    ylabel(sprintf('Ch %d', ch-1));
    ylim([-128 127]);
end
xlabel('Time (s)');
sgtitle('Myo EMG — 8 Channels');
```

---

# 12. EMG Acquisition Validation

### Test 1 — Rest State

Procedure: wear the armband, relax the forearm completely for 10 seconds.

Expected result: low-amplitude EMG across all channels (values close to zero, within ±10).

### Test 2 — Fist Clench

Procedure: close the fist firmly for 3 seconds, then relax for 3 seconds. Repeat 5 times.

Expected result: clear amplitude increase during contraction (values reaching ±50 to ±127), returning to baseline during relaxation.

### Test 3 — Individual Finger Movement

Procedure: extend and flex each finger individually while keeping others relaxed.

Expected result: different channel activation patterns for different fingers, demonstrating spatial selectivity of the 8-electrode array.

### Test 4 — Repeatability

Procedure: repeat the fist clench test from Test 2 across 3 separate sessions (removing and re-wearing the armband each time).

Expected result: similar activation patterns across sessions, though absolute amplitudes may vary due to electrode placement differences.

### Pass Criteria

| Test | Criterion |
|------|-----------|
| Rest | mean |EMG| < 10 across all channels |
| Contraction | peak |EMG| > 50 on at least 3 channels |
| Repeatability | same channels activate for the same gesture across sessions |

---

# 13. Known Limitations and Mitigations

| Source | Effect | Mitigation |
|--------|--------|------------|
| Electrode placement | amplitude variation between sessions | document armband orientation, use logo as reference |
| Muscle cross-talk | adjacent channels pick up nearby muscle signals | accept as inherent to surface EMG, handle in feature extraction |
| Skin impedance | increased noise, reduced amplitude | ensure clean, dry skin before wearing |
| Motion artifacts | transient spikes unrelated to EMG | apply high-pass filter (>20 Hz) in post-processing |
| BLE latency | occasional 10–50 ms jitter in delivery | timestamp in callback, interpolate if needed |
| Myo SDK discontinued | no future updates or bug fixes | archive SDK DLL with project, pin myo-python version |

---

# 14. File Placement

New files created by this module and where they belong in the existing project structure:

```
ekf_madgwick_comparsion/
├── scripts/
│   ├── capture_emg.py              # NEW — standalone Myo EMG acquisition
│   ├── capture_ekf.py              # existing — IMU dual-filter capture
│   └── capture_mad.py              # existing — IMU Madgwick-only capture
├── matlab/
│   ├── plot_emg.m                  # NEW — 8-channel EMG visualization
│   ├── plot_comparison.m           # existing
│   └── ...
├── docs/
│   └── myo_emg_acquisition_module.md   # this document
└── emg_data/                       # NEW — directory for EMG CSV captures
    ├── emg_capture_YYYYMMDD_HHMMSS.csv
    └── ...
```

---

# 15. PC-to-STM32 UART Bridge Protocol (Future Stage)

This section specifies the protocol for forwarding EMG data from the PC to the STM32 over the existing USART2 link (PA2 TX / PA3 RX, 115200 baud, 8N1). This is **not implemented in this stage** but is defined here so that the acquisition pipeline and firmware can be designed for compatibility.

### 15.1 Problem

The STM32's USART2 is currently used for CLI commands (host → STM32) and data streaming (STM32 → host). Adding EMG data injection requires a framing protocol that coexists with the existing CLI without ambiguity.

### 15.2 Packet Format

EMG frames are sent from the PC to the STM32 using a tagged ASCII line format, consistent with the existing `D,` prefix convention used for outbound data:

```
E,<t_ms>,<emg0>,<emg1>,<emg2>,<emg3>,<emg4>,<emg5>,<emg6>,<emg7>\n
```

| Field | Type | Description |
|-------|------|-------------|
| `E` | char | packet tag — distinguishes EMG data from CLI commands |
| `t_ms` | uint32 | PC-side timestamp in milliseconds since acquisition start |
| `emg0`–`emg7` | int8 | 8-channel EMG values (-128 to +127) |
| `\n` | char | line terminator (same as CLI commands) |

Example:

```
E,1523,-12,4,8,-3,6,-1,7,2\n
```

### 15.3 Firmware-Side Parsing

The existing `uart_cli_poll()` assembles complete lines from the ring buffer and dispatches them to `app_cli_handle_line()`. The EMG bridge adds a check **before** the CLI parser:

```
uart_cli_poll() assembles line
    │
    ├── line starts with "E," → parse EMG packet → store in EMG ring buffer
    │                            (bypass CLI parser)
    │
    └── otherwise             → app_cli_handle_line()  (existing CLI path)
```

This requires:
- a new `emg_parse_line()` function in the app layer
- a small ring buffer (16–32 entries) to hold the latest EMG frames
- the fusion module reads from this buffer at its own rate

### 15.4 Timing Considerations

| Parameter | Value |
|-----------|-------|
| EMG frame rate from PC | ~200 Hz |
| EMG packet size | ~30 bytes (worst case: `E,65535,-128,-128,-128,-128,-128,-128,-128,-128\n`) |
| UART throughput at 115200 baud | ~11,520 bytes/s |
| Bandwidth used by EMG stream | ~6,000 bytes/s (30 bytes × 200 Hz) |
| Remaining bandwidth for CLI + IMU stream | ~5,520 bytes/s |

At 115200 baud, the 200 Hz EMG stream consumes approximately 52% of the UART bandwidth. The existing IMU `D,` stream at 100 Hz uses roughly 100 bytes/line × 100 Hz = 10,000 bytes/s when both filters are enabled, which **exceeds the remaining bandwidth**.

**Resolution options** (to be decided in the bridge implementation stage):

1. **Increase baud rate** to 230400 or 460800 — the STM32F411 and most USB-UART adapters support this.
2. **Reduce IMU stream rate** — use `MPU PRINT 2` (50 Hz) or `MPU PRINT 5` (20 Hz) during fusion experiments.
3. **Binary framing** — switch to a compact binary protocol to reduce per-frame overhead.
4. **Separate UART** — use a second UART peripheral (USART1 or USART6) dedicated to EMG input, keeping USART2 for CLI/IMU output.

### 15.5 Clock Synchronization

The PC and STM32 have independent clocks. For fusion, EMG and IMU timestamps must be aligned. Approach:

1. At bridge startup, the PC sends a `SYNC` command.
2. The STM32 responds with its current `HAL_GetTick()` value.
3. The PC records the round-trip time and computes a clock offset.
4. All forwarded `E,` packets use the adjusted timestamp.
5. The fusion module on the STM32 matches EMG frames to IMU samples by nearest timestamp.

This provides millisecond-level alignment, which is sufficient given the ~5 ms EMG period and ~10 ms IMU period.

---

# 16. Expected Outcome

After completing this phase the system should:

- connect to the Myo armband via Myo Connect and myo-python
- stream 8-channel raw EMG signals at ~200 Hz
- log timestamped EMG data to CSV files in `emg_data/`
- visualize EMG signals (all 8 channels) in Python and MATLAB
- pass all four validation tests (rest, contraction, finger isolation, repeatability)
- have a defined UART bridge protocol ready for the fusion stage

---

# 17. Next Development Stage

**EMG Signal Processing and Feature Extraction**

Planned tasks:

- full-wave rectification of raw EMG signals
- envelope detection (moving RMS or low-pass filtered rectified signal)
- window-based feature extraction (MAV, RMS, ZC, WL) with 50–200 ms windows
- gesture classification using extracted features
- real-time feature computation on the PC before UART forwarding

After feature extraction is validated, the processed EMG features (not raw samples) will be forwarded to the STM32 via the UART bridge protocol defined in Section 15. Sending features instead of raw samples significantly reduces bandwidth requirements (~10 Hz feature vectors vs ~200 Hz raw frames).

---

# 18. Relationship to Existing Project Modules

This module integrates alongside the existing firmware without modifying it:

| Existing Module | Relationship |
|-----------------|-------------|
| `imu_app.c` | Unmodified — continues running Madgwick + EKF at 100 Hz |
| `cli_app.c` | Future: add `E,` line detection before CLI dispatch |
| `uart_cli.c` | Future: may need baud rate increase for concurrent EMG + IMU streams |
| `capture_ekf.py` | Unmodified — EMG acquisition is a separate standalone script |
| `app_config.h` | Future: add `RUN_EMG_FUSION` compile flag and EMG buffer size settings |
| `mpu6050.c` | Unmodified — MPU6050 remains the primary IMU source |

---

# 19. Summary

This stage establishes the EMG acquisition infrastructure required for the EMG–IMU sensor fusion system. The Myo armband provides 8-channel surface EMG data via BLE to the PC, where it is logged, visualized, and validated. A UART bridge protocol is specified for future integration with the STM32F411 firmware, enabling real-time fusion of EMG features with the Madgwick/EKF orientation estimates from the MPU6050.

Once the Myo streaming pipeline is verified and EMG signal quality is confirmed, the project proceeds to EMG signal processing (rectification, envelope detection, feature extraction) and ultimately to on-board sensor fusion.
