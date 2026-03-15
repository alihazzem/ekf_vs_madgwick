"""
capture_fusion.py  —  Simultaneously capture Myo EMG (BLE) and STM32 IMU (UART).

Produces two time-synchronised CSV files in ../emg_data/:
  fusion_emg_TIMESTAMP.csv  — EMG stream with PC wall-clock timestamp
  fusion_imu_TIMESTAMP.csv  — IMU stream with PC wall-clock timestamp

Time alignment
--------------
  Both CSVs share a pc_t_ms column derived from the same PC clock origin
  (set at the moment MPU STREAM ON is sent).  In MATLAB, use interp1 or
  nearest-neighbour matching on pc_t_ms to align the two streams.

Usage
-----
  python capture_fusion.py                     # defaults below
  python capture_fusion.py COM3                # override COM port
  python capture_fusion.py COM3 115200 2       # port, baud, IMU print divisor

EMG CSV columns
---------------
  pc_t_ms     : PC wall-clock ms since capture start (common time axis)
  emg_t_ms    : elapsed ms from Myo callback timestamps (native Myo clock)
  emg0–emg7   : 8-channel signed EMG (-128 to +127)

IMU CSV columns (19 fields — identical to capture_ekf.py plus pc_t_ms)
-----------------------------------------------------------------------
  pc_t_ms     : PC wall-clock ms since capture start (common time axis)
  t_ms        : board timestamp from HAL_GetTick()
  ax_raw .. gz_raw             : raw int16 sensor counts
  mad_roll_mdeg .. mad_us      : Madgwick filter output
  ekf_roll_mdeg .. ekf_us      : EKF filter output
  bx_uradps .. bz_uradps       : EKF gyro bias estimate

Dependencies
------------
  pip install myo-python pyserial
  Myo Connect must be running.
"""

import sys
import csv
import os
import time
import threading
import serial
import myo
from myo import Hub, DeviceListener
from datetime import datetime


# ── Configuration ─────────────────────────────────────────────────────────────
PORT      = "COM7"
BAUD      = 115200
PRINT_DIV = 1          # 1 = 100 Hz, 2 = 50 Hz
SDK_PATH  = ""         # set to Myo SDK bin/ path if myo.init() fails
OUT_DIR   = "../emg_data"
# ─────────────────────────────────────────────────────────────────────────────

EMG_HEADER = [
    "pc_t_ms", "emg_t_ms",
    "emg0", "emg1", "emg2", "emg3",
    "emg4", "emg5", "emg6", "emg7",
]

IMU_HEADER = [
    "pc_t_ms", "t_ms",
    "ax_raw", "ay_raw", "az_raw",
    "gx_raw", "gy_raw", "gz_raw",
    "mad_roll_mdeg", "mad_pitch_mdeg", "mad_yaw_mdeg", "mad_us",
    "ekf_roll_mdeg", "ekf_pitch_mdeg", "ekf_yaw_mdeg",
    "traceP_1e6", "ekf_us",
    "bx_uradps", "by_uradps", "bz_uradps",
]

IMU_DATA_FIELDS = len(IMU_HEADER) - 1  # 19 fields after pc_t_ms
DATA_PREFIX     = b"D,"


def send_cmd(ser: serial.Serial, cmd: str):
    ser.write((cmd.strip() + "\r\n").encode())
    time.sleep(0.15)


# ── IMU reader thread ─────────────────────────────────────────────────────────

def imu_reader_thread(ser: serial.Serial, writer: csv.writer,
                      counters: list, running: threading.Event,
                      t_origin: list):
    """
    Reads D, lines from UART and writes them to the IMU CSV.
    t_origin[0] is set by main() when streaming starts — provides the
    common PC clock origin shared with the EMG listener.
    """
    while running.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException:
            running.clear()
            break

        if not raw:
            continue

        line = raw.strip()

        if line.startswith(DATA_PREFIX):
            try:
                parts = line.split(b",")
                if len(parts) == IMU_DATA_FIELDS + 1:
                    fields = [int(p) for p in parts[1:]]
                elif len(parts) == 11:
                    fields = [int(p) for p in parts[1:]] + [0] * (IMU_DATA_FIELDS - 10)
                else:
                    continue
            except ValueError:
                continue

            origin = t_origin[0]
            pc_t   = int((time.time() - origin) * 1000) if origin else 0
            writer.writerow([pc_t] + fields)
            counters[1] += 1
        else:
            text = line.decode(errors="replace")
            if text:
                print(f"\n  [IMU] >> {text}", flush=True)


# ── Myo EMG listener ──────────────────────────────────────────────────────────

class FusionEmgListener(DeviceListener):

    def __init__(self, writer: csv.writer, counters: list,
                 running: threading.Event, t_origin: list):
        super().__init__()
        self._writer   = writer
        self._counters = counters
        self._running  = running
        self._t_origin = t_origin
        self._t_start  = None   # first Myo callback timestamp (µs)

    def on_connected(self, event):
        print("  [EMG] Myo connected — streaming EMG")
        event.device.stream_emg(True)

    def on_disconnected(self, event):
        print("\n  [EMG] Myo disconnected")

    def on_emg(self, event):
        if not self._running.is_set():
            return

        if self._t_start is None:
            self._t_start = event.timestamp

        emg_t_ms = int((event.timestamp - self._t_start) / 1000)

        origin = self._t_origin[0]
        pc_t   = int((time.time() - origin) * 1000) if origin else 0

        self._writer.writerow([pc_t, emg_t_ms] + list(event.emg))
        self._counters[0] += 1


# ── Progress printer ──────────────────────────────────────────────────────────

def progress_thread(counters: list, running: threading.Event):
    """Prints a combined sample count line every second."""
    while running.is_set():
        time.sleep(1.0)
        print(f"  EMG: {counters[0]:5d} samples  |  "
              f"IMU: {counters[1]:5d} samples      ",
              end="\r", flush=True)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    port      = sys.argv[1] if len(sys.argv) > 1 else PORT
    baud      = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD
    print_div = int(sys.argv[3]) if len(sys.argv) > 3 else PRINT_DIV

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir    = os.path.normpath(os.path.join(script_dir, OUT_DIR))
    os.makedirs(out_dir, exist_ok=True)

    emg_path = os.path.join(out_dir, f"fusion_emg_{timestamp}.csv")
    imu_path = os.path.join(out_dir, f"fusion_imu_{timestamp}.csv")

    print(f"\nEMG out : {emg_path}")
    print(f"IMU out : {imu_path}")
    print("─" * 60)

    # ── Open serial ───────────────────────────────────────────────────────────
    print(f"Opening {port} at {baud} baud ...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: cannot open {port} — {e}")
        sys.exit(1)

    # ── Init Myo SDK ──────────────────────────────────────────────────────────
    print("Initialising Myo SDK ...")
    try:
        myo.init(sdk_path=SDK_PATH) if SDK_PATH else myo.init()
    except Exception as e:
        print(f"ERROR: myo.init() failed — {e}")
        print("Set SDK_PATH at the top of this script.")
        ser.close()
        sys.exit(1)

    # ── Open CSV files ────────────────────────────────────────────────────────
    emg_f  = open(emg_path, "w", newline="")
    imu_f  = open(imu_path, "w", newline="")
    emg_wr = csv.writer(emg_f)
    imu_wr = csv.writer(imu_f)
    emg_wr.writerow(EMG_HEADER)
    imu_wr.writerow(IMU_HEADER)

    # counters[0] = EMG samples, counters[1] = IMU samples
    counters = [0, 0]
    running  = threading.Event()
    running.set()
    t_origin = [0.0]   # shared PC clock origin — set when streaming starts

    # ── Start IMU reader thread ───────────────────────────────────────────────
    imu_t = threading.Thread(
        target=imu_reader_thread,
        args=(ser, imu_wr, counters, running, t_origin),
        daemon=True)
    imu_t.start()

    # ── MPU init + calibration ────────────────────────────────────────────────
    time.sleep(0.2)
    send_cmd(ser, "MPU INIT")

    print("\nPlace the board FLAT and STILL for gyro calibration.")
    choice = input("Press Enter to calibrate (3 s), or 's' to skip: ").strip().lower()
    if choice != "s":
        print("Calibrating ...")
        send_cmd(ser, "MPU CAL GYRO 3000")
        time.sleep(3.5)
        print("Calibration done.\n")
    else:
        print("Calibration skipped.\n")

    # ── Start both streams ────────────────────────────────────────────────────
    print("Waiting for Myo armband connection ...")
    hub      = Hub()
    listener = FusionEmgListener(emg_wr, counters, running, t_origin)

    with hub.run_in_background(listener):
        # Give the Myo a moment to connect before starting the IMU stream
        time.sleep(1.0)

        # Set the shared clock origin — both streams timestamp relative to this
        t_origin[0] = time.time()

        send_cmd(ser, f"MPU PRINT {print_div}")
        send_cmd(ser, "MPU STREAM ON")

        # Start progress printer
        prog_t = threading.Thread(
            target=progress_thread, args=(counters, running), daemon=True)
        prog_t.start()

        print("Both streams running. Type CLI commands or press Enter to stop.\n")

        # ── Interactive command loop ──────────────────────────────────────────
        try:
            while running.is_set():
                try:
                    cmd = input()
                except EOFError:
                    break
                cmd = cmd.strip()
                if not cmd:
                    break
                if cmd.lower() in ("q", "quit", "exit"):
                    break
                send_cmd(ser, cmd)
        except KeyboardInterrupt:
            pass

    # ── Shutdown ──────────────────────────────────────────────────────────────
    print(f"\n\nStopping streams ...")
    try:
        send_cmd(ser, "MPU STREAM OFF")
    except Exception:
        pass

    running.clear()
    imu_t.join(timeout=2)
    emg_f.close()
    imu_f.close()
    ser.close()

    print(f"Done.")
    print(f"  EMG : {counters[0]} samples  → {emg_path}")
    print(f"  IMU : {counters[1]} samples  → {imu_path}\n")


if __name__ == "__main__":
    main()
