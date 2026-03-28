"""
capture_ekf.py  —  Capture extended IMU stream (Madgwick + EKF) from STM32 over UART.

Usage
-----
  python capture_ekf.py                              # uses defaults below
  python capture_ekf.py COM3                         # override port
  python capture_ekf.py COM3 115200                  # override port + baud
  python capture_ekf.py COM3 115200 2                # override print divisor (default 1 = 100 Hz)
  python capture_ekf.py COM7 115200 1 raw_vs_ekf     # custom CSV name

How it works
------------
  - Auto-sends MPU INIT / MPU PRINT <N> / MPU STREAM ON.
  - Parses the extended "D," lines emitted by the firmware when both
    RUN_MADGWICK and RUN_EKF are enabled.
  - While capturing you can type any CLI command (e.g. EKF SHOW, EKF DIAG,
    EKF TUNE, MAD SHOW) and press Enter — forwarded to the board.
  - Type  q  or  quit  (or Ctrl+C) to stop.

Output CSV columns (19 numeric fields per row)
----------------------------------------------
  t_ms           : board timestamp (ms)

  -- Raw sensor (int16 counts) --
  ax_raw         : accel X  (÷16384 → g  at ±2g)
  ay_raw         : accel Y
  az_raw         : accel Z
  gx_raw         : gyro  X  (÷131   → dps at ±250 dps)
  gy_raw         : gyro  Y
  gz_raw         : gyro  Z

  -- Madgwick filter --
  mad_roll_mdeg  : roll  (÷1000 → deg)
  mad_pitch_mdeg : pitch (÷1000 → deg, negated convention)
  mad_yaw_mdeg   : yaw   (÷1000 → deg)
  mad_us         : Madgwick step CPU time (µs)

  -- EKF filter --
  ekf_roll_mdeg  : roll  (÷1000 → deg)
  ekf_pitch_mdeg : pitch (÷1000 → deg)
  ekf_yaw_mdeg   : yaw   (÷1000 → deg)
  traceP_1e6     : trace(P) × 1e6 — EKF convergence metric (decreases over time)
  ekf_us         : EKF step CPU time (µs)

  -- EKF gyro bias estimate --
  bx_uradps      : bias X (÷1e6 → rad/s)
  by_uradps      : bias Y
  bz_uradps      : bias Z

Columns are 0 when the respective filter is disabled or not yet valid.

Dependencies
------------
  pip install pyserial
"""

import sys
import csv
import time
import threading
import serial
from datetime import datetime

# ── Configuration ──────────────────────────────────────────────────────────────
PORT        = "COM7"    # change to your port (e.g. /dev/ttyUSB0 on Linux)
BAUD        = 115200
PRINT_DIV   = 1         # 1 = 100 Hz, 2 = 50 Hz, 5 = 20 Hz
# ───────────────────────────────────────────────────────────────────────────────

CSV_HEADER = [
    "t_ms",
    "ax_raw", "ay_raw", "az_raw",
    "gx_raw", "gy_raw", "gz_raw",
    "mad_roll_mdeg", "mad_pitch_mdeg", "mad_yaw_mdeg", "mad_us",
    "ekf_roll_mdeg", "ekf_pitch_mdeg", "ekf_yaw_mdeg",
    "traceP_1e6", "ekf_us",
    "bx_uradps", "by_uradps", "bz_uradps",
]

# Number of numeric fields after the "D" prefix tag
EXPECTED_FIELDS = len(CSV_HEADER)   # 19

DATA_PREFIX = b"D,"


def send_cmd(ser: serial.Serial, cmd: str):
    """Send a CLI command and wait briefly for the board to reply."""
    ser.write((cmd.strip() + "\r\n").encode())
    time.sleep(0.15)


def reader_thread(ser: serial.Serial, writer: csv.writer,
                  counter: list, running: threading.Event):
    """
    Background thread: reads every line from the serial port.
    - 'D,...' lines  → parsed and written to CSV
    - Everything else → printed to console as board reply
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
                # parts[0] = b"D", parts[1..19] = numeric fields
                if len(parts) != EXPECTED_FIELDS + 1:
                    # Tolerate old 10-field firmware (Madgwick-only) gracefully
                    if len(parts) == 11:
                        row = [int(p) for p in parts[1:]] + [0] * (EXPECTED_FIELDS - 10)
                    else:
                        continue
                else:
                    row = [int(p) for p in parts[1:]]
            except ValueError:
                continue

            writer.writerow(row)
            counter[0] += 1
            if counter[0] % 100 == 0:
                print(f"  [{counter[0]} samples | "
                      f"{counter[0] / 100.0:.1f} s @ 100 Hz]      ",
                      end="\r", flush=True)
        else:
            text = line.decode(errors="replace")
            if text:
                print(f"\n  >> {text}", flush=True)


def main():
    port      = sys.argv[1] if len(sys.argv) > 1 else PORT
    baud      = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD
    print_div = int(sys.argv[3]) if len(sys.argv) > 3 else PRINT_DIV

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if len(sys.argv) > 4:
        base_name = sys.argv[4].removesuffix(".csv")
        out_file  = f"{base_name}.csv"
    else:
        out_file  = f"ekf_capture_{timestamp}.csv"

    print(f"\nOpening {port} at {baud} baud ...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: cannot open port — {e}")
        sys.exit(1)

    print(f"Output  : {out_file}")
    print("─" * 60)
    print("Type any CLI command while capturing and press Enter.")
    print("Useful commands: EKF SHOW  EKF DIAG  EKF TUNE  MAD SHOW")
    print("Type  q  or  quit  (or Ctrl+C) to stop.\n")

    f      = open(out_file, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(CSV_HEADER)

    counter = [0]
    running = threading.Event()
    running.set()

    t = threading.Thread(target=reader_thread,
                         args=(ser, writer, counter, running),
                         daemon=True)
    t.start()

    # ── Auto-init sequence ─────────────────────────────────────────────────────
    time.sleep(0.2)
    send_cmd(ser, "MPU INIT")

    # ── Calibration prompt ────────────────────────────────────────────────────
    print("Place the board FLAT and STILL.")
    choice = input("Press Enter to calibrate gyro (3 s), or type 's' to skip: ").strip().lower()
    if choice != "s":
        print("Calibrating — keep the board still ...")
        send_cmd(ser, "MPU CAL GYRO 3000")
        time.sleep(3.5)
        print("Calibration done.\n")
    else:
        print("Calibration skipped.\n")

    # ── Start streaming ───────────────────────────────────────────────────────
    send_cmd(ser, f"MPU PRINT {print_div}")
    send_cmd(ser, "MPU STREAM ON")
    print("Streaming started. Move the board now.")

    # ── Interactive command loop ──────────────────────────────────────────────
    try:
        while running.is_set():
            try:
                cmd = input()
            except EOFError:
                break

            cmd = cmd.strip()
            if not cmd:
                continue
            if cmd.lower() in ("q", "quit", "exit"):
                break

            send_cmd(ser, cmd)

    except KeyboardInterrupt:
        pass

    # ── Shutdown ──────────────────────────────────────────────────────────────
    print(f"\n\nStopping stream ...")
    try:
        send_cmd(ser, "MPU STREAM OFF")
    except Exception:
        pass

    running.clear()
    t.join(timeout=2)
    f.close()
    ser.close()

    print(f"Done. {counter[0]} samples saved to {out_file}\n")


if __name__ == "__main__":
    main()
