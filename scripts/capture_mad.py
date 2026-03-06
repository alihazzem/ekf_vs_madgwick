"""
capture.py  —  Capture IMU stream from STM32 over UART and save to CSV.

Usage
-----
  python capture.py                              # uses defaults below
  python capture.py COM3                         # override port
  python capture.py COM3 115200                  # override port + baud
  python capture.py COM3 115200 2                # override print divisor (default 1 = 100 Hz)
  python capture.py COM3 115200 1 my_test        # custom CSV name → my_test.csv
  python capture.py COM3 115200 1 tilt_30deg     # custom CSV name → tilt_30deg.csv

How it works
------------
  - The script opens the port, automatically sends MPU INIT / MPU STREAM ON /
    MPU PRINT <N> so you don't need a separate terminal.
  - While capturing you can type any CLI command (e.g. MAD RESET, MAD SHOW,
    MPU CAL GYRO 2000) and press Enter — it is forwarded to the board and the
    reply is printed.  Only "D," data lines are written to the CSV.
  - Type  q  or  quit  (or press Ctrl+C) to stop streaming and close the file.

Output CSV columns
------------------
  t_ms        : board timestamp (ms, from HAL_GetTick)
  ax_raw      : accel X raw int16  (÷16384 → g  at ±2g)
  ay_raw      : accel Y raw int16
  az_raw      : accel Z raw int16
  gx_raw      : gyro  X raw int16  (÷131   → dps at ±250dps)
  gy_raw      : gyro  Y raw int16
  gz_raw      : gyro  Z raw int16
  roll_mdeg   : Madgwick roll  (÷1000 → deg)
  pitch_mdeg  : Madgwick pitch (÷1000 → deg, negated — matches MAD SHOW)
  yaw_mdeg    : Madgwick yaw   (÷1000 → deg)

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
    "roll_mdeg", "pitch_mdeg", "yaw_mdeg",
]

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
                if len(parts) != 11:
                    continue
                row = [int(p) for p in parts[1:]]   # skip the "D" tag
            except ValueError:
                continue
            writer.writerow(row)
            counter[0] += 1
            # progress every 100 samples (overwrite same console line)
            if counter[0] % 100 == 0:
                print(f"  [{counter[0]} samples | "
                      f"{counter[0] / 100.0:.1f} s @ 100 Hz]      ",
                      end="\r", flush=True)
        else:
            # board reply to a CLI command — print it
            text = line.decode(errors="replace")
            if text:
                print(f"\n  >> {text}", flush=True)


def main():
    port      = sys.argv[1] if len(sys.argv) > 1 else PORT
    baud      = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD
    print_div = int(sys.argv[3]) if len(sys.argv) > 3 else PRINT_DIV

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if len(sys.argv) > 4:
        # user-supplied name: strip .csv if they included it, then add it back
        base_name = sys.argv[4].removesuffix(".csv")
        out_file  = f"{base_name}.csv"
    else:
        out_file  = f"imu_capture_{timestamp}.csv"

    print(f"\nOpening {port} at {baud} baud ...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: cannot open port — {e}")
        sys.exit(1)

    print(f"Output  : {out_file}")
    print("─" * 56)
    print("Type any CLI command while capturing and press Enter.")
    print("Type  q  or  quit  (or Ctrl+C) to stop.\n")

    f       = open(out_file, "w", newline="")
    writer  = csv.writer(f)
    writer.writerow(CSV_HEADER)

    counter = [0]
    running = threading.Event()
    running.set()

    t = threading.Thread(target=reader_thread,
                         args=(ser, writer, counter, running),
                         daemon=True)
    t.start()

    # ── Auto-init sequence ────────────────────────────────────────────────────
    time.sleep(0.2)   # let the board settle after port open
    send_cmd(ser, "MPU INIT")

    # ── Calibration prompt ───────────────────────────────────────────────────
    print("Place the board FLAT and STILL.")
    choice = input("Press Enter to calibrate gyro (3 s), or type 's' to skip: ").strip().lower()
    if choice != 's':
        print("Calibrating — keep the board still ...")
        send_cmd(ser, "MPU CAL GYRO 3000")
        time.sleep(3.5)   # wait for the 3 s calibration to finish on the board
        print("Calibration done.\n")
    else:
        print("Calibration skipped.\n")

    # ── Start streaming ──────────────────────────────────────────────────────
    send_cmd(ser, f"MPU PRINT {print_div}")
    send_cmd(ser, "MPU STREAM ON")
    print("Streaming started. Move the board now.")

    # ── Interactive command loop ──────────────────────────────────────────────
    try:
        while running.is_set():
            try:
                cmd = input()           # blocks until user presses Enter
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

    # ── Shutdown ─────────────────────────────────────────────────────────────
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
