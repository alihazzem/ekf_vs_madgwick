"""
capture_emg.py  —  Acquire EMG data from the Myo armband and save to CSV.

Usage
-----
  python capture_emg.py                    # default timestamped filename
  python capture_emg.py my_emg_test        # custom CSV name -> my_emg_test.csv

Requirements
------------
  pip install myo-python
  Myo Connect must be running in the system tray.
  myo.dll (Windows) must be on the system PATH or in the script directory.

  If myo.init() raises RuntimeError, set SDK_PATH at the top of this file:
    SDK_PATH = "C:/path/to/myo-sdk/bin"

How it works
------------
  - Initialises the Myo SDK, then starts Hub.run_in_background() with an
    EmgListener that receives on_emg callbacks at ~200 Hz.
  - Each callback writes one row (t_ms + 8 channels) to the CSV.
  - Press Enter (or Ctrl+C) to stop capture and close the file.

Output CSV columns
------------------
  t_ms         : elapsed time (ms) since the first EMG sample
  emg0–emg7    : 8-channel signed 8-bit EMG values (−128 to +127)

Sampling rate  : ~200 Hz (Myo BLE delivery rate)
Output folder  : ../emg_data/  (created automatically if it does not exist)

Dependencies
------------
  pip install myo-python
"""

import sys
import csv
import os
import myo
from myo import Hub, DeviceListener
from datetime import datetime


# ── Configuration ────────────────────────────────────────────────────────────
SDK_PATH = ""
OUT_DIR  = "../emg_data"  # relative to the scripts/ folder
# ─────────────────────────────────────────────────────────────────────────────

CSV_HEADER = [
    "t_ms",
    "emg0", "emg1", "emg2", "emg3",
    "emg4", "emg5", "emg6", "emg7",
]


class EmgListener(DeviceListener):

    def __init__(self, writer: csv.writer, counter: list):
        super().__init__()
        self._writer  = writer
        self._counter = counter
        self._t_start = None   # timestamp of first EMG sample (microseconds)

    def on_connected(self, event):
        print("Myo connected")
        event.device.stream_emg(True)
        print("EMG streaming enabled")

    def on_disconnected(self, event):
        print("\nMyo disconnected")

    def on_emg(self, event):
        if self._t_start is None:
            self._t_start = event.timestamp   # microseconds (Unix epoch-based)

        t_ms = int((event.timestamp - self._t_start) / 1000)
        row  = [t_ms] + list(event.emg)
        self._writer.writerow(row)
        self._counter[0] += 1

        if self._counter[0] % 200 == 0:
            print(f"  [{self._counter[0]} samples | "
                  f"{self._counter[0] / 200.0:.1f} s @ ~200 Hz]      ",
                  end="\r", flush=True)


def main():
    # ── Resolve output path ───────────────────────────────────────────────────
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if len(sys.argv) > 1:
        base_name = sys.argv[1].removesuffix(".csv")
        out_file  = f"{base_name}.csv"
    else:
        out_file  = f"emg_capture_{timestamp}.csv"

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir    = os.path.normpath(os.path.join(script_dir, OUT_DIR))
    os.makedirs(out_dir, exist_ok=True)
    out_path   = os.path.join(out_dir, out_file)

    print(f"\nOutput  : {out_path}")
    print("─" * 60)
    print("Ensure Myo Connect is running and the armband is awake.")
    print("Waiting for Myo connection ...")
    print("Press Enter (or Ctrl+C) to stop capturing.\n")

    # ── Initialise Myo SDK ────────────────────────────────────────────────────
    try:
        if SDK_PATH:
            myo.init(sdk_path=SDK_PATH)
        else:
            myo.init()
    except Exception as e:
        print(f"ERROR: myo.init() failed — {e}")
        print("Make sure Myo Connect is running and myo.dll is on the PATH.")
        print("If needed, set SDK_PATH at the top of this script.")
        sys.exit(1)

    # ── Open CSV ──────────────────────────────────────────────────────────────
    f      = open(out_path, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(CSV_HEADER)

    counter = [0]

    # ── Start BLE hub with listener ───────────────────────────────────────────
    hub      = Hub()
    listener = EmgListener(writer, counter)

    try:
        with hub.run_in_background(listener):
            try:
                input()   # block until Enter is pressed
            except KeyboardInterrupt:
                pass
    except Exception as e:
        print(f"\nERROR during capture: {e}")
    finally:
        f.close()

    print(f"\n\nDone. {counter[0]} samples saved to {out_path}\n")


if __name__ == "__main__":
    main()
