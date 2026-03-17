"""
capture_emg.py  —  Acquire raw 8-channel EMG data from the Myo armband and save to CSV.

Usage
-----
  python capture_emg.py
  python capture_emg.py my_emg_test

Requirements
------------
  pip install myo-python
  Myo Connect must be running.
  myo.dll must be on PATH or in this script directory.

  If myo.init() fails, set:
      SDK_PATH = "C:/path/to/myo-sdk/bin"

Output CSV
----------
  t_ms, emg0..emg7

Where:
  t_ms       elapsed time in ms since first EMG sample
  emg0..7    signed EMG samples from Myo (-128..127 nominally)

Output folder
-------------
  ../emg_data/   relative to this script

Notes
-----
- This script logs raw EMG only.
- Press Enter or Ctrl+C to stop.
- Timing stats are printed at the end using actual Myo event timestamps.
"""

import sys
import csv
import os
import myo
from myo import Hub, DeviceListener
from datetime import datetime


# ── Configuration ────────────────────────────────────────────────────────────
SDK_PATH = ""
OUT_DIR = "../emg_data"
FLUSH_EVERY = 200      # flush file every N samples
PROGRESS_EVERY = 200   # print progress every N samples
# ─────────────────────────────────────────────────────────────────────────────

CSV_HEADER = [
    "t_ms",
    "emg0", "emg1", "emg2", "emg3",
    "emg4", "emg5", "emg6", "emg7",
]


class EmgListener(DeviceListener):
    def __init__(self, writer: csv.writer, csv_file, stats: dict):
        super().__init__()
        self._writer = writer
        self._file = csv_file
        self._stats = stats

        self._t_start = None       # first EMG event timestamp (us)
        self._t_last = None        # last EMG event timestamp (us)
        self._last_dt_us = None

    def on_connected(self, event):
        print("Myo connected")
        event.device.stream_emg(True)
        print("EMG streaming enabled")

    def on_disconnected(self, event):
        print("\nMyo disconnected")
        self._stats["disconnects"] += 1

    def on_emg(self, event):
        ts_us = event.timestamp

        if self._t_start is None:
            self._t_start = ts_us
            self._stats["t_first_us"] = ts_us

        if self._t_last is not None:
            dt_us = ts_us - self._t_last
            self._last_dt_us = dt_us

            if dt_us < self._stats["dt_min_us"]:
                self._stats["dt_min_us"] = dt_us
            if dt_us > self._stats["dt_max_us"]:
                self._stats["dt_max_us"] = dt_us

        self._t_last = ts_us
        self._stats["t_last_us"] = ts_us

        t_ms = (ts_us - self._t_start) / 1000
        row = [t_ms] + list(event.emg)
        self._writer.writerow(row)

        self._stats["samples"] += 1
        n = self._stats["samples"]

        if n % FLUSH_EVERY == 0:
            self._file.flush()

        if n % PROGRESS_EVERY == 0:
            elapsed_s = (ts_us - self._t_start) / 1e6 if self._t_start is not None else 0.0
            rate_hz = (n - 1) / elapsed_s if elapsed_s > 0 and n > 1 else 0.0
            print(
                f"  [{n} samples | {elapsed_s:.2f} s | {rate_hz:.1f} Hz]      ",
                end="\r",
                flush=True
            )


def safe_remove_csv_suffix(name: str) -> str:
    if name.lower().endswith(".csv"):
        return name[:-4]
    return name


def resolve_output_path():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    if len(sys.argv) > 1:
        base_name = safe_remove_csv_suffix(sys.argv[1].strip())
        out_file = f"{base_name}.csv"
    else:
        out_file = f"emg_capture_{timestamp}.csv"

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.normpath(os.path.join(script_dir, OUT_DIR))
    os.makedirs(out_dir, exist_ok=True)

    return os.path.join(out_dir, out_file)


def print_final_stats(stats: dict, out_path: str):
    samples = stats["samples"]
    t_first = stats["t_first_us"]
    t_last = stats["t_last_us"]

    print("\n")

    if samples == 0 or t_first is None or t_last is None:
        print("No EMG samples were captured.")
        print(f"Output file: {out_path}")
        return

    dur_s = (t_last - t_first) / 1e6
    avg_rate = (samples - 1) / dur_s if dur_s > 0 and samples > 1 else 0.0

    dt_min_us = stats["dt_min_us"] if stats["dt_min_us"] != float("inf") else 0.0
    dt_max_us = stats["dt_max_us"] if stats["dt_max_us"] != 0.0 else 0.0

    print("── Capture complete ──────────────────────────────")
    print(f"Output file : {out_path}")
    print(f"Samples     : {samples}")
    print(f"Duration    : {dur_s:.3f} s")
    print(f"Avg rate    : {avg_rate:.2f} Hz")
    print(f"dt min/max  : {dt_min_us:.0f} / {dt_max_us:.0f} us")
    print(f"Disconnects : {stats['disconnects']}")
    print("──────────────────────────────────────────────────\n")


def main():
    out_path = resolve_output_path()

    print(f"\nOutput  : {out_path}")
    print("─" * 60)
    print("Ensure Myo Connect is running and the armband is awake.")
    print("Waiting for Myo connection ...")
    print("Press Enter (or Ctrl+C) to stop capturing.\n")

    try:
        if SDK_PATH:
            myo.init(sdk_path=SDK_PATH)
        else:
            myo.init()
    except Exception as e:
        print(f"ERROR: myo.init() failed — {e}")
        print("Make sure Myo Connect is running and myo.dll is on PATH.")
        print("If needed, set SDK_PATH at the top of this script.")
        sys.exit(1)

    stats = {
        "samples": 0,
        "t_first_us": None,
        "t_last_us": None,
        "dt_min_us": float("inf"),
        "dt_max_us": 0.0,
        "disconnects": 0,
    }

    csv_file = None
    try:
        csv_file = open(out_path, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(CSV_HEADER)

        listener = EmgListener(writer, csv_file, stats)
        hub = Hub()

        with hub.run_in_background(listener):
            try:
                input()
            except KeyboardInterrupt:
                pass

    except Exception as e:
        print(f"\nERROR during capture: {e}")
    finally:
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()

    print_final_stats(stats, out_path)


if __name__ == "__main__":
    main()