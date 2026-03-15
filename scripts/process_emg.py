"""
process_emg.py  —  Signal processing pipeline for raw Myo EMG CSV.

Stages
------
  1. Full-wave rectification   |emg|
  2. Moving-RMS envelope       sqrt(movmean(emg^2, ENV_WIN))
  3. Sliding-window feature extraction:
       MAV  — Mean Absolute Value        mean(|emg|)       per window
       RMS  — Root Mean Square           sqrt(mean(emg^2)) per window
       ZC   — Zero Crossing count        sign changes with dead-zone
       WL   — Waveform Length            sum(|diff(emg)|)  per window

Usage
-----
  python process_emg.py                               # opens file picker prompt
  python process_emg.py emg_test_synthetic.csv        # process specific file
  python process_emg.py emg_test_synthetic.csv 100 50 # custom window/step (ms)

  The input CSV is searched in ../emg_data/ relative to the scripts/ folder.
  If the filename is not found there, it is treated as a full path.

Output (written next to the input file)
----------------------------------------
  <name>_envelope.csv  — moving-RMS envelope at the original sample rate
                         columns: t_ms, env0..env7  (float, same units as raw)

  <name>_features.csv  — windowed features (one row per window step)
                         columns: t_ms, mav0..mav7, rms0..rms7,
                                        zc0..zc7,  wl0..wl7

Default parameters
------------------
  ENV_WIN  = 50 samples  (~250 ms at 200 Hz)  — RMS envelope smoothing
  WIN_MS   = 100 ms                            — feature extraction window
  STEP_MS  = 50  ms                            — feature extraction step (50% overlap)
  ZC_THRESH = 5 counts                         — dead-zone to suppress noise crossings

Dependencies
------------
  pip install numpy
"""

import sys
import os
import csv
import numpy as np

# ── Default parameters ────────────────────────────────────────────────────────
ENV_WIN   = 50     # RMS envelope window (samples, ~250 ms at 200 Hz)
WIN_MS    = 100    # feature window length (ms)
STEP_MS   = 50     # feature window step   (ms)
ZC_THRESH = 5      # zero-crossing dead zone (raw counts)
FS_NOM    = 200.0  # nominal EMG sample rate (Hz)
EMG_DIR   = "../emg_data"
# ─────────────────────────────────────────────────────────────────────────────


def load_csv(path: str):
    """Return (t_ms [N], emg [N x 8], measured_fs)."""
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    t_ms = data[:, 0]
    emg  = data[:, 1:9].astype(np.int16)
    fs   = 1000.0 * (len(t_ms) - 1) / max(t_ms[-1] - t_ms[0], 1)
    return t_ms, emg, fs


def moving_rms(emg: np.ndarray, win: int) -> np.ndarray:
    """
    Per-channel moving-RMS envelope using a causal sliding window.
    Output has the same shape as input [N x 8].
    """
    N, C = emg.shape
    env  = np.zeros((N, C), dtype=float)
    kernel = np.ones(win) / win
    for ch in range(C):
        sq  = emg[:, ch].astype(float) ** 2
        env[:, ch] = np.sqrt(np.convolve(sq, kernel, mode="same"))
    return env


def extract_features(t_ms: np.ndarray, emg: np.ndarray,
                     win_samp: int, step_samp: int):
    """
    Sliding-window feature extraction.
    Returns a list of rows: [t_center_ms, mav×8, rms×8, zc×8, wl×8]
    """
    N, C = emg.shape
    rows = []

    for start in range(0, N - win_samp + 1, step_samp):
        end    = start + win_samp
        w      = emg[start:end, :].astype(float)   # [win x 8]
        t_c    = int((t_ms[start] + t_ms[end - 1]) / 2)

        mav = np.mean(np.abs(w), axis=0)
        rms = np.sqrt(np.mean(w ** 2, axis=0))

        # Zero crossings: count sign changes where both neighbours
        # exceed the dead-zone threshold
        zc = np.zeros(C, dtype=int)
        for ch in range(C):
            s = emg[start:end, ch]
            for i in range(len(s) - 1):
                if (s[i] * s[i + 1] < 0 and
                        (abs(int(s[i])) >= ZC_THRESH or
                         abs(int(s[i + 1])) >= ZC_THRESH)):
                    zc[ch] += 1

        wl = np.sum(np.abs(np.diff(w, axis=0)), axis=0)

        rows.append([t_c] + list(mav) + list(rms) + list(zc) + list(wl))

    return rows


def write_csv(path: str, header: list, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for row in rows:
            w.writerow([f"{v:.4f}" if isinstance(v, float) else v for v in row])


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    emg_dir    = os.path.normpath(os.path.join(script_dir, EMG_DIR))

    # ── Resolve input file ────────────────────────────────────────────────────
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        # Try emg_data/ first, then treat as full path
        candidate = os.path.join(emg_dir, arg if arg.endswith(".csv") else arg + ".csv")
        in_path   = candidate if os.path.exists(candidate) else arg
    else:
        # List available CSVs in emg_data/
        available = [f for f in os.listdir(emg_dir) if f.endswith(".csv")]
        if not available:
            print(f"No CSV files found in {emg_dir}")
            print("Run  generate_emg_test_data.py  or  capture_emg.py  first.")
            sys.exit(1)
        print("Available EMG captures:")
        for i, f in enumerate(available):
            print(f"  [{i}] {f}")
        choice  = input("Select file number (or press Enter for 0): ").strip()
        idx     = int(choice) if choice.isdigit() else 0
        in_path = os.path.join(emg_dir, available[idx])

    if not os.path.exists(in_path):
        print(f"ERROR: file not found — {in_path}")
        sys.exit(1)

    # ── Window parameters ─────────────────────────────────────────────────────
    win_ms  = int(sys.argv[2]) if len(sys.argv) > 2 else WIN_MS
    step_ms = int(sys.argv[3]) if len(sys.argv) > 3 else STEP_MS

    # ── Load ──────────────────────────────────────────────────────────────────
    print(f"\nInput   : {in_path}")
    t_ms, emg, fs = load_csv(in_path)
    n_samp = len(t_ms)
    dur_s  = (t_ms[-1] - t_ms[0]) / 1000.0

    win_samp  = max(1, int(win_ms  / 1000.0 * fs))
    step_samp = max(1, int(step_ms / 1000.0 * fs))

    print(f"Samples : {n_samp}  |  Duration : {dur_s:.2f} s  |  Rate : {fs:.1f} Hz")
    print(f"Window  : {win_ms} ms = {win_samp} samples  |  "
          f"Step : {step_ms} ms = {step_samp} samples")

    # ── Envelope ──────────────────────────────────────────────────────────────
    env   = moving_rms(emg, ENV_WIN)
    base  = os.path.splitext(in_path)[0]

    env_header = ["t_ms"] + [f"env{ch}" for ch in range(8)]
    env_rows   = [[int(t_ms[i])] + [round(env[i, ch], 4) for ch in range(8)]
                  for i in range(n_samp)]
    env_path   = base + "_envelope.csv"
    write_csv(env_path, env_header, env_rows)
    print(f"\nEnvelope: {n_samp} rows  -> {env_path}")

    # ── Features ──────────────────────────────────────────────────────────────
    feat_rows   = extract_features(t_ms, emg, win_samp, step_samp)
    feat_header = (["t_ms"]
                   + [f"mav{ch}" for ch in range(8)]
                   + [f"rms{ch}" for ch in range(8)]
                   + [f"zc{ch}"  for ch in range(8)]
                   + [f"wl{ch}"  for ch in range(8)])
    feat_path   = base + "_features.csv"
    write_csv(feat_path, feat_header, feat_rows)
    print(f"Features: {len(feat_rows)} windows -> {feat_path}\n")


if __name__ == "__main__":
    main()
