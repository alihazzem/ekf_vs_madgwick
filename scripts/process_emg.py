"""
process_emg.py  —  Baseline signal-processing pipeline for raw Myo EMG CSV.

What this script does
---------------------
1) Load raw 8-channel EMG CSV
2) Remove per-channel DC offset
3) Compute causal moving-RMS envelope
4) Extract sliding-window features:
      MAV  = Mean Absolute Value
      RMS  = Root Mean Square
      ZC   = Zero Crossing count with dead-zone
      WL   = Waveform Length

Input CSV format
----------------
Expected columns:
    t_ms, ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7

- t_ms must be in milliseconds
- ch0..ch7 are raw EMG samples

Usage
-----
python process_emg.py
python process_emg.py my_capture.csv
python process_emg.py my_capture.csv 100 50
python process_emg.py my_capture.csv 120 60 250

Arguments
---------
arg1: input csv filename or full path
arg2: feature window length in ms      (default = 100)
arg3: feature step in ms               (default = 50)
arg4: envelope smoothing window in ms  (default = 250)

Default parameters
------------------
WIN_MS      = 100 ms
STEP_MS     = 50 ms
ENV_MS      = 250 ms
ZC_THRESH   = 5 raw counts
FS_NOM      = 200 Hz (used only as fallback)

Outputs
-------
<name>_envelope.csv
    columns: t_ms, env0..env7

<name>_features.csv
    columns:
      t_ms,
      mav0..mav7,
      rms0..rms7,
      zc0..zc7,
      wl0..wl7

Dependencies
------------
pip install numpy
"""

import sys
import os
import csv
import numpy as np

# ── Default parameters ────────────────────────────────────────────────────────
WIN_MS      = 100      # feature window length (ms)
STEP_MS     = 50       # feature step (ms)
ENV_MS      = 250      # envelope RMS smoothing window (ms)
ZC_THRESH   = 5        # zero-crossing dead-zone threshold (raw counts)
FS_NOM      = 200.0    # nominal sample rate fallback (Hz)
EMG_DIR     = "../emg_data"
# ─────────────────────────────────────────────────────────────────────────────


def load_csv(path: str):
    """
    Return:
        t_ms : shape [N]
        emg  : shape [N, 8] as float
        fs   : measured sampling rate (Hz)

    Accepts non-decreasing timestamps.
    Rejects only backward time steps.
    """
    try:
        data = np.loadtxt(path, delimiter=",", skiprows=1)
    except Exception as e:
        raise RuntimeError(f"Failed to read CSV: {e}")

    if data.ndim == 1:
        data = data.reshape(1, -1)

    if data.shape[0] < 2:
        raise ValueError("CSV must contain at least 2 data rows.")

    if data.shape[1] < 9:
        raise ValueError(
            f"CSV must contain at least 9 columns: t_ms + 8 EMG channels. "
            f"Found {data.shape[1]} columns."
        )

    t_ms = data[:, 0].astype(float)
    emg = data[:, 1:9].astype(float)

    dt = np.diff(t_ms)

    # Allow repeated timestamps, reject only backward steps
    if np.any(dt < 0):
        raise ValueError("t_ms must be non-decreasing (found backward timestamp step).")

    # Estimate fs from positive intervals only
    dt_pos = dt[dt > 0]
    if len(dt_pos) == 0:
        fs = FS_NOM
    else:
        fs = 1000.0 / np.mean(dt_pos)

    return t_ms, emg, fs
    """
    Return:
        t_ms : shape [N]
        emg  : shape [N, 8] as float
        fs   : measured sampling rate (Hz)

    Validates:
    - file is non-empty
    - at least 9 columns exist
    - timestamps are strictly increasing
    """
    try:
        data = np.loadtxt(path, delimiter=",", skiprows=1)
    except Exception as e:
        raise RuntimeError(f"Failed to read CSV: {e}")

    if data.ndim == 1:
        # Single-row file becomes 1D after loadtxt
        data = data.reshape(1, -1)

    if data.shape[0] < 2:
        raise ValueError("CSV must contain at least 2 data rows.")

    if data.shape[1] < 9:
        raise ValueError(
            f"CSV must contain at least 9 columns: t_ms + 8 EMG channels. "
            f"Found {data.shape[1]} columns."
        )

    t_ms = data[:, 0].astype(float)
    emg = data[:, 1:9].astype(float)

    dt = np.diff(t_ms)
    if np.any(dt <= 0):
        raise ValueError("t_ms must be strictly increasing.")

    dur_ms = t_ms[-1] - t_ms[0]
    if dur_ms <= 0:
        fs = FS_NOM
    else:
        fs = 1000.0 * (len(t_ms) - 1) / dur_ms

    return t_ms, emg, fs


def remove_dc(emg: np.ndarray) -> np.ndarray:
    """
    Remove per-channel mean (simple DC offset removal).
    """
    return emg - np.mean(emg, axis=0, keepdims=True)


def moving_rms_causal(emg: np.ndarray, win: int) -> np.ndarray:
    """
    True causal moving RMS.
    Each output sample uses only current and past samples.

    For sample i:
        env[i] = sqrt(mean(emg[max(0, i-win+1):i+1]^2))

    Input:
        emg: [N, C]
        win: window length in samples

    Output:
        env: [N, C]
    """
    N, C = emg.shape
    env = np.zeros((N, C), dtype=float)

    for ch in range(C):
        x2 = emg[:, ch] ** 2
        csum = np.cumsum(x2)

        for i in range(N):
            start = max(0, i - win + 1)
            if start == 0:
                s = csum[i]
            else:
                s = csum[i] - csum[start - 1]

            length = i - start + 1
            env[i, ch] = np.sqrt(s / length)

    return env


def zero_crossings_with_deadzone(x: np.ndarray, thresh: float) -> int:
    """
    Count zero crossings using a dead-zone threshold.

    A crossing is counted if:
      - the sign changes between adjacent samples
      - BOTH samples are outside the dead-zone threshold

    This is more conservative than using OR and better suppresses noise.

    Example:
      ... -8, +10 ...  counts if thresh <= 8 and <= 10
      ... -1, +12 ...  does not count if thresh = 5
    """
    count = 0
    for i in range(len(x) - 1):
        a = x[i]
        b = x[i + 1]

        if (a * b < 0) and (abs(a) >= thresh) and (abs(b) >= thresh):
            count += 1

    return count


def extract_features(
    t_ms: np.ndarray,
    emg: np.ndarray,
    win_samp: int,
    step_samp: int,
    zc_thresh: float
):
    """
    Sliding-window feature extraction.

    Returns rows of:
      [t_center_ms,
       mav0..mav7,
       rms0..rms7,
       zc0..zc7,
       wl0..wl7]
    """
    N, C = emg.shape
    rows = []

    for start in range(0, N - win_samp + 1, step_samp):
        end = start + win_samp
        w = emg[start:end, :]                 # [win_samp, C]
        t_center = int(round((t_ms[start] + t_ms[end - 1]) / 2.0))

        mav = np.mean(np.abs(w), axis=0)
        rms = np.sqrt(np.mean(w ** 2, axis=0))
        wl = np.sum(np.abs(np.diff(w, axis=0)), axis=0)

        zc = np.zeros(C, dtype=int)
        for ch in range(C):
            zc[ch] = zero_crossings_with_deadzone(w[:, ch], zc_thresh)

        row = [t_center]
        row.extend(mav.tolist())
        row.extend(rms.tolist())
        row.extend(zc.tolist())
        row.extend(wl.tolist())
        rows.append(row)

    return rows


def write_csv(path: str, header: list, rows):
    """
    Write rows to CSV with consistent float formatting.
    """
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)

        for row in rows:
            out = []
            for v in row:
                if isinstance(v, (float, np.floating)):
                    out.append(f"{float(v):.4f}")
                elif isinstance(v, (int, np.integer)):
                    out.append(int(v))
                else:
                    out.append(v)
            writer.writerow(out)


def resolve_input_file(script_dir: str) -> str:
    """
    Resolve input file from:
    - command line argument
    - or interactive selection from EMG_DIR
    """
    emg_dir = os.path.normpath(os.path.join(script_dir, EMG_DIR))

    if len(sys.argv) > 1:
        arg = sys.argv[1]
        candidate = os.path.join(emg_dir, arg if arg.endswith(".csv") else arg + ".csv")
        return candidate if os.path.exists(candidate) else arg

    if not os.path.isdir(emg_dir):
        raise FileNotFoundError(f"EMG directory not found: {emg_dir}")

    available = sorted([f for f in os.listdir(emg_dir) if f.endswith(".csv")])

    if not available:
        raise FileNotFoundError(
            f"No CSV files found in {emg_dir}\n"
            f"Run your capture script first."
        )

    print("Available EMG captures:")
    for i, fname in enumerate(available):
        print(f"  [{i}] {fname}")

    choice = input("Select file number (or press Enter for 0): ").strip()
    idx = int(choice) if choice.isdigit() else 0

    if idx < 0 or idx >= len(available):
        raise IndexError(f"Invalid selection index: {idx}")

    return os.path.join(emg_dir, available[idx])


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))

    try:
        in_path = resolve_input_file(script_dir)
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    if not os.path.exists(in_path):
        print(f"ERROR: file not found — {in_path}")
        sys.exit(1)

    # Command-line parameters
    win_ms = int(sys.argv[2]) if len(sys.argv) > 2 else WIN_MS
    step_ms = int(sys.argv[3]) if len(sys.argv) > 3 else STEP_MS
    env_ms = int(sys.argv[4]) if len(sys.argv) > 4 else ENV_MS

    if win_ms <= 0 or step_ms <= 0 or env_ms <= 0:
        print("ERROR: win_ms, step_ms, and env_ms must all be > 0.")
        sys.exit(1)

    try:
        t_ms, emg_raw, fs = load_csv(in_path)
    except Exception as e:
        print(f"ERROR while loading CSV: {e}")
        sys.exit(1)

    n_samp = len(t_ms)
    dur_s = (t_ms[-1] - t_ms[0]) / 1000.0

    # Basic preprocessing
    emg = remove_dc(emg_raw)

    # Convert time windows to sample counts using measured fs
    win_samp = max(1, int(round(win_ms * fs / 1000.0)))
    step_samp = max(1, int(round(step_ms * fs / 1000.0)))
    env_samp = max(1, int(round(env_ms * fs / 1000.0)))

    print(f"\nInput    : {in_path}")
    print(f"Samples  : {n_samp}")
    print(f"Duration : {dur_s:.2f} s")
    print(f"Rate     : {fs:.2f} Hz")
    print(f"Feature window : {win_ms} ms -> {win_samp} samples")
    print(f"Feature step   : {step_ms} ms -> {step_samp} samples")
    print(f"Envelope win   : {env_ms} ms -> {env_samp} samples")
    print(f"ZC threshold   : {ZC_THRESH} counts")

    base = os.path.splitext(in_path)[0]

    # Envelope
    env = moving_rms_causal(emg, env_samp)
    env_header = ["t_ms"] + [f"env{ch}" for ch in range(8)]
    env_rows = [
        [int(round(t_ms[i]))] + env[i, :].tolist()
        for i in range(n_samp)
    ]
    env_path = base + "_envelope.csv"
    write_csv(env_path, env_header, env_rows)
    print(f"\nEnvelope: {n_samp} rows -> {env_path}")

    # Features
    feat_rows = extract_features(
        t_ms=t_ms,
        emg=emg,
        win_samp=win_samp,
        step_samp=step_samp,
        zc_thresh=ZC_THRESH
    )

    feat_header = (
        ["t_ms"]
        + [f"mav{ch}" for ch in range(8)]
        + [f"rms{ch}" for ch in range(8)]
        + [f"zc{ch}" for ch in range(8)]
        + [f"wl{ch}" for ch in range(8)]
    )

    feat_path = base + "_features.csv"
    write_csv(feat_path, feat_header, feat_rows)
    print(f"Features: {len(feat_rows)} windows -> {feat_path}\n")


if __name__ == "__main__":
    main()