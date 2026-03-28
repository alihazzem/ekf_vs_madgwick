"""
process_fusion_emg.py  —  EMG processing for merged fusion CSV files.

Purpose
-------
Process only the EMG side of fusion_merged_*.csv while keeping IMU columns
unchanged. Produces:
  1) *_emgproc.csv : original rows + processed EMG signals
  2) *_emgfeat.csv : windowed EMG features (+ optional roll/pitch summaries)

Expected input
--------------
fusion_merged_*.csv from scripts/merge_fusion.py, containing at least:
  pc_t_ms, emg0..emg7
and optionally:
  imu_match, imu_ekf_roll_mdeg, imu_ekf_pitch_mdeg,
  imu_mad_roll_mdeg, imu_mad_pitch_mdeg

Defaults
--------
- High-pass filter: 20 Hz (first-order)
- Notch filter: disabled (set --notch-hz to 50 or 60 to enable)
- RMS envelope window: 150 ms (causal)
- Feature window/step: 100 ms / 50 ms
- Angle source for summaries: EKF

Usage
-----
python process_fusion_emg.py
python process_fusion_emg.py --in ../emg_data/fusion_merged_20260328_124706.csv
python process_fusion_emg.py --win-ms 120 --step-ms 60 --env-ms 200
python process_fusion_emg.py --notch-hz 50 --notch-q 30
python process_fusion_emg.py --angle-src MAD
"""

import argparse
import csv
import glob
import math
import os
from bisect import bisect_left

import numpy as np

DATA_DIR = "../emg_data"
MERGED_GLOB = "fusion_merged_*.csv"


def parse_args():
    p = argparse.ArgumentParser(description="Process EMG in merged fusion CSV files.")
    p.add_argument("--in", dest="in_path", default="", help="Input fusion_merged CSV path")
    p.add_argument("--out-proc", default="", help="Output processed time-series CSV path")
    p.add_argument("--out-feat", default="", help="Output feature CSV path")
    p.add_argument("--win-ms", type=float, default=100.0, help="Feature window size in ms")
    p.add_argument("--step-ms", type=float, default=50.0, help="Feature step in ms")
    p.add_argument("--env-ms", type=float, default=150.0, help="Causal RMS envelope window in ms")
    p.add_argument("--hp-hz", type=float, default=20.0, help="High-pass cutoff in Hz")
    p.add_argument("--notch-hz", type=float, default=0.0, help="Notch frequency in Hz (0 disables)")
    p.add_argument("--notch-q", type=float, default=30.0, help="Notch quality factor")
    p.add_argument("--zc-th", type=float, default=5.0, help="Zero-crossing dead-zone threshold")
    p.add_argument("--angle-src", choices=["EKF", "MAD"], default="EKF", help="Angle source for roll/pitch window summaries")
    return p.parse_args()


def find_latest_merged(script_dir):
    data_dir = os.path.normpath(os.path.join(script_dir, DATA_DIR))
    paths = sorted(glob.glob(os.path.join(data_dir, MERGED_GLOB)))
    if not paths:
        raise FileNotFoundError(f"No merged files found in {data_dir} matching {MERGED_GLOB}")
    return paths[-1]


def estimate_fs_hz(t_ms):
    dt = np.diff(t_ms)
    dt_pos = dt[dt > 0]
    if len(dt_pos) == 0:
        return 200.0
    return 1000.0 / float(np.mean(dt_pos))


def highpass_first_order(x, fs_hz, fc_hz):
    if fc_hz <= 0 or fs_hz <= 0:
        return x.copy()

    dt = 1.0 / fs_hz
    rc = 1.0 / (2.0 * math.pi * fc_hz)
    alpha = rc / (rc + dt)

    y = np.zeros_like(x, dtype=float)
    y[0, :] = x[0, :]
    for i in range(1, x.shape[0]):
        y[i, :] = alpha * (y[i - 1, :] + x[i, :] - x[i - 1, :])
    return y


def notch_biquad(x, fs_hz, f0_hz, q):
    if f0_hz <= 0 or fs_hz <= 0 or f0_hz >= fs_hz * 0.5:
        return x.copy()

    w0 = 2.0 * math.pi * f0_hz / fs_hz
    alpha = math.sin(w0) / (2.0 * q)

    b0 = 1.0
    b1 = -2.0 * math.cos(w0)
    b2 = 1.0
    a0 = 1.0 + alpha
    a1 = -2.0 * math.cos(w0)
    a2 = 1.0 - alpha

    b0 /= a0
    b1 /= a0
    b2 /= a0
    a1 /= a0
    a2 /= a0

    y = np.zeros_like(x, dtype=float)
    for ch in range(x.shape[1]):
        x1 = 0.0
        x2 = 0.0
        y1 = 0.0
        y2 = 0.0
        for n in range(x.shape[0]):
            xn = float(x[n, ch])
            yn = b0 * xn + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
            y[n, ch] = yn
            x2 = x1
            x1 = xn
            y2 = y1
            y1 = yn
    return y


def moving_rms_causal(x, win_samp):
    win_samp = max(1, int(win_samp))
    n, c = x.shape
    env = np.zeros((n, c), dtype=float)

    x2 = x * x
    csum = np.cumsum(x2, axis=0)
    for i in range(n):
        start = i - win_samp + 1
        if start <= 0:
            s = csum[i, :]
            length = i + 1
        else:
            s = csum[i, :] - csum[start - 1, :]
            length = win_samp
        env[i, :] = np.sqrt(s / float(length))
    return env


def zc_deadzone(x, thresh):
    count = 0
    for i in range(len(x) - 1):
        a = x[i]
        b = x[i + 1]
        if (a * b < 0) and (abs(a) >= thresh) and (abs(b) >= thresh):
            count += 1
    return count


def safe_float(v):
    try:
        return float(v)
    except Exception:
        return float("nan")


def nearest_index(sorted_values, target):
    pos = bisect_left(sorted_values, target)
    if pos <= 0:
        return 0
    if pos >= len(sorted_values):
        return len(sorted_values) - 1
    left = pos - 1
    right = pos
    if abs(target - sorted_values[left]) <= abs(sorted_values[right] - target):
        return left
    return right


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for row in rows:
            out = []
            for v in row:
                if isinstance(v, (np.floating, float)):
                    if math.isnan(float(v)):
                        out.append("")
                    else:
                        out.append(f"{float(v):.6f}")
                elif isinstance(v, (np.integer, int)):
                    out.append(int(v))
                else:
                    out.append(v)
            w.writerow(out)


def main():
    args = parse_args()
    script_dir = os.path.dirname(os.path.abspath(__file__))

    in_path = os.path.normpath(args.in_path) if args.in_path else find_latest_merged(script_dir)
    if not os.path.exists(in_path):
        raise FileNotFoundError(f"Input file not found: {in_path}")

    with open(in_path, "r", newline="") as f:
        rdr = csv.DictReader(f)
        if rdr.fieldnames is None:
            raise ValueError("Input CSV has no header")
        header = list(rdr.fieldnames)
        rows = list(rdr)

    if len(rows) < 2:
        raise ValueError("Input CSV must have at least 2 data rows")

    required = ["pc_t_ms"] + [f"emg{i}" for i in range(8)]
    missing = [c for c in required if c not in header]
    if missing:
        raise ValueError(f"Missing required columns: {', '.join(missing)}")

    t_ms = np.array([safe_float(r["pc_t_ms"]) for r in rows], dtype=float)
    if np.any(np.diff(t_ms) < 0):
        raise ValueError("pc_t_ms must be non-decreasing")

    emg_raw = np.array([[safe_float(r[f"emg{i}"]) for i in range(8)] for r in rows], dtype=float)

    fs_hz = estimate_fs_hz(t_ms)
    win_samp = max(1, int(round(args.win_ms * fs_hz / 1000.0)))
    step_samp = max(1, int(round(args.step_ms * fs_hz / 1000.0)))
    env_samp = max(1, int(round(args.env_ms * fs_hz / 1000.0)))

    # EMG processing
    emg_dc = emg_raw - np.mean(emg_raw, axis=0, keepdims=True)
    emg_hp = highpass_first_order(emg_dc, fs_hz, args.hp_hz)
    if args.notch_hz > 0:
        emg_filt = notch_biquad(emg_hp, fs_hz, args.notch_hz, args.notch_q)
    else:
        emg_filt = emg_hp
    emg_rect = np.abs(emg_filt)
    emg_env = moving_rms_causal(emg_filt, env_samp)

    # Output path resolution
    base = os.path.splitext(in_path)[0]
    out_proc = os.path.normpath(args.out_proc) if args.out_proc else base + "_emgproc.csv"
    out_feat = os.path.normpath(args.out_feat) if args.out_feat else base + "_emgfeat.csv"

    # Processed time-series output (preserve all original columns + EMG processed columns)
    proc_header = header[:]
    proc_header += [f"emg_dc{i}" for i in range(8)]
    proc_header += [f"emg_hp{i}" for i in range(8)]
    proc_header += [f"emg_rect{i}" for i in range(8)]
    proc_header += [f"emg_env{i}" for i in range(8)]

    proc_rows = []
    for i, r in enumerate(rows):
        row_out = [r[c] for c in header]
        row_out += emg_dc[i, :].tolist()
        row_out += emg_filt[i, :].tolist()
        row_out += emg_rect[i, :].tolist()
        row_out += emg_env[i, :].tolist()
        proc_rows.append(row_out)

    write_csv(out_proc, proc_header, proc_rows)

    # Feature output
    roll_col = "imu_ekf_roll_mdeg" if args.angle_src == "EKF" else "imu_mad_roll_mdeg"
    pitch_col = "imu_ekf_pitch_mdeg" if args.angle_src == "EKF" else "imu_mad_pitch_mdeg"

    has_roll = roll_col in header
    has_pitch = pitch_col in header
    has_match = "imu_match" in header

    if has_roll:
        roll_deg = np.array([safe_float(r[roll_col]) for r in rows], dtype=float) / 1000.0
    else:
        roll_deg = np.full((len(rows),), np.nan)

    if has_pitch:
        pitch_deg = np.array([safe_float(r[pitch_col]) for r in rows], dtype=float) / 1000.0
    else:
        pitch_deg = np.full((len(rows),), np.nan)

    if has_match:
        imu_match = np.array([safe_float(r["imu_match"]) > 0.5 for r in rows], dtype=float)
    else:
        imu_match = np.ones((len(rows),), dtype=float)

    feat_header = ["t_center_ms", "fs_hz", "angle_src"]
    feat_header += [f"mav{i}" for i in range(8)]
    feat_header += [f"rms{i}" for i in range(8)]
    feat_header += [f"zc{i}" for i in range(8)]
    feat_header += [f"wl{i}" for i in range(8)]
    feat_header += [f"env_mean{i}" for i in range(8)]
    feat_header += [
        "roll_mean_deg", "roll_std_deg", "roll_slope_degps",
        "pitch_mean_deg", "pitch_std_deg", "pitch_slope_degps",
        "imu_match_ratio"
    ]

    feat_rows = []
    n = len(rows)
    for s in range(0, n - win_samp + 1, step_samp):
        e = s + win_samp

        w = emg_filt[s:e, :]
        w_env = emg_env[s:e, :]

        mav = np.mean(np.abs(w), axis=0)
        rms = np.sqrt(np.mean(w * w, axis=0))
        wl = np.sum(np.abs(np.diff(w, axis=0)), axis=0)

        zc = np.zeros((8,), dtype=float)
        for ch in range(8):
            zc[ch] = float(zc_deadzone(w[:, ch], args.zc_th))

        env_mean = np.mean(w_env, axis=0)

        t_center = float(0.5 * (t_ms[s] + t_ms[e - 1]))

        wr = roll_deg[s:e]
        wp = pitch_deg[s:e]
        wt = t_ms[s:e] / 1000.0

        roll_mean = float(np.nanmean(wr)) if np.any(~np.isnan(wr)) else float("nan")
        roll_std = float(np.nanstd(wr)) if np.any(~np.isnan(wr)) else float("nan")
        pitch_mean = float(np.nanmean(wp)) if np.any(~np.isnan(wp)) else float("nan")
        pitch_std = float(np.nanstd(wp)) if np.any(~np.isnan(wp)) else float("nan")

        if (wt[-1] - wt[0]) > 0:
            if np.any(~np.isnan(wr)):
                r0 = wr[0] if not np.isnan(wr[0]) else roll_mean
                r1 = wr[-1] if not np.isnan(wr[-1]) else roll_mean
                roll_slope = float((r1 - r0) / (wt[-1] - wt[0]))
            else:
                roll_slope = float("nan")

            if np.any(~np.isnan(wp)):
                p0 = wp[0] if not np.isnan(wp[0]) else pitch_mean
                p1 = wp[-1] if not np.isnan(wp[-1]) else pitch_mean
                pitch_slope = float((p1 - p0) / (wt[-1] - wt[0]))
            else:
                pitch_slope = float("nan")
        else:
            roll_slope = float("nan")
            pitch_slope = float("nan")

        match_ratio = float(np.mean(imu_match[s:e]))

        row = [t_center, fs_hz, args.angle_src]
        row += mav.tolist()
        row += rms.tolist()
        row += zc.tolist()
        row += wl.tolist()
        row += env_mean.tolist()
        row += [
            roll_mean, roll_std, roll_slope,
            pitch_mean, pitch_std, pitch_slope,
            match_ratio,
        ]
        feat_rows.append(row)

    write_csv(out_feat, feat_header, feat_rows)

    print(f"Input      : {in_path}")
    print(f"Samples    : {len(rows)}")
    print(f"Rate (est) : {fs_hz:.2f} Hz")
    print(f"HPF        : {args.hp_hz:.2f} Hz")
    if args.notch_hz > 0:
        print(f"Notch      : {args.notch_hz:.2f} Hz (Q={args.notch_q:.1f})")
    else:
        print("Notch      : disabled")
    print(f"Win/Step   : {args.win_ms:.1f}/{args.step_ms:.1f} ms ({win_samp}/{step_samp} samples)")
    print(f"Env window : {args.env_ms:.1f} ms ({env_samp} samples)")
    print(f"Out (proc) : {out_proc}")
    print(f"Out (feat) : {out_feat}")
    print(f"Feature rows: {len(feat_rows)}")


if __name__ == "__main__":
    main()
