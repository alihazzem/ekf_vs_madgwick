"""
xcorr_channel_matrix.py

Build full cross-channel correlation matrices for one trial.

Computes, for every pair of channels (0..7):
- normalized peak correlation
- normalized zero-lag correlation
- normalized peak lag
- raw/textbook peak correlation
- raw/textbook zero-lag correlation
- raw/textbook peak lag

Useful for studying spatial coupling between EMG channels
within the same trial.

Example
-------
python xcorr_channel_matrix.py --gesture FIST --session S01 --trial 2 --stage env --start 250 --window 1000 --max-lag 150
"""

import argparse
import csv
import math
import os


DEFAULT_DATA_DIR = "../emg_data"
DEFAULT_OUT_DIR = "../correlation_results"
DEFAULT_LABELS = "trial_labels.csv"


def parse_args():
    p = argparse.ArgumentParser(description="Cross-channel EMG correlation matrix for one trial.")
    p.add_argument("--data-dir", default=DEFAULT_DATA_DIR, help="Data directory")
    p.add_argument("--out-dir", default=DEFAULT_OUT_DIR, help="Output directory")
    p.add_argument("--labels", default=DEFAULT_LABELS, help="trial labels CSV")

    p.add_argument("--gesture", default="FIST", help="Gesture label")
    p.add_argument("--session", default="S01", help="Session ID")
    p.add_argument("--trial", type=int, required=True, help="Trial ID")

    p.add_argument(
        "--stage",
        choices=["raw", "hp", "rect", "env"],
        default="env",
        help="Signal stage",
    )

    p.add_argument("--channels", default="0,1,2,3,4,5,6,7", help="Comma-separated channel indices")
    p.add_argument("--start", type=int, default=0, help="Start sample")
    p.add_argument("--window", type=int, default=0, help="Window length (0 = to end)")
    p.add_argument("--max-lag", type=int, default=150, help="Maximum lag searched")
    p.add_argument("--min-overlap-frac", type=float, default=0.40, help="Minimum overlap fraction")

    p.add_argument("--prefix", default="channel_matrix", help="Output filename prefix")
    return p.parse_args()


def parse_channels(text):
    out = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        ch = int(part)
        if ch < 0 or ch > 7:
            raise ValueError(f"Channel out of range [0..7]: {ch}")
        out.append(ch)
    out = sorted(set(out))
    if not out:
        raise ValueError("No channels selected")
    return out


def stage_prefix(stage):
    return {
        "raw": "emg_raw",
        "hp": "emg_hp",
        "rect": "emg_rect",
        "env": "emg_env",
    }[stage]


def read_csv_dict(path):
    with open(path, "r", newline="") as f:
        r = csv.DictReader(f)
        if r.fieldnames is None:
            raise ValueError(f"CSV has no header: {path}")
        return r.fieldnames, list(r)


def safe_float(x):
    try:
        v = float(x)
        if math.isfinite(v):
            return v
        return None
    except Exception:
        return None


def load_signal(path, col):
    header, rows = read_csv_dict(path)
    if col not in header:
        raise ValueError(f"Column {col} missing in {path}")

    sig = []
    for r in rows:
        v = safe_float(r.get(col, ""))
        if v is not None:
            sig.append(v)

    if len(sig) < 50:
        raise ValueError(f"Too few samples in {path} for {col}")
    return sig


def select_segment(x, start, window):
    n = len(x)
    s = int(start)
    w = int(window)

    if s < 0 or s >= n:
        raise ValueError("Segment start out of range")
    if w < 0:
        raise ValueError("Segment window must be >= 0")

    e = n if w == 0 else min(n, s + w)
    seg = x[s:e]
    if len(seg) < 50:
        raise ValueError("Segment too short (<50 samples)")
    return seg, s, e


def find_trial_row(label_rows, gesture, session, trial_id):
    g = gesture.strip().upper()
    s = session.strip()
    tid = str(trial_id)

    candidates = []
    for r in label_rows:
        if (r.get("gesture_label") or "").strip().upper() != g:
            continue
        if s and (r.get("session_id") or "").strip() != s:
            continue
        if (r.get("trial_id") or "").strip() != tid:
            continue
        candidates.append(r)

    if not candidates:
        raise ValueError(f"No trial found for gesture={g}, session={s}, trial_id={tid}")
    if len(candidates) > 1:
        raise ValueError(f"Ambiguous trial selection for gesture={g}, session={s}, trial_id={tid}")
    return candidates[0]


def normalized_corr(a, b):
    n = len(a)
    if n < 2:
        return None

    ma = sum(a) / n
    mb = sum(b) / n
    num = 0.0
    da = 0.0
    db = 0.0

    for xa, xb in zip(a, b):
        va = xa - ma
        vb = xb - mb
        num += va * vb
        da += va * va
        db += vb * vb

    den = math.sqrt(da * db)
    if den <= 1e-12:
        return None
    return num / den


def raw_corr_mean_normalized(a, b):
    """
    Mean-normalized dot product:
        sum(a[n] * b[n]) / overlap_length

    This matches the interpretation you were using for raw cross-channel results.
    """
    n = len(a)
    if n < 1:
        return None
    s = 0.0
    for xa, xb in zip(a, b):
        s += xa * xb
    return s / n


def xcorr_curve(a, b, max_lag, min_overlap, mode="norm"):
    """
    lag > 0 means signal B is delayed relative to signal A.
    """
    na = len(a)
    nb = len(b)
    out = []

    for lag in range(-max_lag, max_lag + 1):
        a_start = max(0, lag)
        a_end = min(na - 1, nb - 1 + lag)

        overlap = a_end - a_start + 1
        if overlap < min_overlap:
            continue

        b_start = a_start - lag
        b_end = a_end - lag

        seg_a = a[a_start : a_end + 1]
        seg_b = b[b_start : b_end + 1]

        if mode == "norm":
            c = normalized_corr(seg_a, seg_b)
        elif mode == "raw":
            c = raw_corr_mean_normalized(seg_a, seg_b)
        else:
            raise ValueError("Unknown mode")

        if c is None:
            continue
        out.append((lag, c))

    if not out:
        return [], None, None, None

    peak_lag, peak_corr = max(out, key=lambda t: t[1])

    zero_lag = None
    for lag, corr in out:
        if lag == 0:
            zero_lag = corr
            break

    return out, peak_lag, peak_corr, zero_lag


def write_matrix_csv(path, channels, matrix):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["channel"] + [f"ch{c}" for c in channels])
        for i, ch in enumerate(channels):
            row = [f"ch{ch}"]
            for j in range(len(channels)):
                row.append(matrix[i][j])
            w.writerow(row)


def main():
    args = parse_args()
    channels = parse_channels(args.channels)

    if args.max_lag <= 0:
        raise ValueError("max-lag must be > 0")
    if not (0.0 < args.min_overlap_frac <= 1.0):
        raise ValueError("min-overlap-frac must be in (0,1]")

    prefix = stage_prefix(args.stage)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.normpath(os.path.join(script_dir, args.data_dir))
    out_dir = os.path.normpath(os.path.join(script_dir, args.out_dir))
    labels_path = os.path.join(data_dir, args.labels)

    os.makedirs(out_dir, exist_ok=True)

    if not os.path.exists(labels_path):
        raise FileNotFoundError(f"Labels file not found: {labels_path}")

    header, label_rows = read_csv_dict(labels_path)
    required = ["timestamp", "gesture_label", "trial_id", "session_id", "emg_online_file"]
    miss = [c for c in required if c not in header]
    if miss:
        raise ValueError(f"Labels file missing columns: {', '.join(miss)}")

    row = find_trial_row(label_rows, args.gesture, args.session, args.trial)
    data_file = os.path.join(data_dir, (row.get("emg_online_file") or "").strip())

    if not os.path.exists(data_file):
        raise FileNotFoundError(f"Trial file missing: {data_file}")

    # Load all selected channel segments once
    signals = {}
    for ch in channels:
        col = f"{prefix}{ch}"
        sig = load_signal(data_file, col)
        seg, s, e = select_segment(sig, args.start, args.window)
        signals[ch] = seg

    n = len(channels)

    peak_corr_norm = [[None] * n for _ in range(n)]
    zero_lag_norm = [[None] * n for _ in range(n)]
    peak_lag_norm = [[None] * n for _ in range(n)]

    peak_corr_raw = [[None] * n for _ in range(n)]
    zero_lag_raw = [[None] * n for _ in range(n)]
    peak_lag_raw = [[None] * n for _ in range(n)]

    overlap_min = max(2, int(round(args.min_overlap_frac * len(next(iter(signals.values()))))))

    for i, ch_i in enumerate(channels):
        for j, ch_j in enumerate(channels):
            a = signals[ch_i]
            b = signals[ch_j]

            _, lag_n, corr_n, zero_n = xcorr_curve(a, b, args.max_lag, overlap_min, mode="norm")
            _, lag_r, corr_r, zero_r = xcorr_curve(a, b, args.max_lag, overlap_min, mode="raw")

            peak_corr_norm[i][j] = "" if corr_n is None else f"{corr_n:.6f}"
            zero_lag_norm[i][j] = "" if zero_n is None else f"{zero_n:.6f}"
            peak_lag_norm[i][j] = "" if lag_n is None else lag_n

            peak_corr_raw[i][j] = "" if corr_r is None else f"{corr_r:.6f}"
            zero_lag_raw[i][j] = "" if zero_r is None else f"{zero_r:.6f}"
            peak_lag_raw[i][j] = "" if lag_r is None else lag_r

    base = f"{args.prefix}_{args.gesture.upper()}_{args.session}_trial{args.trial}_{args.stage}"

    write_matrix_csv(os.path.join(out_dir, f"{base}_peak_corr_norm.csv"), channels, peak_corr_norm)
    write_matrix_csv(os.path.join(out_dir, f"{base}_zero_lag_norm.csv"), channels, zero_lag_norm)
    write_matrix_csv(os.path.join(out_dir, f"{base}_peak_lag_norm.csv"), channels, peak_lag_norm)

    write_matrix_csv(os.path.join(out_dir, f"{base}_peak_corr_raw.csv"), channels, peak_corr_raw)
    write_matrix_csv(os.path.join(out_dir, f"{base}_zero_lag_raw.csv"), channels, zero_lag_raw)
    write_matrix_csv(os.path.join(out_dir, f"{base}_peak_lag_raw.csv"), channels, peak_lag_raw)

    summary_path = os.path.join(out_dir, f"{base}_summary.csv")
    with open(summary_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric", "value"])
        w.writerow(["gesture", args.gesture.upper()])
        w.writerow(["session", args.session])
        w.writerow(["trial", args.trial])
        w.writerow(["timestamp", row.get("timestamp", "")])
        w.writerow(["stage", args.stage])
        w.writerow(["channels", ",".join(str(c) for c in channels)])
        w.writerow(["start", args.start])
        w.writerow(["window", args.window])
        w.writerow(["max_lag", args.max_lag])
        w.writerow(["min_overlap_frac", args.min_overlap_frac])
        w.writerow(["overlap_min", overlap_min])
        w.writerow(["mode", "cross-channel matrix (same trial, all channel pairs)"])
        w.writerow(["norm_method", "Pearson-style lagged normalized correlation"])
        w.writerow(["raw_method", "mean-normalised dot product / overlap_length"])

    print("\nCross-channel matrix complete")
    print(f"Gesture/session : {args.gesture.upper()} / {args.session}")
    print(f"Trial           : {args.trial}")
    print(f"Stage           : {args.stage}")
    print(f"Window          : start={args.start}, len={args.window}")
    print(f"Channels        : {channels}")
    print(f"Summary         : {summary_path}")
    print(f"Saved 6 matrix CSV files to: {out_dir}")


if __name__ == "__main__":
    main()