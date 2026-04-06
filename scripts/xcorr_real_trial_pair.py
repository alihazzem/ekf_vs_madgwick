"""
xcorr_real_trial_pair_both.py  (CORRECTED)

Realistic cross-correlation between two captured trials (no synthetic shifting).

CORRECTIONS vs original:
  1) raw_corr now divides by overlap length so the score does not shrink
     artificially at large lags (was biased toward lag=0 before).
  2) normalized_corr now uses the textbook / MATLAB-xcorr definition:
       - global means subtracted once (not per-window)
       - denominator is sqrt(R_aa(0) * R_bb(0)), i.e. geometric mean of the
         two zero-lag autocorrelations, guaranteeing output in [-1, 1] and
         consistent scale across all lags.
  3) Both xcorr_curve_* functions now receive precomputed global statistics
     so they don't recompute them inside the lag loop.

Computes BOTH:
  1) Mean-normalised cross-correlation  (raw sum of products / overlap length)
  2) Standard normalised cross-correlation (Pearson / MATLAB-style)

Outputs (in ../correlation_results by default):
  - summary CSV        : overall metrics and selected windows
  - by-channel CSV     : peak lag/correlation for each channel (raw + normalised)
  - curve CSV          : full lag-correlation curve for one channel (raw + normalised)

Usage example
-------------
python xcorr_real_trial_pair.py --gesture FIST --session S01 --trial-a 2 --trial-b 4 --start-a 250 --window-a 2200 --start-b 220 --window-b 2500 --max-lag 250 --stage env
"""

import argparse
import csv
import math
import os


DEFAULT_DATA_DIR = "../emg_data"
DEFAULT_OUT_DIR  = "../correlation_results"
DEFAULT_LABELS   = "trial_labels.csv"


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description="Cross-correlate two real captured trials.")
    p.add_argument("--data-dir",  default=DEFAULT_DATA_DIR)
    p.add_argument("--out-dir",   default=DEFAULT_OUT_DIR)
    p.add_argument("--labels",    default=DEFAULT_LABELS)

    p.add_argument("--gesture",  default="FIST")
    p.add_argument("--session",  default="S01")
    p.add_argument("--trial-a",  type=int, required=True)
    p.add_argument("--trial-b",  type=int, required=True)

    p.add_argument("--stage", choices=["raw", "hp", "rect", "env"], default="env")
    p.add_argument("--channels", default="0,1,2,3,4,5,6,7")

    p.add_argument("--start-a",  type=int,   default=0)
    p.add_argument("--window-a", type=int,   default=0, help="0 = to end of file")
    p.add_argument("--start-b",  type=int,   default=0)
    p.add_argument("--window-b", type=int,   default=0, help="0 = to end of file")

    p.add_argument("--max-lag",          type=int,   default=250)
    p.add_argument("--min-overlap-frac", type=float, default=0.40)
    p.add_argument("--curve-channel",   type=int,   default=0)

    p.add_argument("--summary-out",    default="real_pair_xcorr_summary.csv")
    p.add_argument("--by-channel-out", default="real_pair_xcorr_by_channel.csv")
    p.add_argument("--curve-out",      default="real_pair_xcorr_curve.csv")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def parse_channels(text):
    out = sorted({int(p.strip()) for p in text.split(",") if p.strip()})
    for ch in out:
        if ch < 0 or ch > 7:
            raise ValueError(f"Channel out of range [0..7]: {ch}")
    if not out:
        raise ValueError("No channels selected")
    return out


def stage_prefix(stage):
    return {"raw": "emg_raw", "hp": "emg_hp", "rect": "emg_rect", "env": "emg_env"}[stage]


def read_csv_dict(path):
    with open(path, "r", newline="") as f:
        r = csv.DictReader(f)
        if r.fieldnames is None:
            raise ValueError(f"CSV has no header: {path}")
        return r.fieldnames, list(r)


def safe_float(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else None
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Signal stats  (computed ONCE per segment, not per lag window)
# ---------------------------------------------------------------------------

def signal_stats(x):
    """
    Returns (mean, energy, std) where
        energy = Σ (x[n] - mean)²          (zero-lag autocorrelation energy)
        std    = sqrt(energy / n)
    All three are needed for standard normalised XCorr.
    """
    n = len(x)
    if n < 2:
        raise ValueError("Segment too short to compute stats")
    m = sum(x) / n
    energy = sum((v - m) ** 2 for v in x)
    std = math.sqrt(energy / n)
    return m, energy, std


# ---------------------------------------------------------------------------
# Point-wise correlation functions  (operate on two equal-length lists)
# ---------------------------------------------------------------------------

def raw_corr_normalised(a, b):
    """
    Mean-normalised raw cross-correlation:
        (1/N) * Σ a[n] * b[n]

    Dividing by N removes the overlap-length bias that was present in the
    original script (longer overlap → larger dot product regardless of
    actual correlation).  This makes raw scores comparable across lags.
    """
    n = len(a)
    if n == 0:
        return None
    return sum(xa * xb for xa, xb in zip(a, b)) / n


def standard_norm_corr(a, b, mean_a, mean_b, energy_a, energy_b):
    """
    Standard (MATLAB-style) normalised cross-correlation for the overlap
    segment, using GLOBAL means and energies of the full segments A and B.

    Formula:
        R(lag) = Σ (a[n] - mean_a) * (b[n] - mean_b)
                 ──────────────────────────────────────
                       sqrt(energy_a * energy_b)

    • mean_a, mean_b  : global means of the full (not windowed) segments
    • energy_a        : Σ (a[n] - mean_a)²  over the full segment A
    • energy_b        : Σ (b[n] - mean_b)²  over the full segment B

    Output is in [-1, 1] and is consistent across all lags because the
    denominator is fixed (does not change with lag).

    The original code computed the mean of the overlap slice at each lag,
    which inflated or deflated scores depending on local signal structure
    at that particular lag — that is sliding Pearson, not standard XCorr.
    """
    den = math.sqrt(energy_a * energy_b)
    if den <= 1e-12:
        return None
    num = sum((xa - mean_a) * (xb - mean_b) for xa, xb in zip(a, b))
    return num / den


# ---------------------------------------------------------------------------
# Cross-correlation sweep functions
# ---------------------------------------------------------------------------

def xcorr_curve_raw(a, b, max_lag, min_overlap):
    """
    Sweeps lag from -max_lag to +max_lag.
    At each lag computes mean-normalised raw cross-correlation.

    lag > 0  →  B delayed relative to A
    lag < 0  →  A delayed relative to B
    """
    na, nb = len(a), len(b)
    out = []

    for lag in range(-max_lag, max_lag + 1):
        a_start = max(0, lag)
        a_end   = min(na - 1, nb - 1 + lag)
        overlap = a_end - a_start + 1

        if overlap < min_overlap:
            continue

        b_start = a_start - lag
        b_end   = a_end   - lag

        seg_a = a[a_start: a_end + 1]
        seg_b = b[b_start: b_end + 1]

        c = raw_corr_normalised(seg_a, seg_b)
        if c is not None:
            out.append((lag, c))

    if not out:
        return [], None, None

    peak_lag, peak_corr = max(out, key=lambda t: t[1])
    return out, peak_lag, peak_corr


def xcorr_curve_normalised(a, b, max_lag, min_overlap, mean_a, mean_b, energy_a, energy_b):
    """
    Sweeps lag from -max_lag to +max_lag.
    At each lag computes standard normalised cross-correlation using
    GLOBAL mean_a, mean_b, energy_a, energy_b (fixed, not recomputed per lag).
    """
    na, nb = len(a), len(b)
    out = []

    for lag in range(-max_lag, max_lag + 1):
        a_start = max(0, lag)
        a_end   = min(na - 1, nb - 1 + lag)
        overlap = a_end - a_start + 1

        if overlap < min_overlap:
            continue

        b_start = a_start - lag
        b_end   = a_end   - lag

        seg_a = a[a_start: a_end + 1]
        seg_b = b[b_start: b_end + 1]

        c = standard_norm_corr(seg_a, seg_b, mean_a, mean_b, energy_a, energy_b)
        if c is not None:
            out.append((lag, c))

    if not out:
        return [], None, None

    peak_lag, peak_corr = max(out, key=lambda t: t[1])
    return out, peak_lag, peak_corr


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def load_signal(path, col):
    header, rows = read_csv_dict(path)
    if col not in header:
        raise ValueError(f"Column {col} missing in {path}")
    sig = [v for r in rows for v in [safe_float(r.get(col, ""))] if v is not None]
    if len(sig) < 100:
        raise ValueError(f"Too few samples in {path} for {col}")
    return sig


def select_segment(x, start, window):
    n = len(x)
    s = int(start)
    w = int(window)
    if s < 0 or s >= n:
        raise ValueError("Segment start out of range")
    e = n if w == 0 else min(n, s + w)
    seg = x[s:e]
    if len(seg) < 100:
        raise ValueError("Segment too short (<100 samples)")
    return seg, s, e


def find_trial_row(label_rows, gesture, session, trial_id):
    g, s, tid = gesture.strip().upper(), session.strip(), str(trial_id)
    candidates = [
        r for r in label_rows
        if (r.get("gesture_label") or "").strip().upper() == g
        and (r.get("session_id")    or "").strip() == s
        and (r.get("trial_id")      or "").strip() == tid
    ]
    if not candidates:
        raise ValueError(f"No trial: gesture={g}, session={s}, trial_id={tid}")
    if len(candidates) > 1:
        raise ValueError(f"Ambiguous trial: gesture={g}, session={s}, trial_id={tid}")
    return candidates[0]


def curve_to_dict(curve):
    return {lag: corr for lag, corr in curve}


def get_zero_lag(curve):
    return next((c for l, c in curve if l == 0), None)


# ---------------------------------------------------------------------------
# Writers
# ---------------------------------------------------------------------------

def write_summary(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric", "value"])
        w.writerows(rows)


def write_by_channel(path, rows):
    cols = [
        "channel",
        "peak_lag_norm", "peak_corr_norm",
        "peak_lag_raw",  "peak_corr_raw",
        "zero_lag_norm", "zero_lag_raw",
        "mean_a", "mean_b",
        "energy_a", "energy_b",
        "n_a", "n_b", "overlap_min",
    ]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)


def write_curve(path, rows):
    cols = [
        "lag",
        "corr_norm", "corr_raw",
        "peak_lag_norm", "peak_corr_norm",
        "peak_lag_raw",  "peak_corr_raw",
        "zero_lag_norm", "zero_lag_raw",
        "channel",
        "trial_a_timestamp", "trial_b_timestamp",
        "segment_a_start", "segment_a_end",
        "segment_b_start", "segment_b_end",
        "stage",
    ]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args     = parse_args()
    channels = parse_channels(args.channels)

    if args.curve_channel not in channels:
        raise ValueError("--curve-channel must be one of the selected channels")
    if args.max_lag <= 0:
        raise ValueError("--max-lag must be > 0")
    if not (0.0 < args.min_overlap_frac <= 1.0):
        raise ValueError("--min-overlap-frac must be in (0, 1]")

    prefix     = stage_prefix(args.stage)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir   = os.path.normpath(os.path.join(script_dir, args.data_dir))
    out_dir    = os.path.normpath(os.path.join(script_dir, args.out_dir))
    labels_path = os.path.join(data_dir, args.labels)

    os.makedirs(out_dir, exist_ok=True)

    summary_path    = os.path.join(out_dir, args.summary_out)
    by_channel_path = os.path.join(out_dir, args.by_channel_out)
    curve_path      = os.path.join(out_dir, args.curve_out)

    if not os.path.exists(labels_path):
        raise FileNotFoundError(f"Labels file not found: {labels_path}")

    header, label_rows = read_csv_dict(labels_path)
    required = ["timestamp", "gesture_label", "trial_id", "session_id", "emg_online_file"]
    miss = [c for c in required if c not in header]
    if miss:
        raise ValueError(f"Labels file missing columns: {', '.join(miss)}")

    row_a = find_trial_row(label_rows, args.gesture, args.session, args.trial_a)
    row_b = find_trial_row(label_rows, args.gesture, args.session, args.trial_b)

    file_a = os.path.join(data_dir, (row_a.get("emg_online_file") or "").strip())
    file_b = os.path.join(data_dir, (row_b.get("emg_online_file") or "").strip())

    for path, label in [(file_a, "A"), (file_b, "B")]:
        if not os.path.exists(path):
            raise FileNotFoundError(f"Trial {label} file missing: {path}")

    by_channel_rows = []
    curve_rows      = []

    peak_corrs_norm, abs_lags_norm = [], []
    peak_corrs_raw,  abs_lags_raw  = [], []

    first_curve_done = False

    for ch in channels:
        col = f"{prefix}{ch}"

        sig_a = load_signal(file_a, col)
        sig_b = load_signal(file_b, col)

        seg_a, a_s, a_e = select_segment(sig_a, args.start_a, args.window_a)
        seg_b, b_s, b_e = select_segment(sig_b, args.start_b, args.window_b)

        # ── Precompute global stats ONCE per segment ──────────────────────
        mean_a, energy_a, _ = signal_stats(seg_a)
        mean_b, energy_b, _ = signal_stats(seg_b)

        overlap_min = max(2, int(round(args.min_overlap_frac * min(len(seg_a), len(seg_b)))))

        # ── Standard normalised XCorr (MATLAB-style, fixed denominator) ───
        curve_norm, peak_lag_norm, peak_corr_norm = xcorr_curve_normalised(
            seg_a, seg_b, args.max_lag, overlap_min,
            mean_a, mean_b, energy_a, energy_b
        )

        # ── Mean-normalised raw XCorr (÷ overlap_length, unbiased) ────────
        curve_raw, peak_lag_raw, peak_corr_raw = xcorr_curve_raw(
            seg_a, seg_b, args.max_lag, overlap_min
        )

        if peak_lag_norm is None:
            raise ValueError(f"No valid normalised xcorr points for channel {ch}")
        if peak_lag_raw is None:
            raise ValueError(f"No valid raw xcorr points for channel {ch}")

        zero_lag_norm = get_zero_lag(curve_norm)
        zero_lag_raw  = get_zero_lag(curve_raw)

        peak_corrs_norm.append(float(peak_corr_norm))
        abs_lags_norm.append(abs(float(peak_lag_norm)))
        peak_corrs_raw.append(float(peak_corr_raw))
        abs_lags_raw.append(abs(float(peak_lag_raw)))

        by_channel_rows.append({
            "channel":        ch,
            "peak_lag_norm":  peak_lag_norm,
            "peak_corr_norm": peak_corr_norm,
            "peak_lag_raw":   peak_lag_raw,
            "peak_corr_raw":  peak_corr_raw,
            "zero_lag_norm":  zero_lag_norm,
            "zero_lag_raw":   zero_lag_raw,
            "mean_a":         f"{mean_a:.6f}",
            "mean_b":         f"{mean_b:.6f}",
            "energy_a":       f"{energy_a:.4f}",
            "energy_b":       f"{energy_b:.4f}",
            "n_a":            len(seg_a),
            "n_b":            len(seg_b),
            "overlap_min":    overlap_min,
        })

        if (not first_curve_done) and ch == args.curve_channel:
            norm_map = curve_to_dict(curve_norm)
            raw_map  = curve_to_dict(curve_raw)
            all_lags = sorted(set(norm_map) | set(raw_map))

            for lag in all_lags:
                curve_rows.append({
                    "lag":               lag,
                    "corr_norm":         norm_map.get(lag, ""),
                    "corr_raw":          raw_map.get(lag, ""),
                    "peak_lag_norm":     peak_lag_norm,
                    "peak_corr_norm":    peak_corr_norm,
                    "peak_lag_raw":      peak_lag_raw,
                    "peak_corr_raw":     peak_corr_raw,
                    "zero_lag_norm":     zero_lag_norm,
                    "zero_lag_raw":      zero_lag_raw,
                    "channel":           ch,
                    "trial_a_timestamp": row_a.get("timestamp", ""),
                    "trial_b_timestamp": row_b.get("timestamp", ""),
                    "segment_a_start":   a_s,
                    "segment_a_end":     a_e,
                    "segment_b_start":   b_s,
                    "segment_b_end":     b_e,
                    "stage":             args.stage,
                })
            first_curve_done = True

    # ── Aggregate metrics ─────────────────────────────────────────────────
    n_ch = len(channels)
    mean_peak_corr_norm = sum(peak_corrs_norm) / n_ch
    mean_abs_lag_norm   = sum(abs_lags_norm)   / n_ch
    mean_peak_corr_raw  = sum(peak_corrs_raw)  / n_ch
    mean_abs_lag_raw    = sum(abs_lags_raw)    / n_ch

    summary_rows = [
        ("gesture",               args.gesture.upper()),
        ("session",               args.session),
        ("trial_a",               args.trial_a),
        ("trial_b",               args.trial_b),
        ("timestamp_a",           row_a.get("timestamp", "")),
        ("timestamp_b",           row_b.get("timestamp", "")),
        ("stage",                 args.stage),
        ("channels",              ",".join(str(c) for c in channels)),
        ("start_a",               args.start_a),
        ("window_a",              args.window_a),
        ("start_b",               args.start_b),
        ("window_b",              args.window_b),
        ("max_lag",               args.max_lag),
        ("min_overlap_frac",      args.min_overlap_frac),
        ("norm_method",           "standard_xcorr (global mean + sqrt(Eaa*Ebb) denom)"),
        ("raw_method",            "mean_normalised (dot_product / overlap_length)"),
        ("mean_peak_corr_norm",   f"{mean_peak_corr_norm:.6f}"),
        ("mean_abs_peak_lag_norm",f"{mean_abs_lag_norm:.6f}"),
        ("mean_peak_corr_raw",    f"{mean_peak_corr_raw:.6f}"),
        ("mean_abs_peak_lag_raw", f"{mean_abs_lag_raw:.6f}"),
    ]

    write_summary(summary_path, summary_rows)
    write_by_channel(by_channel_path, by_channel_rows)
    write_curve(curve_path, curve_rows)

    print("\nReal-trial pair xcorr complete (CORRECTED)")
    print(f"Gesture/session         : {args.gesture.upper()} / {args.session}")
    print(f"Trial pair              : {args.trial_a} vs {args.trial_b}")
    print(f"Timestamps              : {row_a.get('timestamp','')} vs {row_b.get('timestamp','')}")
    print(f"Stage                   : {args.stage}")
    print(f"Window A (start, len)   : ({args.start_a}, {args.window_a})")
    print(f"Window B (start, len)   : ({args.start_b}, {args.window_b})")
    print(f"Mean peak corr norm     : {mean_peak_corr_norm:.4f}")
    print(f"Mean abs peak lag norm  : {mean_abs_lag_norm:.4f}")
    print(f"Mean peak corr raw      : {mean_peak_corr_raw:.4f}")
    print(f"Mean abs peak lag raw   : {mean_abs_lag_raw:.4f}")
    print(f"Summary CSV             : {summary_path}")
    print(f"By-channel CSV          : {by_channel_path}")
    print(f"Curve CSV               : {curve_path}")


if __name__ == "__main__":
    main()