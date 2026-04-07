"""
xcorr_real_trial_pair_both.py  (CORRECTED + per-signal channel selection)

Cross-correlate any two (trial, channel) combinations.

Examples
--------
# Same gesture, different trials, same channel (original use case):
python xcorr_real_trial_pair.py --gesture FIST --session S01 --trial-a 2 --trial-b 2 --channel-a 3 --channel-b 6 --stage env

# Same trial, different channels (cross-channel comparison):
python xcorr_real_trial_pair_both.py \
    --gesture FIST --session S01 \
    --trial-a 2 --trial-b 2 \
    --channel-a 0 --channel-b 3 \
    --stage env

# Different trial AND different channel:
python xcorr_real_trial_pair_both.py \
    --gesture FIST --session S01 \
    --trial-a 2 --trial-b 4 \
    --channel-a 1 --channel-b 5 \
    --stage env

CORRECTIONS (vs original):
  1) raw_corr divides by overlap length → removes lag=0 bias.
  2) normalised_corr uses global mean + fixed sqrt(Eaa*Ebb) denominator
     (MATLAB xcorr 'normalized' equivalent), not sliding Pearson.
  3) Global stats computed once per segment, not inside the lag loop.
  4) --channel-a / --channel-b replace --channels, enabling independent
     channel selection for each of the two signals.

     ## python xcorr_real_trial_pair.py  --gesture FIST --session S01 --trial-a 2 --trial-b 2 --channel-a 0 --channel-b 0 --stage env --start-a 220 --window-a 2200 --start-b 250 --window-b 2200 --max-lag 250 --min-overlap-frac 0.40 --summary-out real_pair_fist_t2_ch0_vs_ch1_summary.csv --curve-out real_pair_fist_t2_ch0_vs_ch1_curve.csv
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
    p = argparse.ArgumentParser(
        description="Cross-correlate any two (trial, channel) combinations."
    )
    p.add_argument("--data-dir",  default=DEFAULT_DATA_DIR)
    p.add_argument("--out-dir",   default=DEFAULT_OUT_DIR)
    p.add_argument("--labels",    default=DEFAULT_LABELS)

    p.add_argument("--gesture", default="FIST")
    p.add_argument("--session", default="S01")

    # Trial selection — may be the same trial
    p.add_argument("--trial-a", type=int, required=True,
                   help="Trial ID for signal A (may equal --trial-b)")
    p.add_argument("--trial-b", type=int, required=True,
                   help="Trial ID for signal B (may equal --trial-a)")

    # Independent channel selection per signal
    p.add_argument("--channel-a", type=int, required=True,
                   help="Channel index [0..7] to read from trial A")
    p.add_argument("--channel-b", type=int, required=True,
                   help="Channel index [0..7] to read from trial B")

    p.add_argument("--stage", choices=["raw", "hp", "rect", "env"], default="env")

    p.add_argument("--start-a",  type=int,   default=0)
    p.add_argument("--window-a", type=int,   default=0, help="0 = to end of file")
    p.add_argument("--start-b",  type=int,   default=0)
    p.add_argument("--window-b", type=int,   default=0, help="0 = to end of file")

    p.add_argument("--max-lag",          type=int,   default=250)
    p.add_argument("--min-overlap-frac", type=float, default=0.40)

    p.add_argument("--summary-out", default="real_pair_xcorr_summary.csv")
    p.add_argument("--curve-out",   default="real_pair_xcorr_curve.csv")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def validate_channel(ch, label):
    if ch < 0 or ch > 7:
        raise ValueError(f"--channel-{label} must be in [0..7], got {ch}")


def stage_prefix(stage):
    return {
        "raw":  "emg_raw",
        "hp":   "emg_hp",
        "rect": "emg_rect",
        "env":  "emg_env",
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
        return v if math.isfinite(v) else None
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Signal stats  (computed ONCE per segment, not per lag window)
# ---------------------------------------------------------------------------

def signal_stats(x):
    """
    Returns (mean, energy, std) where
        energy = Σ (x[n] - mean)²     (zero-lag autocorrelation energy)
    """
    n = len(x)
    if n < 2:
        raise ValueError("Segment too short to compute stats")
    m = sum(x) / n
    energy = sum((v - m) ** 2 for v in x)
    std = math.sqrt(energy / n)
    return m, energy, std


# ---------------------------------------------------------------------------
# Point-wise correlation functions
# ---------------------------------------------------------------------------

def raw_corr_normalised(a, b):
    """
    Mean-normalised raw cross-correlation:
        (1/N) * Σ a[n] * b[n]

    Dividing by N removes the overlap-length bias (longer overlap → larger
    dot product regardless of actual correlation).
    """
    n = len(a)
    if n == 0:
        return None
    return sum(xa * xb for xa, xb in zip(a, b)) / n


def standard_norm_corr(a, b, mean_a, mean_b, energy_a, energy_b):
    """
    Standard (MATLAB-style) normalised cross-correlation.

        R(lag) = Σ (a[n] - μ_a)(b[n] - μ_b)
                 ─────────────────────────────
                      sqrt(E_a · E_b)

    μ_a, μ_b  : global means of full segments (fixed, not per-lag window)
    E_a, E_b  : global energies Σ(x - μ)²    (fixed denominator across lags)

    Output is in [-1, 1] and consistent across all lags.
    """
    den = math.sqrt(energy_a * energy_b)
    if den <= 1e-12:
        return None
    num = sum((xa - mean_a) * (xb - mean_b) for xa, xb in zip(a, b))
    return num / den


# ---------------------------------------------------------------------------
# Cross-correlation sweep
# ---------------------------------------------------------------------------

def _overlap_slices(na, nb, lag):
    """Return (a_start, a_end, b_start, b_end, overlap) for a given lag."""
    a_start = max(0, lag)
    a_end   = min(na - 1, nb - 1 + lag)
    overlap = a_end - a_start + 1
    b_start = a_start - lag
    b_end   = a_end   - lag
    return a_start, a_end, b_start, b_end, overlap


def xcorr_curve_raw(a, b, max_lag, min_overlap):
    """
    Sweeps lag in [-max_lag, +max_lag].
    lag > 0  →  B delayed relative to A
    lag < 0  →  A delayed relative to B
    """
    na, nb = len(a), len(b)
    out = []
    for lag in range(-max_lag, max_lag + 1):
        a_s, a_e, b_s, b_e, overlap = _overlap_slices(na, nb, lag)
        if overlap < min_overlap:
            continue
        c = raw_corr_normalised(a[a_s:a_e+1], b[b_s:b_e+1])
        if c is not None:
            out.append((lag, c))
    if not out:
        return [], None, None
    peak_lag, peak_corr = max(out, key=lambda t: t[1])
    return out, peak_lag, peak_corr


def xcorr_curve_normalised(a, b, max_lag, min_overlap,
                            mean_a, mean_b, energy_a, energy_b):
    """
    Sweeps lag in [-max_lag, +max_lag].
    Uses fixed global stats so the denominator is constant across all lags.
    """
    na, nb = len(a), len(b)
    out = []
    for lag in range(-max_lag, max_lag + 1):
        a_s, a_e, b_s, b_e, overlap = _overlap_slices(na, nb, lag)
        if overlap < min_overlap:
            continue
        c = standard_norm_corr(
            a[a_s:a_e+1], b[b_s:b_e+1],
            mean_a, mean_b, energy_a, energy_b
        )
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
        raise ValueError(f"Column '{col}' missing in {path}")
    sig = [v for r in rows for v in [safe_float(r.get(col, ""))] if v is not None]
    if len(sig) < 100:
        raise ValueError(f"Too few samples in {path} for column '{col}'")
    return sig


def select_segment(x, start, window):
    n = len(x)
    s = int(start)
    w = int(window)
    if s < 0 or s >= n:
        raise ValueError(f"Segment start {s} out of range [0, {n})")
    e = n if w == 0 else min(n, s + w)
    seg = x[s:e]
    if len(seg) < 100:
        raise ValueError(f"Segment too short ({len(seg)} samples, need >= 100)")
    return seg, s, e


def find_trial_row(label_rows, gesture, session, trial_id):
    g   = gesture.strip().upper()
    s   = session.strip()
    tid = str(trial_id)
    candidates = [
        r for r in label_rows
        if (r.get("gesture_label") or "").strip().upper() == g
        and (r.get("session_id")    or "").strip() == s
        and (r.get("trial_id")      or "").strip() == tid
    ]
    if not candidates:
        raise ValueError(f"No trial: gesture={g}, session={s}, trial_id={tid}")
    if len(candidates) > 1:
        raise ValueError(f"Ambiguous: gesture={g}, session={s}, trial_id={tid}")
    return candidates[0]


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


def write_curve(path, rows):
    cols = [
        "lag",
        "corr_norm", "corr_raw",
        "peak_lag_norm", "peak_corr_norm",
        "peak_lag_raw",  "peak_corr_raw",
        "zero_lag_norm", "zero_lag_raw",
        "trial_a", "channel_a",
        "trial_b", "channel_b",
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
    args = parse_args()

    validate_channel(args.channel_a, "a")
    validate_channel(args.channel_b, "b")

    if args.max_lag <= 0:
        raise ValueError("--max-lag must be > 0")
    if not (0.0 < args.min_overlap_frac <= 1.0):
        raise ValueError("--min-overlap-frac must be in (0, 1]")

    # Describe the comparison mode clearly in the output
    same_trial   = (args.trial_a == args.trial_b)
    same_channel = (args.channel_a == args.channel_b)
    if same_trial and same_channel:
        mode = "self (same trial, same channel)"
    elif same_trial:
        mode = "cross-channel (same trial, different channels)"
    elif same_channel:
        mode = "cross-trial (different trials, same channel)"
    else:
        mode = "cross-trial + cross-channel"

    prefix      = stage_prefix(args.stage)
    script_dir  = os.path.dirname(os.path.abspath(__file__))
    data_dir    = os.path.normpath(os.path.join(script_dir, args.data_dir))
    out_dir     = os.path.normpath(os.path.join(script_dir, args.out_dir))
    labels_path = os.path.join(data_dir, args.labels)

    os.makedirs(out_dir, exist_ok=True)
    summary_path = os.path.join(out_dir, args.summary_out)
    curve_path   = os.path.join(out_dir, args.curve_out)

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

    for fpath, label in [(file_a, "A"), (file_b, "B")]:
        if not os.path.exists(fpath):
            raise FileNotFoundError(f"Trial {label} file missing: {fpath}")

    # ── Load signals — each from its own (trial, channel) pair ───────────
    col_a = f"{prefix}{args.channel_a}"
    col_b = f"{prefix}{args.channel_b}"

    sig_a = load_signal(file_a, col_a)
    sig_b = load_signal(file_b, col_b)

    seg_a, a_s, a_e = select_segment(sig_a, args.start_a, args.window_a)
    seg_b, b_s, b_e = select_segment(sig_b, args.start_b, args.window_b)

    # ── Precompute global stats ONCE per segment ──────────────────────────
    mean_a, energy_a, _ = signal_stats(seg_a)
    mean_b, energy_b, _ = signal_stats(seg_b)

    overlap_min = max(2, int(round(args.min_overlap_frac * min(len(seg_a), len(seg_b)))))

    # ── Run both XCorr methods ────────────────────────────────────────────
    curve_norm, peak_lag_norm, peak_corr_norm = xcorr_curve_normalised(
        seg_a, seg_b, args.max_lag, overlap_min,
        mean_a, mean_b, energy_a, energy_b
    )
    curve_raw, peak_lag_raw, peak_corr_raw = xcorr_curve_raw(
        seg_a, seg_b, args.max_lag, overlap_min
    )

    if peak_lag_norm is None:
        raise ValueError("No valid normalised xcorr points — check window and max-lag settings")
    if peak_lag_raw is None:
        raise ValueError("No valid raw xcorr points — check window and max-lag settings")

    zero_lag_norm = get_zero_lag(curve_norm)
    zero_lag_raw  = get_zero_lag(curve_raw)

    # ── Build curve rows ──────────────────────────────────────────────────
    norm_map = {lag: corr for lag, corr in curve_norm}
    raw_map  = {lag: corr for lag, corr in curve_raw}
    all_lags = sorted(set(norm_map) | set(raw_map))

    curve_rows = [
        {
            "lag":               lag,
            "corr_norm":         norm_map.get(lag, ""),
            "corr_raw":          raw_map.get(lag, ""),
            "peak_lag_norm":     peak_lag_norm,
            "peak_corr_norm":    peak_corr_norm,
            "peak_lag_raw":      peak_lag_raw,
            "peak_corr_raw":     peak_corr_raw,
            "zero_lag_norm":     zero_lag_norm,
            "zero_lag_raw":      zero_lag_raw,
            "trial_a":           args.trial_a,
            "channel_a":         args.channel_a,
            "trial_b":           args.trial_b,
            "channel_b":         args.channel_b,
            "trial_a_timestamp": row_a.get("timestamp", ""),
            "trial_b_timestamp": row_b.get("timestamp", ""),
            "segment_a_start":   a_s,
            "segment_a_end":     a_e,
            "segment_b_start":   b_s,
            "segment_b_end":     b_e,
            "stage":             args.stage,
        }
        for lag in all_lags
    ]

    # ── Summary ───────────────────────────────────────────────────────────
    summary_rows = [
        ("gesture",          args.gesture.upper()),
        ("session",          args.session),
        ("mode",             mode),
        ("trial_a",          args.trial_a),
        ("channel_a",        args.channel_a),
        ("trial_b",          args.trial_b),
        ("channel_b",        args.channel_b),
        ("timestamp_a",      row_a.get("timestamp", "")),
        ("timestamp_b",      row_b.get("timestamp", "")),
        ("stage",            args.stage),
        ("start_a",          args.start_a),
        ("window_a",         args.window_a),
        ("start_b",          args.start_b),
        ("window_b",         args.window_b),
        ("n_a",              len(seg_a)),
        ("n_b",              len(seg_b)),
        ("mean_a",           f"{mean_a:.6f}"),
        ("mean_b",           f"{mean_b:.6f}"),
        ("energy_a",         f"{energy_a:.4f}"),
        ("energy_b",         f"{energy_b:.4f}"),
        ("max_lag",          args.max_lag),
        ("min_overlap_frac", args.min_overlap_frac),
        ("overlap_min",      overlap_min),
        ("norm_method",      "standard_xcorr (global mean + sqrt(Eaa*Ebb) denom)"),
        ("raw_method",       "mean_normalised (dot_product / overlap_length)"),
        ("peak_lag_norm",    peak_lag_norm),
        ("peak_corr_norm",   f"{peak_corr_norm:.6f}"),
        ("zero_lag_norm",    f"{zero_lag_norm:.6f}" if zero_lag_norm is not None else ""),
        ("peak_lag_raw",     peak_lag_raw),
        ("peak_corr_raw",    f"{peak_corr_raw:.6f}"),
        ("zero_lag_raw",     f"{zero_lag_raw:.6f}" if zero_lag_raw is not None else ""),
    ]

    write_summary(summary_path, summary_rows)
    write_curve(curve_path, curve_rows)

    print(f"\nXCorr complete")
    print(f"Mode                    : {mode}")
    print(f"Gesture / session       : {args.gesture.upper()} / {args.session}")
    print(f"Signal A                : trial {args.trial_a}, channel {args.channel_a}  [{col_a}]")
    print(f"Signal B                : trial {args.trial_b}, channel {args.channel_b}  [{col_b}]")
    print(f"Timestamps              : {row_a.get('timestamp','')} vs {row_b.get('timestamp','')}")
    print(f"Stage                   : {args.stage}")
    print(f"Window A (start, len)   : ({args.start_a}, {args.window_a})")
    print(f"Window B (start, len)   : ({args.start_b}, {args.window_b})")
    print(f"Segment lengths         : {len(seg_a)} vs {len(seg_b)} samples")
    print(f"Mean A / B              : {mean_a:.4f} / {mean_b:.4f}")
    print(f"Energy A / B            : {energy_a:.2f} / {energy_b:.2f}")
    print(f"Peak corr norm          : {peak_corr_norm:.4f}  at lag {peak_lag_norm}")
    print(f"Zero lag  norm          : {zero_lag_norm:.4f}" if zero_lag_norm is not None else "Zero lag norm           : N/A")
    print(f"Peak corr raw           : {peak_corr_raw:.4f}  at lag {peak_lag_raw}")
    print(f"Summary CSV             : {summary_path}")
    print(f"Curve CSV               : {curve_path}")


if __name__ == "__main__":
    main()