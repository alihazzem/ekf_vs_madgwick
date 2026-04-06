# EMG Cross-Correlation Script — Logic Explained

**File:** `xcorr_real_trial_pair_both.py` (corrected version)  
**Purpose:** Measure temporal similarity between two captured EMG gesture trials using two flavors of cross-correlation.

---

## Table of Contents

1. [What Problem Does This Script Solve?](#1-what-problem-does-this-script-solve)
2. [High-Level Data Flow](#2-high-level-data-flow)
3. [Signal Stages](#3-signal-stages)
4. [Segment Selection](#4-segment-selection)
5. [Global Signal Statistics](#5-global-signal-statistics)
6. [The Lag Sweep — Core XCorr Logic](#6-the-lag-sweep--core-xcorr-logic)
7. [Method 1 — Mean-Normalised Raw XCorr](#7-method-1--mean-normalised-raw-xcorr)
8. [Method 2 — Standard Normalised XCorr (MATLAB-style)](#8-method-2--standard-normalised-xcorr-matlab-style)
9. [Why Two Methods?](#9-why-two-methods)
10. [Minimum Overlap Guard](#10-minimum-overlap-guard)
11. [Peak Detection](#11-peak-detection)
12. [Output Files](#12-output-files)
13. [Key Metrics Explained](#13-key-metrics-explained)
14. [What Was Wrong in the Original — And How It Was Fixed](#14-what-was-wrong-in-the-original--and-how-it-was-fixed)
15. [Practical Interpretation Guide](#15-practical-interpretation-guide)

---

## 1. What Problem Does This Script Solve?

When a person performs the same gesture (e.g., making a fist) across two different trials, the EMG signals captured will look similar in shape but will **not be time-aligned**. Trial A might start contracting at sample 250; Trial B might start at sample 220. Even after windowing, there is an unknown timing offset between them.

Cross-correlation answers two questions:

- **How similar are the two trials?** (similarity score)
- **What is the timing offset (lag) between them?** (lag in samples)

If the sampling rate is 1000 Hz, a peak lag of 30 means Trial B is 30 ms ahead of Trial A at the point of best alignment.

This analysis is done **per channel** (up to 8 EMG channels), giving a picture of how consistently all muscles fire between the two trials.

---

## 2. High-Level Data Flow

```
trial_labels.csv
      │
      ├── find_trial_row()  →  locates metadata for Trial A and Trial B
      │
      ├── emg file path A,  emg file path B
      │
      └── for each channel (0–7):
                │
                ├── load_signal()           read column emg_envN from CSV
                ├── select_segment()        slice [start, start+window)
                ├── signal_stats()          compute mean + energy ONCE globally
                │
                ├── xcorr_curve_raw()       sweep lag → mean-normalised dot product
                └── xcorr_curve_normalised() sweep lag → standard normalised corr
                          │
                          └── collect peak_lag, peak_corr, zero_lag per channel
                                    │
                                    └── write 3 CSV output files
```

The file `trial_labels.csv` acts as an **index** that maps `(gesture, session, trial_id)` to an actual data file path. This decouples metadata from signal data and lets the script locate any trial by its logical identity rather than a raw filename.

---

## 3. Signal Stages

The `--stage` argument controls which column is read from the data CSV:

| Stage flag | Column prefix | Description |
|---|---|---|
| `raw` | `emg_raw` | Raw ADC samples, noisy |
| `hp` | `emg_hp` | High-pass filtered — removes DC offset and slow drift |
| `rect` | `emg_rect` | Rectified — absolute value, all non-negative |
| `env` | `emg_env` | Envelope — smoothed rectified signal (**recommended**) |

The envelope stage is best for cross-correlation because it captures the **shape of muscle activation** — the broad rise and fall of effort — rather than the raw high-frequency oscillations. Two envelopes from the same gesture look like similar bumps; two raw signals from the same gesture look like two different noise bursts even if they are fundamentally correlated.

---

## 4. Segment Selection

Whole-trial files may include a rest period before the gesture and a release phase after. Comparing those regions adds noise to the correlation. The script lets you window each trial independently:

```
--start-a  250   --window-a  2200   →  Trial A: samples [250, 2450)
--start-b  220   --window-b  2500   →  Trial B: samples [220, 2720)
```

The `select_segment()` function enforces:

- Start index must be within bounds.
- `window=0` means "from start to end of file" (no truncation).
- The resulting segment must have at least 100 samples, or an error is raised.

Each channel uses the same windowing parameters. The two segments do **not** need to be the same length — the lag sweep handles unequal lengths naturally.

---

## 5. Global Signal Statistics

**This is one of the key corrections from the original script.**

Before the lag sweep begins, the mean and energy of each full segment are computed **once**:

```python
def signal_stats(x):
    n      = len(x)
    mean   = sum(x) / n
    energy = sum((v - mean) ** 2 for v in x)   # Σ (x[n] - mean)²
    std    = sqrt(energy / n)
    return mean, energy, std
```

These three values are computed once and passed into the cross-correlation functions. They are **not recomputed inside the lag loop**. This matters because:

- Recomputing the mean from each overlap slice (as the original did) gives a different mean at every lag, which is **sliding Pearson correlation**, not standard cross-correlation.
- Using a fixed global mean and energy ensures the correlation score has a consistent denominator across all lags, which is required for the result to be mathematically valid as a lag estimator.

---

## 6. The Lag Sweep — Core XCorr Logic

For both methods, the same windowing logic is used to find the overlapping portion of the two segments at a given lag:

```python
for lag in range(-max_lag, max_lag + 1):

    a_start = max(0, lag)
    a_end   = min(na - 1, nb - 1 + lag)
    overlap = a_end - a_start + 1

    b_start = a_start - lag
    b_end   = a_end   - lag

    seg_a = a[a_start : a_end + 1]
    seg_b = b[b_start : b_end + 1]
```

### Lag Convention

| Lag value | Meaning |
|---|---|
| `lag > 0` | Trial B is delayed relative to Trial A |
| `lag < 0` | Trial A is delayed relative to Trial B |
| `lag = 0` | No offset — signals compared sample-for-sample |

### How the Overlap Window Moves

At `lag = 0`, both signals are compared from their common start for `min(na, nb)` samples.

At `lag = +5`, signal B is conceptually shifted 5 samples to the right, so:
- We skip the first 5 samples of A (`a_start = 5`)
- We start B from sample 0 (`b_start = 0`)
- The overlap region is the portion where both signals have valid data

At `lag = -5`, it is the reverse — A shifts right, B shifts left.

This is a correct implementation of the standard discrete cross-correlation overlap arithmetic.

---

## 7. Method 1 — Mean-Normalised Raw XCorr

```python
def raw_corr_normalised(a, b):
    n = len(a)
    return sum(xa * xb for xa, xb in zip(a, b)) / n
```

This computes:

```
R_raw(lag) = (1/N) * Σ a[n] * b[n]
```

where N is the length of the overlap at that lag. **Dividing by N is the correction.** In the original script, there was no division, which caused a systematic bias: at large lags, fewer samples contribute to the sum, making the raw score artificially small. This meant the peak was nearly always reported at or near lag=0, regardless of the true alignment.

With the division, each lag produces an **average energy per sample**, making scores comparable across all lags. The output is in physical EMG-squared units (e.g., µV²), not bounded to [-1, 1].

This method is useful when:
- You want to weight channels by their actual signal amplitude.
- You are comparing trials from the same session (similar amplitude scale).

---

## 8. Method 2 — Standard Normalised XCorr (MATLAB-style)

```python
def standard_norm_corr(a, b, mean_a, mean_b, energy_a, energy_b):
    den = sqrt(energy_a * energy_b)
    num = sum((xa - mean_a) * (xb - mean_b) for xa, xb in zip(a, b))
    return num / den
```

This computes:

```
         Σ (a[n] - μ_a) · (b[n+lag] - μ_b)
R(lag) = ─────────────────────────────────────
               sqrt(E_a · E_b)
```

where:
- `μ_a`, `μ_b` are the **global means** of the full segments A and B
- `E_a = Σ (a[n] - μ_a)²` is the **global energy** (zero-lag autocorrelation) of segment A
- `E_b = Σ (b[n] - μ_b)²` is the **global energy** of segment B

### Key Properties

- **Output is in [-1, 1]** — 1.0 is perfect correlation, -1.0 is perfect anti-correlation, 0.0 is no linear relationship.
- **Denominator is fixed across all lags** — because `E_a` and `E_b` are computed from the full segments, not from the overlap window. This means a score of 0.9 at lag=+20 is directly comparable to a score of 0.85 at lag=0.
- **Amplitude-independent** — a loud trial and a quiet trial of the same gesture will score ~1.0 if their shapes match, even if one signal is twice the amplitude.
- **Session-comparable** — scores are meaningful across sessions and subjects without normalization by electrode position.

### Equivalent Formulation

This is mathematically equivalent to `xcorr(a, b, 'normalized')` in MATLAB, or `scipy.signal.correlate` followed by division by `sqrt(autocorr_a(0) * autocorr_b(0))`.

---

## 9. Why Two Methods?

| Property | Method 1 (Raw ÷ N) | Method 2 (Standard Norm) |
|---|---|---|
| Output range | Unbounded (µV²) | [-1, 1] |
| Amplitude sensitive | Yes | No |
| Denominator varies with lag | No (fixed after ÷N) | No (global energy) |
| Session-comparable | Partially | Yes |
| Best for | Energy-weighted alignment | Shape-based alignment |
| Equivalent to | Normalised dot product | MATLAB `xcorr 'normalized'` |

Using both lets you cross-check: if Method 1 and Method 2 agree on the peak lag, the alignment is robust. If they disagree, it may indicate that one channel has a large amplitude outlier driving the raw result.

---

## 10. Minimum Overlap Guard

At extreme lags (e.g., `lag = max_lag`), the overlap window becomes very short. Computing a correlation on 3 samples is statistically meaningless and numerically unstable.

```python
overlap_min = max(2, int(round(min_overlap_frac * min(len(seg_a), len(seg_b)))))
```

With `min_overlap_frac = 0.40` and `min(na, nb) = 2000`, only lags where the overlap exceeds **800 samples** are evaluated. Lags with insufficient overlap are silently skipped and do not appear in the output curve. This protects against spurious peak detections at the tails of the lag range.

---

## 11. Peak Detection

After the full lag sweep, the peak is found by a simple maximum search:

```python
peak_lag, peak_corr = max(out, key=lambda t: t[1])
```

The lag that yields the highest correlation score is the **best-alignment lag**. The correlation value at that lag is the **best-case similarity** between the two trials.

The script also records the zero-lag value separately:

```python
zero_lag_norm = next((c for l, c in curve if l == 0), None)
```

This lets you compare:
- `peak_corr_norm` — similarity at the best alignment
- `zero_lag_norm` — similarity at zero delay (raw time comparison)

A large gap between these two values means the trials are similar in shape but significantly offset in time. A small gap means they were already well-aligned when captured.

---

## 12. Output Files

### `real_pair_xcorr_summary.csv`

One row per metric. Records all parameters and the aggregate means across channels.

Key fields: `mean_peak_corr_norm`, `mean_abs_peak_lag_norm`, `mean_peak_corr_raw`, `mean_abs_peak_lag_raw`, plus the normalization method names for documentation.

### `real_pair_xcorr_by_channel.csv`

One row per channel. Useful for identifying which channels are most consistent between trials, and which may have electrode contact issues.

Key fields: `peak_lag_norm`, `peak_corr_norm`, `peak_lag_raw`, `peak_corr_raw`, `zero_lag_norm`, `zero_lag_raw`, `mean_a`, `mean_b`, `energy_a`, `energy_b`.

The global stats (`mean_a`, `energy_a`, etc.) are now exported per channel so you can audit the normalization directly.

### `real_pair_xcorr_curve.csv`

Full lag sweep for one selected channel (`--curve-channel`). One row per lag value. This is what you plot in MATLAB or Python to visualize the correlation shape.

Key fields: `lag`, `corr_norm`, `corr_raw`, `peak_lag_norm`, `peak_lag_raw`.

**How to interpret the curve plot:**
- A **sharp, tall peak** = strong, unambiguous alignment. The two trials are similar and the lag is well-determined.
- A **broad, flat peak** = the signals are somewhat correlated over a wide lag range. Alignment is uncertain.
- A **flat, near-zero curve** = the two trials do not look similar at any lag. Possible bad capture.
- **Multiple peaks** = the gesture has a repeating activation pattern (e.g., alternating finger flexion). Take care when interpreting peak lag.

---

## 13. Key Metrics Explained

| Metric | Interpretation |
|---|---|
| `peak_corr_norm` > 0.85 | Trials are highly repeatable — the gesture shape is consistent |
| `peak_corr_norm` 0.65–0.85 | Moderate consistency — acceptable for robust classifiers |
| `peak_corr_norm` < 0.65 | Poor consistency — check electrode placement, gesture execution |
| `peak_lag_norm` at 1000 Hz | Each unit = 1 ms of timing offset between trials |
| `zero_lag_norm` ≈ `peak_corr_norm` | Trials were well-aligned at capture |
| `zero_lag_norm` ≪ `peak_corr_norm` | Trials have a systematic timing offset |
| `mean_abs_peak_lag_norm` large | Onset detection or windowing needs improvement |

---

## 14. What Was Wrong in the Original — And How It Was Fixed

### Bug 1 — Raw XCorr Biased Toward Lag = 0

**Original:**
```python
def raw_corr(a, b):
    return sum(xa * xb for xa, xb in zip(a, b))   # no division
```

At lag=0, the overlap is maximum (e.g., 2000 samples). At lag=200, the overlap is smaller (e.g., 1800 samples). The sum at lag=0 naturally has more terms and will almost always be larger — not because the signals are better aligned there, but simply because more samples are included. The peak lag would always be reported near 0, regardless of true alignment.

**Fix:**
```python
def raw_corr_normalised(a, b):
    return sum(xa * xb for xa, xb in zip(a, b)) / len(a)
```

Dividing by the overlap length converts the dot product to a **per-sample average**, making it comparable across all lags.

---

### Bug 2 — "Normalised" Was Actually Sliding Pearson

**Original:**
```python
def normalized_corr(a, b):
    ma = sum(a) / n    # ← mean of the OVERLAP SLICE at this lag
    mb = sum(b) / n    # ← mean of the OVERLAP SLICE at this lag
    ...
    den = sqrt(da * db)    # ← energy of the OVERLAP SLICE
```

At every lag, the mean was recomputed from the current overlap window. This means the denominator was different at every lag — the correlation was re-centered and re-scaled for each lag window. This is **sliding Pearson correlation**, which can inflate or deflate scores depending on the local statistics of each slice, and makes scores across lags incomparable.

**Fix:**
```python
# Computed ONCE before the lag loop:
mean_a, energy_a, _ = signal_stats(seg_a)
mean_b, energy_b, _ = signal_stats(seg_b)

# Inside the lag loop — denominator is FIXED:
def standard_norm_corr(a, b, mean_a, mean_b, energy_a, energy_b):
    den = sqrt(energy_a * energy_b)   # ← constant across all lags
    num = sum((xa - mean_a) * (xb - mean_b) for xa, xb in zip(a, b))
    return num / den
```

Global means and a fixed denominator ensure the correlation score is consistent and mathematically valid as a lag estimator.

---

### Improvement — Global Stats Exported to CSV

The corrected by-channel CSV now includes `mean_a`, `mean_b`, `energy_a`, `energy_b` per channel. This allows post-hoc auditing of the normalization and helps identify channels with very low energy (near-zero signal, possibly disconnected electrode).

---

## 15. Practical Interpretation Guide

### Scenario A — High Correlation, Low Lag
```
mean_peak_corr_norm = 0.91
mean_abs_peak_lag_norm = 12.0
```
The two trials are very similar in shape. There is a 12 ms timing offset, likely from slight variation in when the gesture was initiated. The windowing parameters are good.

### Scenario B — High Correlation, High Lag
```
mean_peak_corr_norm = 0.88
mean_abs_peak_lag_norm = 180.0
```
The gesture shape is consistent, but the onset timing varies significantly between trials. Consider tighter windowing to isolate the active contraction phase, or use onset detection to auto-align before windowing.

### Scenario C — Low Correlation Across All Channels
```
mean_peak_corr_norm = 0.41
mean_abs_peak_lag_norm = 83.0
```
The two trials do not look similar. Possible causes: electrode shift between trials, muscle fatigue, inconsistent gesture execution, or one trial capturing a failed attempt. Discard one or both trials and re-capture.

### Scenario D — High Correlation on Some Channels, Low on Others

Check the `by_channel` CSV. Channels with low `peak_corr_norm` and very different `energy_a` vs `energy_b` likely have poor electrode contact on one of the two trials. The other channels can still be used for classification if they are consistent.