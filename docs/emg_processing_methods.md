# EMG Processing Methods Used in This Project

## Scope
This document explains the EMG processing steps used in the current pipeline and why each step is needed.

Current implementation is in:
- scripts/process_fusion_emg.py (merged EMG + IMU pipeline)
- scripts/process_emg.py (EMG-only baseline pipeline)

The IMU side is not changed by these EMG steps.

---

## 1) DC Offset Removal
### What it does
For each EMG channel, subtract the channel mean:

x_dc[n] = x[n] - mean(x)

### Why we need it
Raw EMG can have baseline shift from electrode/skin impedance and acquisition bias.

### What it fixes
- Removes constant offset
- Re-centers signal around zero
- Improves stability of later filters and zero-crossing metrics

---

## 2) High-Pass Filtering (default 20 Hz)
### What it does
Applies a first-order high-pass filter to each channel.

### Why we need it
Surface EMG contains low-frequency motion artifact (cable movement, skin motion, slow drift), especially below about 20 Hz.

### What it fixes
- Suppresses baseline wander
- Suppresses low-frequency movement artifact
- Keeps the more informative EMG band content

---

## 3) Optional Notch Filtering (50/60 Hz)
### What it does
Applies a narrow notch (biquad) at mains frequency when enabled.

### Why we need it
Depending on environment and wiring, mains interference can contaminate EMG channels.

### What it fixes
- Reduces narrowband powerline noise
- Improves SNR for feature extraction

### Notes
- Disabled by default
- Enable only when clear 50/60 Hz contamination is observed

---

## 4) Full-Wave Rectification
### What it does
Converts bipolar EMG to unipolar magnitude:

x_rect[n] = |x_filt[n]|

### Why we need it
Muscle activity is represented by signal energy/amplitude; signed polarity is less useful for envelope features.

### What it fixes
- Prevents positive and negative phases from cancelling each other
- Makes amplitude-based metrics (MAV/RMS/envelope) meaningful

---

## 5) Causal RMS Envelope
### What it does
Computes trailing-window RMS over filtered EMG (default 150 ms):

env[n] = sqrt(mean(x_filt[n-k+1 : n]^2))

### Why we need it
Raw EMG is spiky and high variance. Envelope gives smooth activation trend over time.

### What it fixes
- Reduces short-term jitter
- Highlights activation bursts and contraction timing
- Provides a robust activity profile for plotting and fusion context

---

## 6) Windowing
### What it does
Segments EMG into short overlapping windows (default 100 ms window, 50 ms step).

### Why we need it
Feature extraction and fusion models use local time context, not single samples.

### What it fixes
- Converts high-rate raw stream into structured analysis units
- Balances temporal resolution and feature stability

---

## 7) Feature Extraction per Window
Features are computed per channel (8 channels):

- MAV (Mean Absolute Value)
- RMS (Root Mean Square)
- ZC (Zero Crossings with dead-zone threshold)
- WL (Waveform Length)
- env_mean (mean envelope in window)

### Why we need them
These features capture complementary aspects of EMG:
- MAV/RMS: signal amplitude and energy
- ZC: frequency-like activity changes
- WL: waveform complexity and motor unit activity richness
- env_mean: smooth activation strength

### What they fix
- Reduce dimensionality from raw time-series
- Improve robustness vs noisy sample-level representation
- Provide model-ready inputs for fusion/classification

---

## 8) Synchronization Context (already available)
Although IMU is not processed here, EMG windows can include synchronized roll/pitch summaries from the merged timeline.

### Why we need this
Sensor fusion requires both modalities aligned in the same window/time reference.

### What it fixes
- Prevents temporal mismatch between EMG activity and orientation state

---

## Output Files Produced by EMG Processing
For each fusion_merged file:

1. *_emgproc.csv
- Original merged columns + processed EMG columns:
  emg_dc0..7, emg_hp0..7, emg_rect0..7, emg_env0..7

2. *_emgfeat.csv
- One row per window with EMG features and optional angle summaries

---

## Practical Defaults in This Project
- HPF cutoff: 20 Hz
- Notch: off by default
- RMS envelope window: 150 ms
- Feature window: 100 ms
- Feature step: 50 ms
- ZC threshold: 5 counts

These are baseline defaults and can be tuned after collecting more labeled data.
