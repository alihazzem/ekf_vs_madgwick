# Gesture Data Collection Plan and Next Steps

## Purpose
Now that synchronized EMG + IMU acquisition is working, this phase builds a reliable labeled dataset for signal processing and sensor fusion experiments.

---

## Part A: What You Need To Do Now

## 1) Define Gesture Set
Start with a small, repeatable set:
- Rest
- Fist clench
- Hand open
- Wrist flexion
- Wrist extension
- Optional: pronation/supination

### Why
A compact set gives cleaner class boundaries and faster iteration.

---

## 2) Fixed Trial Structure (Gesture + Rest)
Recommended per trial:
- 2 s neutral preparation
- 3 s gesture hold or controlled movement
- 3 s rest

Repeat each gesture for 15 to 20 trials in one session.
Collect at least 2 to 3 sessions on different times/days.

### Why
- Rest segments are needed as baseline and negative class
- Repetition is required for statistical reliability
- Multi-session data improves generalization

---

## 3) Keep Mounting and Calibration Consistent
Before each session:
- Fix Myo position on upper forearm
- Fix STM32 and MPU in the same physical orientation as previous session
- Run gyro calibration while still
- Record one neutral pose for reference

### Why
This minimizes variance caused by hardware placement changes.

---

## 4) Use Current Pipeline for Every Trial
For each recording block:
1. capture_fusion.py
2. merge_fusion.py
3. process_fusion_emg.py

### Why
This keeps one consistent data path from raw acquisition to feature extraction.

---

## 5) Label Every Trial Immediately
Maintain a metadata CSV with at least:
- file_timestamp
- gesture_label
- trial_id
- session_id
- notes (placement quality, unusual motion, noise)

### Why
Unlabeled or poorly labeled trials reduce model value and create analysis ambiguity.

---

## Part B: Immediate Quality Checks

After each session, verify:
- EMG baseline at rest is low and stable
- Gesture windows show clear envelope increase
- imu_match_ratio in feature file is high
- No obvious clipping or channel dropouts

### Why
Early rejection of poor trials saves time and prevents dataset contamination.

---

## Part C: What Is Next (Future Work)

## 1) Build Master Labeled Dataset
Combine all *_emgfeat.csv files with metadata labels into one dataset.

### Why
Training and evaluation require a single consistent table.

---

## 2) Baseline Modeling
Compare:
- EMG-only features
- EMG + roll/pitch fusion features

Evaluate with trial/session-level split (not random row split).

### Why
This quantifies the actual benefit of sensor fusion.

---

## 3) Fusion-Oriented Improvements
After baseline is stable:
- tune feature windows
- test optional notch filtering
- add EMG activity index for adaptive fusion logic

### Why
These steps can improve robustness during dynamic movement and contraction.

---

## 4) Advanced EKF Integration (Later)
Use EMG activity intensity as context to adapt IMU measurement trust during high-motion phases.

### Why
Potentially better roll/pitch stability in challenging motion segments.

---

## Practical Weekly Execution Suggestion
Week 1:
- collect labeled multi-session data with current pipeline

Week 2:
- build master dataset and run baseline EMG vs fused-feature models

Week 3:
- tune preprocessing and fusion settings using quantitative metrics

---

## Success Criteria for This Phase
- complete labeled dataset across multiple sessions
- reproducible feature extraction outputs
- baseline report showing whether fusion improves performance vs EMG-only
