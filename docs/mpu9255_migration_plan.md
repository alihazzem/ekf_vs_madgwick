# MPU-6050 to GY-91 (MPU-9255) Migration Plan (EKF + Madgwick, 9-DoF Yaw)

## 1) Objective

Migrate the current IMU fusion stack from MPU-6050 (6-axis) to the GY-91 module (MPU-9255 9-axis: accel + gyro + AK8963 magnetometer) so yaw is referenced to magnetic north and no longer drifts like gyro-only heading.

> **Note — GY-91 module details:**
> The GY-91 board carries an **MPU-9255** (not MPU-9250) and a **BMP280** barometric pressure sensor.
> The MPU-9255 is register-compatible with the MPU-9250; the only firmware-visible difference is the WHO_AM_I value (`0x73` instead of `0x71`).
> The **BMP280 barometer is present on the board but will NOT be used** in this project — its I2C address (0x76/0x77) and chip-select pin (CSB) are simply left unconnected / ignored in software.

This plan is built specifically for your current project architecture (STM32F411 bare-metal, 100 Hz loop, dual filter pipeline, UART CLI, CSV logging).

---

## 2) Success Criteria (Definition of Done)

1. System boots and detects MPU-9255 and AK8963 consistently.
2. 9-axis data stream is stable at 100 Hz (or chosen synchronized rate) with no missed-read bursts.
3. Madgwick runs in MARG mode (gyro + accel + mag) and provides stable absolute yaw.
4. EKF includes magnetometer measurement update (minimum 7-state + mag update; optional 10-state with mag bias).
5. Magnetometer hard-iron and soft-iron calibration is applied in runtime.
6. CLI can run full calibration, view calibration values, and enable/disable magnetometer fusion.
7. CSV output includes raw and calibrated mag plus filter diagnostic terms.
8. Static yaw drift (after calibration) is bounded by target: <= 1.5 deg/min indoor, <= 0.8 deg/min low-interference environment.
9. Heading repeatability test (0 / 90 / 180 / 270 deg placements) passes target: <= 5 deg RMS error.
10. Existing roll/pitch performance does not regress versus MPU-6050 baseline.

---

## 3) Current Baseline (What Changes)

Current system characteristics:

- Sensor driver: MPU6050 only.
- Fusion input: accel + gyro only.
- Madgwick mode: IMU-only.
- EKF model: 7-state quaternion + gyro bias, accel update only.
- Yaw behavior: relative heading with drift (no absolute heading reference).

Target behavior after migration:

- Sensor driver: GY-91 module — MPU-9255 accel/gyro + AK8963 magnetometer (BMP280 barometer ignored).
- Fusion input: accel + gyro + mag.
- Madgwick mode: MARG update path.
- EKF: add magnetometer measurement update (and optionally mag bias states).
- Yaw behavior: globally referenced heading with controlled drift.

---

## 4) Architecture Decision

## Recommended path

Implement in two stages for low risk:

1. Stage A (required): Keep EKF state at 7, add magnetometer measurement update and gating.
2. Stage B (optional): Extend EKF to 10 states by adding magnetometer bias states if indoor interference still causes heading bias.

Why this is recommended:

- Stage A is much faster to validate and already gives large yaw improvement.
- Stage B can be added only if needed, avoiding unnecessary complexity early.

---

## 5) Hardware and Register Plan

## 5.0 Module overview — GY-91

The GY-91 module exposes the following pins:

| Pin | Function | Notes |
|-----|----------|-------|
| VIN | Power supply | 3.3 V – 5 V (onboard regulator) |
| GND | Ground | |
| 3V3 | Regulated 3.3 V output | Can also be used as 3.3 V input |
| SCL | I2C clock / SPI clock | |
| SDA | I2C data / SPI MOSI | |
| SAO/SDO | I2C addr select / SPI MISO | Directly affects MPU I2C address (0x68 low / 0x69 high) |
| NCS | Chip select — MPU-9255 | Pull high or leave open for I2C mode |
| CSB | Chip select — BMP280 | **Not used** — pull high or leave open |

ICs on board:

- **MPU-9255** — 9-axis IMU (accel + gyro + AK8963 magnetometer die)
- **BMP280** — barometric pressure / temperature sensor → **explicitly ignored in this project**

## 5.1 Electrical and bus assumptions

1. Keep I2C bus at 400 kHz (same as existing).
2. Confirm board wiring for GY-91 module (SCL/SDA, VIN or 3V3, GND). NCS and CSB should be pulled high or left floating for I2C mode.
3. Confirm logic level compatibility (3.3 V).
4. BMP280 will share the I2C bus at address 0x76 or 0x77 — it is safe to ignore; the firmware simply never addresses it.

## 5.2 Device IDs

1. MPU-9255 WHO_AM_I expected: **0x73** (register 117 / 0x75). Note: MPU-9250 returns 0x71 — accept both if future cross-compatibility is desired.
2. AK8963 WHO_AM_I expected: 0x48 (same magnetometer die as MPU-9250).

## 5.3 MPU-9255 configuration

Planned equivalent settings to preserve current behavior:

1. Sample divider and DLPF set for 100 Hz output path.
2. Accel full-scale: +/-2g.
3. Gyro full-scale: +/-250 dps.
4. Enable bypass mode so STM32 can access AK8963 directly over the same I2C lines:
   - USER_CTRL.I2C_MST_EN = 0
   - INT_PIN_CFG.BYPASS_EN = 1

## 5.4 AK8963 configuration

1. Read sensitivity adjustment (ASA) from fuse ROM.
2. Set continuous measurement mode, 16-bit output, 100 Hz (CNTL1 = 0x16).
3. Read sequence each sample: ST1 -> HXL..HZH -> ST2.
4. Discard sample on overflow or invalid status flags.

Scaling model:

- Raw scale at 16-bit mode: 0.15 uT/LSB.
- Apply ASA correction per axis.
- Then apply hard/soft iron calibration matrix.

---

## 6) File-by-File Change Plan

## 6.1 New driver modules

1. Create Core/Inc/drivers/mpu9255.h
2. Create Core/Src/drivers/mpu9255.c
3. Create Core/Inc/drivers/ak8963.h
4. Create Core/Src/drivers/ak8963.c

> No driver is created for the BMP280 — it is intentionally unused.

Suggested APIs:

- mpu9255_whoami (accept 0x73; optionally also 0x71 for MPU-9250 compat)
- mpu9255_init_100hz
- mpu9255_read_accel_gyro_raw
- mpu9255_enable_bypass
- ak8963_whoami
- ak8963_read_asa
- ak8963_init_continuous_100hz
- ak8963_read_raw

## 6.2 Application and data model changes

1. Update Core/Inc/app/imu_types.h:
   - Add raw and calibrated magnetometer fields.
   - Add validity/status bits for mag sample.
2. Update Core/Src/app/imu_app.c:
   - Replace MPU6050 read path with MPU9255 + AK8963 fused read flow.
   - Add mag unit conversion and calibration application.
   - Add mag axis remap (separate map from accel/gyro if needed).
   - Add fallback behavior when mag sample invalid.
3. Update Core/Inc/app/app_config.h:
   - Sensor select macro (MPU6050 legacy or GY91/MPU9255 new).
   - Mag noise/gating thresholds.
   - Optional declination constant.
   - BMP280 is NOT referenced anywhere — no barometer config needed.

## 6.3 Filter modules

1. Update Core/Inc/filters/madgwick.h and Core/Src/filters/madgwick.c:
   - Add madgwick_update_marg (9-axis update).
   - Keep madgwick_update_imu for fallback and A/B testing.
2. Update Core/Inc/filters/ekf.h and Core/Src/filters/ekf.c:
   - Add magnetometer measurement update routine.
   - Add R_mag tuning and innovation gating.
   - Optional: add 3 mag bias states in Stage B.

## 6.4 CLI and telemetry

1. Update Core/Src/app/cli_app.c:
   - New commands for mag calibration and diagnostics.
2. Update streaming line format in Core/Src/app/imu_app.c:
   - Include mx_raw,my_raw,mz_raw and mx_cal,my_cal,mz_cal.
   - Include mag valid flag and yaw diagnostics.
3. Update Python/MATLAB tooling under scripts and matlab:
   - Parse new CSV schema.
   - Add heading validation plots.

## 6.5 Documentation

1. Update docs/README.md with GY-91 / MPU-9255 architecture notes.
2. Update docs/modules.md to include new drivers.
3. Update docs/filter_explanation.md for 9-axis fusion math.
4. Keep this migration plan as execution checklist.

---

## 7) Detailed Implementation Phases

## Phase 0: Baseline freeze (before any code migration)

Tasks:

1. Tag or branch current MPU-6050 stable state.
2. Capture baseline logs for roll/pitch/yaw drift and CPU timing.
3. Save baseline metrics table for regression checks.

Deliverables:

- Baseline CSV + plots.
- Baseline timing and drift summary.

Exit criteria:

- You can compare post-migration behavior quantitatively.

## Phase 1: Driver bring-up (GY-91: MPU-9255 + AK8963)

Tasks:

1. Implement MPU-9255 init and accel/gyro burst read (registers identical to MPU-9250; accept WHO_AM_I = 0x73).
2. Implement bypass mode and AK8963 bring-up.
3. Implement AK8963 status-safe read.
4. Verify both IDs over CLI diagnostic command.
5. Confirm BMP280 is visible on the bus (optional sanity check) but do NOT initialize or read it.

Deliverables:

- Stable raw accel/gyro/mag read at target rate.

Exit criteria:

- No sporadic I2C lockups for 10+ minutes continuous run.

## Phase 2: Coordinate frames and scaling

Tasks:

1. Convert raw accel/gyro units exactly as current baseline.
2. Convert raw mag to uT including ASA correction.
3. Determine mag axis remap to body frame.
4. Validate sign conventions by simple rotations around each body axis.

Deliverables:

- Consistent body-frame vectors for all sensors.

Exit criteria:

- Axis sanity tests pass (right-hand rule and expected sign).

## Phase 3: Magnetometer calibration pipeline

Tasks:

1. Add mag data collection mode (CLI command).
2. Collect figure-8 / full-orientation cloud.
3. Compute hard-iron offset vector b and soft-iron matrix S offline.
4. Store calibration in firmware config (initially compile-time).
5. Apply calibration online:
   - m_corr = S * (m_raw - b)

Future enhancement:

- Store calibration in flash so it persists without recompiling.

Deliverables:

- Calibrated magnetic field sphere/ellipsoid corrected.

Exit criteria:

- Norm variation reduced significantly (target <= 10 to 15 percent in normal environment).

## Phase 4: Madgwick 9-axis integration

Tasks:

1. Add MARG update function using accel + gyro + mag.
2. Keep beta ramp and zeta bias adaptation behavior.
3. Add magnetic anomaly gating:
   - reject mag update if norm out of expected band or innovation too large.
4. Add fallback to IMU-only update when mag invalid.

Deliverables:

- Madgwick yaw locks to heading and recovers after motion.

Exit criteria:

- Static heading drift within target; no instability in roll/pitch.

## Phase 5: EKF magnetometer update

## Stage A (required): 7-state + mag measurement update

Tasks:

1. Keep state x = [q, b_g].
2. Add magnetometer measurement model h_mag(q) using earth magnetic reference vector.
3. Add Jacobian H_mag wrt quaternion states.
4. Perform sequential updates each cycle:
   - accel update
   - mag update
5. Add adaptive R_mag and innovation gating.

Deliverables:

- EKF yaw absolute referencing with controlled correction strength.

Exit criteria:

- Yaw RMS error and drift improve significantly over 6-axis EKF.

## Stage B (optional): 10-state EKF with mag bias

Tasks:

1. Extend state to include b_mx,b_my,b_mz.
2. Add process model for slow-varying mag bias.
3. Update H_mag and covariance dimensions.
4. Re-tune Q and R.

When to do Stage B:

- Only if Stage A shows persistent heading bias in real environment that calibration + gating cannot solve.

## Phase 6: Runtime controls, logging, tooling

Tasks:

1. Add CLI commands:
   - imu mag on/off
   - imu magcal start/stop/dump
   - imu mag status
   - ekf rmag set
   - mad magreject set
2. Extend CSV protocol with mag channels and flags.
3. Update scripts and MATLAB plots to include heading error metrics.

Deliverables:

- End-to-end observability and tunability from CLI.

Exit criteria:

- You can tune and validate without reflashing for every parameter change.

## Phase 7: Validation and acceptance testing

Test suite:

1. Static drift test (5 to 10 min).
2. Cardinal heading placement test (0/90/180/270 deg).
3. Slow yaw sweep test (manual turntable style).
4. Aggressive motion test (to verify gating/fallback).
5. Magnetic disturbance test (bring metal object near sensor, then remove).
6. Thermal warm-up test (first 5 minutes after power on).

Metrics to log:

1. Yaw drift rate (deg/min).
2. Heading RMS error at cardinal poses.
3. Recovery time after disturbance.
4. EKF trace(P), innovation magnitudes, reject counters.
5. CPU time per filter step.

Exit criteria:

- All success criteria in Section 2 met.

## Phase 8: Deployment and fallback strategy

Tasks:

1. Keep compile-time switch for legacy MPU-6050 path until GY-91 / MPU-9255 path is fully accepted.
2. Preserve IMU-only fallback mode in filters.
3. Document known magnetic-interference limitations.

Fallback policy:

- If mag quality is bad, auto-fallback to gyro+accel and set heading quality flag low.

---

## 8) EKF and Madgwick Tuning Plan (Practical)

## Madgwick

1. Start from current beta and zeta values.
2. Tune mag fusion gain to avoid yaw oscillation.
3. Add separate thresholds for:
   - accel trust window
   - mag trust window
4. Validate beta ramp does not over-correct heading on startup.

## EKF

1. Keep current accel noise parameters initially.
2. Introduce R_mag diagonal and set conservative high value first.
3. Gradually lower R_mag until yaw responsiveness is acceptable without noise.
4. Use innovation gating for outlier rejection.
5. If using Stage B, set mag bias random walk low and re-evaluate convergence speed.

---

## 9) Risk Register and Mitigation

1. Risk: Magnetometer heavily distorted by nearby ferromagnetic materials.
   - Mitigation: strong gating, calibration, quality flag, IMU-only fallback.
2. Risk: Axis misalignment between mag and accel/gyro causes wrong yaw.
   - Mitigation: dedicated axis validation routine and remap matrix tests.
3. Risk: EKF complexity increase impacts CPU and numeric stability.
   - Mitigation: stage rollout, static scratch buffers, covariance symmetry enforcement.
4. Risk: Calibration not persisted across resets.
   - Mitigation: add flash storage after first stable release.
5. Risk: Indoor environment changes invalidate calibration quality.
   - Mitigation: quick recalibration command and calibration versioning.

---

## 10) Proposed Command Additions (CLI)

1. imu mag status
2. imu mag on
3. imu mag off
4. imu magcal start
5. imu magcal stop
6. imu magcal dump
7. imu magcal apply bx by bz s11 s12 s13 s21 s22 s23 s31 s32 s33
8. ekf rmag set value
9. ekf maggate set threshold
10. mad maggate set threshold
11. imu heading quality

---

## 11) Implementation Checklist

- [ ] Baseline metrics captured on MPU-6050
- [ ] MPU-9255 accel/gyro driver integrated (GY-91 module)
- [ ] AK8963 driver integrated
- [ ] 9-axis sample path added in imu_app
- [ ] Mag unit conversion and ASA correction validated
- [ ] Hard-iron and soft-iron calibration applied
- [ ] Madgwick MARG mode implemented
- [ ] EKF magnetometer update implemented (Stage A)
- [ ] Optional EKF Stage B evaluated
- [ ] CLI controls for mag and calibration added
- [ ] CSV and scripts updated
- [ ] Validation suite completed
- [ ] Documentation updated

---

## 12) Suggested Execution Timeline

1. Day 1 to 2: Phase 0 + Phase 1
2. Day 3: Phase 2
3. Day 4 to 5: Phase 3
4. Day 6: Phase 4
5. Day 7 to 8: Phase 5 Stage A
6. Day 9: Phase 6
7. Day 10: Phase 7 + release decision

Optional Stage B (10-state EKF): add 2 to 4 additional days.

---

## 13) Final Notes

1. Treat magnetometer quality as dynamic, not always trustworthy.
2. Keep dual-mode operation (9-axis and IMU-only) for robustness.
3. Validate yaw with objective metrics, not only visual smoothness.
4. Preserve your current strong timing/diagnostic instrumentation during migration.
5. The BMP280 barometer on the GY-91 module is intentionally unused — do not add any barometer code or configuration.
