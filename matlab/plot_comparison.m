% plot_comparison.m  —  Side-by-side Madgwick vs EKF comparison
%
% Usage
% -----
%   1. Run  capture_ekf.py  with BOTH RUN_MADGWICK=1 and RUN_EKF=1
%   2. Place the generated CSV in the matlab/ folder (or update CSV_FILE)
%   3. Run this script in MATLAB
%
% This script produces 4 comparison panels:
%   [1] Roll:   Madgwick vs EKF overlaid
%   [2] Pitch:  Madgwick vs EKF overlaid
%   [3] EKF convergence — trace(P) over time (no Madgwick equivalent)
%   [4] CPU cost — mad_us vs ekf_us per step (shows computational trade-off)
%
% A second figure shows the EKF's gyro bias estimate (Kalman-optimal) vs the
% Madgwick zeta-based bias (not in the stream — use EKF BIAS via CLI for that).

clear; clc; close all;

% ── Select CSV file ───────────────────────────────────────────────────────────

CSV_FILE = "mad_vs_ekf.csv";   % leave empty "" to open a file picker

if CSV_FILE == ""
    [fname, fpath] = uigetfile("*.csv", "Select comparison CSV");
    if isequal(fname, 0), disp("No file selected."); return; end
    CSV_FILE = fullfile(fpath, fname);
end

% ── Load data ─────────────────────────────────────────────────────────────────

T = readtable(CSV_FILE);

t_ms = T.t_ms;
t_s  = (t_ms - t_ms(1)) / 1000.0;

ANGLE_SCALE  = 1.0 / 1000.0;
TRACEP_SCALE = 1.0 / 1e6;
BIAS_SCALE   = 1.0 / 1e6;        % µrad/s → rad/s

mad_roll  = T.mad_roll_mdeg  * ANGLE_SCALE;
mad_pitch = T.mad_pitch_mdeg * ANGLE_SCALE;
mad_yaw   = T.mad_yaw_mdeg   * ANGLE_SCALE;
mad_us    = T.mad_us;

ekf_roll  = T.ekf_roll_mdeg  * ANGLE_SCALE;
ekf_pitch = T.ekf_pitch_mdeg * ANGLE_SCALE;
ekf_yaw   = T.ekf_yaw_mdeg   * ANGLE_SCALE;
traceP    = T.traceP_1e6 * TRACEP_SCALE;
ekf_us    = T.ekf_us;

bx = T.bx_uradps * BIAS_SCALE;   % rad/s
by = T.by_uradps * BIAS_SCALE;
bz = T.bz_uradps * BIAS_SCALE;

% ── Stats ──────────────────────────────────────────────────────────────────────

n_samp   = height(T);
dur_s    = t_s(end);
avg_rate = n_samp / dur_s;

fprintf("── Samples / timing ──────────────────────────────\n");
fprintf("Samples  : %d\n",    n_samp);
fprintf("Duration : %.2f s\n", dur_s);
fprintf("Avg rate : %.1f Hz\n", avg_rate);
fprintf("\n── CPU cost (µs / step at 100 Hz, budget = 10000 µs) ──\n");
fprintf("Madgwick : mean=%.1f  max=%.0f  (%.3f%% of budget)\n", ...
        mean(mad_us), max(mad_us), mean(mad_us)/100.0);
fprintf("EKF      : mean=%.1f  max=%.0f  (%.3f%% of budget)\n", ...
        mean(ekf_us), max(ekf_us), mean(ekf_us)/100.0);
fprintf("Ratio    : EKF is %.1fx more expensive than Madgwick\n", ...
        mean(ekf_us) / max(mean(mad_us), 1));

% ── Figure 1: Angle + convergence + CPU ───────────────────────────────────────

fig1 = figure("Name", "Madgwick vs EKF — Comparison", "NumberTitle", "off");
fig1.Position = [60 60 1200 870];

% Panel 1 — Roll
sp1 = subplot(4, 1, 1);
plot(t_s, mad_roll,  "b",  "LineWidth", 1.0, "DisplayName", "Madgwick"); hold on;
plot(t_s, ekf_roll,  "r--","LineWidth", 1.2, "DisplayName", "EKF");
yline(0, "k--", "LineWidth", 0.5, "HandleVisibility", "off");
ylabel("Roll (deg)");
title("Roll — Madgwick vs EKF");
legend("Location", "best"); grid on;

% Panel 2 — Pitch
sp2 = subplot(4, 1, 2);
plot(t_s, mad_pitch, "b",  "LineWidth", 1.0, "DisplayName", "Madgwick"); hold on;
plot(t_s, ekf_pitch, "r--","LineWidth", 1.2, "DisplayName", "EKF");
yline(0, "k--", "LineWidth", 0.5, "HandleVisibility", "off");
ylabel("Pitch (deg)");
title("Pitch — Madgwick vs EKF");
legend("Location", "best"); grid on;

% Panel 3 — EKF trace(P): convergence metric
%   Madgwick has no equivalent — this is a key EKF advantage
sp3 = subplot(4, 1, 3);
semilogy(t_s, traceP + 1e-9, "m-", "LineWidth", 1.4);
ylabel("trace(P)  [log]");
title("EKF trace(P)  —  statistical convergence (Madgwick has no equivalent)");
xline(0, "k:");
grid on;

% Panel 4 — CPU cost per step
sp4 = subplot(4, 1, 4);
plot(t_s, mad_us, "b",  "DisplayName", sprintf("Madgwick (mean %.1f µs)", mean(mad_us)));
hold on;
plot(t_s, ekf_us, "r--","DisplayName", sprintf("EKF      (mean %.1f µs)", mean(ekf_us)));
ylabel("Step time (µs)");
xlabel("Time (s)");
title("CPU cost per step  —  measured on hardware at 100 Hz");
legend("Location", "best"); grid on;

linkaxes([sp1 sp2 sp3 sp4], "x");
sgtitle(sprintf("Madgwick vs EKF Comparison  |  %d samples  |  %.1f Hz", ...
                n_samp, avg_rate));

[~, csv_name, ~] = fileparts(CSV_FILE);
png1 = csv_name + "_angles.png";
exportgraphics(fig1, png1, "Resolution", 150);
fprintf("Saved : %s\n", png1);

% ── Figure 2: Gyro bias estimate ──────────────────────────────────────────────
%   Shows EKF Kalman-optimal bias convergence
%   Madgwick's zeta-based bias is internal and not in the stream,
%   but can be checked with MAD SHOW after capture.

fig2 = figure("Name", "EKF Gyro Bias Estimate", "NumberTitle", "off");
fig2.Position = [80 80 1000 400];

plot(t_s, bx * 1e3, "r", "DisplayName", "b_x (mrad/s)"); hold on;
plot(t_s, by * 1e3, "g", "DisplayName", "b_y (mrad/s)");
plot(t_s, bz * 1e3, "b", "DisplayName", "b_z (mrad/s)");
yline(0, "k--", "LineWidth", 0.5, "HandleVisibility", "off");
xlabel("Time (s)");
ylabel("Bias (mrad/s)");
title("EKF Gyro Bias Estimate  —  Kalman-optimal tracking");
legend("Location", "best"); grid on;

png2 = csv_name + "_bias.png";
exportgraphics(fig2, png2, "Resolution", 150);
fprintf("Saved : %s\n", png2);

% ── Figure 3: Angle difference (Madgwick - EKF) ───────────────────────────────
%   Shows divergence between the two filters (where they disagree)

fig3 = figure("Name", "Madgwick - EKF Angle Difference", "NumberTitle", "off");
fig3.Position = [100 100 1000 450];

sp_r = subplot(2, 1, 1);
plot(t_s, mad_roll - ekf_roll, "k", "LineWidth", 0.8);
yline(0, "b--", "LineWidth", 0.5);
ylabel("Δ Roll (deg)");
title("Roll difference: Madgwick − EKF  (diverges during dynamic motion)");
grid on;

sp_p = subplot(2, 1, 2);
plot(t_s, mad_pitch - ekf_pitch, "k", "LineWidth", 0.8);
yline(0, "b--", "LineWidth", 0.5);
ylabel("Δ Pitch (deg)");
xlabel("Time (s)");
title("Pitch difference: Madgwick − EKF");
grid on;

linkaxes([sp_r sp_p], "x");
sgtitle("Where the filters diverge shows the EKF adaptive-R effect during motion");

png3 = csv_name + "_diff.png";
exportgraphics(fig3, png3, "Resolution", 150);
fprintf("Saved : %s\n", png3);
