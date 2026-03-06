% plot_ekf.m  —  Plot raw sensor data + EKF output from captured CSV
%
% Usage
% -----
%   1. Run  capture_ekf.py  while the board streams with RUN_EKF = 1
%   2. Place the generated CSV in the matlab/ folder (or update CSV_FILE)
%   3. Run this script in MATLAB
%
% CSV columns (from capture_ekf.py / extended D, stream)
% -------------------------------------------------------
%   t_ms                           board timestamp (ms)
%   ax_raw  ay_raw  az_raw         accel  int16   (÷16384 → g)
%   gx_raw  gy_raw  gz_raw         gyro   int16   (÷131   → dps)
%   mad_roll  mad_pitch  mad_yaw   Madgwick angles (mdeg, may be 0)
%   mad_us                         Madgwick CPU time (µs)
%   ekf_roll  ekf_pitch  ekf_yaw   EKF angles (mdeg)
%   traceP_1e6                     trace(P) × 1e6
%   ekf_us                         EKF CPU time (µs)
%   bx_uradps  by_uradps  bz_uradps  EKF gyro bias (µrad/s)

clear; clc; close all;

% ── Select CSV file ──────────────────────────────────────────────────────────

CSV_FILE = "raw_vs_ekf.csv";   % leave empty "" to open a file picker

if CSV_FILE == ""
    [fname, fpath] = uigetfile("*.csv", "Select EKF capture CSV");
    if isequal(fname, 0), disp("No file selected."); return; end
    CSV_FILE = fullfile(fpath, fname);
end

% ── Load data ─────────────────────────────────────────────────────────────────

T = readtable(CSV_FILE);

t_ms = T.t_ms;
t_s  = (t_ms - t_ms(1)) / 1000.0;

ACCEL_SCALE = 1.0 / 16384.0;
GYRO_SCALE  = 1.0 / 131.0;
ANGLE_SCALE = 1.0 / 1000.0;    % mdeg → deg
TRACEP_SCALE = 1.0 / 1e6;      % undo ×1e6
BIAS_SCALE  = 1.0 / 1e6;       % µrad/s → rad/s

% Sensor → body remapping (same as imu_app.c)
ax = -T.ay_raw * ACCEL_SCALE;
ay = -T.az_raw * ACCEL_SCALE;
az =  T.ax_raw * ACCEL_SCALE;

gx = -T.gy_raw * GYRO_SCALE;
gy = -T.gz_raw * GYRO_SCALE;
gz =  T.gx_raw * GYRO_SCALE;

ekf_roll  = T.ekf_roll_mdeg  * ANGLE_SCALE;
ekf_pitch = T.ekf_pitch_mdeg * ANGLE_SCALE;
ekf_yaw   = T.ekf_yaw_mdeg   * ANGLE_SCALE;

traceP    = T.traceP_1e6 * TRACEP_SCALE;
ekf_us    = T.ekf_us;

bx = T.bx_uradps * BIAS_SCALE;   % rad/s
by = T.by_uradps * BIAS_SCALE;
bz = T.bz_uradps * BIAS_SCALE;

% ── Stats ─────────────────────────────────────────────────────────────────────

n_samp   = height(T);
dur_s    = t_s(end);
avg_rate = n_samp / dur_s;
fprintf("Samples  : %d\n",    n_samp);
fprintf("Duration : %.2f s\n", dur_s);
fprintf("Avg rate : %.1f Hz\n", avg_rate);
fprintf("EKF us   : mean=%.1f  max=%.0f\n", mean(ekf_us), max(ekf_us));

% ── Plot ──────────────────────────────────────────────────────────────────────

fig = figure("Name", "Raw vs EKF", "NumberTitle", "off");
fig.Position = [80 80 1200 850];

% ── Subplot 1: Accelerometer ──────────────────────────────────────────────────
sp1 = subplot(4, 1, 1);
plot(t_s, ax, "r", "DisplayName", "ax"); hold on;
plot(t_s, ay, "g", "DisplayName", "ay");
plot(t_s, az, "b", "DisplayName", "az");
yline(0, "k--", "LineWidth", 0.5, "HandleVisibility", "off");
ylabel("Accel (g)");
title("Accelerometer  (flat: az ≈ +1 g)");
legend("Location", "best"); grid on;

% ── Subplot 2: Gyroscope ──────────────────────────────────────────────────────
sp2 = subplot(4, 1, 2);
plot(t_s, gx, "r", "DisplayName", "gx"); hold on;
plot(t_s, gy, "g", "DisplayName", "gy");
plot(t_s, gz, "b", "DisplayName", "gz");
yline(0, "k--", "LineWidth", 0.5, "HandleVisibility", "off");
ylabel("Gyro (dps)");
title("Gyroscope");
legend("Location", "best"); grid on;

% ── Subplot 3: EKF Euler angles ───────────────────────────────────────────────
sp3 = subplot(4, 1, 3);
plot(t_s, ekf_roll,  "r", "DisplayName", "roll");   hold on;
plot(t_s, ekf_pitch, "g", "DisplayName", "pitch");
plot(t_s, ekf_yaw,   "b", "DisplayName", "yaw");
yline(0, "k--", "LineWidth", 0.5, "HandleVisibility", "off");
ylabel("Angle (deg)");
title("EKF output  —  Euler angles");
legend("Location", "best"); grid on;

% ── Subplot 4: Convergence (trace(P)) + Bias ──────────────────────────────────
sp4 = subplot(4, 1, 4);
yyaxis left;
semilogy(t_s, traceP + 1e-9, "m-", "LineWidth", 1.2, "DisplayName", "trace(P)");
ylabel("trace(P)  [log]");

yyaxis right;
plot(t_s, bx * 1000, "r--", "DisplayName", "bias x (mrad/s)"); hold on;
plot(t_s, by * 1000, "g--", "DisplayName", "bias y (mrad/s)");
plot(t_s, bz * 1000, "b--", "DisplayName", "bias z (mrad/s)");
ylabel("Gyro bias (mrad/s)");

xlabel("Time (s)");
title("EKF convergence  —  trace(P) decreases as filter settles");
legend("Location", "best"); grid on;

linkaxes([sp1 sp2 sp3 sp4], "x");
sgtitle(sprintf("Raw vs EKF  |  %d samples  |  %.1f Hz  |  EKF avg %.1f µs/step", ...
                n_samp, avg_rate, mean(ekf_us)));

% ── Save PNG ──────────────────────────────────────────────────────────────────
[~, csv_name, ~] = fileparts(CSV_FILE);
png_file = csv_name + ".png";
exportgraphics(fig, png_file, "Resolution", 150);
fprintf("Saved : %s\n", png_file);
