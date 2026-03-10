% plot_imu.m  —  Plot captured IMU data + Madgwick output
%
% Usage
% -----
%   1. Run capture.py on your PC while the board is streaming
%   2. Place the generated CSV in the same folder as this script, or update
%      CSV_FILE below with the full path
%   3. Run this script in MATLAB
%
% CSV columns (from capture.py)
% ------------------------------
%   t_ms                          board timestamp (ms)
%   ax_raw  ay_raw  az_raw        accel  int16  (÷16384 → g  at ±2g)
%   gx_raw  gy_raw  gz_raw        gyro   int16  (÷131   → dps at ±250dps)
%   roll_mdeg  pitch_mdeg  yaw_mdeg   Madgwick angles (÷1000 → degrees)

clear; clc; close all;

% ── Select CSV file ─────────────────────────────────────────────────────────

CSV_FILE = "ekf_capture_20260310_142321.csv";   % leave empty to open a file picker

if CSV_FILE == ""
    [fname, fpath] = uigetfile("*.csv", "Select IMU capture CSV");
    if isequal(fname, 0)
        disp("No file selected. Exiting.");
        return;
    end
    CSV_FILE = fullfile(fpath, fname);
end

% ── Load data ────────────────────────────────────────────────────────────────

T = readtable(CSV_FILE);

t_ms = T.t_ms;
t_s  = (t_ms - t_ms(1)) / 1000.0;   % relative time in seconds

% Convert raw to physical units
ACCEL_SCALE = 1.0 / 16384.0;   % g  (±2g range, 16-bit)
GYRO_SCALE  = 1.0 / 131.0;     % dps (±250dps range, 16-bit)
ANGLE_SCALE = 1.0 / 1000.0;    % degrees (from millidegrees)

% Apply the same sensor→body remapping used in imu_app.c:
%   ax_body =  -ay_sensor
%   ay_body =  -az_sensor
%   az_body =   ax_sensor
% (board flat → ax_body ≈ +1 g, az_body ≈ 0 g)
ax = -T.ay_raw * ACCEL_SCALE;
ay = -T.az_raw * ACCEL_SCALE;
az =  T.ax_raw * ACCEL_SCALE;

% Gyro remapping (same permutation):
%   wx_body = -gy_sensor
%   wy_body = -gz_sensor
%   wz_body =  gx_sensor
gx = -T.gy_raw * GYRO_SCALE;
gy = -T.gz_raw * GYRO_SCALE;
gz =  T.gx_raw * GYRO_SCALE;

roll  = T.roll_mdeg  * ANGLE_SCALE;
pitch = T.pitch_mdeg * ANGLE_SCALE;
yaw   = T.yaw_mdeg   * ANGLE_SCALE;

% ── Print basic stats ────────────────────────────────────────────────────────

n_samples  = height(T);
duration_s = t_s(end);
avg_rate   = n_samples / duration_s;

fprintf("Samples  : %d\n",   n_samples);
fprintf("Duration : %.2f s\n", duration_s);
fprintf("Avg rate : %.1f Hz\n", avg_rate);

% ── Plot ─────────────────────────────────────────────────────────────────────

fig = figure("Name", "IMU Capture", "NumberTitle", "off");
fig.Position = [100 100 1100 750];

% ─── Subplot 1: Accelerometer ────────────────────────────────────────────────
ax1 = subplot(3, 1, 1);
plot(t_s, ax, "r",  "DisplayName", "ax");  hold on;
plot(t_s, ay, "g",  "DisplayName", "ay");
plot(t_s, az, "b",  "DisplayName", "az");
yline(0, "k--", "LineWidth", 0.5, "HandleVisibility", "off");
xlabel("Time (s)");
ylabel("Accel (g)");
title("Accelerometer — body frame  (flat: ax≈0, ay≈0, az≈+1 g)");
legend("Location", "best");
grid on;

% ─── Subplot 2: Gyroscope ─────────────────────────────────────────────────────
ax2 = subplot(3, 1, 2);
plot(t_s, gx, "r",  "DisplayName", "gx");  hold on;
plot(t_s, gy, "g",  "DisplayName", "gy");
plot(t_s, gz, "b",  "DisplayName", "gz");
yline(0, "k--", "LineWidth", 0.5, "HandleVisibility", "off");
xlabel("Time (s)");
ylabel("Gyro (dps)");
title("Gyroscope — body frame");
legend("Location", "best");
grid on;

% ─── Subplot 3: Madgwick Euler angles ─────────────────────────────────────────
ax3 = subplot(3, 1, 3);
plot(t_s, roll,  "r",  "DisplayName", "roll");   hold on;
plot(t_s, pitch, "g",  "DisplayName", "pitch");
plot(t_s, yaw,   "b",  "DisplayName", "yaw");
yline(0, "k--", "LineWidth", 0.5, "HandleVisibility", "off");
xlabel("Time (s)");
ylabel("Angle (deg)");
title("Madgwick filter output");
legend("Location", "best");
grid on;

linkaxes([ax1 ax2 ax3], "x");   % sync x-axis zoom across all subplots
sgtitle(sprintf("IMU Capture  |  %d samples  |  %.1f Hz avg", n_samples, avg_rate))
