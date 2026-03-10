% plot_accel_lpf.m  —  Plot raw + low-pass filtered acceleration from CSV

clear; clc; close all;

% ── Select CSV file ──────────────────────────────────────────────────────

CSV_FILE = "ekf_capture_1.csv";   % leave "" to open picker

if CSV_FILE == ""
    [fname, fpath] = uigetfile("*.csv", "Select CSV file");
    if isequal(fname,0)
        disp("No file selected.");
        return;
    end
    CSV_FILE = fullfile(fpath,fname);
end

% ── Load data ────────────────────────────────────────────────────────────

T = readtable(CSV_FILE);

t_ms = T.t_ms;
t_s  = (t_ms - t_ms(1)) / 1000.0;

ACCEL_SCALE = 1 / 16384;

% Sensor → body remapping (same as imu_app.c)
ax = -T.ay_raw * ACCEL_SCALE;
ay = -T.az_raw * ACCEL_SCALE;
az =  T.ax_raw * ACCEL_SCALE;

% ── Sampling rate estimation ─────────────────────────────────────────────

dt = mean(diff(t_s));
fs = 1 / dt;

fprintf("Estimated sampling rate: %.2f Hz\n", fs);

% ── Low-pass filter design (10 Hz) ───────────────────────────────────────

fc = 10;   % cutoff frequency

[b,a] = butter(4, fc/(fs/2), "low");

% zero-phase filtering
ax_lp = filtfilt(b,a,ax);
ay_lp = filtfilt(b,a,ay);
az_lp = filtfilt(b,a,az);

% ── Plot RAW acceleration ────────────────────────────────────────────────

figure("Name","Raw Acceleration","NumberTitle","off");
plot(t_s, ax, "r", "DisplayName","ax"); hold on;
plot(t_s, ay, "g", "DisplayName","ay");
plot(t_s, az, "b", "DisplayName","az");

yline(0,"k--","HandleVisibility","off");

xlabel("Time (s)");
ylabel("Acceleration (g)");
title("Raw Accelerometer Data");
legend("Location","best");
grid on;

% ── Plot LOW-PASS filtered acceleration ──────────────────────────────────

figure("Name","Low-Pass Filtered Acceleration","NumberTitle","off");

plot(t_s, ax_lp, "r", "DisplayName","ax filtered"); hold on;
plot(t_s, ay_lp, "g", "DisplayName","ay filtered");
plot(t_s, az_lp, "b", "DisplayName","az filtered");

yline(0,"k--","HandleVisibility","off");

xlabel("Time (s)");
ylabel("Acceleration (g)");
title("Acceleration After Low-Pass Filter");
legend("Location","best");
grid on;