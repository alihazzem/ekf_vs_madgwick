% plot_fusion_roll_pitch_emg.m
% Plot roll and pitch together with 8-channel EMG from merged fusion CSV.
%
% Expected input: fusion_merged_*.csv (produced by scripts/merge_fusion.py)
% Required columns:
%   pc_t_ms, emg0..emg7,
%   imu_match,
%   imu_ekf_roll_mdeg + imu_ekf_pitch_mdeg  (default)
%   or imu_mad_roll_mdeg + imu_mad_pitch_mdeg
%
% Notes:
% - Yaw is intentionally not plotted.
% - If imu_match == 0 for some rows, roll/pitch are set to NaN.

clear; clc; close all;

% ── User settings ────────────────────────────────────────────────────────────
CSV_FILE = "../emg_data/fusion_merged_20260328_130530.csv";              % leave empty to auto-pick latest fusion_merged_*.csv
ANGLE_SRC = "EKF";          % "EKF" or "MAD"
USE_MATCHED_ONLY = false;   % true = keep only rows where imu_match == 1
EMG_RMS_WIN_MS = 150;       % moving RMS window for EMG summary panel
% ─────────────────────────────────────────────────────────────────────────────

% Resolve default input file
if CSV_FILE == ""
    d = dir("../emg_data/fusion_merged_*.csv");
    if isempty(d)
        [fname, fpath] = uigetfile("*.csv", "Select merged fusion CSV", "../emg_data");
        if isequal(fname, 0)
            disp("No file selected.");
            return;
        end
        CSV_FILE = fullfile(fpath, fname);
    else
        [~, idx] = max([d.datenum]);
        CSV_FILE = fullfile(d(idx).folder, d(idx).name);
    end
end

fprintf("Using file: %s\n", CSV_FILE);

T = readtable(CSV_FILE);
vars = string(T.Properties.VariableNames);

% Validate EMG columns
req_emg = ["pc_t_ms", compose("emg%d", 0:7)];
missing_emg = setdiff(req_emg, vars);
if ~isempty(missing_emg)
    error("Missing required EMG columns: %s", strjoin(missing_emg, ", "));
end

% Angle source selection
src = upper(string(ANGLE_SRC));
switch src
    case "EKF"
        roll_col = "imu_ekf_roll_mdeg";
        pitch_col = "imu_ekf_pitch_mdeg";
    case "MAD"
        roll_col = "imu_mad_roll_mdeg";
        pitch_col = "imu_mad_pitch_mdeg";
    otherwise
        error("ANGLE_SRC must be 'EKF' or 'MAD'.");
end

req_ang = [roll_col, pitch_col];
missing_ang = setdiff(req_ang, vars);
if ~isempty(missing_ang)
    error("Missing required angle columns: %s", strjoin(missing_ang, ", "));
end

if any(vars == "imu_match")
    imu_match = logical(T.imu_match);
else
    imu_match = true(height(T), 1);
end

if USE_MATCHED_ONLY
    T = T(imu_match, :);
    imu_match = true(height(T), 1);
end

if height(T) < 2
    error("Not enough rows to plot.");
end

% Time axis
pc_t_ms = double(T.pc_t_ms);
t_s = (pc_t_ms - pc_t_ms(1)) / 1000.0;

% Signals
emg = [T.emg0, T.emg1, T.emg2, T.emg3, T.emg4, T.emg5, T.emg6, T.emg7];
roll_deg = double(T.(roll_col)) / 1000.0;
pitch_deg = double(T.(pitch_col)) / 1000.0;

% Hide unmatched angle points on plot
roll_deg(~imu_match) = NaN;
pitch_deg(~imu_match) = NaN;

% Sampling estimate for RMS window
dt = diff(pc_t_ms);
dt_pos = dt(dt > 0);
if isempty(dt_pos)
    fs = 200;
else
    fs = 1000 / mean(dt_pos);
end

rms_win = max(1, round(EMG_RMS_WIN_MS * fs / 1000));
emg_rms = zeros(size(emg));
for ch = 1:8
    emg_rms(:, ch) = sqrt(movmean(double(emg(:, ch)).^2, [rms_win-1 0]));
end
emg_rms_mean = mean(emg_rms, 2);

% Stats
dur_s = t_s(end);
fprintf("Samples: %d\n", height(T));
fprintf("Duration: %.2f s\n", dur_s);
fprintf("Estimated rate: %.1f Hz\n", fs);
fprintf("Angle source: %s (yaw intentionally not plotted)\n", src);
fprintf("Matched rows: %.2f%%\n", 100 * mean(imu_match));

% Colors
C_ROLL = [0.85 0.20 0.20];
C_PITCH = [0.10 0.45 0.85];
C_EMG = [0.15 0.15 0.15];

% ── Figure 1: Roll/Pitch + EMG RMS summary ─────────────────────────────────
fig1 = figure("Name", "Roll/Pitch + EMG Summary", "NumberTitle", "off");
fig1.Position = [80 80 1200 520];

ax1 = subplot(2,1,1);
plot(t_s, roll_deg, "Color", C_ROLL, "LineWidth", 1.2, "DisplayName", "Roll"); hold on;
plot(t_s, pitch_deg, "Color", C_PITCH, "LineWidth", 1.2, "DisplayName", "Pitch");
yline(0, "k--", "LineWidth", 0.5, "HandleVisibility", "off");
grid on;
ylabel("Angle (deg)");
title(sprintf("%s Roll/Pitch (Yaw excluded)", src));
legend("Location", "best");

ax2 = subplot(2,1,2);
plot(t_s, emg_rms_mean, "Color", C_EMG, "LineWidth", 1.1);
grid on;
xlabel("Time (s)");
ylabel("Mean EMG RMS");
title(sprintf("EMG activity summary (mean RMS of 8 channels, window=%d ms)", EMG_RMS_WIN_MS));

linkaxes([ax1 ax2], "x");

% ── Figure 2: 8-channel EMG with roll/pitch overlay ─────────────────────────
fig2 = figure("Name", "EMG Channels with Roll/Pitch", "NumberTitle", "off");
fig2.Position = [90 40 1300 960];

sp = gobjects(8,1);
for ch = 1:8
    sp(ch) = subplot(8,1,ch);

    yyaxis left;
    plot(t_s, emg(:, ch), "Color", [0.20 0.20 0.20], "LineWidth", 0.5);
    ylabel(sprintf("EMG%d", ch-1));
    ylim([-128 127]);

    yyaxis right;
    plot(t_s, roll_deg, "Color", C_ROLL, "LineStyle", "-", "LineWidth", 0.8, "HandleVisibility", "off"); hold on;
    plot(t_s, pitch_deg, "Color", C_PITCH, "LineStyle", "-", "LineWidth", 0.8, "HandleVisibility", "off");
    yline(0, "k--", "LineWidth", 0.4, "HandleVisibility", "off");
    ylabel("deg");

    grid on;
    if ch < 8
        set(gca, "XTickLabel", []);
    end
end

xlabel("Time (s)");
linkaxes(sp, "x");
sgtitle(sprintf("EMG (left axis) with %s Roll/Pitch overlay (right axis)", src));

% Legend helper in separate tiny figure-less axes approach
% Keep script simple: add legend to top subplot only.
subplot(8,1,1);
yyaxis right;
hold on;
plot(nan, nan, "Color", C_ROLL, "LineWidth", 1.2, "DisplayName", "Roll");
plot(nan, nan, "Color", C_PITCH, "LineWidth", 1.2, "DisplayName", "Pitch");
legend("Location", "northeast");

