% plot_emg_features.m  —  Plot windowed EMG features from process_emg.py output
%
% Usage
% -----
%   1. Run  process_emg.py  on a raw EMG capture CSV.
%   2. Place the generated  *_features.csv  in the  emg_data/  folder,
%      or update CSV_FILE below to any absolute or relative path.
%   3. Run this script in MATLAB.
%
% This script produces two figures:
%   Figure 1  —  4-panel feature time series (MAV, RMS, ZC, WL)
%                All 8 channels overlaid per panel — shows when each gesture
%                activates which channels over time.
%   Figure 2  —  Per-channel activity bar chart (mean MAV per channel)
%                Shows the spatial distribution of muscle activation —
%                useful for identifying the most informative channels.
%
% Features CSV columns (from process_emg.py)
% ------------------------------------------
%   t_ms          window centre timestamp (ms)
%   mav0–mav7     Mean Absolute Value   per channel per window
%   rms0–rms7     Root Mean Square      per channel per window
%   zc0–zc7       Zero Crossing count   per channel per window
%   wl0–wl7       Waveform Length       per channel per window

clear; clc; close all;

% ── Select CSV file ───────────────────────────────────────────────────────────

CSV_FILE = "../emg_data/emg_test_synthetic_features.csv";   % leave "" to open a file picker

if CSV_FILE == ""
    [fname, fpath] = uigetfile("*.csv", "Select EMG features CSV");
    if isequal(fname, 0), disp("No file selected."); return; end
    CSV_FILE = fullfile(fpath, fname);
end

% ── Load data ─────────────────────────────────────────────────────────────────

T = readtable(CSV_FILE);

t_ms = T.t_ms;
t_s  = t_ms / 1000.0;

% Extract feature blocks [N_windows x 8]
mav = table2array(T(:, 2:9));
rms = table2array(T(:, 10:17));
zc  = table2array(T(:, 18:25));
wl  = table2array(T(:, 26:33));

% ── Stats ─────────────────────────────────────────────────────────────────────

n_win  = height(T);
dur_s  = t_s(end);
dt_s   = mean(diff(t_s));

[~, peak_ch]  = max(mean(mav, 1));   % channel with highest mean MAV
[~, peak_win] = max(max(mav, [], 2)); % window with highest peak MAV

fprintf("── EMG features stats ────────────────────────────\n");
fprintf("Windows  : %d\n",     n_win);
fprintf("Duration : %.2f s\n", dur_s);
fprintf("Step     : %.0f ms\n", dt_s * 1000);
fprintf("\n── Per-channel mean MAV (overall activity) ───────\n");
for ch = 0:7
    fprintf("  Ch %d : MAV=%.2f  RMS=%.2f  ZC=%.1f  WL=%.1f\n", ...
        ch, mean(mav(:,ch+1)), mean(rms(:,ch+1)), ...
        mean(zc(:,ch+1)), mean(wl(:,ch+1)));
end
fprintf("\nMost active channel : Ch %d (highest mean MAV)\n", peak_ch - 1);
fprintf("Peak activity at    : t = %.2f s\n\n", t_s(peak_win));

% ── Colour palette (8 channels) ───────────────────────────────────────────────

COLORS = {"#d62728", "#2ca02c", "#1f77b4", "#ff7f0e", ...
          "#9467bd", "#8c564b", "#e377c2", "#17becf"};

% ── Figure 1: Feature time series ─────────────────────────────────────────────

fig1 = figure("Name", "EMG Features — Time Series", "NumberTitle", "off");
fig1.Position = [60 60 1200 850];

% Panel 1 — MAV
sp1 = subplot(4, 1, 1);
hold on;
for ch = 0:7
    plot(t_s, mav(:, ch+1), "Color", COLORS{ch+1}, "LineWidth", 1.0, ...
         "DisplayName", sprintf("Ch %d", ch));
end
hold off;
ylabel("MAV (counts)");
title("Mean Absolute Value — muscle contraction amplitude");
legend("Location", "northeast", "NumColumns", 4);
grid on;

% Panel 2 — RMS
sp2 = subplot(4, 1, 2);
hold on;
for ch = 0:7
    plot(t_s, rms(:, ch+1), "Color", COLORS{ch+1}, "LineWidth", 1.0, ...
         "HandleVisibility", "off");
end
hold off;
ylabel("RMS (counts)");
title("Root Mean Square — signal power per window");
grid on;

% Panel 3 — ZC
sp3 = subplot(4, 1, 3);
hold on;
for ch = 0:7
    plot(t_s, zc(:, ch+1), "Color", COLORS{ch+1}, "LineWidth", 1.0, ...
         "HandleVisibility", "off");
end
hold off;
ylabel("ZC (count)");
title("Zero Crossings — frequency content indicator");
grid on;

% Panel 4 — WL
sp4 = subplot(4, 1, 4);
hold on;
for ch = 0:7
    plot(t_s, wl(:, ch+1), "Color", COLORS{ch+1}, "LineWidth", 1.0, ...
         "HandleVisibility", "off");
end
hold off;
ylabel("WL (counts)");
xlabel("Time (s)");
title("Waveform Length — signal complexity per window");
grid on;

linkaxes([sp1 sp2 sp3 sp4], "x");
sgtitle(sprintf("EMG Features  |  %d windows  |  step = %.0f ms  |  %.1f s total", ...
                n_win, dt_s * 1000, dur_s));

% ── Figure 2: Per-channel activity bar chart ──────────────────────────────────
%
%   Shows the mean MAV for each channel across the full recording.
%   Taller bars = more active muscle region = more informative channel
%   for gesture classification.

fig2 = figure("Name", "EMG Channel Activity Profile", "NumberTitle", "off");
fig2.Position = [80 80 900 420];

mean_mav = mean(mav, 1);   % [1 x 8]

bar_h = bar(0:7, mean_mav, "FaceColor", "flat");
for ch = 0:7
    bar_h.CData(ch+1, :) = sscanf(COLORS{ch+1}(2:end), "%2x%2x%2x")' / 255;
end

xlabel("EMG Channel");
ylabel("Mean MAV (counts)");
title("Per-Channel Activity Profile  —  mean MAV across full recording");
xticks(0:7);
xticklabels(arrayfun(@(x) sprintf("Ch %d", x), 0:7, "UniformOutput", false));
grid on;
sgtitle("Channel Activity Profile  |  higher = more muscle activation detected");
