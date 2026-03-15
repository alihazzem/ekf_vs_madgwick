% plot_emg.m  —  Plot 8-channel Myo EMG data from a captured CSV
%
% Usage
% -----
%   1. Run  capture_emg.py  (or  generate_emg_test_data.py  for synthetic data)
%   2. Place the CSV in the  emg_data/  folder (one level above matlab/)
%      OR update CSV_FILE below to any absolute or relative path.
%   3. Run this script in MATLAB.
%
% This script produces two figures:
%   Figure 1  —  8-channel raw EMG time series (one subplot per channel)
%   Figure 2  —  Moving-RMS envelope for all 8 channels overlaid
%
% CSV columns (from capture_emg.py / generate_emg_test_data.py)
% --------------------------------------------------------------
%   t_ms            elapsed time (ms) since first sample
%   emg0 – emg7     8 EMG channels, signed 8-bit (-128 to +127)
%
% Sampling rate: ~200 Hz (Myo BLE delivery)

clear; clc; close all;

% ── Select CSV file ───────────────────────────────────────────────────────────

CSV_FILE = "../emg_data/emg_test_synthetic.csv";   % leave "" to open a file picker

if CSV_FILE == ""
    [fname, fpath] = uigetfile("*.csv", "Select EMG capture CSV");
    if isequal(fname, 0), disp("No file selected."); return; end
    CSV_FILE = fullfile(fpath, fname);
end

% ── Load data ─────────────────────────────────────────────────────────────────

T = readtable(CSV_FILE);

t_ms = T.t_ms;
t_s  = (t_ms - t_ms(1)) / 1000.0;

% Stack channels into a matrix [N x 8] for convenience
emg = [T.emg0, T.emg1, T.emg2, T.emg3, ...
       T.emg4, T.emg5, T.emg6, T.emg7];

% ── Stats ─────────────────────────────────────────────────────────────────────

n_samp   = height(T);
dur_s    = t_s(end);
avg_rate = n_samp / dur_s;

fprintf("── EMG capture stats ─────────────────────────────\n");
fprintf("Samples  : %d\n",    n_samp);
fprintf("Duration : %.2f s\n", dur_s);
fprintf("Avg rate : %.1f Hz\n", avg_rate);
fprintf("\n── Per-channel amplitude (raw counts) ────────────\n");
fprintf("%-6s  %8s  %8s  %8s\n", "Ch", "Min", "Max", "Mean|x|");
for ch = 0:7
    v = emg(:, ch+1);
    fprintf("emg%-3d  %8d  %8d  %8.1f\n", ch, min(v), max(v), mean(abs(v)));
end
fprintf("\n");

% ── Colour palette (8 channels) ───────────────────────────────────────────────

COLORS = {"#d62728", "#2ca02c", "#1f77b4", "#ff7f0e", ...
          "#9467bd", "#8c564b", "#e377c2", "#17becf"};

% ── Figure 1: 8-channel raw EMG time series ───────────────────────────────────

fig1 = figure("Name", "Myo EMG — 8 Channels", "NumberTitle", "off");
fig1.Position = [60 60 1200 920];

sp = gobjects(8, 1);
for ch = 0:7
    sp(ch+1) = subplot(8, 1, ch+1);
    plot(t_s, emg(:, ch+1), "Color", COLORS{ch+1}, "LineWidth", 0.5);
    ylim([-128 127]);
    yline(0, "k--", "LineWidth", 0.4, "HandleVisibility", "off");
    ylabel(sprintf("Ch %d", ch), "FontSize", 8);
    grid on;
    if ch < 7
        set(gca, "XTickLabel", []);   % hide x labels except bottom panel
    end
end
xlabel("Time (s)");
linkaxes(sp, "x");
sgtitle(sprintf("Myo EMG — 8 Channels  |  %d samples  |  %.1f Hz  |  %.1f s", ...
                n_samp, avg_rate, dur_s));

% ── Figure 2: Moving-RMS envelope ─────────────────────────────────────────────
%
%   Computes the root-mean-square amplitude in a 50-sample (250 ms) sliding
%   window — the standard EMG linear envelope.  Overlaying all 8 channels
%   on one axis makes it easy to identify which channels activate for each
%   gesture.

RMS_WIN = 50;   % samples (~250 ms at 200 Hz)

fig2 = figure("Name", "Myo EMG — RMS Envelope", "NumberTitle", "off");
fig2.Position = [80 80 1200 450];

hold on;
for ch = 0:7
    rms_env = sqrt(movmean(emg(:, ch+1).^2, RMS_WIN));
    plot(t_s, rms_env, "Color", COLORS{ch+1}, "LineWidth", 1.2, ...
         "DisplayName", sprintf("Ch %d", ch));
end
hold off;

ylabel("RMS amplitude (counts)");
xlabel("Time (s)");
legend("Location", "northeast", "NumColumns", 4);
grid on;
title(sprintf("Moving-RMS Envelope  (window = %d samples = %.0f ms)", ...
              RMS_WIN, RMS_WIN / avg_rate * 1000));
