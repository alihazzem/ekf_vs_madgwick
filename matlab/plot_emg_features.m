% plot_emg_features.m  —  Plot windowed EMG features from process_emg.py output

clear; clc; close all;

% ── Select CSV file ───────────────────────────────────────────────────────────
CSV_FILE = "../emg_data/emg_capture_test_features.csv";   % leave "" to open picker

if isempty(CSV_FILE)
    [fname, fpath] = uigetfile("*.csv", "Select EMG features CSV");
    if isequal(fname, 0)
        disp("No file selected.");
        return;
    end
    CSV_FILE = fullfile(fpath, fname);
end

% ── Load data ─────────────────────────────────────────────────────────────────
T = readtable(CSV_FILE);

% Basic validation
requiredVars = ["t_ms", ...
    compose("mav%d", 0:7), ...
    compose("rms%d", 0:7), ...
    compose("zc%d", 0:7), ...
    compose("wl%d", 0:7)];

missingVars = setdiff(requiredVars, string(T.Properties.VariableNames));
if ~isempty(missingVars)
    error("Missing required columns in CSV: %s", strjoin(missingVars, ", "));
end

t_ms = T.t_ms;
t_s  = t_ms / 1000.0;

% Extract by column names (more robust than fixed indices)
mav = table2array(T(:, compose("mav%d", 0:7)));
rms = table2array(T(:, compose("rms%d", 0:7)));
zc  = table2array(T(:, compose("zc%d", 0:7)));
wl  = table2array(T(:, compose("wl%d", 0:7)));

% ── Stats ─────────────────────────────────────────────────────────────────────
n_win = height(T);
dur_s = t_s(end) - t_s(1);

if numel(t_s) >= 2
    dt_s = mean(diff(t_s));
else
    dt_s = NaN;
end

[~, peak_ch] = max(mean(mav, 1));      % channel with highest mean MAV
[~, peak_win] = max(sum(mav, 2));      % window with highest total MAV across channels

fprintf("── EMG features stats ────────────────────────────\n");
fprintf("Windows  : %d\n", n_win);
fprintf("Duration : %.2f s\n", dur_s);

if ~isnan(dt_s)
    fprintf("Step     : %.0f ms\n", dt_s * 1000);
else
    fprintf("Step     : N/A (only one window)\n");
end

fprintf("\n── Per-channel mean features ─────────────────────\n");
for ch = 0:7
    fprintf("  Ch %d : MAV=%.2f  RMS=%.2f  ZC=%.1f  WL=%.1f\n", ...
        ch, mean(mav(:, ch+1)), mean(rms(:, ch+1)), ...
        mean(zc(:, ch+1)), mean(wl(:, ch+1)));
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
ylabel("MAV (raw units)");
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
ylabel("RMS (raw units)");
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
title("Zero Crossings — rough frequency/activity indicator");
grid on;

% Panel 4 — WL
sp4 = subplot(4, 1, 4);
hold on;
for ch = 0:7
    plot(t_s, wl(:, ch+1), "Color", COLORS{ch+1}, "LineWidth", 1.0, ...
         "HandleVisibility", "off");
end
hold off;
ylabel("WL (raw units)");
xlabel("Time (s)");
title("Waveform Length — signal complexity per window");
grid on;

linkaxes([sp1 sp2 sp3 sp4], "x");

if ~isnan(dt_s)
    sgtitle(sprintf("EMG Features  |  %d windows  |  step = %.0f ms  |  %.1f s total", ...
        n_win, dt_s * 1000, dur_s));
else
    sgtitle(sprintf("EMG Features  |  %d window  |  %.1f s total", ...
        n_win, dur_s));
end

% ── Figure 2: Per-channel activity bar chart ──────────────────────────────────
fig2 = figure("Name", "EMG Channel Activity Profile", "NumberTitle", "off");
fig2.Position = [80 80 900 420];

mean_mav = mean(mav, 1);

bar_h = bar(0:7, mean_mav, "FaceColor", "flat");
for ch = 0:7
    bar_h.CData(ch+1, :) = sscanf(COLORS{ch+1}(2:end), "%2x%2x%2x")' / 255;
end

xlabel("EMG Channel");
ylabel("Mean MAV (raw units)");
title("Per-Channel Activity Profile — mean MAV across full recording");
xticks(0:7);
xticklabels(arrayfun(@(x) sprintf("Ch %d", x), 0:7, "UniformOutput", false));
grid on;
sgtitle("Channel Activity Profile | higher = more muscle activation detected");