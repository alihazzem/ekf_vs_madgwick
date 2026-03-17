% plot_emg.m  —  Plot 8-channel Myo EMG data from a captured CSV

clear; clc; close all;

% ── Select CSV file ───────────────────────────────────────────────────────────
CSV_FILE = "../emg_data/at_rest.csv";   % leave "" to open a file picker

if isempty(CSV_FILE)
    [fname, fpath] = uigetfile("*.csv", "Select EMG capture CSV");
    if isequal(fname, 0)
        disp("No file selected.");
        return;
    end
    CSV_FILE = fullfile(fpath, fname);
end

% ── Load data ─────────────────────────────────────────────────────────────────
T = readtable(CSV_FILE);

requiredVars = ["t_ms", compose("emg%d", 0:7)];
missingVars = setdiff(requiredVars, string(T.Properties.VariableNames));
if ~isempty(missingVars)
    error("Missing required columns in CSV: %s", strjoin(missingVars, ", "));
end

t_ms = T.t_ms;
t_s  = (t_ms - t_ms(1)) / 1000.0;

emg = [T.emg0, T.emg1, T.emg2, T.emg3, ...
       T.emg4, T.emg5, T.emg6, T.emg7];

% ── Stats ─────────────────────────────────────────────────────────────────────
n_samp = height(T);
dur_s  = t_s(end);

if dur_s > 0
    avg_rate = (n_samp - 1) / dur_s;
else
    avg_rate = NaN;
end

fprintf("── EMG capture stats ─────────────────────────────\n");
fprintf("Samples  : %d\n", n_samp);
fprintf("Duration : %.2f s\n", dur_s);

if ~isnan(avg_rate)
    fprintf("Avg rate : %.1f Hz\n", avg_rate);
else
    fprintf("Avg rate : N/A\n");
end

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
    ylim([-128 127]);   % valid for raw Myo int8 EMG
    yline(0, "k--", "LineWidth", 0.4, "HandleVisibility", "off");
    ylabel(sprintf("Ch %d", ch), "FontSize", 8);
    grid on;
    if ch < 7
        set(gca, "XTickLabel", []);
    end
end
xlabel("Time (s)");
linkaxes(sp, "x");

if ~isnan(avg_rate)
    sgtitle(sprintf("Myo EMG — 8 Channels  |  %d samples  |  %.1f Hz  |  %.1f s", ...
        n_samp, avg_rate, dur_s));
else
    sgtitle(sprintf("Myo EMG — 8 Channels  |  %d samples  |  %.1f s", ...
        n_samp, dur_s));
end

% ── Figure 2: Moving-RMS envelope ─────────────────────────────────────────────
RMS_WIN_MS = 250;

if ~isnan(avg_rate)
    RMS_WIN = max(1, round(RMS_WIN_MS * avg_rate / 1000));
else
    RMS_WIN = 50;
end

fig2 = figure("Name", "Myo EMG — RMS Envelope", "NumberTitle", "off");
fig2.Position = [80 80 1200 450];

hold on;
for ch = 0:7
    % causal / trailing moving RMS
    rms_env = sqrt(movmean(double(emg(:, ch+1)).^2, [RMS_WIN-1 0]));
    plot(t_s, rms_env, "Color", COLORS{ch+1}, "LineWidth", 1.2, ...
         "DisplayName", sprintf("Ch %d", ch));
end
hold off;

ylabel("RMS amplitude (raw counts)");
xlabel("Time (s)");
legend("Location", "northeast", "NumColumns", 4);
grid on;
title(sprintf("Moving RMS Envelope (window = %d samples = %.0f ms)", ...
      RMS_WIN, RMS_WIN / avg_rate * 1000));