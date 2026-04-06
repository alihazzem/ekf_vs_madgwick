% plot_xcorr_real_pair_both_simple.m
% Minimal visualization for real-trial pair cross-correlation output
% from the UPDATED Python script that exports BOTH:
%   - normalized cross-correlation
%   - textbook/raw cross-correlation
%
% Reads:
%   ../correlation_results/<base>_curve.csv
%   ../correlation_results/<base>_summary.csv
%
% Shows:
%   - normalized xcorr curve + detected peak
%   - raw/textbook xcorr curve + detected peak

clear; clc; close all;

RESULTS_DIR = "../correlation_results";
BASE_NAME   = "real_pair_fist_t2_t4";   % change this if needed

CURVE_CSV   = fullfile(RESULTS_DIR, BASE_NAME + "_curve.csv");
SUMMARY_CSV = fullfile(RESULTS_DIR, BASE_NAME + "_summary.csv");

DOCK_FIGURES = false;
FIG_SIZE = [1100 700];

if DOCK_FIGURES
    set(groot, "DefaultFigureWindowStyle", "docked");
else
    set(groot, "DefaultFigureWindowStyle", "normal");
end

if ~isfile(CURVE_CSV)
    error("Curve CSV not found: %s", CURVE_CSV);
end
if ~isfile(SUMMARY_CSV)
    error("Summary CSV not found: %s", SUMMARY_CSV);
end

C = readtable(CURVE_CSV, "TextType", "string");
S = readtable(SUMMARY_CSV, "TextType", "string");

if isempty(C)
    error("Curve CSV is empty: %s", CURVE_CSV);
end

% ---- Required curve columns check ----
requiredCurveCols = [
    "lag", ...
    "corr_norm", "corr_raw", ...
    "peak_lag_norm", "peak_corr_norm", ...
    "peak_lag_raw",  "peak_corr_raw", ...
    "zero_lag_norm", "zero_lag_raw", ...
    "trial_a_timestamp", "trial_b_timestamp", ...
    "channel", "stage", ...
    "segment_a_start", "segment_a_end", ...
    "segment_b_start", "segment_b_end"
];

missingCurve = requiredCurveCols(~ismember(requiredCurveCols, string(C.Properties.VariableNames)));
if ~isempty(missingCurve)
    error("Curve CSV missing columns: %s", strjoin(missingCurve, ", "));
end

% ---- Read values from first row (same metadata repeated for all rows) ----
peakLagNorm  = C.peak_lag_norm(1);
peakCorrNorm = C.peak_corr_norm(1);

peakLagRaw   = C.peak_lag_raw(1);
peakCorrRaw  = C.peak_corr_raw(1);

zeroLagNorm  = C.zero_lag_norm(1);
zeroLagRaw   = C.zero_lag_raw(1);

trialA = string(C.trial_a_timestamp(1));
trialB = string(C.trial_b_timestamp(1));
ch     = C.channel(1);
stg    = string(C.stage(1));

segA = sprintf("[%d:%d]", C.segment_a_start(1), C.segment_a_end(1));
segB = sprintf("[%d:%d]", C.segment_b_start(1), C.segment_b_end(1));

% ---- Summary metrics ----
meanPeakCorrNorm = summaryValueToDouble(S, "mean_peak_corr_norm");
meanAbsLagNorm   = summaryValueToDouble(S, "mean_abs_peak_lag_norm");

meanPeakCorrRaw  = summaryValueToDouble(S, "mean_peak_corr_raw");
meanAbsLagRaw    = summaryValueToDouble(S, "mean_abs_peak_lag_raw");

% ---- Create figure ----
fig = figure("Name", "Real Pair XCorr (Normalized + Raw)", "NumberTitle", "off");
if ~DOCK_FIGURES
    screen = get(groot, "ScreenSize");
    w = min(FIG_SIZE(1), max(500, screen(3) - 80));
    h = min(FIG_SIZE(2), max(350, screen(4) - 120));
    fig.Position = [120 80 w h];
    movegui(fig, "onscreen");
end

tiledlayout(2,1, "TileSpacing", "compact", "Padding", "compact");

%% =========================
%  Top: Normalized xcorr
%  =========================
nexttile;

hCurveNorm = plot(C.lag, C.corr_norm, ...
    "LineWidth", 1.6, ...
    "Color", [0.1 0.45 0.75], ...
    "DisplayName", "Normalized xcorr");
grid on;
hold on;

hPeakLineNorm = xline(peakLagNorm, "-.", ...
    "LineWidth", 1.2, ...
    "Color", [0.85 0.2 0.2], ...
    "DisplayName", sprintf("Peak lag = %d", peakLagNorm));

hPeakPointNorm = plot(peakLagNorm, peakCorrNorm, "o", ...
    "MarkerSize", 8, ...
    "MarkerFaceColor", [0.85 0.2 0.2], ...
    "MarkerEdgeColor", [0.85 0.2 0.2], ...
    "DisplayName", sprintf("Peak corr = %.3f", peakCorrNorm));

if ~isnan(zeroLagNorm)
    hZeroNorm = plot(0, zeroLagNorm, "s", ...
        "MarkerSize", 7, ...
        "MarkerFaceColor", [0.2 0.6 0.2], ...
        "MarkerEdgeColor", [0.2 0.6 0.2], ...
        "DisplayName", sprintf("Zero-lag corr = %.3f", zeroLagNorm));
else
    hZeroNorm = gobjects(1);
end

yline(0, "k--", "LineWidth", 0.5, "HandleVisibility", "off");

xlabel("Lag (samples)");
ylabel("Normalized correlation");

title({
    sprintf("Normalized Cross-Correlation | Channel %d | %s", ch, upper(stg)), ...
    sprintf("A=%s seg%s  vs  B=%s seg%s | mean peak corr=%.3f | mean abs lag=%.2f", ...
        trialA, segA, trialB, segB, meanPeakCorrNorm, meanAbsLagNorm)
}, "Interpreter", "none");

if isgraphics(hZeroNorm)
    legend([hCurveNorm, hPeakLineNorm, hPeakPointNorm, hZeroNorm], ...
        "Location", "southoutside", ...
        "Orientation", "horizontal", ...
        "Box", "off");
else
    legend([hCurveNorm, hPeakLineNorm, hPeakPointNorm], ...
        "Location", "southoutside", ...
        "Orientation", "horizontal", ...
        "Box", "off");
end

%% =========================
%  Bottom: Raw/textbook xcorr
%  =========================
nexttile;

hCurveRaw = plot(C.lag, C.corr_raw, ...
    "LineWidth", 1.6, ...
    "Color", [0.65 0.25 0.75], ...
    "DisplayName", "Raw/textbook xcorr");
grid on;
hold on;

hPeakLineRaw = xline(peakLagRaw, "-.", ...
    "LineWidth", 1.2, ...
    "Color", [0.85 0.2 0.2], ...
    "DisplayName", sprintf("Peak lag = %d", peakLagRaw));

hPeakPointRaw = plot(peakLagRaw, peakCorrRaw, "o", ...
    "MarkerSize", 8, ...
    "MarkerFaceColor", [0.85 0.2 0.2], ...
    "MarkerEdgeColor", [0.85 0.2 0.2], ...
    "DisplayName", sprintf("Peak corr = %.3f", peakCorrRaw));

if ~isnan(zeroLagRaw)
    hZeroRaw = plot(0, zeroLagRaw, "s", ...
        "MarkerSize", 7, ...
        "MarkerFaceColor", [0.2 0.6 0.2], ...
        "MarkerEdgeColor", [0.2 0.6 0.2], ...
        "DisplayName", sprintf("Zero-lag corr = %.3f", zeroLagRaw));
else
    hZeroRaw = gobjects(1);
end

yline(0, "k--", "LineWidth", 0.5, "HandleVisibility", "off");

xlabel("Lag (samples)");
ylabel("Raw / textbook correlation");

title({
    sprintf("Raw/Textbook Cross-Correlation | Channel %d | %s", ch, upper(stg)), ...
    sprintf("A=%s seg%s  vs  B=%s seg%s | mean peak corr=%.3f | mean abs lag=%.2f", ...
        trialA, segA, trialB, segB, meanPeakCorrRaw, meanAbsLagRaw)
}, "Interpreter", "none");

if isgraphics(hZeroRaw)
    legend([hCurveRaw, hPeakLineRaw, hPeakPointRaw, hZeroRaw], ...
        "Location", "southoutside", ...
        "Orientation", "horizontal", ...
        "Box", "off");
else
    legend([hCurveRaw, hPeakLineRaw, hPeakPointRaw], ...
        "Location", "southoutside", ...
        "Orientation", "horizontal", ...
        "Box", "off");
end

%% ---- Console summary ----
fprintf("\nReal pair xcorr (updated)\n");
fprintf("Curve file             : %s\n", CURVE_CSV);
fprintf("Summary file           : %s\n", SUMMARY_CSV);

fprintf("\n--- Normalized ---\n");
fprintf("Detected peak lag      : %d\n", peakLagNorm);
fprintf("Detected peak corr     : %.6f\n", peakCorrNorm);
fprintf("Zero-lag corr          : %.6f\n", zeroLagNorm);
fprintf("mean_peak_corr_norm    : %.6f\n", meanPeakCorrNorm);
fprintf("mean_abs_peak_lag_norm : %.6f\n", meanAbsLagNorm);

fprintf("\n--- Raw / Textbook ---\n");
fprintf("Detected peak lag      : %d\n", peakLagRaw);
fprintf("Detected peak corr     : %.6f\n", peakCorrRaw);
fprintf("Zero-lag corr          : %.6f\n", zeroLagRaw);
fprintf("mean_peak_corr_raw     : %.6f\n", meanPeakCorrRaw);
fprintf("mean_abs_peak_lag_raw  : %.6f\n", meanAbsLagRaw);


function out = summaryValueToDouble(T, metricName)
out = NaN;

if ~all(ismember(["metric", "value"], string(T.Properties.VariableNames)))
    return;
end

metricCol = string(T.metric);
idx = find(metricCol == string(metricName), 1);
if isempty(idx)
    return;
end

raw = T.value(idx);
if iscell(raw)
    raw = raw{1};
end

if isnumeric(raw)
    out = double(raw);
else
    out = str2double(string(raw));
end
end