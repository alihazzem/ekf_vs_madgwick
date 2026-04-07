clear; clc; close all;

RESULTS_DIR = "../correlation_results";
BASE_NAME   = "real_pair_fist_t2_ch0_vs_ch1";
CURVE_CSV   = fullfile(RESULTS_DIR, BASE_NAME + "_curve.csv");
SUMMARY_CSV = fullfile(RESULTS_DIR, BASE_NAME + "_summary.csv");

% ── Load curve (preserve underscore column names) ────────────────────────
C = readtable(CURVE_CSV, "TextType", "string", ...
              "VariableNamingRule", "preserve");

% ── Extract peak metadata safely via unique() ─────────────────────────────
peakLagNorm  = unique(C.("peak_lag_norm"));
peakCorrNorm = unique(C.("peak_corr_norm"));
peakLagRaw   = unique(C.("peak_lag_raw"));
peakCorrRaw  = unique(C.("peak_corr_raw"));

assert(isscalar(peakLagNorm),  "peak_lag_norm is not constant across rows");
assert(isscalar(peakLagRaw),   "peak_lag_raw is not constant across rows");

% ── Plot ──────────────────────────────────────────────────────────────────
figure("Name", "XCorr: " + BASE_NAME);

subplot(2,1,1);
plot(C.("lag"), C.("corr_norm"), 'LineWidth', 1.5); grid on; hold on;
xline(peakLagNorm, '--r', 'LineWidth', 1.2);
plot(peakLagNorm, peakCorrNorm, 'or', 'MarkerFaceColor', 'r', 'MarkerSize', 7);
yline(0, 'k--');
ylim([-1 1]);
title('Normalized Cross-Correlation  (standard, MATLAB-style)');
xlabel('Lag (samples)'); ylabel('R_{norm}');

subplot(2,1,2);
plot(C.("lag"), C.("corr_raw"), 'LineWidth', 1.5); grid on; hold on;
xline(peakLagRaw, '--r', 'LineWidth', 1.2);
plot(peakLagRaw, peakCorrRaw, 'or', 'MarkerFaceColor', 'r', 'MarkerSize', 7);
yline(0, 'k--');
title('Mean-Normalised Raw Cross-Correlation  (\div overlap length)');
xlabel('Lag (samples)'); ylabel('R_{raw}');