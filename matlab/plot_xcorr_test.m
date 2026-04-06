clear; clc; close all;

RESULTS_DIR = "../correlation_results";
BASE_NAME   = "real_pair_xcorr";   % change here

CURVE_CSV   = fullfile(RESULTS_DIR, BASE_NAME + "_curve.csv");
SUMMARY_CSV = fullfile(RESULTS_DIR, BASE_NAME + "_summary.csv");

C = readtable(CURVE_CSV, "TextType", "string");

peakLagNorm  = C.peak_lag_norm(1);
peakCorrNorm = C.peak_corr_norm(1);

peakLagRaw   = C.peak_lag_raw(1);
peakCorrRaw  = C.peak_corr_raw(1);

figure;

subplot(2,1,1);
plot(C.lag, C.corr_norm, 'LineWidth', 1.5); grid on; hold on;
xline(peakLagNorm, '--r');
plot(peakLagNorm, peakCorrNorm, 'or', 'MarkerFaceColor', 'r');
yline(0, 'k--');
title('Normalized cross-correlation');
xlabel('Lag'); ylabel('corr\_norm');

subplot(2,1,2);
plot(C.lag, C.corr_raw, 'LineWidth', 1.5); grid on; hold on;
xline(peakLagRaw, '--r');
plot(peakLagRaw, peakCorrRaw, 'or', 'MarkerFaceColor', 'r');
yline(0, 'k--');
title('Raw (textbook) cross-correlation');
xlabel('Lag'); ylabel('corr\_raw');