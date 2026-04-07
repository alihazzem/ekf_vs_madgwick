% plot_channel_corr_heatmap.m
% Visualize cross-channel correlation matrix as a heatmap

clear; clc; close all;

RESULTS_DIR = "../correlation_results";

% Change this to match your output filename
FILE_NAME = "channel_matrix_FIST_S01_trial2_env_peak_corr_norm.csv";

CSV_PATH = fullfile(RESULTS_DIR, FILE_NAME);

if ~isfile(CSV_PATH)
    error("File not found: %s", CSV_PATH);
end

% Read table
T = readtable(CSV_PATH, "TextType", "string");

% Extract channel labels (first column)
row_labels = string(T{:,1});
col_labels = string(T.Properties.VariableNames(2:end));

% Convert data to numeric matrix
data = T{:,2:end};
data = str2double(string(data));

% Create figure
figure('Name', 'EMG Channel Correlation Heatmap', 'NumberTitle', 'off');

imagesc(data);
colorbar;

% Set colormap
colormap(jet);

% Set axis labels
xticks(1:length(col_labels));
yticks(1:length(row_labels));

xticklabels(col_labels);
yticklabels(row_labels);

xlabel('Channel');
ylabel('Channel');

title('Cross-Channel Normalized Correlation (Peak)');

axis square;

% Show values inside cells
for i = 1:size(data,1)
    for j = 1:size(data,2)
        val = data(i,j);
        if ~isnan(val)
            text(j, i, sprintf('%.2f', val), ...
                'HorizontalAlignment', 'center', ...
                'Color', 'white', ...
                'FontSize', 9, ...
                'FontWeight', 'bold');
        end
    end
end

% Improve contrast (optional)
caxis([0 1]);  % normalized correlation range

grid on;