% plot_one_trial_all_channels_pairs.m
% Plot one selected gesture + one selected trial, showing all 8 EMG channels
% as 4 figures, with 2 channels per figure.
%
% For each channel:
%   - top subplot: raw EMG
%   - bottom subplot: selected filtered EMG stage (HP / RECT / ENV)
%
% Layout:
%   Figure 1: ch0, ch1
%   Figure 2: ch2, ch3
%   Figure 3: ch4, ch5
%   Figure 4: ch6, ch7

clear; clc; close all;

% ── User settings ────────────────────────────────────────────────────────────
LABELS_CSV    = "../emg_data/trial_labels.csv";
FILTER_SOURCE = "env";     % "hp", "rect", "env"
GESTURE       = "FIST";    % e.g. "FIST"
SESSION_ID    = "S01";     % e.g. "S01", or "" to ignore session
TRIAL_ID      = "4";       % trial to plot as string or number
CHANNEL_PAIRS = [0 1; 2 3; 4 5; 6 7];

PLOT_STRIDE   = 1;         % 1 = full resolution
DOCK_FIGURES  = false;
FIG_SIZE      = [1300 850];
SAVE_PNG      = false;
OUT_DIR       = "../emg_data/trial_channel_pair_plots";
% ─────────────────────────────────────────────────────────────────────────────

if DOCK_FIGURES
    set(groot, "DefaultFigureWindowStyle", "docked");
else
    set(groot, "DefaultFigureWindowStyle", "normal");
end

if ~isfile(LABELS_CSV)
    error("Labels file not found: %s", LABELS_CSV);
end

L = readtable(LABELS_CSV, "TextType", "string");
if isempty(L)
    error("Labels file is empty: %s", LABELS_CSV);
end

required = ["gesture_label", "trial_id", "session_id", "emg_online_file"];
for i = 1:numel(required)
    if ~any(string(L.Properties.VariableNames) == required(i))
        error("Missing required labels column: %s", required(i));
    end
end

gesture_upper = upper(strtrim(string(GESTURE)));
trial_str = string(TRIAL_ID);

rows = upper(string(L.gesture_label)) == gesture_upper & string(L.trial_id) == trial_str;

if SESSION_ID ~= ""
    rows = rows & string(L.session_id) == string(SESSION_ID);
end

G = L(rows, :);

if isempty(G)
    error("No matching row found for gesture=%s, session=%s, trial=%s", ...
        gesture_upper, string(SESSION_ID), trial_str);
end

if height(G) > 1
    warning("Multiple matching rows found. Using the first one.");
    G = G(1, :);
end

switch upper(string(FILTER_SOURCE))
    case "HP"
        filt_prefix = "emg_hp";
        filt_label = "High-Pass";
    case "RECT"
        filt_prefix = "emg_rect";
        filt_label = "Rectified";
    case "ENV"
        filt_prefix = "emg_env";
        filt_label = "Envelope";
    otherwise
        error("FILTER_SOURCE must be 'hp', 'rect', or 'env'.");
end

if SAVE_PNG && ~isfolder(OUT_DIR)
    mkdir(OUT_DIR);
end

if PLOT_STRIDE < 1
    PLOT_STRIDE = 1;
end

fpath = fullfile("..", "emg_data", string(G.emg_online_file(1)));
if ~isfile(fpath)
    error("EMG file not found: %s", fpath);
end

T = readtable(fpath);
vars = string(T.Properties.VariableNames);

if ~any(vars == "pc_t_ms")
    error("Column pc_t_ms not found in file: %s", fpath);
end

t_s = (double(T.pc_t_ms) - double(T.pc_t_ms(1))) / 1000.0;
idx = 1:PLOT_STRIDE:numel(t_s);

fprintf("\nLoaded file: %s\n", fpath);
fprintf("Gesture: %s | Session: %s | Trial: %s\n", gesture_upper, string(G.session_id(1)), trial_str);

for p = 1:size(CHANNEL_PAIRS, 1)
    chA = CHANNEL_PAIRS(p, 1);
    chB = CHANNEL_PAIRS(p, 2);

    fig = figure("Name", sprintf("%s | Trial %s | Ch%d-Ch%d", gesture_upper, trial_str, chA, chB), ...
                 "NumberTitle", "off");

    if ~DOCK_FIGURES
        screen = get(groot, "ScreenSize");
        w = min(FIG_SIZE(1), max(500, screen(3) - 80));
        h = min(FIG_SIZE(2), max(400, screen(4) - 120));
        fig.Position = [120 80 w h];
        movegui(fig, "onscreen");
    end

    tl = tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");

    % -------- Channel A raw --------
    ax1 = nexttile(tl, 1);
    hold(ax1, "on"); grid(ax1, "on");
    raw_col_A = sprintf("emg_raw%d", chA);
    fil_col_A = sprintf("%s%d", filt_prefix, chA);

    if ~any(vars == raw_col_A) || ~any(vars == fil_col_A)
        title(ax1, sprintf("Missing columns for ch%d", chA));
    else
        rawA = double(T.(raw_col_A));
        plot(ax1, t_s(idx), rawA(idx), "LineWidth", 0.9);
        yline(ax1, 0, "k--", "LineWidth", 0.4, "HandleVisibility", "off");
        title(ax1, sprintf("Channel %d | Raw", chA));
        ylabel(ax1, "Raw");
    end

    % -------- Channel B raw --------
    ax2 = nexttile(tl, 2);
    hold(ax2, "on"); grid(ax2, "on");
    raw_col_B = sprintf("emg_raw%d", chB);
    fil_col_B = sprintf("%s%d", filt_prefix, chB);

    if ~any(vars == raw_col_B) || ~any(vars == fil_col_B)
        title(ax2, sprintf("Missing columns for ch%d", chB));
    else
        rawB = double(T.(raw_col_B));
        plot(ax2, t_s(idx), rawB(idx), "LineWidth", 0.9);
        yline(ax2, 0, "k--", "LineWidth", 0.4, "HandleVisibility", "off");
        title(ax2, sprintf("Channel %d | Raw", chB));
        ylabel(ax2, "Raw");
    end

    % -------- Channel A filtered --------
    ax3 = nexttile(tl, 3);
    hold(ax3, "on"); grid(ax3, "on");

    if ~any(vars == fil_col_A)
        title(ax3, sprintf("Missing filtered column for ch%d", chA));
    else
        filA = double(T.(fil_col_A));
        plot(ax3, t_s(idx), filA(idx), "LineWidth", 1.0);
        yline(ax3, 0, "k--", "LineWidth", 0.4, "HandleVisibility", "off");
        title(ax3, sprintf("Channel %d | %s", chA, filt_label));
        xlabel(ax3, "Time (s)");
        ylabel(ax3, filt_label);
    end

    % -------- Channel B filtered --------
    ax4 = nexttile(tl, 4);
    hold(ax4, "on"); grid(ax4, "on");

    if ~any(vars == fil_col_B)
        title(ax4, sprintf("Missing filtered column for ch%d", chB));
    else
        filB = double(T.(fil_col_B));
        plot(ax4, t_s(idx), filB(idx), "LineWidth", 1.0);
        yline(ax4, 0, "k--", "LineWidth", 0.4, "HandleVisibility", "off");
        title(ax4, sprintf("Channel %d | %s", chB, filt_label));
        xlabel(ax4, "Time (s)");
        ylabel(ax4, filt_label);
    end

    sgtitle(sprintf("Gesture %s | Session %s | Trial %s | Channels %d & %d", ...
        gesture_upper, string(G.session_id(1)), trial_str, chA, chB));

    if SAVE_PNG
        out_name = sprintf("%s_sess_%s_trial_%s_ch%d_ch%d_%s.png", ...
            lower(gesture_upper), lower(string(G.session_id(1))), trial_str, chA, chB, lower(FILTER_SOURCE));
        exportgraphics(fig, fullfile(OUT_DIR, out_name), "Resolution", 150);
    end
end

fprintf("\nDone.\n");