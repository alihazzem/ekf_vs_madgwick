% plot_emg_by_gesture.m
% Plot EMG per gesture using labeled trial metadata.
%
% This script supports two modes:
%   1) "gesture_overview" (default, lightweight): one figure per gesture
%      using one selected channel.
%   2) "per_channel" (heavier): one figure per gesture per channel.
%
% In each figure:
%   - top subplot: raw EMG
%   - bottom subplot: selected filtered EMG stage (HP/RECT/ENV)
% Trials of the same gesture are overlaid in each subplot.
%
% Data source:
%   ../emg_data/trial_labels.csv
%   referenced fusion_emg_online_*.csv files

clear; clc; close all;

% ── User settings ────────────────────────────────────────────────────────────
LABELS_CSV = "../emg_data/trial_labels.csv";
FILTER_SOURCE = "env";      % "hp", "rect", "env"
SESSION_ID = "";            % e.g. "S01"; leave "" for all sessions
GESTURES = [];              % [] => auto from labels (REST/FIST/OPEN/PINCH...)
PLOT_MODE = "gesture_overview";  % "gesture_overview" or "per_channel"
OVERVIEW_CHANNEL = 3;       % channel used in gesture_overview mode
CHANNELS = 0:7;             % channels to plot
MAX_TRIALS_PER_GESTURE = 3; % 0 = all (larger values increase plotting load)
PLOT_STRIDE = 4;            % plot every Nth sample for speed (1 = full resolution)
SHOW_LEGEND = true;        % legends are expensive with many trials
SAVE_PNG = false;           % true to save png files
OUT_DIR = "../emg_data/gesture_plots";
DOCK_FIGURES = false;       % true: open figures docked inside MATLAB desktop
FIG_SIZE = [1200 700];      % [width height] when not docked
% ─────────────────────────────────────────────────────────────────────────────

if DOCK_FIGURES
    set(groot, "DefaultFigureWindowStyle", "docked");
else
    set(groot, "DefaultFigureWindowStyle", "normal");
end

if ~isfile(LABELS_CSV)
    error("Labels file not found: %s", LABELS_CSV);
end

L = readtable(LABELS_CSV);
if isempty(L)
    error("Labels file is empty: %s", LABELS_CSV);
end

required = ["gesture_label", "trial_id", "session_id", "emg_online_file"];
for i = 1:numel(required)
    if ~any(string(L.Properties.VariableNames) == required(i))
        error("Missing required labels column: %s", required(i));
    end
end

if SESSION_ID ~= ""
    L = L(string(L.session_id) == SESSION_ID, :);
end

if isempty(L)
    error("No label rows found after session filter.");
end

if isempty(GESTURES)
    gest = unique(string(L.gesture_label));
else
    gest = string(GESTURES);
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

for g = 1:numel(gest)
    g_name = upper(strtrim(gest(g)));
    G = L(upper(string(L.gesture_label)) == g_name, :);

    if isempty(G)
        fprintf("Skipping %s (no trials found).\n", g_name);
        continue;
    end

    if MAX_TRIALS_PER_GESTURE > 0 && height(G) > MAX_TRIALS_PER_GESTURE
        G = G(1:MAX_TRIALS_PER_GESTURE, :);
    end

    fprintf("\nGesture: %s  |  Trials: %d\n", g_name, height(G));

    if lower(string(PLOT_MODE)) == "gesture_overview"
        ch_list = OVERVIEW_CHANNEL;
    else
        ch_list = CHANNELS;
    end

    for ch = ch_list
        fig = figure("Name", sprintf("%s - Ch %d", g_name, ch), "NumberTitle", "off");

        if ~DOCK_FIGURES
            screen = get(groot, "ScreenSize"); % [left bottom width height]
            w = min(FIG_SIZE(1), max(400, screen(3) - 80));
            h = min(FIG_SIZE(2), max(300, screen(4) - 120));
            fig.Position = [120 80 w h];
            movegui(fig, "onscreen");
        end

        tiledlayout(2, 1, "TileSpacing", "compact", "Padding", "compact");

        ax1 = nexttile;
        hold(ax1, "on");
        grid(ax1, "on");
        ylabel(ax1, "Raw");
        title(ax1, sprintf("%s | Channel %d | Raw", g_name, ch));

        ax2 = nexttile;
        hold(ax2, "on");
        grid(ax2, "on");
        ylabel(ax2, filt_label);
        xlabel(ax2, "Time (s)");
        title(ax2, sprintf("%s | Channel %d | %s", g_name, ch, filt_label));

        trial_colors = lines(height(G));
        legend_entries = strings(height(G), 1);

        for r = 1:height(G)
            fpath = fullfile("..", "emg_data", string(G.emg_online_file(r)));
            if ~isfile(fpath)
                fprintf("  Missing file: %s\n", fpath);
                continue;
            end

            T = readtable(fpath);
            vars = string(T.Properties.VariableNames);

            raw_col = sprintf("emg_raw%d", ch);
            fil_col = sprintf("%s%d", filt_prefix, ch);
            if ~any(vars == raw_col) || ~any(vars == fil_col) || ~any(vars == "pc_t_ms")
                fprintf("  Missing cols in %s (skip)\n", fpath);
                continue;
            end

            t_s = (double(T.pc_t_ms) - double(T.pc_t_ms(1))) / 1000.0;
            raw = double(T.(raw_col));
            fil = double(T.(fil_col));

            idx = 1:PLOT_STRIDE:numel(t_s);

            c = trial_colors(r, :);
            plot(ax1, t_s(idx), raw(idx), "Color", c, "LineWidth", 0.8);
            plot(ax2, t_s(idx), fil(idx), "Color", c, "LineWidth", 0.9);

            legend_entries(r) = sprintf("Trial %s", string(G.trial_id(r)));
        end

        yline(ax1, 0, "k--", "LineWidth", 0.4, "HandleVisibility", "off");
        yline(ax2, 0, "k--", "LineWidth", 0.4, "HandleVisibility", "off");

        if SHOW_LEGEND
            legend(ax2, legend_entries, "Location", "northeastoutside");
        end
        sgtitle(sprintf("Gesture %s | Channel %d | Raw vs %s (separate)", g_name, ch, filt_label));

        if SAVE_PNG
            out_name = sprintf("%s_ch%d_%s_raw_vs_%s.png", lower(g_name), ch, lower(PLOT_MODE), lower(FILTER_SOURCE));
            exportgraphics(fig, fullfile(OUT_DIR, out_name), "Resolution", 150);
        end
    end
end

fprintf("\nDone.\n");
