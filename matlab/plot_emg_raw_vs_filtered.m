% plot_emg_raw_vs_filtered.m
% Plot raw EMG versus processed EMG stages from fusion_emg_online_*.csv.
%
% Expected input file columns (from capture_fusion.py):
%   pc_t_ms, emg_raw0..emg_raw7, emg_hp0..emg_hp7, emg_rect0..emg_rect7, emg_env0..emg_env7
%
% Default behavior:
%   - auto-picks latest ../emg_data/fusion_emg_online_*.csv
%   - Figure 1: raw vs selected stage (HP/RECT/ENV)
%   - Figure 2: full chain per channel (raw, HP, rectified, envelope)

clear; clc; close all;

% ── User settings ────────────────────────────────────────────────────────────
CSV_FILE = "";                % leave empty to auto-pick latest online file
FILTER_SOURCE = "env";        % "hp", "rect", or "env" (final output is usually "env")
CHANNELS = 0:7;                % channels to plot
USE_CHANNEL_GROUPS = true;     % true: split channels into multiple clearer figures
CHANNEL_GROUP_SIZE = 4;        % channels per figure when grouping is enabled
SINGLE_CHANNEL = -1;           % set 0..7 to plot one channel only; -1 disables
COMPARE_FINAL_ONLY = true;     % true: show raw vs final selected output only
SHOW_FULL_CHAIN = false;       % set true only when debugging each stage
DOCK_FIGURES = false;          % true: open figures docked inside MATLAB desktop
FIG_SIZE = [1200 700];         % [width height] for normal windows
% ─────────────────────────────────────────────────────────────────────────────

if DOCK_FIGURES
    set(groot, "DefaultFigureWindowStyle", "docked");
else
    set(groot, "DefaultFigureWindowStyle", "normal");
end

if SINGLE_CHANNEL >= 0
    CHANNELS = SINGLE_CHANNEL;
    USE_CHANNEL_GROUPS = false;
end

% Resolve input file
if CSV_FILE == ""
    d = dir("../emg_data/fusion_emg_online_*.csv");
    if isempty(d)
        [fname, fpath] = uigetfile("*.csv", "Select fusion_emg_online CSV", "../emg_data");
        if isequal(fname, 0)
            disp("No file selected.");
            return;
        end
        CSV_FILE = fullfile(fpath, fname);
    else
        [~, idx] = max([d.datenum]);
        CSV_FILE = fullfile(d(idx).folder, d(idx).name);
    end
end

fprintf("Using file: %s\n", CSV_FILE);

T = readtable(CSV_FILE);
vars = string(T.Properties.VariableNames);

if ~any(vars == "pc_t_ms")
    error("Missing required column: pc_t_ms");
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

% Validate columns for requested channels
for ch = CHANNELS
    raw_col = sprintf("emg_raw%d", ch);
    fil_col = sprintf("%s%d", filt_prefix, ch);

    if ~any(vars == string(raw_col))
        error("Missing required column: %s", raw_col);
    end
    if ~any(vars == string(fil_col))
        error("Missing required column: %s", fil_col);
    end

    if SHOW_FULL_CHAIN
        hp_col = sprintf("emg_hp%d", ch);
        rect_col = sprintf("emg_rect%d", ch);
        env_col = sprintf("emg_env%d", ch);

        if ~any(vars == string(hp_col))
            error("Missing required column: %s", hp_col);
        end
        if ~any(vars == string(rect_col))
            error("Missing required column: %s", rect_col);
        end
        if ~any(vars == string(env_col))
            error("Missing required column: %s", env_col);
        end
    end
end

% Time axis (seconds)
t_ms = double(T.pc_t_ms);
t_s = (t_ms - t_ms(1)) / 1000.0;

n_samp = height(T);
dur_s = t_s(end);

dt = diff(t_ms);
dt_pos = dt(dt > 0);
if isempty(dt_pos)
    fs = NaN;
else
    fs = 1000.0 / mean(dt_pos);
end

fprintf("Samples  : %d\n", n_samp);
fprintf("Duration : %.2f s\n", dur_s);
if ~isnan(fs)
    fprintf("Rate     : %.1f Hz\n", fs);
else
    fprintf("Rate     : N/A\n");
end
fprintf("Filter   : %s\n", filt_label);
if COMPARE_FINAL_ONLY
    fprintf("Mode     : Raw vs final output only\n");
else
    fprintf("Mode     : Detailed comparison\n");
end

% Build channel groups for clearer plotting
ch_list = CHANNELS(:)';
if USE_CHANNEL_GROUPS
    groups = {};
    start_idx = 1;
    while start_idx <= numel(ch_list)
        stop_idx = min(start_idx + CHANNEL_GROUP_SIZE - 1, numel(ch_list));
        groups{end+1} = ch_list(start_idx:stop_idx); %#ok<AGROW>
        start_idx = stop_idx + 1;
    end
else
    groups = {ch_list};
end

% Figure 1: one plot per channel with separate raw and filtered subplots
for i = 1:numel(ch_list)
    ch = ch_list(i);

    raw_col = sprintf("emg_raw%d", ch);
    fil_col = sprintf("%s%d", filt_prefix, ch);

    raw = double(T.(raw_col));
    fil = double(T.(fil_col));

    fig = figure("Name", sprintf("EMG Ch %d: Raw and %s", ch, filt_label), ...
        "NumberTitle", "off");
    if ~DOCK_FIGURES
        screen = get(groot, "ScreenSize"); % [left bottom width height]
        w = min(FIG_SIZE(1), max(400, screen(3) - 80));
        h = min(FIG_SIZE(2), max(300, screen(4) - 120));
        fig.Position = [120 80 w h];
        movegui(fig, "onscreen");
    end

    tiledlayout(2, 1, "TileSpacing", "compact", "Padding", "compact");

    nexttile;
    plot(t_s, raw, "Color", [0.20 0.20 0.20], "LineWidth", 0.7);
    yline(0, "k--", "LineWidth", 0.4, "HandleVisibility", "off");
    grid on;
    ylabel("Raw");
    title(sprintf("Channel %d - Raw", ch));

    nexttile;
    plot(t_s, fil, "Color", [0.85 0.20 0.20], "LineWidth", 0.9);
    yline(0, "k--", "LineWidth", 0.4, "HandleVisibility", "off");
    grid on;
    ylabel(filt_label);
    xlabel("Time (s)");
    title(sprintf("Channel %d - %s", ch, filt_label));

    if ~isnan(fs)
        sgtitle(sprintf("EMG Channel %d  |  Raw vs %s (separate)  |  %.1f Hz", ch, filt_label, fs));
    else
        sgtitle(sprintf("EMG Channel %d  |  Raw vs %s (separate)", ch, filt_label));
    end
end

% Figure 2: full chain overlay for each channel
if SHOW_FULL_CHAIN && ~COMPARE_FINAL_ONLY
    for g = 1:numel(groups)
        ch_group = groups{g};
        n_ch = numel(ch_group);

        fig2 = figure("Name", sprintf("EMG Full Processing Chain (Group %d/%d)", g, numel(groups)), ...
            "NumberTitle", "off");
        if ~DOCK_FIGURES
            screen = get(groot, "ScreenSize");
            w = min(1400, max(400, screen(3) - 80));
            h = min(900, max(300, screen(4) - 120));
            fig2.Position = [80 50 w h];
            movegui(fig2, "onscreen");
        end

        sp2 = gobjects(n_ch, 1);
        for i = 1:n_ch
            ch = ch_group(i);

            raw = double(T.(sprintf("emg_raw%d", ch)));
            hp = double(T.(sprintf("emg_hp%d", ch)));
            rect = double(T.(sprintf("emg_rect%d", ch)));
            env = double(T.(sprintf("emg_env%d", ch)));

            sp2(i) = subplot(n_ch, 1, i);

            yyaxis left;
            plot(t_s, raw, "Color", [0.25 0.25 0.25], "LineWidth", 0.5, "DisplayName", "Raw");
            hold on;
            plot(t_s, hp, "Color", [0.00 0.45 0.74], "LineWidth", 0.8, "DisplayName", "HP");
            yline(0, "k--", "LineWidth", 0.4, "HandleVisibility", "off");
            ylabel(sprintf("Ch %d", ch));

            yyaxis right;
            plot(t_s, rect, "Color", [0.85 0.33 0.10], "LineWidth", 0.7, "DisplayName", "Rect");
            plot(t_s, env, "Color", [0.49 0.18 0.56], "LineWidth", 1.0, "DisplayName", "Env");
            ylabel("Rect/Env");

            grid on;
            if i < n_ch
                set(gca, "XTickLabel", []);
            end

            if i == 1
                yyaxis left;
                hRaw = plot(nan, nan, "Color", [0.25 0.25 0.25], "LineWidth", 0.8, "DisplayName", "Raw");
                hHP = plot(nan, nan, "Color", [0.00 0.45 0.74], "LineWidth", 1.0, "DisplayName", "HP");
                yyaxis right;
                hRect = plot(nan, nan, "Color", [0.85 0.33 0.10], "LineWidth", 1.0, "DisplayName", "Rect");
                hEnv = plot(nan, nan, "Color", [0.49 0.18 0.56], "LineWidth", 1.0, "DisplayName", "Env");
                legend([hRaw hHP hRect hEnv], "Location", "northeast");
            end
        end

        xlabel("Time (s)");
        linkaxes(sp2, "x");

        ch_txt = sprintf("%d ", ch_group);
        if ~isnan(fs)
            sgtitle(sprintf("EMG Processing Chain (Raw -> HP -> Rect -> Env)  |  Group %d/%d  |  Ch: [%s] | %.1f Hz", ...
                g, numel(groups), strtrim(ch_txt), fs));
        else
            sgtitle(sprintf("EMG Processing Chain (Raw -> HP -> Rect -> Env)  |  Group %d/%d  |  Ch: [%s]", ...
                g, numel(groups), strtrim(ch_txt)));
        end
    end
end

