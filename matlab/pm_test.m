% Predictive Maintenance - FFT Anomaly Detection
% Matches current STM32 firmware:
% - 1024 samples per transmitted buffer
% - 1000 Hz sample rate
% - serial framing with BEGIN_BUFFER / END_BUFFER
% - MATLAB sends one line back after every buffer: "STATE,distance,block"
%   STATE is CALIB / OK / ALARM. The STM32 forwards it to the TTGO over I2C.

clearvars; close all; clc;

%% Configuration
PORT_STM32    = "COM8";    % <-- check Device Manager for the ST-LINK Virtual COM Port
BAUD          = 115200;    % must match STM32 serial_vcp.cpp
fs            = 1000;      % STM32 uses 1 ms sample period
N             = 128;       % must match STM32 kSampleCount
threshold     = 50;        % anomaly threshold
axis_sel      = 'Z';       % 'X', 'Y', or 'Z'
calib_blocks  = 5;         % number of healthy blocks for baseline
serialTimeout = 15;        % seconds
spec_ymax     = 200;       % FFT spectrum zoom: Y-axis max (lower = more zoomed in)

%% Derived values
df   = fs / N;
f_ax = (0:N/2) * df;

% CSV format from STM32: time_ms,x_mg,y_mg,z_mg
switch upper(axis_sel)
    case 'X'
        col = 2;
    case 'Y'
        col = 3;
    case 'Z'
        col = 4;
    otherwise
        error('axis_sel must be ''X'', ''Y'', or ''Z''.');
end

%% Connect to STM32
sStm = serialport(PORT_STM32, BAUD);
sStm.Timeout = serialTimeout;
configureTerminator(sStm, "LF");
flush(sStm);

% Send "STATE,distance,block" to the STM32, which forwards it to the TTGO.
% STATE is one of: CALIB, OK, ALARM (also IDLE at boot, set by the firmware).
sendStatus = @(state, dist, block) writeline(sStm, sprintf('%s,%.2f,%d', state, dist, block));

%% Figure: status dashboard + live spectrum + distance history
% Colours (state -> RGB)
COL_BG    = [0.12 0.12 0.14];   % dark background
COL_PANEL = [0.18 0.18 0.21];
COL_CALIB = [0.20 0.55 0.90];   % blue
COL_OK    = [0.20 0.70 0.35];   % green
COL_ALARM = [0.90 0.25 0.25];   % red
COL_TXT   = [0.92 0.92 0.94];

fig = figure('Name', 'PSCD Predictive Maintenance', 'NumberTitle', 'off', ...
             'Color', COL_BG, 'Position', [80 80 1000 720]);

% --- Big status banner (top) ---
hBanner = annotation(fig, 'textbox', [0.04 0.90 0.92 0.075], ...
    'String', 'CONNECTING...', 'FontSize', 22, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'Color', COL_TXT, 'BackgroundColor', COL_CALIB, ...
    'EdgeColor', 'none', 'FaceAlpha', 1);

% Helper to update the banner in one call
setBanner = @(state, col, txt) set(hBanner, 'String', txt, 'BackgroundColor', col);

% --- FFT spectrum (middle) ---
axSpec = axes(fig, 'Position', [0.08 0.50 0.86 0.33], ...
              'Color', COL_PANEL, 'XColor', COL_TXT, 'YColor', COL_TXT, ...
              'GridColor', [0.4 0.4 0.45]);
hold(axSpec, 'on');
hSpec = area(axSpec, f_ax, zeros(1, N/2+1), 'FaceColor', [0.30 0.65 0.95], ...
             'FaceAlpha', 0.5, 'EdgeColor', [0.40 0.75 1.0], 'LineWidth', 1.2);
hBase = plot(axSpec, f_ax, zeros(1, N/2+1), '--', 'Color', [1.0 0.75 0.30], 'LineWidth', 1.5);
hold(axSpec, 'off');
xlabel(axSpec, 'Frequency [Hz]'); ylabel(axSpec, 'Amplitude');
title(axSpec, 'Live FFT Spectrum', 'Color', COL_TXT);
legend(axSpec, {'Live', 'Healthy baseline'}, 'TextColor', COL_TXT, ...
       'Color', COL_PANEL, 'EdgeColor', [0.4 0.4 0.45], 'Location', 'northeast');
grid(axSpec, 'on'); xlim(axSpec, [0, fs/2]); ylim(axSpec, [0, spec_ymax]);

% --- Distance history (bottom) ---
axDist = axes(fig, 'Position', [0.08 0.08 0.86 0.33], ...
              'Color', COL_PANEL, 'XColor', COL_TXT, 'YColor', COL_TXT, ...
              'GridColor', [0.4 0.4 0.45]);
hold(axDist, 'on');
hThreshLine = yline(axDist, threshold, '-', sprintf('threshold = %.0f', threshold), ...
                    'Color', COL_ALARM, 'LineWidth', 1.5, ...
                    'LabelHorizontalAlignment', 'left');
hDist = plot(axDist, NaN, NaN, '-', 'Color', [0.6 0.8 1.0], 'LineWidth', 1.5);
hHead = plot(axDist, NaN, NaN, 'o', 'MarkerSize', 9, 'MarkerFaceColor', COL_OK, ...
             'MarkerEdgeColor', 'w', 'LineWidth', 1.2);   % current value marker
hold(axDist, 'off');
xlabel(axDist, 'Block'); ylabel(axDist, 'Euclidean distance');
title(axDist, 'Anomaly Detection', 'Color', COL_TXT);
grid(axDist, 'on');

%% Calibration
setBanner('CALIB', COL_CALIB, 'CLICK COMMAND WINDOW + PRESS ENTER TO CALIBRATE');
% input() reliably waits for ENTER in the Command Window. (pause is flaky
% because keypresses go to whichever window has focus, often the figure.)
input('Keep the board still, then click the Command Window and press ENTER to calibrate...', 's');

% Protocol: the STM32 sends a buffer, then waits for exactly ONE reply line
% before sending the next. So we read one buffer, then send one status line --
% strictly one-to-one, no priming line (that would desync the handshake).
baseline = zeros(1, N/2+1);

for k = 1:calib_blocks
    x = readStmBuffer(sStm, N, col);
    P = singleSided(x, N);

    baseline = baseline + P;

    % Build a little ASCII progress bar for the banner
    nfill = round(20 * k / calib_blocks);
    bar   = [repmat('#', 1, nfill), repmat('-', 1, 20 - nfill)];
    setBanner('CALIB', COL_CALIB, ...
        sprintf('CALIBRATING   [%s]   block %d / %d', bar, k, calib_blocks));

    set(hSpec, 'YData', P);
    drawnow;

    fprintf('  calibration block %d/%d\n', k, calib_blocks);

    % Release the next STM32 buffer and show calibration progress on the TTGO.
    sendStatus('CALIB', 0, k);
end

baseline = baseline / calib_blocks;
set(hBase, 'YData', baseline);
setBanner('OK', COL_OK, 'CALIBRATION DONE  -  monitoring...');
fprintf('Calibration done.\n');

%% Monitoring
fprintf('Monitoring... shake the board to trigger the alarm.\n');

WINDOW = 100;          % show only the most recent N blocks (keeps the plot fast)
dist  = [];
blocks = [];
block = 0;

while ishandle(fig)
    x = readStmBuffer(sStm, N, col);
    P = singleSided(x, N);
    d = norm(P - baseline);

    block = block + 1;
    dist(end+1)   = d;     %#ok<AGROW>
    blocks(end+1) = block; %#ok<AGROW>

    % Keep only the last WINDOW points so rendering stays snappy over time
    if numel(dist) > WINDOW
        dist   = dist(end-WINDOW+1:end);
        blocks = blocks(end-WINDOW+1:end);
    end

    set(hSpec, 'YData', P);
    set(hDist, 'XData', blocks, 'YData', dist);
    set(hHead, 'XData', block, 'YData', d);   % highlight the current value
    xlim(axDist, [blocks(1), max(blocks(end), blocks(1)+1)]);

    if d > threshold
        setBanner('ALARM', COL_ALARM, ...
            sprintf('ALARM    distance %.1f  >  %.0f    |    block %d', d, threshold, block));
        set(hHead, 'MarkerFaceColor', COL_ALARM);
        sendStatus('ALARM', d, block);
    else
        setBanner('OK', COL_OK, ...
            sprintf('OK    distance %.1f  /  %.0f    |    block %d', d, threshold, block));
        set(hHead, 'MarkerFaceColor', COL_OK);
        sendStatus('OK', d, block);
    end

    drawnow limitrate;     % faster, smoother updates than plain drawnow
    fprintf('d = %.2f   (threshold = %.2f)\n', d, threshold);
end

%% Helper functions
function P = singleSided(x, N)
    Y = fft(x) / N;
    P = abs(Y(1:N/2+1));
    P(2:end-1) = 2 * P(2:end-1);
end

function x = readStmBuffer(s, N, col)
    x = zeros(1, N);
    n = 0;
    inBuffer = false;

    while true
        try
            raw = readline(s);
        catch
            error(['Serial timeout while waiting for STM32 data. ' ...
                   'Check the COM port, baud rate, cable, and that the board is running.']);
        end

        line = strtrim(string(raw));
        if strlength(line) == 0
            continue;
        end

        if line == "BEGIN_BUFFER"
            inBuffer = true;
            n = 0;
            continue;
        end

        if ~inBuffer
            continue;
        end

        if line == "END_BUFFER"
            if n == N
                return;
            end
            error('STM32 buffer ended early: received %d of %d samples.', n, N);
        end

        % Each in-buffer line is a single Z-axis value. Any non-numeric line
        % (header / status / error text) parses to NaN and is skipped.
        v = str2double(line);
        if isnan(v)
            continue;
        end

        n = n + 1;
        if n > N
            error('Received more than %d samples before END_BUFFER. Check N in MATLAB and STM32.', N);
        end

        x(n) = v;
    end
end
