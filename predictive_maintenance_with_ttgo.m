% Predictive Maintenance - FFT Anomaly Detection
% PSCD Group 1 - March 2026
%
% Reads accelerometer data from STM32 via serial,
% computes FFT spectrum, detects anomalies using
% Euclidean distance, sends result to TTGO display.
%
% 1. Plug in STM32 + TTGO via USB
% 2. Run this script
% 3. Press ENTER to start calibration
% 4. Keep board still for 10 seconds
% 5. Shake the board to see ALARM on TTGO

clearvars;  close all;  clc;

%% Configuration
PORT_STM32  = '/dev/ttyACM0';
PORT_TTGO   = '/dev/ttyACM1';
BAUD        = 115200;
fs          = 500;            % sample rate [Hz]
N           = 512;            % block size (samples per FFT)
threshold   = 50;             % anomaly threshold
axis_sel    = 'Z';            % which axis: 'X', 'Y', or 'Z'
calib_time  = 10;             % calibration duration [s]

%% Frequency axis (single-sided, 0 to fs/2)
df   = fs / N;                         % frequency resolution [Hz]
f_ax = (0 : N/2) * df;                 % frequency vector [Hz]

%% Connect serial ports
fprintf('Connecting STM32 on %s ... ', PORT_STM32);
sStm = serialport(PORT_STM32, BAUD);
configureTerminator(sStm, "LF");
flush(sStm);
fprintf('OK\n');

fprintf('Connecting TTGO on %s ... ', PORT_TTGO);
sTtgo = serialport(PORT_TTGO, BAUD);
configureTerminator(sTtgo, "LF");
fprintf('OK\n\n');

cleanupObj = onCleanup(@() cellfun(@delete, {sStm, sTtgo}));

%% Setup figure
figure('Name', 'PSCD Predictive Maintenance', 'Position', [100 100 900 600]);

subplot(2,1,1);
hSpec = plot(f_ax, zeros(1, N/2+1), 'b-', 'LineWidth', 1);
hold on;
hBase = plot(f_ax, zeros(1, N/2+1), 'r--', 'LineWidth', 1.5);
hold off;
xlabel('Frequency [Hz]');  ylabel('Magnitude');
title('FFT Spectrum');     legend('Live', 'Baseline');
grid on;

subplot(2,1,2);
hDist = plot(NaN, NaN, 'k-o', 'LineWidth', 1, 'MarkerSize', 3);
hold on;
yline(threshold, 'r--', 'LineWidth', 1.5);
hold off;
xlabel('Block number');    ylabel('Euclidean distance');
title(sprintf('Anomaly Detection (threshold = %d)', threshold));
grid on;

%% Wait for user
writeline(sTtgo, 'IDLE,0,0');
fprintf('Press ENTER to start %d-second calibration...\n', calib_time);
pause();

%% Phase 1: Calibration - record baseline spectrum
fprintf('[CALIBRATION] Recording for %d seconds, keep board still...\n', calib_time);
writeline(sTtgo, 'CALIB,0,0');

buf         = [];              % sample buffer
calib_specs = [];              % store calibration spectra (rows)
t_start     = tic;

while toc(t_start) < calib_time
    % Read available serial data
    while sStm.NumBytesAvailable > 0
        line = readline(sStm);
        vals = str2double( strsplit( strtrim(line), ',' ) );
        if numel(vals) == 4 && ~any(isnan(vals))
            switch axis_sel
                case 'X';  buf(end+1) = vals(2);
                case 'Y';  buf(end+1) = vals(3);
                case 'Z';  buf(end+1) = vals(4);
            end
        end
    end

    % Process complete blocks
    while length(buf) >= N
        x   = buf(1:N);
        buf = buf(N+1:end);

        % FFT: single-sided amplitude spectrum (same as DSP course)
        Y  = fft(x) / N;
        P1 = abs( Y(1:N/2+1) );
        P1(2:end-1) = 2 * P1(2:end-1);

        calib_specs = [calib_specs; P1];
        set(hSpec, 'YData', P1);
        fprintf('  Calibration block %d\n', size(calib_specs, 1));
    end

    drawnow limitrate;
    pause(0.02);
end

if isempty(calib_specs)
    error('No data received. Check STM32 and serial port.');
end

% Baseline = average of all calibration spectra
baseline = mean(calib_specs, 1);
set(hBase, 'YData', baseline);
fprintf('[CALIBRATION] Done. Baseline from %d blocks.\n\n', size(calib_specs, 1));

%% Phase 2: Monitoring - detect anomalies
fprintf('[MONITORING] Running... shake the board to trigger alarm.\n');
writeline(sTtgo, 'OK,0,0');

dist_history = [];
block_num    = 0;
alarm_on     = false;

while ishandle(gcf)
    % Read serial
    while sStm.NumBytesAvailable > 0
        line = readline(sStm);
        vals = str2double( strsplit( strtrim(line), ',' ) );
        if numel(vals) == 4 && ~any(isnan(vals))
            switch axis_sel
                case 'X';  buf(end+1) = vals(2);
                case 'Y';  buf(end+1) = vals(3);
                case 'Z';  buf(end+1) = vals(4);
            end
        end
    end

    % Process blocks
    while length(buf) >= N
        x   = buf(1:N);
        buf = buf(N+1:end);
        block_num = block_num + 1;

        % FFT (same computation as calibration)
        Y  = fft(x) / N;
        P1 = abs( Y(1:N/2+1) );
        P1(2:end-1) = 2 * P1(2:end-1);

        % Euclidean distance between live spectrum and baseline
        d = sqrt( sum( (P1 - baseline).^2 ) );
        dist_history(end+1) = d;

        % Update plots
        set(hSpec, 'YData', P1);
        set(hDist, 'XData', 1:length(dist_history), 'YData', dist_history);

        % Check threshold
        if d > threshold
            if ~alarm_on
                fprintf('[ALARM]   Block %d, distance = %.1f\n', block_num, d);
            end
            alarm_on = true;
            writeline(sTtgo, sprintf('ALARM,%.2f,%d', d, block_num));
        else
            if alarm_on
                fprintf('[OK]      Block %d, distance = %.1f\n', block_num, d);
            end
            alarm_on = false;
            writeline(sTtgo, sprintf('OK,%.2f,%d', d, block_num));
        end
    end

    drawnow limitrate;
    pause(0.02);
end

fprintf('Done.\n');
