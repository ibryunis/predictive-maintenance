%% Predictive Maintenance - FFT anomaly detection
% Yunis Ibrahimov - Group 1
% Streams x,y,z from the STM32, runs an FFT, and compares each block to a
% healthy baseline. Too far from the baseline (Euclidean distance) = alarm.

clearvars; close all; clc;        % clear workspace

% --- settings ---
port      = "COM8";               % ST-LINK serial port
baud      = 115200;               % must match the STM32
fs        = 1000;                 % sampling frequency [Hz]
N         = 128;                  % samples per block
threshold = 50;                   % alarm level (tune at the rig)
nCalib    = 15;                   % blocks averaged for the baseline
nSmooth   = 5;                    % blocks averaged before deciding
fr        = (0:N/2)*fs/N;         % single-sided frequency vector [Hz]

% --- connect to the STM32 ---
stm = serialport(port, baud);     % open the port
configureTerminator(stm, "CR/LF"); % STM32 ends lines with \r\n
stm.Timeout = 15;                 % serial timeout [s]
flush(stm);                       % drop old data

% --- plots ---
fig = figure('Name', 'Predictive Maintenance');     % new figure
subplot(211);                                        % spectrum plot
hLive = plot(fr, zeros(1,N/2+1), 'b');  hold on;     % live spectrum
hBase = plot(fr, zeros(1,N/2+1), 'r--'); hold off;   % baseline
xlim([0 fs/2]); ylim([0 200]); grid on;              % axes
xlabel('frequency [Hz]'); ylabel('amplitude'); legend('live','baseline');

subplot(212);                                        % distance plot
hDist = plot(NaN, NaN, 'b'); hold on;                % distance history
yline(threshold, 'r', 'threshold'); hold off;        % alarm line
xlabel('block'); ylabel('distance'); grid on;

% --- calibrate on the healthy fan (press the button) ---
btn = uicontrol('Style','pushbutton','String','CALIBRATE', ...   % button
    'Units','normalized', 'Position',[0.02 0.95 0.15 0.05], ...
    'Callback', @(~,~) uiresume(fig));
sgtitle('Press CALIBRATE (healthy fan, steady)');    % prompt
uiwait(fig);                                         % wait for the click
if ~ishandle(fig); return; end                       % window closed -> stop

baseline = zeros(1, N/2+1);                          % baseline spectrum
for k = 1:nCalib                                     % average healthy blocks
    P = ampSpectrum(readBlock(stm, N), N);           % one block spectrum
    baseline = baseline + P;                         % accumulate
    writeline(stm, sprintf('CALIB,0,%d', k));        % release the next block
end
baseline = baseline / nCalib;                        % mean = the baseline
set(hBase, 'YData', baseline);                       % show it
set(btn, 'String', 'CALIBRATED');                    % update the button

% --- monitoring loop ---
dist = [];                                           % distance history
blk  = 0;                                            % block counter
while ishandle(fig)                                  % until the window closes
    P  = ampSpectrum(readBlock(stm, N), N);          % live spectrum
    d  = norm(P - baseline);                         % distance from baseline
    dist(end+1) = d;                                 %#ok<SAGROW>
    ds = mean(dist(max(1,end-nSmooth+1):end));       % smooth the last few
    blk = blk + 1;                                   % next block

    if ds > threshold                                % decide the state
        state = 'ALARM'; col = 'r';                  % fault
    else
        state = 'OK';    col = [0 0.6 0];            % healthy
    end
    writeline(stm, sprintf('%s,%.2f,%d', state, ds, blk));   % reply to the STM32

    lo = max(1, blk-99);                             % last 100 points
    set(hLive, 'YData', P);                          % update spectrum
    set(hDist, 'XData', lo:blk, 'YData', dist(lo:end));      % update distance
    sgtitle(sprintf('%s   d=%.1f / %d   block %d', state, ds, threshold, blk), 'Color', col);
    drawnow limitrate;                               % refresh
    fprintf('d = %.2f   smoothed = %.2f\n', d, ds);  % print for tuning
end

% --- single-sided amplitude spectrum of x,y,z combined ---
function P = ampSpectrum(xyz, N)
    P = zeros(1, N/2+1);              % output spectrum
    for a = 1:3                       % each axis
        X = abs(fft(xyz(:,a))/N).';   % normalized magnitude
        X = X(1:N/2+1);               % keep 0..fs/2
        X(2:end-1) = 2*X(2:end-1);    % single-sided
        P = P + X.^2;                 % sum the axes (power)
    end
    P = sqrt(P);                      % combined magnitude
    P(1) = 0;                         % drop gravity (0 Hz)
end

% --- read one block of N "x,y,z" lines from the STM32 ---
function xyz = readBlock(s, N)
    while readline(s) ~= "BEGIN_BUFFER"; end          % wait for the start marker
    xyz = zeros(N, 3);                                % block buffer
    for i = 1:N                                       % read N samples
        xyz(i,:) = str2double(split(readline(s), ","))';   % parse "x,y,z"
    end
    readline(s);                                      % discard END_BUFFER
end
