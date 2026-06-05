% Predictive Maintenance - FFT Anomaly Detection
% Group 1 - 2026
%
% The STM32 streams accelerometer samples (x,y,z in mg) to the laptop. For each
% block we run an FFT, combine the three axes into one vibration spectrum, and
% compare it to a healthy baseline. If the live spectrum drifts too far from the
% baseline (Euclidean distance over a threshold) we raise an alarm. The state is
% sent back to the STM32, which forwards it to the TTGO display.

%% Configuration
clearvars;  close all;  clc;

PORT      = "COM8";     % ST-LINK virtual COM port (check Device Manager)
BAUD      = 115200;     % must match the STM32 firmware
fs        = 1000;       % [Hz] sample rate (STM32 samples every 1 ms)
N         = 128;        % samples per block (must match STM32 kSampleCount)
threshold = 50;         % alarm level (tune at the rig)
nCalib    = 15;         % healthy blocks averaged for the baseline
nSmooth   = 5;          % blocks of distance averaged before deciding
specYmax  = 200;        % [-] y-axis zoom for the spectrum plot
WIN       = 100;        % distance points kept on screen

df   = fs / N;          % [Hz] frequency resolution
freq = 0 : df : fs/2;   % [Hz] single-sided frequency axis

%% Connect to the STM32
stm = serialport( PORT, BAUD );
stm.Timeout = 15;
configureTerminator( stm, "LF" );
flush( stm );

% Reply sent once per block: "STATE,distance,block"  (STATE = CALIB / OK / ALARM)
sendStatus = @(state, d, blk) writeline( stm, sprintf('%s,%.2f,%d', state, d, blk) );

%% Set up the plots
fig = figure( 'Name', 'PSCD Predictive Maintenance', 'NumberTitle', 'off' );

% Live spectrum: current block vs healthy baseline
subplot( 2, 1, 1 );
hLive = plot( freq, zeros(1, N/2+1), 'b-',  'LineWidth', 1.5 );  hold on;
hBase = plot( freq, zeros(1, N/2+1), 'r--', 'LineWidth', 1.5 );  hold off;
xlabel('frequency   [Hz]');  ylabel('amplitude');  grid on;
xlim([ 0 fs/2 ]);  ylim([ 0 specYmax ]);
legend('live', 'baseline');  title('FFT spectrum');

% Distance history with the alarm threshold
subplot( 2, 1, 2 );
hDist = plot( NaN, NaN, 'b-', 'LineWidth', 1.5 );  hold on;
yline( threshold, 'r-', 'threshold' );  hold off;
xlabel('block');  ylabel('distance');  grid on;
title('anomaly distance');

%% Calibration  (press the button)
% IMPORTANT: calibrate on the HEALTHY fan running steadily. The baseline becomes
% "normal", so anything different later (a fault, or the fan stopping) alarms.
btn = uicontrol( fig, 'Style', 'pushbutton', 'String', 'CALIBRATE', ...
                 'FontSize', 12, 'FontWeight', 'bold', ...
                 'Units', 'normalized', 'Position', [0.02 0.945 0.15 0.05], ...
                 'Callback', @(~,~) uiresume(fig) );
sgtitle('Press CALIBRATE  (healthy fan, steady)');
uiwait( fig );                          % wait here until the button is clicked
if ~ishandle( fig );  return;  end      % window closed -> stop
set( btn, 'Enable', 'off', 'String', 'CALIBRATING...' );

baseline = zeros( 1, N/2+1 );
for k = 1 : nCalib
    P = singleSided( readBlock(stm, N), N );
    baseline = baseline + P;
    set( hLive, 'YData', P );  drawnow;
    sendStatus( 'CALIB', 0, k );        % release the next STM32 block
end
baseline = baseline / nCalib;
set( hBase, 'YData', baseline );
set( btn, 'String', 'CALIBRATED' );

%% Monitoring loop
recent = [];        % last nSmooth distances (for smoothing)
dist   = [];        % distance history (for the plot)
block  = 0;
while ishandle( fig )
    P = singleSided( readBlock(stm, N), N );
    d = norm( P - baseline );

    % Average the last nSmooth distances so one noisy block cannot false-alarm
    recent = [recent d];                          % add this block's distance
    recent = recent( max(1, end-nSmooth+1) : end );   % keep only the last nSmooth
    dSmooth = mean( recent );

    block = block + 1;
    dist(end+1) = dSmooth;                        %#ok<AGROW>

    % Decide the state and report it
    if dSmooth > threshold
        state = 'ALARM';  col = 'r';
    else
        state = 'OK';     col = [0 0.6 0];
    end
    sendStatus( state, dSmooth, block );

    % Update the plots (show only the most recent WIN points)
    lo = max( 1, block - WIN + 1 );
    set( hLive, 'YData', P );
    set( hDist, 'XData', lo:block, 'YData', dist(lo:end) );
    sgtitle( sprintf('%s    distance %.1f / %.0f    block %d', ...
             state, dSmooth, threshold, block), 'Color', col );
    drawnow limitrate;

    fprintf('d = %.2f   smoothed = %.2f   threshold = %g\n', d, dSmooth, threshold);
end

%% Helper functions
function P = singleSided( xyz, N )
    % Combine X, Y, Z into one vibration spectrum. The per-bin magnitude
    % sqrt(X^2+Y^2+Z^2) does not depend on orientation; the DC (0 Hz) bin is
    % zeroed afterwards to remove gravity.
    P = zeros( 1, N/2+1 );
    for a = 1 : 3
        X  = fft( xyz(:, a) ) / N;          % normalized FFT of one axis
        Xa = abs( X(1:N/2+1) ).';           % keep 0..fs/2 as a row
        Xa(2:end-1) = 2 * Xa(2:end-1);      % single-sided: double inner bins
        P = P + Xa.^2;
    end
    P = sqrt( P );
    P(1) = 0;                               % remove gravity
end

function xyz = readBlock( s, N )
    % Read N "x,y,z" lines between BEGIN_BUFFER and END_BUFFER.
    xyz = zeros( N, 3 );
    n = 0;  inBlock = false;
    while true
        try
            line = strtrim( string( readline(s) ) );
        catch
            error('Serial timeout. Check the COM port, baud, cable, and board.');
        end

        if strlength(line) == 0
            continue;                       % blank / timeout line
        elseif line == "BEGIN_BUFFER"
            inBlock = true;  n = 0;
        elseif line == "END_BUFFER"
            if n == N;  return;  end
            error('Block ended early: %d of %d samples.', n, N);
        elseif inBlock
            v = str2double( split(line, ",") );
            if numel(v) == 3 && ~any(isnan(v))
                n = n + 1;
                xyz(n, :) = v.';
            end
        end
    end
end
