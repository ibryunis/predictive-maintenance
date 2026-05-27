% Predictive Maintenance - FFT Anomaly Detection (no TTGO in MATLAB)
% PSCD Group 1 - March 2026
%
% Reads accelerometer data from the STM32 via serial, computes the
% single-sided FFT spectrum, and compares it to a healthy baseline
% with the Euclidean distance. Sends a state byte to the STM32
% (1 = good, 3 = alarm); the STM32 forwards it to the TTGO.

% Initialisation
clearvars;  close all;  clc;

%% Configuration
PORT_STM32  = '/dev/ttyACM0';
BAUD        = 115200;
fs          = 500;        % sample rate [Hz]
N           = 512;        % samples per FFT block
threshold   = 50;         % anomaly threshold
axis_sel    = 'Z';        % accelerometer axis: 'X', 'Y' or 'Z'
calib_blocks = 10;        % how many blocks to average for the baseline

df   = fs / N;            % frequency resolution [Hz]
f_ax = ( 0 : N/2 ) * df;  % single-sided frequency axis [Hz]

% Which CSV column holds the chosen axis (time_ms,x_mg,y_mg,z_mg)
switch axis_sel
    case 'X';  col = 2;
    case 'Y';  col = 3;
    case 'Z';  col = 4;
end

%% Connect to the STM32
sStm = serialport( PORT_STM32, BAUD );
configureTerminator( sStm, "LF" );
flush( sStm );

% Send one state byte to the STM32 (1 = good/green, 3 = alarm/red on TTGO)
sendState = @(s) write( sStm, uint8(s), "uint8" );

%% Figure: live spectrum + distance history
figure('Name','PSCD Predictive Maintenance');

subplot(211);
hSpec = plot( f_ax, zeros(1,N/2+1), 'b-' );  hold on;
hBase = plot( f_ax, zeros(1,N/2+1), 'r--' ); hold off;
xlabel('frequency [Hz]');  ylabel('amplitude');  grid on;
title('FFT spectrum');     legend('live','baseline');

subplot(212);
hDist = plot( NaN, NaN, 'k-o' );  hold on;
yline( threshold, 'r--' );        hold off;
xlabel('block');  ylabel('Euclidean distance');  grid on;
title( sprintf('Anomaly detection (threshold = %d)', threshold) );

%% Calibration: average the spectrum over several healthy blocks
sendState( 1 );
fprintf('Press ENTER, then keep the board still for calibration...\n');
pause();

baseline = zeros( 1, N/2+1 );
for k = 1 : calib_blocks
    P        = singleSided( readBlock(sStm, N, col), N );
    baseline = baseline + P;
    set( hSpec, 'YData', P );  drawnow;
    fprintf('  calibration block %d/%d\n', k, calib_blocks );
end
baseline = baseline / calib_blocks;
set( hBase, 'YData', baseline );
fprintf('Calibration done.\n');

%% Monitoring: compare each new block to the baseline
fprintf('Monitoring... shake the board to trigger the alarm.\n');
dist = [];
while ishandle( gcf )
    P = singleSided( readBlock(sStm, N, col), N );
    d = sqrt( sum( (P - baseline).^2 ) );      % Euclidean distance
    dist(end+1) = d;

    set( hSpec, 'YData', P );
    set( hDist, 'XData', 1:numel(dist), 'YData', dist );
    drawnow;

    if d > threshold
        sendState( 3 );        % alarm -> red
    else
        sendState( 1 );        % good  -> green
    end
end

%% Helper functions
function P = singleSided( x, N )
    % Single-sided amplitude spectrum (same as the DSP course)
    Y = fft( x ) / N;
    P = abs( Y(1:N/2+1) );
    P(2:end-1) = 2 * P(2:end-1);
end

function x = readBlock( s, N, col )
    % Read N accelerometer samples from one "time_ms,x,y,z" CSV stream
    x = zeros( 1, N );
    n = 0;
    while n < N
        v = str2double( strsplit( strtrim( readline(s) ), ',' ) );
        if numel(v) == 4 && ~any( isnan(v) )
            n = n + 1;
            x(n) = v(col);
        end
    end
end
