PSCD Predictive Maintenance - Demo Package
==========================================

Hardware:
  - STM32 B-L475E-IOT01A (built-in LSM6DSL accelerometer)
  - TTGO T-Display (ESP32 + ST7789 screen)

Wiring (STM32 <-> TTGO, I2C):
  STM32 PB9 (D14, SDA)  <->  TTGO GPIO 21 (SDA)
  STM32 PB8 (D15, SCL)  <->  TTGO GPIO 22 (SCL)
  STM32 GND             <->  TTGO GND
  (TTGO also needs its own USB for 5V power.)

How it works:
  STM32 samples the accelerometer at 1000 Hz, sends 1024-sample buffers
  to MATLAB over USB (framed with BEGIN_BUFFER / END_BUFFER). MATLAB runs
  an FFT, compares the live spectrum to a healthy baseline using Euclidean
  distance, and sends one state byte back per buffer:
     1 = good   -> TTGO green
     2 = alarm  -> TTGO orange

Setup (Windows laptop)
----------------------
1. Find the COM port:
   Device Manager -> Ports (COM & LPT) ->
   "STMicroelectronics STLink Virtual COM Port (COMx)".
   If it is NOT COM5, edit it in TWO places:
     - matlab/pm_test.m       line: PORT_STM32 = "COMx";
     - stm32/platformio.ini   line: monitor_port = COMx

2. Flash the STM32:
   Open the stm32/ folder in VS Code + PlatformIO, click Upload.

3. Flash the TTGO (only if it does not already have firmware):
   Open the ttgo/ folder in VS Code + PlatformIO, click Upload.

4. Run the demo:
   Open matlab/pm_test.m in MATLAB, run it, press ENTER, keep the board
   still for the 10 calibration blocks, then shake it. The screen should
   change and the "d = ..." value in MATLAB should jump above the threshold.

Tuning
------
If shaking does not trigger the alarm, watch the "d = ..." values printed
in MATLAB (still vs shaking) and set 'threshold' in pm_test.m to about 3x
the resting value.

MATLAB requires the Instrument Control Toolbox (for serialport).
Do NOT rename pm_test.m to test.m - "test" collides with a built-in.
