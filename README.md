# PSCD Predictive Maintenance - Group 1

A proof-of-concept that detects a machine fault from vibration. An accelerometer
on the STM32 streams vibration to a laptop; MATLAB runs an FFT, compares the live
spectrum to a healthy baseline, and raises an alarm when they differ too much.
The alarm state is shown on the TTGO screen and sent to a phone over Bluetooth.

## Parts

| Folder    | Runs on            | Job                                                        |
|-----------|--------------------|-----------------------------------------------------------|
| `stm32/`  | STM32 B-L475E      | Sample the accelerometer, stream blocks, forward the state |
| `matlab/` | Laptop (MATLAB)    | FFT + baseline + distance + alarm decision (the dashboard)  |
| `ttgo/`   | TTGO T-Display     | Show the state on screen + notify a phone over BLE          |

## Hardware and wiring

- **STM32 B-L475E-IOT01A** - built-in LSM6DSL accelerometer.
- **TTGO T-Display** - ESP32 + colour screen.

```
STM32 PB9 (SDA)  <->  TTGO GPIO 21 (SDA)
STM32 PB8 (SCL)  <->  TTGO GPIO 22 (SCL)
STM32 GND        <->  TTGO GND          (common ground is required)
```
The TTGO also needs its own USB cable for power.

## How it works

1. The STM32 samples X, Y, Z every 1 ms (1000 Hz) and sends a block of 128
   samples to MATLAB over USB serial, framed as:
   `BEGIN_BUFFER` / `x,y,z` lines / `END_BUFFER`.
2. MATLAB combines the three axes into one vibration spectrum (FFT), removes
   gravity, and measures the Euclidean distance to a healthy baseline.
3. MATLAB sends one reply line back per block: `STATE,distance,block`
   (STATE = `CALIB` / `OK` / `ALARM`).
4. The STM32 forwards that line to the TTGO over I2C.
5. The TTGO shows the state (green / red / blue) plus the distance and block,
   and notifies a connected phone over BLE (device `PCSD_GRP1_TTGO`).

## Build and flash (PlatformIO in VS Code)

Open the `stm32/` folder, plug in the STM32, press **Upload**.
Open the `ttgo/` folder, plug in the TTGO, press **Upload**.

## Run the demo

1. In MATLAB, open `matlab/pm_test.m` and check `PORT` matches the ST-LINK COM
   port (Device Manager on Windows, or `/dev/ttyACM0` on Linux).
2. Run the script - a window with a **CALIBRATE** button appears.
3. Press the **reset** button on the STM32 so it starts a fresh stream.
4. With the fan **healthy and running steadily**, click **CALIBRATE**.
5. Introduce the fault (tape/coin on a blade) - the state should switch to ALARM.

## Key settings (must match across files)

| Setting   | Value  | Where                                   |
|-----------|--------|-----------------------------------------|
| Sample rate | 1000 Hz | STM32 1 ms timer, MATLAB `fs`         |
| Block size  | 128     | STM32 `N`, MATLAB `N`                 |
| Baud        | 115200  | STM32 `Serial_Setup`, MATLAB `BAUD`  |
| Threshold   | 50      | MATLAB `threshold` (tune at the rig)  |

## Notes

- **Always calibrate on the healthy fan.** The baseline becomes "normal", so
  anything different later (a fault, or the fan stopping) reads as an alarm.
- **`threshold` may need tuning** at the real rig: watch the `d = ...` value
  printed in the MATLAB Command Window (healthy vs faulty) and set it in between.
