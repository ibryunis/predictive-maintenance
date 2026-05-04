# PSCD Predictive Maintenance

FFT-based vibration anomaly detection for the PSCD course (Group 1, March 2026).

An accelerometer on a rotating machine streams samples to a laptop. MATLAB runs an FFT, compares the spectrum to a stored baseline using Euclidean distance, and raises an alarm on two outputs: the on-board LED of the STM32 board and a TTGO T-Display screen.

## Hardware

- **B-L475E-IOT01A** (STM32 IoT Discovery Kit) with built-in **LSM6DSL** accelerometer (I2C)
- **TTGO T-Display** (ESP32 + ST7789 1.14" colour TFT) as a remote alarm screen
- Two USB cables to the laptop

## Repo layout

```
ttgo_firmware/                  PlatformIO project for the TTGO display
  src/main.cpp                  Receives "STATE,distance,blocknum" over USB serial
  platformio.ini                Board + display config (ST7789, 135x240)
predictive_maintenance_with_ttgo.m   MATLAB script (FFT + anomaly detection)
```

## How to run

1. Flash the TTGO with the firmware in `ttgo_firmware/` (PlatformIO: `pio run -t upload`).
2. Plug the STM32 board into the laptop. It should show up as `/dev/ttyACM0`.
3. Plug the TTGO into the laptop. It should show up as `/dev/ttyACM1`.
4. Open `predictive_maintenance_with_ttgo.m` in MATLAB and run it.
5. Press ENTER to start a 10-second calibration. Keep the board still.
6. After calibration, monitoring starts. Shake the board to trigger ALARM on the TTGO.

If the ports come up in a different order, edit `PORT_STM32` and `PORT_TTGO` at the top of the MATLAB script.

## Group 1

Yunis Ibrahimov, Bogdan Ghetu, Nikito Wilson, Onno Oerbekke, Akram Al-Hebshi.
