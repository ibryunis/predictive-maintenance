/*
 * PSCD Predictive Maintenance - TTGO T-Display
 * Receives a single status byte from the STM32 over I2C (forwarded from MATLAB)
 * and shows a colored full-screen state.
 *
 * State byte: 0 = IDLE (grey), 1 = CALIB (blue), 2 = OK (green), 3 = ALARM (red)
 *
 * Group 1 - 2026
 */

#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>

static constexpr uint8_t I2C_SLAVE_ADDRESS = 0x55;
static constexpr uint8_t SDA_PIN = 21;
static constexpr uint8_t SCL_PIN = 22;

TFT_eSPI tft = TFT_eSPI();

// Colors
#define OK_COLOR    0x07E0   // green
#define ALARM_COLOR 0xF800   // red
#define CALIB_COLOR 0x04BF   // blue
#define IDLE_COLOR  0x8410   // gray

volatile uint8_t receivedValue = 0;
volatile bool    newValue      = false;
int lastDrawn = -1;

void receiveEvent(int byteCount)
{
    // Take the last byte of the transaction as the state code.
    while (Wire.available())
    {
        receivedValue = (uint8_t)Wire.read();
    }
    newValue = true;
}

void drawState(uint8_t value)
{
    uint16_t bg;
    const char *line1;
    const char *line2;

    switch (value)
    {
    case 1:  bg = CALIB_COLOR; line1 = "CALIBRATING"; line2 = "keep still";       break;
    case 2:  bg = OK_COLOR;    line1 = "OK";          line2 = "healthy";          break;
    case 3:  bg = ALARM_COLOR; line1 = "ALARM";       line2 = "maintenance!";     break;
    default: bg = IDLE_COLOR;  line1 = "IDLE";        line2 = "waiting...";       break;
    }

    tft.fillScreen(bg);
    tft.setTextColor(TFT_WHITE, bg);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(line1, 120, 50, 4);   // big state name
    tft.drawString(line2, 120, 95, 2);   // small subtitle
}

void setup()
{
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);          // backlight on

    tft.init();
    tft.setRotation(1);                  // landscape 240x135

    // Startup screen
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("PSCD Group 1", 120, 45, 4);
    tft.drawString("Waiting for data...", 120, 90, 2);

    Wire.begin(I2C_SLAVE_ADDRESS, SDA_PIN, SCL_PIN, 100000);
    Wire.onReceive(receiveEvent);
}

void loop()
{
    if (!newValue)
    {
        return;
    }

    noInterrupts();
    uint8_t value = receivedValue;
    newValue = false;
    interrupts();

    if ((int)value != lastDrawn)
    {
        drawState(value);
        lastDrawn = value;
    }
}
