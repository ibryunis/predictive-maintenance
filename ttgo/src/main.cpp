/*
 * PSCD Predictive Maintenance - TTGO T-Display
 * Receives status from the STM32 over I2C (forwarded from MATLAB).
 * Displays: state, distance, block count.
 *
 * Protocol (one line per update, sent by the STM32 over I2C):
 *   "STATE,distance,block"
 *   e.g. "ALARM,87.32,15" or "OK,12.50,16" or "CALIB,0,3"
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
#define BG_COLOR    TFT_BLACK
#define OK_COLOR    0x07E0   // green
#define ALARM_COLOR 0xF800   // red
#define CALIB_COLOR 0x04BF   // blue
#define IDLE_COLOR  0x8410   // gray
#define TEXT_COLOR  TFT_WHITE

// I2C message buffer.
// On ESP32, Wire.onReceive runs in a separate driver task, NOT a maskable ISR,
// so noInterrupts() would not protect lineBuf. Use a portMUX critical section,
// which serializes across both cores and the I2C task.
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool lineReady = false;
char          lineBuf[64];   // last complete message, guarded by rxMux

// State tracking (avoid pointless redraws)
String lastState = "";
float  lastDist  = 0;
int    lastBlock = 0;

void receiveEvent(int byteCount)
{
    // The STM32 sends one complete "STATE,distance,block" message per I2C
    // transaction, so treat each callback as one full message. Line endings
    // are ignored, so it works with or without a trailing newline.
    char tmp[64];
    uint8_t i = 0;

    while (Wire.available())
    {
        char c = (char)Wire.read();
        if (c == '\n' || c == '\r')
        {
            continue;
        }
        if (i < sizeof(tmp) - 1)
        {
            tmp[i++] = c;
        }
    }
    tmp[i] = '\0';

    if (i > 0)
    {
        taskENTER_CRITICAL(&rxMux);
        for (uint8_t k = 0; k <= i; k++)
        {
            lineBuf[k] = tmp[k];
        }
        lineReady = true;
        taskEXIT_CRITICAL(&rxMux);
    }
}

void drawScreen(const String &state, float dist, int block)
{
    uint16_t stateColor = IDLE_COLOR;
    if (state == "OK")    stateColor = OK_COLOR;
    if (state == "ALARM") stateColor = ALARM_COLOR;
    if (state == "CALIB") stateColor = CALIB_COLOR;

    // Top bar with state
    tft.fillRect(0, 0, 240, 60, stateColor);
    tft.setTextColor(TFT_WHITE, stateColor);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);
    tft.drawString(state, 120, 20, 4);

    // Distance value
    tft.fillRect(0, 60, 240, 50, BG_COLOR);
    tft.setTextColor(TEXT_COLOR, BG_COLOR);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Distance:", 120, 70, 2);

    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", dist);
    tft.setTextColor(stateColor, BG_COLOR);
    tft.drawString(buf, 120, 95, 4);

    // Block counter (kept above y=134 so it isn't clipped on the 135 px tall panel)
    tft.fillRect(0, 110, 240, 25, BG_COLOR);
    tft.setTextColor(IDLE_COLOR, BG_COLOR);
    tft.setTextDatum(MC_DATUM);
    snprintf(buf, sizeof(buf), "Block: %d", block);
    tft.drawString(buf, 120, 121, 2);
}

void setup()
{
    // Turn the backlight on, otherwise the panel stays black.
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    tft.init();
    tft.setRotation(1);    // landscape
    tft.fillScreen(BG_COLOR);

    // Startup screen
    tft.setTextColor(TEXT_COLOR, BG_COLOR);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("PSCD Group 1", 120, 40, 4);
    tft.drawString("Predictive", 120, 70, 2);
    tft.drawString("Maintenance", 120, 90, 2);
    tft.setTextColor(IDLE_COLOR, BG_COLOR);
    tft.drawString("Waiting for data...", 120, 120, 2);

    Wire.begin(I2C_SLAVE_ADDRESS, SDA_PIN, SCL_PIN, 100000);
    Wire.onReceive(receiveEvent);
}

void loop()
{
    if (!lineReady)
    {
        return;
    }

    char local[64];
    taskENTER_CRITICAL(&rxMux);
    for (uint8_t i = 0; i < sizeof(local); i++)
    {
        local[i] = lineBuf[i];
        if (lineBuf[i] == '\0') break;
    }
    lineReady = false;
    taskEXIT_CRITICAL(&rxMux);

    String line = String(local);
    line.trim();

    // Parse: "STATE,distance,block"
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);

    if (c1 > 0 && c2 > c1)
    {
        String state = line.substring(0, c1);
        float  dist  = line.substring(c1 + 1, c2).toFloat();
        int    block = line.substring(c2 + 1).toInt();

        if (state != lastState || fabsf(dist - lastDist) > 0.1f || block != lastBlock)
        {
            drawScreen(state, dist, block);
            lastState = state;
            lastDist  = dist;
            lastBlock = block;
        }
    }
}
