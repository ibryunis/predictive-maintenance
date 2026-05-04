/*
 * PSCD Predictive Maintenance - TTGO T-Display
 * Receives status from MATLAB over USB serial
 * Displays: state, distance, block count
 *
 * Protocol from MATLAB (one line per update):
 *   "STATE,distance,blocknum"
 *   e.g. "ALARM,87.32,15" or "OK,12.50,16" or "CALIB,0,3"
 *
 * Group 1 - March 2026
 */

#include <TFT_eSPI.h>
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();

// Colors
#define BG_COLOR    TFT_BLACK
#define OK_COLOR    0x07E0   // green
#define ALARM_COLOR 0xF800   // red
#define CALIB_COLOR 0x04BF   // blue
#define IDLE_COLOR  0x8410   // gray
#define TEXT_COLOR  TFT_WHITE

// State tracking
String lastState   = "";
float  lastDist    = 0;
int    lastBlock   = 0;
bool   needRedraw  = true;

void drawScreen(const String &state, float dist, int block) {
    // Background
    uint16_t stateColor = IDLE_COLOR;
    if (state == "OK")     stateColor = OK_COLOR;
    if (state == "ALARM")  stateColor = ALARM_COLOR;
    if (state == "CALIB")  stateColor = CALIB_COLOR;

    // Top bar with state
    tft.fillRect(0, 0, 240, 60, stateColor);
    tft.setTextColor(TFT_WHITE, stateColor);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);
    tft.drawString(state, 120, 20, 4);  // font 4 = 26px

    // Distance value
    tft.fillRect(0, 60, 240, 50, BG_COLOR);
    tft.setTextColor(TEXT_COLOR, BG_COLOR);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Distance:", 120, 70, 2);
    
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", dist);
    tft.setTextColor(stateColor, BG_COLOR);
    tft.drawString(buf, 120, 95, 4);

    // Block counter
    tft.fillRect(0, 115, 240, 20, BG_COLOR);
    tft.setTextColor(IDLE_COLOR, BG_COLOR);
    tft.setTextDatum(MC_DATUM);
    snprintf(buf, sizeof(buf), "Block: %d", block);
    tft.drawString(buf, 120, 125, 2);
}

void setup() {
    Serial.begin(115200);

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
}

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();

        // Parse: "STATE,distance,block"
        int c1 = line.indexOf(',');
        int c2 = line.indexOf(',', c1 + 1);

        if (c1 > 0 && c2 > c1) {
            String state = line.substring(0, c1);
            float dist   = line.substring(c1 + 1, c2).toFloat();
            int block    = line.substring(c2 + 1).toInt();

            // Only redraw if something changed
            if (state != lastState || abs(dist - lastDist) > 0.1 || block != lastBlock) {
                drawScreen(state, dist, block);
                lastState = state;
                lastDist  = dist;
                lastBlock = block;
            }
        }
    }
}
