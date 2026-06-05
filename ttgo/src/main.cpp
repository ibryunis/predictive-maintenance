/*
 * PSCD Predictive Maintenance - TTGO T-Display
 * Group 1 - 2026
 *
 * Receives "STATE,distance,block" from the STM32 over I2C (forwarded from
 * MATLAB), shows it on the screen, and also sends it to a phone over BLE.
 *   e.g. "ALARM,87.32,15"  "OK,12.50,16"  "CALIB,0,3"
 *
 * BLE:  device PCSD_GRP1_TTGO
 *       service        4fafc201-1fb5-459e-8fcc-c5c9c331914b
 *       characteristic beb5483e-36e1-4688-b7f5-ea07361b26a8  (notify)
 */

#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

// ====================== Settings ======================
static const uint8_t I2C_ADDRESS = 0x55;
static const uint8_t SDA_PIN = 21;
static const uint8_t SCL_PIN = 22;

#define OK_COLOR    0x07E0   // green
#define ALARM_COLOR 0xF800   // red
#define CALIB_COLOR 0x04BF   // blue
#define IDLE_COLOR  0x8410   // gray

TFT_eSPI tft = TFT_eSPI();

// BLE
BLECharacteristic* pChar = nullptr;
bool phoneConnected = false;

// Last received message, shared between the I2C callback and loop().
// Wire.onReceive runs in a separate ESP32 task, so guard it with a critical
// section (portMUX) instead of noInterrupts().
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool lineReady = false;
char lineBuf[64];

// Last drawn values, so we only redraw on a real change
String lastState = "";
float  lastDist  = 0;
int    lastBlock = 0;

// ====================== BLE connect/disconnect ======================
class ServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer*) override    { phoneConnected = true; }
    void onDisconnect(BLEServer*) override { phoneConnected = false; BLEDevice::startAdvertising(); }
};

// ====================== I2C receive ======================
// The STM32 sends one full message per I2C transaction. Read it into a local
// buffer, then copy it into the shared one under a short critical section.
void onI2CReceive(int count)
{
    (void)count;
    char tmp[64];
    uint8_t i = 0;

    while (Wire.available())
    {
        char c = (char)Wire.read();
        if (c == '\n' || c == '\r') { continue; }      // skip line endings
        if (i < sizeof(tmp) - 1) { tmp[i++] = c; }
    }
    tmp[i] = '\0';

    if (i > 0)
    {
        taskENTER_CRITICAL(&rxMux);
        memcpy(lineBuf, tmp, i + 1);
        lineReady = true;
        taskEXIT_CRITICAL(&rxMux);
    }
}

// ====================== Screen ======================
void drawScreen(const String& state, float dist, int block)
{
    uint16_t color = IDLE_COLOR;
    char buf[16];
    if      (state == "OK")    { color = OK_COLOR; }
    else if (state == "ALARM") { color = ALARM_COLOR; }
    else if (state == "CALIB") { color = CALIB_COLOR; }

    tft.setTextDatum(MC_DATUM);

    // State (coloured top bar)
    tft.fillRect(0, 0, 240, 60, color);
    tft.setTextColor(TFT_WHITE, color);
    tft.drawString(state, 120, 20, 4);

    // Distance
    tft.fillRect(0, 60, 240, 50, TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Distance:", 120, 70, 2);
    snprintf(buf, sizeof(buf), "%.1f", dist);
    tft.setTextColor(color, TFT_BLACK);
    tft.drawString(buf, 120, 95, 4);

    // Block counter
    tft.fillRect(0, 110, 240, 25, TFT_BLACK);
    tft.setTextColor(IDLE_COLOR, TFT_BLACK);
    snprintf(buf, sizeof(buf), "Block: %d", block);
    tft.drawString(buf, 120, 121, 2);
}

// ====================== Setup ======================
void setup()
{
    // Display (drive the backlight high or the panel stays black)
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("PSCD Group 1", 120, 40, 4);
    tft.drawString("Predictive",   120, 70, 2);
    tft.drawString("Maintenance",  120, 90, 2);
    tft.setTextColor(IDLE_COLOR, TFT_BLACK);
    tft.drawString("Waiting for data...", 120, 120, 2);

    // I2C slave
    Wire.begin(I2C_ADDRESS, SDA_PIN, SCL_PIN, 100000);
    Wire.onReceive(onI2CReceive);

    // BLE server with one notify characteristic
    BLEDevice::init("PCSD_GRP1_TTGO");
    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    BLEService* pService = pServer->createService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
    pChar = pService->createCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8",
                                           BLECharacteristic::PROPERTY_NOTIFY);
    pChar->addDescriptor(new BLE2902());
    pService->start();
    BLEDevice::startAdvertising();
}

// ====================== Loop ======================
void loop()
{
    if (!lineReady) { return; }

    // Copy the latest message out of the shared buffer
    char msg[64];
    taskENTER_CRITICAL(&rxMux);
    strncpy(msg, lineBuf, sizeof(msg) - 1);
    msg[sizeof(msg) - 1] = '\0';
    lineReady = false;
    taskEXIT_CRITICAL(&rxMux);

    String line = String(msg);
    line.trim();

    // Parse "STATE,distance,block"
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    if (c1 <= 0 || c2 <= c1) { return; }   // need two commas in order

    String state = line.substring(0, c1);
    float  dist  = line.substring(c1 + 1, c2).toFloat();
    int    block = line.substring(c2 + 1).toInt();

    // Only act on a real change
    bool changed = (state != lastState) ||
                   (fabsf(dist - lastDist) > 0.1f) ||
                   (block != lastBlock);
    if (!changed) { return; }

    drawScreen(state, dist, block);
    if (phoneConnected && pChar != nullptr)
    {
        pChar->setValue(msg);     // send the same message to the phone
        pChar->notify();
    }

    lastState = state;
    lastDist  = dist;
    lastBlock = block;
}
