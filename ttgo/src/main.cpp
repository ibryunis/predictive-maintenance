/*
 * Predictive Maintenance - TTGO T-Display - Group 1
 *
 * Receives "STATE,distance,block" from the STM32 over I2C (forwarded from
 * MATLAB), shows it on the screen, and notifies a phone over Bluetooth (BLE).
 *   e.g. "ALARM,87.32,15"  "OK,12.50,16"  "CALIB,0,3"
 *
 * The two long BLE strings below are UUIDs - one-off random IDs that just name
 * our data channel. Not secret; the phone app uses the same strings to find us.
 */

#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

// --- settings ---
#define I2C_ADDR 0x55
#define SDA_PIN  21
#define SCL_PIN  22
#define OK_COLOR    0x07E0   // green
#define ALARM_COLOR 0xF800   // red
#define CALIB_COLOR 0x04BF   // blue
#define IDLE_COLOR  0x8410   // gray

TFT_eSPI tft = TFT_eSPI();
BLECharacteristic* ble = nullptr;
bool phoneConnected = false;

// The I2C message, shared between the receive callback and loop(). onReceive
// runs in a separate ESP32 task, so guard the buffer with a critical section.
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool lineReady = false;
char lineBuf[64];

// --- BLE connect / disconnect ---
class ServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer*) override    { phoneConnected = true; }
    void onDisconnect(BLEServer*) override { phoneConnected = false; BLEDevice::startAdvertising(); }
};

// --- I2C receive (one full message per transaction) ---
void onI2CReceive(int)
{
    char tmp[64];
    uint8_t i = 0;
    while (Wire.available())
    {
        char c = (char)Wire.read();
        if (c == '\n' || c == '\r') continue;          // skip line endings
        if (i < sizeof(tmp) - 1) tmp[i++] = c;
    }
    tmp[i] = '\0';
    if (i == 0) return;

    taskENTER_CRITICAL(&mux);
    memcpy(lineBuf, tmp, i + 1);
    lineReady = true;
    taskEXIT_CRITICAL(&mux);
}

// --- draw the screen: coloured state bar, distance, block ---
void drawScreen(const String& state, float dist, int block)
{
    uint16_t color = IDLE_COLOR;
    if      (state == "OK")    color = OK_COLOR;
    else if (state == "ALARM") color = ALARM_COLOR;
    else if (state == "CALIB") color = CALIB_COLOR;

    char buf[16];
    tft.setTextDatum(MC_DATUM);

    tft.fillRect(0, 0, 240, 60, color);                // state bar
    tft.setTextColor(TFT_WHITE, color);
    tft.drawString(state, 120, 20, 4);

    tft.fillRect(0, 60, 240, 50, TFT_BLACK);           // distance
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Distance:", 120, 70, 2);
    snprintf(buf, sizeof(buf), "%.1f", dist);
    tft.setTextColor(color, TFT_BLACK);
    tft.drawString(buf, 120, 95, 4);

    tft.fillRect(0, 110, 240, 25, TFT_BLACK);          // block counter
    tft.setTextColor(IDLE_COLOR, TFT_BLACK);
    snprintf(buf, sizeof(buf), "Block: %d", block);
    tft.drawString(buf, 120, 121, 2);
}

void setup()
{
    pinMode(TFT_BL, OUTPUT);                            // backlight on
    digitalWrite(TFT_BL, HIGH);
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("PSCD Group 1", 120, 50, 4);
    tft.setTextColor(IDLE_COLOR, TFT_BLACK);
    tft.drawString("Waiting for data...", 120, 90, 2);

    Wire.begin(I2C_ADDR, SDA_PIN, SCL_PIN, 100000);    // I2C slave
    Wire.onReceive(onI2CReceive);

    // BLE: one "notify" channel the phone subscribes to. The UUIDs name it.
    BLEDevice::init("PCSD_GRP1_TTGO");                  // name shown when scanning
    BLEServer* server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());
    BLEService* service = server->createService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
    ble = service->createCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8",
                                        BLECharacteristic::PROPERTY_NOTIFY);
    ble->addDescriptor(new BLE2902());
    service->start();
    BLEDevice::startAdvertising();
}

void loop()
{
    if (!lineReady) return;                             // nothing new

    char msg[64];                                       // copy out the message
    taskENTER_CRITICAL(&mux);
    strncpy(msg, lineBuf, sizeof(msg) - 1);
    msg[sizeof(msg) - 1] = '\0';
    lineReady = false;
    taskEXIT_CRITICAL(&mux);

    String line = String(msg);
    int c1 = line.indexOf(',');                         // parse "STATE,dist,block"
    int c2 = line.indexOf(',', c1 + 1);
    if (c1 <= 0 || c2 <= c1) return;                    // need two commas

    String state = line.substring(0, c1);
    float  dist  = line.substring(c1 + 1, c2).toFloat();
    int    block = line.substring(c2 + 1).toInt();

    drawScreen(state, dist, block);
    if (phoneConnected && ble)                          // send to the phone too
    {
        ble->setValue(msg);
        ble->notify();
    }
}
