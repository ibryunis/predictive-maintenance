#include <cstdio>
#include <stdint.h>

#include "accelerometer_module.h"
#include "board_support.h"
#include "data_logger.h"
#include "i2c_ttgo.h"
#include "serial_vcp.h"
#include "stm32l4xx_hal.h"

namespace
{
constexpr uint16_t kSampleCount = 1024;
constexpr uint32_t kSamplePeriodMs = 1;

AccelerometerSample g_sample_buffer[kSampleCount] = {};
}

static void ReportError(SerialVCP& serial, const char* message);
static void ReportI2CError(SerialVCP& serial, uint32_t error);
static void ErrorLoop(SerialVCP& serial, const char* message);
static bool FillSampleBuffer(Accelerometer& accelerometer,
                             AccelerometerSample* samples,
                             uint16_t count);
static void WaitForMatlabLine(SerialVCP& serial, char* buffer, uint16_t max_len);

int main(void)
{
    Board_Setup();
    const bool ttgo_ready = TTGO_I2C_Init();

    SerialVCP pc_serial;

    if (!pc_serial.Init())
    {
        ErrorLoop(pc_serial, "ST-LINK VCP init failed");
    }

    if (!ttgo_ready)
    {
        ErrorLoop(pc_serial, "TTGO I2C init failed");
    }

    Accelerometer accelerometer;
    if (!accelerometer.Init())
    {
        ErrorLoop(pc_serial, accelerometer.GetLastError());
    }

    if (!SampleTimer_Start1ms())
    {
        ErrorLoop(pc_serial, "1 ms sample timer init failed");
    }

    DataLogger logger(pc_serial);

    // Boot ping: show an "idle" screen on the TTGO right away so we can
    // confirm the I2C link works before any data flows. If the screen stays
    // on its startup message, the wiring/power to the TTGO is the problem.
    TTGO_SendLine("IDLE,0,0");

    char matlab_line[64] = {};

    while (1)
    {
        if (!FillSampleBuffer(accelerometer, g_sample_buffer, kSampleCount))
        {
            ErrorLoop(pc_serial, accelerometer.GetLastError());
        }

        pc_serial.Write("BEGIN_BUFFER\r\n");
        logger.PrintHeader();
        logger.LogBuffer(0U, g_sample_buffer, kSampleCount, kSamplePeriodMs);
        pc_serial.Write("END_BUFFER\r\n");

        // MATLAB replies with one line: "STATE,distance,block" (e.g. "OK,12.5,16").
        // Receiving it both releases the next buffer and tells us what to show.
        WaitForMatlabLine(pc_serial, matlab_line, sizeof(matlab_line));

        if (!TTGO_SendLine(matlab_line))
        {
            ReportI2CError(pc_serial, TTGO_GetLastError());
        }
    }
}

static bool FillSampleBuffer(Accelerometer& accelerometer,
                             AccelerometerSample* samples,
                             uint16_t count)
{
    while (SampleTimer_TakeTick())
    {
    }

    for (uint16_t i = 0; i < count; ++i)
    {
        while (!SampleTimer_TakeTick())
        {
        }

        if (!accelerometer.Read(samples[i]))
        {
            return false;
        }
    }

    return true;
}

static void WaitForMatlabLine(SerialVCP& serial, char* buffer, uint16_t max_len)
{
    uint16_t n = 0;

    while (1)
    {
        uint8_t received = 0;
        if (!serial.ReadByte(received))
        {
            continue;
        }

        if (received == '\n')
        {
            buffer[n] = '\0';
            return;
        }

        if (received == '\r')
        {
            continue;
        }

        if (n < (max_len - 1U))
        {
            buffer[n++] = (char)received;
        }
    }
}

static void ReportI2CError(SerialVCP& serial, uint32_t error)
{
    char message[64] = {};
    std::snprintf(message,
                  sizeof(message),
                  "ERROR,TTGO I2C send failed,0x%08lX\r\n",
                  static_cast<unsigned long>(error));
    serial.Write(message);

    if ((error & HAL_I2C_ERROR_AF) != 0U)
    {
        serial.Write("ERROR,No ACK from TTGO; check address, wiring, and TTGO SDA/SCL pins\r\n");
    }
}

static void ReportError(SerialVCP& serial, const char* message)
{
    if (!serial.IsReady())
    {
        return;
    }

    serial.Write("ERROR,");
    serial.Write(message);
    serial.Write("\r\n");
}

static void ErrorLoop(SerialVCP& serial, const char* message)
{
    ReportError(serial, message);

    while (1)
    {
        HAL_Delay(100);
    }
}
