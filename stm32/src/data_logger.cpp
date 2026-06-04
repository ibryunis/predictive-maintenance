#include "data_logger.h"

#include <cstdio>

#include "serial_vcp.h"

DataLogger::DataLogger(SerialVCP& serial)
    : serial_(serial)
{
}

void DataLogger::PrintHeader()
{
    serial_.Write("time_ms,x_mg,y_mg,z_mg\r\n");
}

void DataLogger::LogSample(uint32_t time_ms, const AccelerometerSample& sample)
{
    // Send all three axes (one line per sample) so MATLAB can analyse X, Y and Z.
    (void)time_ms;

    char line[32];
    std::snprintf(line, sizeof(line), "%d,%d,%d\r\n",
                  (int)sample.x_mg, (int)sample.y_mg, (int)sample.z_mg);
    serial_.Write(line);
}

void DataLogger::LogBuffer(uint32_t start_time_ms,
                           const AccelerometerSample* samples,
                           uint16_t count,
                           uint32_t sample_period_ms)
{
    for (uint16_t i = 0; i < count; i++)
    {
        const uint32_t time_ms = start_time_ms + ((uint32_t)i * sample_period_ms);
        LogSample(time_ms, samples[i]);
    }
}
