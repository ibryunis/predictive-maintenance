#pragma once

#include <stdint.h>

#include "accelerometer_module.h"

class SerialVCP;

class DataLogger
{
public:
    DataLogger(SerialVCP& serial);

    void PrintHeader();
    void LogSample(uint32_t time_ms, const AccelerometerSample& sample);
    void LogBuffer(uint32_t start_time_ms,
                   const AccelerometerSample* samples,
                   uint16_t count,
                   uint32_t sample_period_ms);

private:
    SerialVCP& serial_;
};
