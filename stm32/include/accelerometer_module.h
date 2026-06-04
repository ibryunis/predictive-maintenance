#pragma once

#include <stdint.h>

struct AccelerometerSample
{
    int16_t x_mg;
    int16_t y_mg;
    int16_t z_mg;
};

class Accelerometer
{
public:
    bool Init();
    bool Read(AccelerometerSample& sample);
    const char* GetLastError() const;

private:
    bool ready_ = false;
    const char* last_error_ = "Accelerometer not initialized";
};
