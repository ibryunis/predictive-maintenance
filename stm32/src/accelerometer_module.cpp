#include "accelerometer_module.h"

extern "C" {
#include "stm32l475e_iot01_accelero.h"
}

bool Accelerometer::Init()
{
    if (BSP_ACCELERO_Init() != ACCELERO_OK)
    {
        ready_ = false;
        last_error_ = "Accelerometer init failed";
        return false;
    }

    ready_ = true;
    last_error_ = "OK";
    return true;
}

bool Accelerometer::Read(AccelerometerSample& sample)
{
    if (!ready_)
    {
        last_error_ = "Accelerometer not initialized";
        return false;
    }

    int16_t xyz[3] = {0, 0, 0};
    BSP_ACCELERO_AccGetXYZ(xyz);

    sample.x_mg = xyz[0];
    sample.y_mg = xyz[1];
    sample.z_mg = xyz[2];

    last_error_ = "OK";
    return true;
}

const char* Accelerometer::GetLastError() const
{
    return last_error_;
}
