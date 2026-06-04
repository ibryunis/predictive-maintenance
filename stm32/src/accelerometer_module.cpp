#include "accelerometer_module.h"

extern "C" {
#include "stm32l475e_iot01_accelero.h"
// Low-level sensor register write, implemented by the board BSP.
void SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
}

bool Accelerometer::Init()
{
    if (BSP_ACCELERO_Init() != ACCELERO_OK)
    {
        ready_ = false;
        last_error_ = "Accelerometer init failed";
        return false;
    }

    // The BSP configures the LSM6DSL at only 52 Hz, which limits vibration
    // sensing to ~26 Hz. Raise the output data rate to 1.66 kHz so higher-
    // frequency vibration (fan blade-pass ~150 Hz, phone motor ~175 Hz) is
    // captured. CTRL1_XL = 0x80 -> ODR_XL = 1.66 kHz, FS_XL = +/-2g (unchanged,
    // so the BSP's mg scaling stays valid).
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, 0x80);

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
