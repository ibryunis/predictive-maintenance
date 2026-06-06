// Board-support layer for the B-L475E-IOT01A board. It knows how this exact
// board is wired - which I2C bus the sensor sits on - and provides the
// SENSOR_IO_* helpers that the accelerometer driver uses to talk to the chip.
//
// Same idea as the other two shim files: the real driver lives in the
// STM32Cube framework, and this one line pulls it into the build (path set in
// platformio.ini, .../Drivers/BSP/B-L475E-IOT01).
#include "stm32l475e_iot01.c"
