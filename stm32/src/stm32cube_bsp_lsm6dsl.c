// Low-level driver for the accelerometer chip itself (the LSM6DSL).
// It reads and writes the sensor's registers over I2C.
//
// PlatformIO only compiles .c files that sit in this src/ folder, but this
// driver ships inside the STM32Cube framework. This one line pulls it into the
// build. The file it points to is found through the include path in
// platformio.ini (.../Drivers/BSP/Components/lsm6dsl).
#include "lsm6dsl.c"
