// The simple accelerometer functions that main.cpp actually calls:
// BSP_ACCELERO_Init() and BSP_ACCELERO_AccGetXYZ(). They sit on top of the
// board layer (stm32l475e_iot01) and the chip driver (lsm6dsl) and hide the
// register details.
//
// Like the other two shim files, this one line pulls ST's driver source into
// the build so PlatformIO compiles it.
#include "stm32l475e_iot01_accelero.c"
