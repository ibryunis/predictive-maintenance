#pragma once

#include <stdbool.h>
#include <stdint.h>

bool TTGO_I2C_Init(void);
bool TTGO_SendState(uint8_t state);       // 0=IDLE 1=CALIB 2=OK 3=ALARM
uint32_t TTGO_GetLastError(void);
