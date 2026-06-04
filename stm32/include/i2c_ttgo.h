#pragma once

#include <stdbool.h>
#include <stdint.h>

bool TTGO_I2C_Init(void);
bool TTGO_SendLine(const char* text);     // forwards "STATE,distance,block" to the TTGO
uint32_t TTGO_GetLastError(void);
