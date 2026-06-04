#pragma once

#include <stdbool.h>
#include <stdint.h>

void Board_Setup(void);
void Board_SystemClock_Config(void);

bool SampleTimer_Start1ms(void);
bool SampleTimer_TakeTick(void);
