#pragma once

#include "stm32l4xx_hal.h"

class SerialVCP
{
public:
    bool Init();
    bool IsReady() const;
    bool ReadByte(uint8_t& value);
    void Write(const char* text);

private:
    UART_HandleTypeDef uart_ = {};
    bool ready_ = false;
};
