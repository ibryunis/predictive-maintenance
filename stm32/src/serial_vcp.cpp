#include "serial_vcp.h"

#include <cstring>

bool SerialVCP::Init()
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &gpio);

    uart_.Instance = USART1;
    uart_.Init.BaudRate = 115200;
    uart_.Init.WordLength = UART_WORDLENGTH_8B;
    uart_.Init.StopBits = UART_STOPBITS_1;
    uart_.Init.Parity = UART_PARITY_NONE;
    uart_.Init.Mode = UART_MODE_TX_RX;
    uart_.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_.Init.OverSampling = UART_OVERSAMPLING_16;

    ready_ = (HAL_UART_Init(&uart_) == HAL_OK);
    return ready_;
}

bool SerialVCP::IsReady() const
{
    return ready_;
}

bool SerialVCP::ReadByte(uint8_t& value)
{
    if (!ready_)
    {
        return false;
    }

    return HAL_UART_Receive(&uart_, &value, 1, 100) == HAL_OK;
}

void SerialVCP::Write(const char* text)
{
    if (!ready_ || (text == nullptr))
    {
        return;
    }

    HAL_UART_Transmit(&uart_, 
        (uint8_t*)text, 
        (uint16_t)std::strlen(text), 
        200);
}
