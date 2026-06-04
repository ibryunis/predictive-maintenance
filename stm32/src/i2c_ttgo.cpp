#include "i2c_ttgo.h"

#include "stm32l4xx_hal.h"

namespace
{
I2C_HandleTypeDef g_ttgo_i2c = {};
uint32_t g_ttgo_last_error = HAL_I2C_ERROR_NONE;
constexpr uint16_t kTtgoAddress = 0x55U << 1U; // STM32 HAL expects the 8-bit shifted address.
}

bool TTGO_I2C_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;          // PB8=SCL, PB9=SDA
    gpio.Mode = GPIO_MODE_AF_OD;                 // I2C must be open-drain
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;

    HAL_GPIO_Init(GPIOB, &gpio);

    g_ttgo_i2c.Instance = I2C1;
    g_ttgo_i2c.Init.Timing = 0x00303D5B;         // 100 kHz-ish for 4 MHz clock
    g_ttgo_i2c.Init.OwnAddress1 = 0;
    g_ttgo_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_ttgo_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_ttgo_i2c.Init.OwnAddress2 = 0;
    g_ttgo_i2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    g_ttgo_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_ttgo_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&g_ttgo_i2c) != HAL_OK)
    {
        g_ttgo_last_error = HAL_I2C_GetError(&g_ttgo_i2c);
        return false;
    }

    HAL_I2CEx_ConfigAnalogFilter(&g_ttgo_i2c, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&g_ttgo_i2c, 0);
    g_ttgo_last_error = HAL_I2C_ERROR_NONE;

    return true;
}

bool TTGO_SendState(uint8_t state)
{
    // Single-byte transfer: most reliable for the ESP32 I2C slave.
    const HAL_StatusTypeDef result =
        HAL_I2C_Master_Transmit(&g_ttgo_i2c, kTtgoAddress, &state, 1, 100);

    if (result == HAL_OK)
    {
        g_ttgo_last_error = HAL_I2C_ERROR_NONE;
        return true;
    }

    g_ttgo_last_error = HAL_I2C_GetError(&g_ttgo_i2c);
    return false;
}

uint32_t TTGO_GetLastError(void)
{
    return g_ttgo_last_error;
}
