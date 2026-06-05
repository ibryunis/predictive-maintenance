/*
 * PSCD Predictive Maintenance - STM32 B-L475E-IOT01A
 * Group 1 - 2026
 *
 * What it does:
 *   - samples the on-board LSM6DSL accelerometer every 1 ms (TIM7)
 *   - sends a block of N samples to the laptop over USB serial, framed as
 *       BEGIN_BUFFER / "x,y,z" lines / END_BUFFER
 *   - waits for one reply line from MATLAB: "STATE,distance,block"
 *   - forwards that line to the TTGO display over I2C
 *
 * Everything lives in this one file, split into clearly labelled sections.
 */

#include <cstdio>
#include <cstring>
#include <stdint.h>

#include "stm32l4xx_hal.h"

extern "C" {
#include "stm32l475e_iot01_accelero.h"
void SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
}

// ====================== Configuration ======================
static const uint16_t N         = 128;          // samples per block (must match MATLAB)
static const uint16_t TTGO_ADDR = 0x55 << 1;    // HAL uses the 8-bit address
static int16_t        g_block[N][3];             // x,y,z block buffer

static UART_HandleTypeDef g_uart = {};
static I2C_HandleTypeDef  g_i2c  = {};
static TIM_HandleTypeDef  g_tim  = {};
static volatile bool      g_tick = false;        // set by TIM7 every 1 ms

// ====================== Clock + 1 ms timer ======================
static void Clock_Setup(void)
{
    RCC_OscInitTypeDef osc = {};
    RCC_ClkInitTypeDef clk = {};

    osc.OscillatorType   = RCC_OSCILLATORTYPE_MSI;   // 4 MHz internal clock
    osc.MSIState         = RCC_MSI_ON;
    osc.MSIClockRange    = RCC_MSIRANGE_6;           // 4 MHz
    osc.PLL.PLLState     = RCC_PLL_OFF;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}

static void Timer_Start1ms(void)
{
    __HAL_RCC_TIM7_CLK_ENABLE();

    g_tim.Instance         = TIM7;
    g_tim.Init.Prescaler   = 2000 - 1;   // 4 MHz -> 2 kHz timer clock
    g_tim.Init.Period      = 2 - 1;      // 2 kHz -> 1 ms period
    g_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_Base_Init(&g_tim);

    HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
    HAL_TIM_Base_Start_IT(&g_tim);
}

// Interrupt handlers (C linkage so the vector table finds them)
extern "C" void SysTick_Handler(void)  { HAL_IncTick(); }
extern "C" void TIM7_IRQHandler(void)  { HAL_TIM_IRQHandler(&g_tim); }
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &g_tim) { g_tick = true; }   // keep the ISR tiny: just a flag
}

// Return true once per 1 ms tick (clears the flag).
static bool TakeTick(void)
{
    __disable_irq();
    bool happened = g_tick;
    g_tick = false;
    __enable_irq();
    return happened;
}

// ====================== Accelerometer ======================
static void Accel_Setup(void)
{
    BSP_ACCELERO_Init();
    // BSP defaults to 52 Hz; raise to 1.66 kHz (CTRL1_XL = 0x80, still +/-2g)
    // so fan blade-pass (~150 Hz) and similar vibration are captured.
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, 0x80);
}

// Fill one block, one sample per 1 ms tick.
static void Accel_FillBlock(void)
{
    while (TakeTick()) { }                 // drop any stale tick

    for (uint16_t i = 0; i < N; i++)
    {
        while (!TakeTick()) { }            // wait for the next 1 ms tick
        BSP_ACCELERO_AccGetXYZ(g_block[i]);
    }
}

// ====================== Serial (USB VCP, USART1) ======================
static void Serial_Setup(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;   // PB6=TX, PB7=RX
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &gpio);

    g_uart.Instance        = USART1;
    g_uart.Init.BaudRate   = 115200;
    g_uart.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart.Init.StopBits   = UART_STOPBITS_1;
    g_uart.Init.Parity     = UART_PARITY_NONE;
    g_uart.Init.Mode       = UART_MODE_TX_RX;
    HAL_UART_Init(&g_uart);
}

static void Serial_Write(const char* text)
{
    HAL_UART_Transmit(&g_uart, (uint8_t*)text, strlen(text), 200);
}

// Read one line from MATLAB into buf (without the newline).
static void Serial_ReadLine(char* buf, uint16_t max_len)
{
    uint16_t n = 0;
    while (1)
    {
        uint8_t c = 0;
        if (HAL_UART_Receive(&g_uart, &c, 1, 100) != HAL_OK) { continue; }
        if (c == '\n') { buf[n] = '\0'; return; }
        if (c == '\r') { continue; }
        if (n < max_len - 1) { buf[n++] = (char)c; }
    }
}

// ====================== I2C to the TTGO ======================
static void Ttgo_Setup(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;   // PB8=SCL, PB9=SDA
    gpio.Mode      = GPIO_MODE_AF_OD;           // I2C is open-drain
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    g_i2c.Instance            = I2C1;
    g_i2c.Init.Timing         = 0x00303D5B;     // ~100 kHz at a 4 MHz clock
    g_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    HAL_I2C_Init(&g_i2c);
}

// Send "STATE,distance,block" (plus a newline) to the TTGO.
static void Ttgo_SendLine(const char* text)
{
    char frame[66];                             // text + newline
    int len = snprintf(frame, sizeof(frame), "%s\n", text);
    HAL_I2C_Master_Transmit(&g_i2c, TTGO_ADDR, (uint8_t*)frame, len, 100);
}

// ====================== Main ======================
int main(void)
{
    HAL_Init();
    Clock_Setup();
    Serial_Setup();
    Ttgo_Setup();
    Accel_Setup();
    Timer_Start1ms();

    Ttgo_SendLine("IDLE,0,0");          // show an idle screen so we know I2C works

    char line[32];                      // one "x,y,z" line to send
    char reply[64];                     // MATLAB's reply line

    while (1)
    {
        Accel_FillBlock();

        // Send the block to MATLAB
        Serial_Write("BEGIN_BUFFER\r\n");
        for (uint16_t i = 0; i < N; i++)
        {
            snprintf(line, sizeof(line), "%d,%d,%d\r\n",
                     g_block[i][0], g_block[i][1], g_block[i][2]);
            Serial_Write(line);
        }
        Serial_Write("END_BUFFER\r\n");

        // Wait for MATLAB's reply, then forward it to the TTGO
        Serial_ReadLine(reply, sizeof(reply));
        Ttgo_SendLine(reply);
    }
}
