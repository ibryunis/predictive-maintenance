/*
 * Predictive Maintenance - STM32 B-L475E-IOT01A - Group 1
 *
 * Samples the on-board LSM6DSL every 1 ms, sends a block of N samples to MATLAB
 * over USB serial (BEGIN_BUFFER / "x,y,z" lines / END_BUFFER), waits for one
 * reply line "STATE,distance,block", and forwards it to the TTGO over I2C.
 */

#include <cstdio>
#include <cstring>
#include <stdint.h>
#include "stm32l4xx_hal.h"

extern "C" {
#include "stm32l475e_iot01_accelero.h"
void SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
}

// --- settings ---
static const uint16_t N         = 128;          // samples per block (match MATLAB)
static const uint16_t TTGO_ADDR = 0x55 << 1;    // HAL wants the 8-bit address
static int16_t        block[N][3];               // x,y,z block buffer

static UART_HandleTypeDef uart = {};
static I2C_HandleTypeDef  i2c  = {};
static TIM_HandleTypeDef  tim  = {};
static volatile bool      tick = false;          // set by TIM7 every 1 ms

// --- clock: 4 MHz internal (MSI) ---
static void clockSetup()
{
    RCC_OscInitTypeDef osc = {};
    osc.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    osc.MSIState       = RCC_MSI_ON;
    osc.MSIClockRange  = RCC_MSIRANGE_6;         // 4 MHz
    osc.PLL.PLLState   = RCC_PLL_OFF;
    HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {};
    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}

// --- 1 ms timer (TIM7) ---
static void timerSetup()
{
    __HAL_RCC_TIM7_CLK_ENABLE();
    tim.Instance         = TIM7;
    tim.Init.Prescaler   = 2000 - 1;             // 4 MHz -> 2 kHz
    tim.Init.Period      = 2 - 1;                // 2 kHz -> 1 ms
    tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_Base_Init(&tim);
    HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
    HAL_TIM_Base_Start_IT(&tim);
}

extern "C" void SysTick_Handler() { HAL_IncTick(); }
extern "C" void TIM7_IRQHandler() { HAL_TIM_IRQHandler(&tim); }
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &tim) tick = true;               // keep the ISR tiny: just a flag
}

// Return true once per 1 ms tick, then clear the flag.
static bool takeTick()
{
    __disable_irq();
    bool t = tick; tick = false;
    __enable_irq();
    return t;
}

// --- accelerometer ---
static void accelSetup()
{
    BSP_ACCELERO_Init();
    // BSP defaults to 52 Hz; raise to 1.66 kHz (CTRL1_XL=0x80) so fan vibration
    // (~150 Hz blade-pass) is captured. Still +/-2 g.
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, 0x80);
}

// Fill one block, one sample per 1 ms tick.
static void fillBlock()
{
    while (takeTick()) { }                        // drop a stale tick
    for (uint16_t i = 0; i < N; i++)
    {
        while (!takeTick()) { }                   // wait for the next tick
        BSP_ACCELERO_AccGetXYZ(block[i]);
    }
}

// --- serial to MATLAB (USART1 = USB virtual COM, PB6/PB7) ---
static void serialSetup()
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {};
    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;     // PB6=TX, PB7=RX
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &gpio);

    uart.Instance        = USART1;
    uart.Init.BaudRate   = 115200;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits   = UART_STOPBITS_1;
    uart.Init.Parity     = UART_PARITY_NONE;
    uart.Init.Mode       = UART_MODE_TX_RX;
    HAL_UART_Init(&uart);
}

static void serialWrite(const char* text)
{
    HAL_UART_Transmit(&uart, (uint8_t*)text, strlen(text), 200);
}

// Read one line from MATLAB into buf (newline stripped).
static void serialReadLine(char* buf, uint16_t maxLen)
{
    uint16_t n = 0;
    while (1)
    {
        uint8_t c = 0;
        if (HAL_UART_Receive(&uart, &c, 1, 100) != HAL_OK)
        {
            __HAL_UART_CLEAR_OREFLAG(&uart);   // clear overrun so RX can't wedge
            continue;
        }
        if (c == '\n') { buf[n] = '\0'; return; }
        if (c == '\r') continue;
        if (n < maxLen - 1) buf[n++] = (char)c;
    }
}

// --- I2C to the TTGO (I2C1, PB8/PB9) ---
static void ttgoSetup()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {};
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;     // PB8=SCL, PB9=SDA
    gpio.Mode      = GPIO_MODE_AF_OD;             // I2C is open-drain
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    i2c.Instance            = I2C1;
    i2c.Init.Timing         = 0x00303D5B;         // ~100 kHz at 4 MHz
    i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    HAL_I2C_Init(&i2c);
}

// Send "STATE,distance,block\n" to the TTGO.
static void ttgoSend(const char* text)
{
    char frame[66];
    int len = snprintf(frame, sizeof(frame), "%s\n", text);
    HAL_I2C_Master_Transmit(&i2c, TTGO_ADDR, (uint8_t*)frame, len, 100);
}

int main()
{
    HAL_Init();
    clockSetup();
    serialSetup();
    ttgoSetup();
    accelSetup();
    timerSetup();
    ttgoSend("IDLE,0,0");                         // show an idle screen

    char line[32], reply[64];
    while (1)
    {
        fillBlock();

        serialWrite("BEGIN_BUFFER\r\n");           // send the block to MATLAB
        for (uint16_t i = 0; i < N; i++)
        {
            snprintf(line, sizeof(line), "%d,%d,%d\r\n",
                     block[i][0], block[i][1], block[i][2]);
            serialWrite(line);
        }
        serialWrite("END_BUFFER\r\n");

        serialReadLine(reply, sizeof(reply));      // MATLAB's reply
        ttgoSend(reply);                           // forward it to the TTGO
    }
}
