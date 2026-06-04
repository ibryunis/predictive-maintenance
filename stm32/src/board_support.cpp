#include "board_support.h"

#include "stm32l4xx_hal.h"

static TIM_HandleTypeDef g_sample_timer = {};
static volatile bool g_sample_timer_tick = false;

void Board_Setup(void)
{
    HAL_Init();
    Board_SystemClock_Config();
}

bool SampleTimer_Start1ms(void)
{
    __HAL_RCC_TIM7_CLK_ENABLE();

    g_sample_timer_tick = false;
    g_sample_timer.Instance = TIM7;
    g_sample_timer.Init.Prescaler = 2000U - 1U; // 2 kHz timer clock from 4 MHz MSI
    g_sample_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_sample_timer.Init.Period = 2U - 1U; // 1 ms update period; HAL does not support ARR = 0
    g_sample_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_sample_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&g_sample_timer) != HAL_OK)
    {
        return false;
    }

    HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    __HAL_TIM_CLEAR_FLAG(&g_sample_timer, TIM_FLAG_UPDATE);
    return HAL_TIM_Base_Start_IT(&g_sample_timer) == HAL_OK;
}

bool SampleTimer_TakeTick(void)
{
    const uint32_t old_primask = __get_PRIMASK();
    __disable_irq();

    const bool tick_happened = g_sample_timer_tick;
    g_sample_timer_tick = false;

    if (old_primask == 0U)
    {
        __enable_irq();
    }

    return tick_happened;
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

extern "C" void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_sample_timer);
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &g_sample_timer)
    {
        // Keep the interrupt simple: only set a flag.
        g_sample_timer_tick = true;
    }
}

void Board_SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_MSI; // 0x00000010U
    osc.MSIState = RCC_MSI_ON;
    osc.MSICalibrationValue = 0;
    osc.MSIClockRange = RCC_MSIRANGE_6; // pg 226
    osc.PLL.PLLState = RCC_PLL_OFF;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
        while (1) {}
    }

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK)
    {
        while (1) {}
    }
}
