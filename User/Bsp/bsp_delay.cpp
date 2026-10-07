#include "bsp_delay.hpp"
#include "stm32f4xx_hal.h"
#include "main.h"

namespace
{
    uint32_t s_cycles_per_us = 0U;
    bool s_delay_ready = false;
}

extern "C" void delay_init(void)
{
    s_delay_ready = false;

    SystemCoreClockUpdate();

    s_cycles_per_us = SystemCoreClock / 1000000U;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    __DSB();
    __ISB();

    const uint32_t before = DWT->CYCCNT;

    for (volatile uint32_t i = 0U; i < 32U; ++i)
    {
        __NOP();
    }

    s_delay_ready =
        s_cycles_per_us != 0U &&
        DWT->CYCCNT != before;
}

extern "C" bool delay_is_ready(void)
{
    return s_delay_ready &&
           (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U &&
           (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U;
}

extern "C" void delay_us(uint16_t nus)
{
    if (nus == 0U)
    {
        return;
    }

    if (!delay_is_ready())
    {
        /* Fail explicitly instead of silently skipping a delay. */
        Error_Handler();
        return;
    }

    const uint32_t ticks =
        static_cast<uint32_t>(nus) * s_cycles_per_us;

    const uint32_t start = DWT->CYCCNT;

    while (static_cast<uint32_t>(DWT->CYCCNT - start) < ticks)
    {
        __NOP();
    }
}

extern "C" void delay_ms(uint16_t nms)
{
    while (nms > 0U)
    {
        delay_us(1000U);
        --nms;
    }
}