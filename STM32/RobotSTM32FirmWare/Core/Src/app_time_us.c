/**
 * @file app_time_us.c
 */

#include "app_time_us.h"
#include "stm32h7xx.h"

void app_time_us_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t app_time_us_now(void)
{
    const uint32_t cycles_per_us = SystemCoreClock / 1000000u;
    if (cycles_per_us == 0u) {
        return 0u;
    }
    return DWT->CYCCNT / cycles_per_us;
}
