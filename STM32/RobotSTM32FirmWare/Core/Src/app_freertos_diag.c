/**
 * @file app_freertos_diag.c
 */

#include "app_freertos_diag.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stm32h7xx.h"

volatile uint32_t g_cortex_cpuid;
volatile uint32_t g_freertos_assert_line;
volatile const char *g_freertos_assert_file;
volatile uint32_t g_freertos_assert_count;
volatile uint32_t g_default_task_loop_count;
volatile uint32_t g_default_task_dwt_cycles;
volatile uint32_t g_fdcan1_it1_count;
volatile uint32_t g_fdcan2_it1_count;

void app_freertos_diag_capture_cpuid(void)
{
    g_cortex_cpuid = *(volatile uint32_t *)0xE000ED00u;
}

void app_freertos_assert_failed(const char *file, int line)
{
    g_freertos_assert_file = file;
    g_freertos_assert_line = (uint32_t)line;
    g_freertos_assert_count++;
    taskDISABLE_INTERRUPTS();
    for (;;) {
        __NOP();
    }
}

void app_freertos_diag_default_task_heartbeat(void)
{
    g_default_task_loop_count++;

    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0u) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u) {
        DWT->CYCCNT = 0u;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    g_default_task_dwt_cycles = DWT->CYCCNT;
}
