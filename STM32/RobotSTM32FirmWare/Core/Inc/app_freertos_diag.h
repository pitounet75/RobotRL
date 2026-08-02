/**
 * @file app_freertos_diag.h
 * @brief FreeRTOS assert hook and runtime freeze diagnostics.
 */
#ifndef APP_FREERTOS_DIAG_H
#define APP_FREERTOS_DIAG_H

#include <stdint.h>

extern volatile uint32_t g_cortex_cpuid;
extern volatile uint32_t g_freertos_assert_line;
extern volatile const char *g_freertos_assert_file;
extern volatile uint32_t g_freertos_assert_count;
extern volatile uint32_t g_default_task_loop_count;
extern volatile uint32_t g_default_task_dwt_cycles;
extern volatile uint32_t g_fdcan1_it1_count;
extern volatile uint32_t g_fdcan2_it1_count;

void app_freertos_diag_capture_cpuid(void);
void app_freertos_assert_failed(const char *file, int line);
void app_freertos_diag_default_task_heartbeat(void);

#endif /* APP_FREERTOS_DIAG_H */
