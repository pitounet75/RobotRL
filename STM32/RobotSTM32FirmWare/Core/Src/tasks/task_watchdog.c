/**
 * @file task_watchdog.c
 */

#include "tasks/tasks.h"

#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"

void task_watchdog(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(APP_WATCHDOG_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&wake, period);
        /* TODO: missed deadlines, estop, clamp motors */
    }
}
