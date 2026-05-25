/**
 * @file task_telemetry.c
 */

#include "tasks/tasks.h"

#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"

void task_telemetry(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(APP_TELEMETRY_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&wake, period);
        /* TODO: copy snapshots to USART1 ESP32 telemetry frame */
    }
}
