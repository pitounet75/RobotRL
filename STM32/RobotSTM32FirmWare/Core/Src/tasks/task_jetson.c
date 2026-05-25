/**
 * @file task_jetson.c
 */

#include "tasks/tasks.h"

#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"

void task_jetson(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(APP_JETSON_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&wake, period);
        /* TODO: USART2 command parser (pitch_ref, vel_ref) */
    }
}
