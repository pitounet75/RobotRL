/**
 * @file task_telemetry.c
 * @brief UART4 BalanceFrame push @ APP_TELEMETRY_PERIOD_MS (default 500 Hz).
 */

#include "tasks/tasks.h"

#include "app_config.h"
#include "app_telemetry.h"

#include "FreeRTOS.h"
#include "task.h"

void task_telemetry(void *argument)
{
    (void)argument;

    if (!app_telemetry_init()) {
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(APP_TELEMETRY_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&wake, period);
        app_telemetry_tx_poll();
        app_telemetry_poll_rx();
        app_telemetry_publish_balance_frame();
    }
}
