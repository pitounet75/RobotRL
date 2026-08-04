/**
 * @file task_telemetry.c
 * @brief Sole telemetry owner: protocol service @ 1 kHz, BalanceFrame @ 500 Hz.
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
    const TickType_t service_period = pdMS_TO_TICKS(1u);
    uint32_t publish_elapsed_ms = 0u;

#if APP_TELEMETRY_WAIT_BRIDGE_READY
    (void)app_telemetry_wait_bridge_ready(APP_TELEMETRY_BRIDGE_READY_TIMEOUT_MS);
#else
    vTaskDelay(pdMS_TO_TICKS(APP_TELEMETRY_STARTUP_DELAY_MS));
#endif
    wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&wake, service_period);
        app_telemetry_tick_1ms();
        app_telemetry_tx_poll();
        app_telemetry_poll_rx();

        publish_elapsed_ms++;
        if (publish_elapsed_ms >= APP_TELEMETRY_PERIOD_MS) {
            publish_elapsed_ms -= APP_TELEMETRY_PERIOD_MS;
            app_telemetry_publish_balance_frame();
        }
    }
}
