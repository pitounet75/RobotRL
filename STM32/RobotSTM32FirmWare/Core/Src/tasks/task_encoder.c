/**
 * @file task_encoder.c
 * @brief 500 Hz TIM2 AB quadrature read and publish.
 */

#include "tasks/tasks.h"

#include "app_config.h"
#include "app_samples.h"
#include "wheel_encoder_abz.h"

#include "FreeRTOS.h"
#include "task.h"

volatile uint32_t g_encoder_read_count;

void task_encoder(void *argument)
{
    (void)argument;

    uint32_t pub_seq = 0u;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(APP_ENCODER_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&wake, period);

        wheel_encoder_sample_t raw;
        if (!wheel_encoder_abz_sample(&raw)) {
            continue;
        }

        g_encoder_read_count++;

        app_encoder_sample_t out = {0};
        out.t_us = raw.t_us;
        out.seq = ++pub_seq;
        out.count = raw.count;
        out.pos_turns = raw.pos_turns;
        out.vel_turns_s = raw.vel_turns_s;
        out.valid = raw.valid;
        app_samples_encoder_publish(&out);
    }
}
