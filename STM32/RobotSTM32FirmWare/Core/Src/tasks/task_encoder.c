/**
 * @file task_encoder.c
 * @brief 500 Hz TIM2/TIM4 AB quadrature read and publish.
 */

#include "tasks/tasks.h"

#include "app_config.h"
#include "app_samples.h"
#include "wheel_encoder_abz.h"

#include "FreeRTOS.h"
#include "task.h"

volatile uint32_t g_encoder_read_count;
volatile uint32_t g_encoder_tim2_valid;
volatile uint32_t g_encoder_tim4_valid;

void task_encoder(void *argument)
{
    (void)argument;

    uint32_t pub_seq = 0u;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(APP_ENCODER_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&wake, period);

        wheel_encoder_sample_t raw[WHEEL_ENCODER_COUNT];
        if (!wheel_encoder_abz_sample_all(raw)) {
            continue;
        }

        g_encoder_read_count++;

        app_encoder_bank_sample_t bank = {0};
        bank.t_us = raw[WHEEL_ENCODER_TIM2].t_us;
        bank.seq = ++pub_seq;

        for (uint32_t i = 0u; i < WHEEL_ENCODER_COUNT && i < APP_LOCAL_ENCODER_COUNT; i++) {
            bank.encoder[i].t_us = raw[i].t_us;
            bank.encoder[i].seq = bank.seq;
            bank.encoder[i].count = raw[i].count;
            bank.encoder[i].raw_count = raw[i].raw_count;
            bank.encoder[i].raw_delta_count = raw[i].raw_delta_count;
            bank.encoder[i].delta_count = raw[i].delta_count;
            bank.encoder[i].timer_bits = raw[i].timer_bits;
            bank.encoder[i].pos_turns = raw[i].pos_turns;
            bank.encoder[i].vel_turns_s = raw[i].vel_turns_s;
            bank.encoder[i].valid = raw[i].valid;
        }

        g_encoder_tim2_valid = bank.encoder[WHEEL_ENCODER_TIM2].valid ? 1u : 0u;
        g_encoder_tim4_valid = bank.encoder[WHEEL_ENCODER_TIM4].valid ? 1u : 0u;

        app_samples_encoder_bank_publish(&bank);
        app_samples_encoder_publish(&bank.encoder[WHEEL_ENCODER_TIM2]);
    }
}
