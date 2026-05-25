/**
 * @file wheel_encoder_abz.c
 */

#include "wheel_encoder_abz.h"
#include "app_config.h"
#include "app_time_us.h"
#include "tim.h"

static int32_t s_last_count;
static uint32_t s_last_t_us;
static bool s_have_prev;

void wheel_encoder_abz_init(void)
{
    (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    s_last_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    s_last_t_us = app_time_us_now();
    s_have_prev = false;
}

bool wheel_encoder_abz_sample(wheel_encoder_sample_t *out)
{
    if (out == NULL) {
        return false;
    }

    const int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    const uint32_t t_us = app_time_us_now();

    out->t_us = t_us;
    out->count = count;
    out->pos_turns = (float)count / (float)WHEEL_ENCODER_CPR;
    out->valid = false;
    out->vel_turns_s = 0.0f;

    if (!s_have_prev) {
        s_last_count = count;
        s_last_t_us = t_us;
        s_have_prev = true;
        return true;
    }

    const int32_t dcount = count - s_last_count;
    const uint32_t dt_us = t_us - s_last_t_us;
    s_last_count = count;
    s_last_t_us = t_us;

    if (dt_us == 0u) {
        return true;
    }

    const float dt_s = (float)dt_us * 1.0e-6f;
    out->vel_turns_s = ((float)dcount / (float)WHEEL_ENCODER_CPR) / dt_s;
    out->valid = true;
    return true;
}
