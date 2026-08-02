/**

 * @file wheel_encoder_abz.c

 */



#include "wheel_encoder_abz.h"

#include "app_config.h"
#include "app_ctrl_params.h"

#include "app_time_us.h"

#include "app_wheel_config.h"

#include "tim.h"



typedef struct {

    TIM_HandleTypeDef *htim;

    uint32_t cpr;

    uint8_t timer_bits;

    uint32_t raw_mask;

    uint32_t last_raw;

    uint32_t last_t_us;

    int64_t count_accum;

    bool have_prev;

    float vel_lpf_turns_s;

    bool vel_lpf_valid;

} encoder_state_t;



static encoder_state_t s_encoder[WHEEL_ENCODER_COUNT] = {

    [WHEEL_ENCODER_TIM2] = {

        .htim = &htim2,

        .cpr = WHEEL_ENCODER_CPR,

        .timer_bits = 32u,

        .raw_mask = 0xFFFFFFFFu,

    },

    [WHEEL_ENCODER_TIM4] = {

        .htim = &htim4,

        .cpr = WHEEL_ENCODER_CPR,

        .timer_bits = 16u,

        .raw_mask = 0x0000FFFFu,

    },

};



volatile int64_t g_encoder_tim2_count;

volatile int64_t g_encoder_tim4_count;

volatile uint32_t g_encoder_tim2_raw_count;

volatile uint32_t g_encoder_tim4_raw_count;

volatile int32_t g_encoder_tim2_raw_delta_count;

volatile int32_t g_encoder_tim4_raw_delta_count;

volatile int32_t g_encoder_tim2_delta_count;

volatile int32_t g_encoder_tim4_delta_count;

volatile int32_t g_encoder_tim2_sign = 1;

volatile int32_t g_encoder_tim4_sign = 1;

volatile float g_encoder_tim2_vel_raw_turns_s;

volatile float g_encoder_tim4_vel_raw_turns_s;



static float filter_velocity(encoder_state_t *enc, float vel_raw_turns_s)
{
    const float alpha = app_ctrl_params_snapshot()->wheel_encoder_vel_lpf_alpha;
    if (alpha <= 0.0f) {
        return vel_raw_turns_s;
    }

    if (!enc->vel_lpf_valid) {
        enc->vel_lpf_turns_s = vel_raw_turns_s;
        enc->vel_lpf_valid = true;
        return vel_raw_turns_s;
    }

    enc->vel_lpf_turns_s = alpha * enc->vel_lpf_turns_s
                         + (1.0f - alpha) * vel_raw_turns_s;
    return enc->vel_lpf_turns_s;
}



static int32_t signed_delta(uint32_t raw, uint32_t last_raw, uint8_t timer_bits)

{

    const uint32_t diff = raw - last_raw;



    if (timer_bits == 16u) {

        return (int32_t)(int16_t)(uint16_t)diff;

    }



    return (int32_t)diff;

}



static void update_debug(wheel_encoder_id_t encoder, const wheel_encoder_sample_t *sample)

{

    if (encoder == WHEEL_ENCODER_TIM2) {

        g_encoder_tim2_count = sample->count;

        g_encoder_tim2_raw_count = sample->raw_count;

        g_encoder_tim2_raw_delta_count = sample->raw_delta_count;

        g_encoder_tim2_delta_count = sample->delta_count;

    } else if (encoder == WHEEL_ENCODER_TIM4) {

        g_encoder_tim4_count = sample->count;

        g_encoder_tim4_raw_count = sample->raw_count;

        g_encoder_tim4_raw_delta_count = sample->raw_delta_count;

        g_encoder_tim4_delta_count = sample->delta_count;

    }

}



void wheel_encoder_abz_init(void)

{

    (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);



    for (uint32_t i = 0u; i < WHEEL_ENCODER_COUNT; i++) {

        encoder_state_t *enc = &s_encoder[i];

        enc->last_raw = ((uint32_t)__HAL_TIM_GET_COUNTER(enc->htim)) & enc->raw_mask;

        enc->last_t_us = app_time_us_now();

        enc->count_accum = 0;

        enc->have_prev = false;

        enc->vel_lpf_turns_s = 0.0f;

        enc->vel_lpf_valid = false;

    }



    g_encoder_tim2_count = 0;

    g_encoder_tim4_count = 0;

    g_encoder_tim2_raw_count = s_encoder[WHEEL_ENCODER_TIM2].last_raw;

    g_encoder_tim4_raw_count = s_encoder[WHEEL_ENCODER_TIM4].last_raw;

    g_encoder_tim2_raw_delta_count = 0;

    g_encoder_tim4_raw_delta_count = 0;

    g_encoder_tim2_delta_count = 0;

    g_encoder_tim4_delta_count = 0;

    g_encoder_tim2_sign = app_wheel_encoder_sign(WHEEL_ENCODER_TIM2);

    g_encoder_tim4_sign = app_wheel_encoder_sign(WHEEL_ENCODER_TIM4);

}



bool wheel_encoder_abz_sample(wheel_encoder_sample_t *out)

{

    return wheel_encoder_abz_sample_encoder(WHEEL_ENCODER_TIM2, out);

}



bool wheel_encoder_abz_sample_encoder(wheel_encoder_id_t encoder, wheel_encoder_sample_t *out)

{

    if (out == NULL) {

        return false;

    }

    if ((uint32_t)encoder >= WHEEL_ENCODER_COUNT) {

        return false;

    }



    encoder_state_t *enc = &s_encoder[encoder];

    const uint32_t raw = ((uint32_t)__HAL_TIM_GET_COUNTER(enc->htim)) & enc->raw_mask;

    const uint32_t t_us = app_time_us_now();



    out->t_us = t_us;

    out->raw_count = raw;

    out->raw_delta_count = 0;

    out->delta_count = 0;

    out->timer_bits = enc->timer_bits;

    out->count = enc->count_accum;

    out->pos_turns = (float)enc->count_accum / (float)enc->cpr;

    out->valid = false;

    out->vel_turns_s = 0.0f;



    if (!enc->have_prev) {

        enc->last_raw = raw;

        enc->last_t_us = t_us;

        enc->have_prev = true;

        update_debug(encoder, out);

        return true;

    }



    const int32_t raw_dcount = signed_delta(raw, enc->last_raw, enc->timer_bits);

    const int32_t dcount = (int32_t)app_wheel_encoder_sign(encoder) * raw_dcount;

    const uint32_t dt_us = t_us - enc->last_t_us;

    enc->last_raw = raw;

    enc->last_t_us = t_us;

    enc->count_accum += (int64_t)dcount;



    out->raw_delta_count = raw_dcount;

    out->delta_count = dcount;

    out->count = enc->count_accum;

    out->pos_turns = (float)enc->count_accum / (float)enc->cpr;



    if (dt_us == 0u) {

        update_debug(encoder, out);

        return true;

    }



    const float dt_s = (float)dt_us * 1.0e-6f;

    const float vel_raw = ((float)dcount / (float)enc->cpr) / dt_s;

    if (encoder == WHEEL_ENCODER_TIM2) {

        g_encoder_tim2_vel_raw_turns_s = vel_raw;

    } else if (encoder == WHEEL_ENCODER_TIM4) {

        g_encoder_tim4_vel_raw_turns_s = vel_raw;

    }



    out->vel_turns_s = filter_velocity(enc, vel_raw);

    out->valid = true;

    update_debug(encoder, out);

    return true;

}



bool wheel_encoder_abz_sample_all(wheel_encoder_sample_t out[WHEEL_ENCODER_COUNT])

{

    if (out == NULL) {

        return false;

    }



    bool ok = true;

    for (uint32_t i = 0u; i < WHEEL_ENCODER_COUNT; i++) {

        ok = wheel_encoder_abz_sample_encoder((wheel_encoder_id_t)i, &out[i]) && ok;

    }

    return ok;

}

