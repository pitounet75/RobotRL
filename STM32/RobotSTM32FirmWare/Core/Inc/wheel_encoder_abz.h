/**
 * @file wheel_encoder_abz.h
 * @brief ABZ wheel encoders on hardware quadrature counters.
 *
 * CubeMX TIM2: 32-bit counter. TIM4: 16-bit counter.
 * Z index is not wired in CubeMX; add EXTI on the index GPIO if needed.
 */
#ifndef WHEEL_ENCODER_ABZ_H
#define WHEEL_ENCODER_ABZ_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    WHEEL_ENCODER_TIM2 = 0,
    WHEEL_ENCODER_TIM4 = 1,
    WHEEL_ENCODER_COUNT = 2,
} wheel_encoder_id_t;

typedef struct {
    uint32_t t_us;
    int64_t count;
    uint32_t raw_count;
    int32_t raw_delta_count;
    int32_t delta_count;
    uint8_t timer_bits;
    float pos_turns;
    float vel_turns_s;
    bool valid;
} wheel_encoder_sample_t;

void wheel_encoder_abz_init(void);
bool wheel_encoder_abz_sample(wheel_encoder_sample_t *out);
bool wheel_encoder_abz_sample_encoder(wheel_encoder_id_t encoder, wheel_encoder_sample_t *out);
bool wheel_encoder_abz_sample_all(wheel_encoder_sample_t out[WHEEL_ENCODER_COUNT]);

#endif /* WHEEL_ENCODER_ABZ_H */
