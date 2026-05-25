/**
 * @file wheel_encoder_abz.h
 * @brief ABZ wheel encoder on TIM2 (hardware quadrature counter).
 *
 * CubeMX TIM2: ENCODERMODE_TI12 (A+B quadrature on PA0/PA1).
 * Z index is not wired in CubeMX; add EXTI on the index GPIO if needed.
 */
#ifndef WHEEL_ENCODER_ABZ_H
#define WHEEL_ENCODER_ABZ_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t t_us;
    int32_t count;
    float pos_turns;
    float vel_turns_s;
    bool valid;
} wheel_encoder_sample_t;

void wheel_encoder_abz_init(void);
bool wheel_encoder_abz_sample(wheel_encoder_sample_t *out);

#endif /* WHEEL_ENCODER_ABZ_H */
