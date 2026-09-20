/**
 * @file ctrl_motor_correction.h
 * @brief Per-wheel dτ = Kp·(α_ref − α_meas). α_meas is also an ABS input.
 */
#ifndef CTRL_MOTOR_CORRECTION_H
#define CTRL_MOTOR_CORRECTION_H

#include "control_strategy.h"

#include <stdbool.h>

typedef struct {
    float dtau_l_nm;
    float dtau_r_nm;
    float alpha_l_rads2;
    float alpha_r_rads2;
} ctrl_motor_correction_out_t;

void ctrl_motor_correction_reset(void);
void ctrl_motor_correction_step(const control_strategy_input_t *in, float u_nm, bool allow,
                                ctrl_motor_correction_out_t *out);

#endif /* CTRL_MOTOR_CORRECTION_H */
