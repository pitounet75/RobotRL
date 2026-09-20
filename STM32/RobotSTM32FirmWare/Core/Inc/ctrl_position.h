/**
 * @file ctrl_position.h
 * @brief Outer loop: teleop v_ref or x → bounded v_ref.
 */
#ifndef CTRL_POSITION_H
#define CTRL_POSITION_H

#include "control_strategy.h"

#include <stdbool.h>

typedef struct {
    float vel_ref_turns_s;
    float x_m;
    float x_err_ema_m;
    bool pos_mode;
} ctrl_position_out_t;

void ctrl_position_reset(void);
void ctrl_position_step(const control_strategy_input_t *in, bool integrator_trust,
                        bool reanchor_pos, float pos_offset_turns_new, float x_m_frozen_m,
                        ctrl_position_out_t *out);

#endif /* CTRL_POSITION_H */
