/**
 * @file ctrl_velocity.h
 * @brief Cascade v → pitch: slew, leaky I, D on v̇, accel FF.
 */
#ifndef CTRL_VELOCITY_H
#define CTRL_VELOCITY_H

#include "control_strategy.h"

#include <stdbool.h>

typedef struct {
    float vel_ref_turns_s;
    float vel_ref_dot;
    float pitch_trim_rad;
} ctrl_velocity_out_t;

void ctrl_velocity_reset(void);
void ctrl_velocity_step(const control_strategy_input_t *in, float vel_ref_cmd,
                        bool integrator_trust, float recover_ramp, bool reset_integrators,
                        float vel_wheel_touch_turns_s, float pitch_trim_frozen_rad,
                        ctrl_velocity_out_t *out);

#endif /* CTRL_VELOCITY_H */
