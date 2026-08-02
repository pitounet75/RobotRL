/**
 * @file control_strategy_linear.c
 * @brief F407-inspired linear state feedback on pitch, pitch_rate, wheel velocity.
 */

#include "control_strategy.h"

#include "app_config.h"
#include "app_ctrl_params.h"

#include <math.h>

static float s_u_prev;

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

void control_strategy_linear_reset(void)
{
    s_u_prev = 0.0f;
}

static float linear_theta_term(float theta_err)
{
    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
    if (p->linear_theta_func == 1u) {
        return atanf(theta_err);
    }
    return theta_err;
}

void control_strategy_linear_update(const control_strategy_input_t *in, control_strategy_output_t *out)
{
    if (in == NULL || out == NULL) {
        return;
    }

    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
    const float theta_err = in->pitch_ref_rad - in->pitch_rad;
    const float theta_term = linear_theta_term(theta_err);

    const float u_raw = p->linear_k_pitch * theta_term
                      - p->linear_k_pitch_rate * in->pitch_rate_rads
                      + p->linear_k_vel * (in->vel_ref_turns_s - in->vel_wheel_turns_s);

    const float alpha = p->linear_output_alpha;
    const float u = alpha * s_u_prev + (1.0f - alpha) * u_raw;
    s_u_prev = u;

    const float cmd = clampf(u, -p->cmd_max_torque_nm, p->cmd_max_torque_nm);

    out->u_balance = p->linear_k_pitch * theta_term
                   - p->linear_k_pitch_rate * in->pitch_rate_rads;
    out->u_vel = p->linear_k_vel * (in->vel_ref_turns_s - in->vel_wheel_turns_s);
    out->cmd = cmd;
    out->ok = true;
    out->estop = false;
    out->torque_left_nm = cmd;
    out->torque_right_nm = cmd;
}
