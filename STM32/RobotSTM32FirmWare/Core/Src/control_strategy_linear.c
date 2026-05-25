/**
 * @file control_strategy_linear.c
 * @brief F407-inspired linear state feedback on pitch, pitch_rate, wheel velocity.
 */

#include "control_strategy.h"

#include "app_config.h"

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

void control_strategy_linear_update(const control_strategy_input_t *in, control_strategy_output_t *out)
{
    if (in == NULL || out == NULL) {
        return;
    }

    const float theta_err = in->pitch_ref_rad - in->pitch_rad;

    /* Same structure as RobotBalanceRange doBalanceImpl2 (signs tunable via gains). */
    const float u_raw = APP_CTRL_LINEAR_K_PITCH * theta_err
                      - APP_CTRL_LINEAR_K_PITCH_RATE * in->pitch_rate_rads
                      + APP_CTRL_LINEAR_K_VEL * (in->vel_ref_turns_s - in->vel_wheel_turns_s);

    const float alpha = APP_CTRL_LINEAR_OUTPUT_ALPHA;
    const float u = alpha * s_u_prev + (1.0f - alpha) * u_raw;
    s_u_prev = u;

    const float cmd = clampf(u, -APP_CTRL_CMD_MAX_TURNS_S, APP_CTRL_CMD_MAX_TURNS_S);

    out->u_balance = APP_CTRL_LINEAR_K_PITCH * theta_err
                   - APP_CTRL_LINEAR_K_PITCH_RATE * in->pitch_rate_rads;
    out->u_vel = APP_CTRL_LINEAR_K_VEL * (in->vel_ref_turns_s - in->vel_wheel_turns_s);
    out->cmd = cmd;
    out->ok = true;
    out->estop = false;
    out->vel_left_turns_s = cmd;
    out->vel_right_turns_s = cmd;
}
