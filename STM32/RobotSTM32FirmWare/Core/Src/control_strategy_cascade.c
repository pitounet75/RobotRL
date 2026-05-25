/**
 * @file control_strategy_cascade.c
 * @brief Velocity loop sets pitch_ref; single pitch PID drives wheels.
 */

#include "control_strategy.h"

#include "app_config.h"
#include "pid_controller.h"

#include <math.h>

static pid_controller_t s_pid_pitch;
static pid_controller_t s_pid_vel;

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

void control_strategy_cascade_reset(void)
{
    pid_init(&s_pid_pitch,
             APP_CTRL_PITCH_KP,
             APP_CTRL_PITCH_KI,
             APP_CTRL_PITCH_KD,
             -APP_CTRL_CMD_MAX_TURNS_S,
             APP_CTRL_CMD_MAX_TURNS_S);
    pid_init(&s_pid_vel,
             APP_CTRL_CASCADE_VEL_KP,
             APP_CTRL_CASCADE_VEL_KI,
             APP_CTRL_CASCADE_VEL_KD,
             -APP_CTRL_CASCADE_PITCH_REF_MAX_RAD,
             APP_CTRL_CASCADE_PITCH_REF_MAX_RAD);
}

void control_strategy_cascade_update(const control_strategy_input_t *in, control_strategy_output_t *out)
{
    if (in == NULL || out == NULL) {
        return;
    }

    const float pitch_trim = pid_update(&s_pid_vel,
                                        in->vel_ref_turns_s,
                                        in->vel_wheel_turns_s,
                                        0.0f,
                                        in->dt_s);

    const float pitch_ref = clampf(in->pitch_ref_rad + pitch_trim,
                                   -APP_CTRL_CASCADE_PITCH_REF_MAX_RAD,
                                   APP_CTRL_CASCADE_PITCH_REF_MAX_RAD);

    out->u_vel = pitch_trim;
    out->u_balance = pid_update(&s_pid_pitch,
                                pitch_ref,
                                in->pitch_rad,
                                in->pitch_rate_rads,
                                in->dt_s);
    out->cmd = out->u_balance;
    out->ok = true;
    out->estop = false;
    out->vel_left_turns_s = out->cmd;
    out->vel_right_turns_s = out->cmd;
}
