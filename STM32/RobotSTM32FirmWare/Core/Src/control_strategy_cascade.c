/**
 * @file control_strategy_cascade.c
 * @brief Velocity loop sets pitch_ref; single pitch PID drives wheels.
 */

#include "control_strategy.h"

#include "app_config.h"
#include "app_ctrl_params.h"
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
    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
    pid_init(&s_pid_pitch,
             p->pitch_kp,
             p->pitch_ki,
             p->pitch_kd,
             -p->cmd_max_torque_nm,
             p->cmd_max_torque_nm);
    pid_init(&s_pid_vel,
             p->cascade_vel_kp,
             p->cascade_vel_ki,
             p->cascade_vel_kd,
             -p->cascade_pitch_ref_max_rad,
             p->cascade_pitch_ref_max_rad);
}

void control_strategy_cascade_update(const control_strategy_input_t *in, control_strategy_output_t *out)
{
    if (in == NULL || out == NULL) {
        return;
    }

    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();

    const float pitch_trim = -pid_update(&s_pid_vel,
                                         in->vel_ref_turns_s,
                                         in->vel_wheel_turns_s,
                                         0.0f,
                                         in->dt_s);

    const float pitch_ref = clampf(in->pitch_ref_rad + pitch_trim,
                                   -p->cascade_pitch_ref_max_rad,
                                   p->cascade_pitch_ref_max_rad);

    out->u_vel = pitch_trim;
    out->u_balance = pid_update(&s_pid_pitch,
                                pitch_ref,
                                in->pitch_rad,
                                in->pitch_rate_rads,
                                in->dt_s);
    out->cmd = out->u_balance;
    out->ok = true;
    out->estop = false;
    out->torque_left_nm = out->cmd;
    out->torque_right_nm = out->cmd;
}
