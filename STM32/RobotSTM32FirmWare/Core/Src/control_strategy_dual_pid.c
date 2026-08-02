/**
 * @file control_strategy_dual_pid.c
 * @brief Parallel pitch PID + velocity PID, summed command (turn/s).
 */

#include "control_strategy.h"

#include "app_config.h"
#include "app_ctrl_params.h"
#include "pid_controller.h"

#include <stddef.h>

static pid_controller_t s_pid_pitch;
static pid_controller_t s_pid_vel;

void control_strategy_dual_pid_reset(void)
{
    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
    pid_init(&s_pid_pitch,
             p->pitch_kp,
             p->pitch_ki,
             p->pitch_kd,
             -p->cmd_max_torque_nm,
             p->cmd_max_torque_nm);
    pid_init(&s_pid_vel,
             p->vel_kp,
             p->vel_ki,
             p->vel_kd,
             -p->cmd_max_torque_nm,
             p->cmd_max_torque_nm);
}

void control_strategy_dual_pid_update(const control_strategy_input_t *in, control_strategy_output_t *out)
{
    if (in == NULL || out == NULL) {
        return;
    }

    out->u_balance = pid_update(&s_pid_pitch,
                                in->pitch_ref_rad,
                                in->pitch_rad,
                                in->pitch_rate_rads,
                                in->dt_s);

    out->u_vel = pid_update(&s_pid_vel,
                            in->vel_ref_turns_s,
                            in->vel_wheel_turns_s,
                            0.0f,
                            in->dt_s);

    out->cmd = out->u_balance + out->u_vel;
    out->ok = true;
    out->estop = false;
    out->torque_left_nm = out->cmd;
    out->torque_right_nm = out->cmd;
}
