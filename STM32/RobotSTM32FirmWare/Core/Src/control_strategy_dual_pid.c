/**
 * @file control_strategy_dual_pid.c
 * @brief Parallel pitch PID + velocity PID, summed command (turn/s).
 */

#include "control_strategy.h"

#include "app_config.h"
#include "pid_controller.h"

#include <stddef.h>

static pid_controller_t s_pid_pitch;
static pid_controller_t s_pid_vel;

void control_strategy_dual_pid_reset(void)
{
    pid_init(&s_pid_pitch,
             APP_CTRL_PITCH_KP,
             APP_CTRL_PITCH_KI,
             APP_CTRL_PITCH_KD,
             -APP_CTRL_CMD_MAX_TURNS_S,
             APP_CTRL_CMD_MAX_TURNS_S);
    pid_init(&s_pid_vel,
             APP_CTRL_VEL_KP,
             APP_CTRL_VEL_KI,
             APP_CTRL_VEL_KD,
             -APP_CTRL_CMD_MAX_TURNS_S,
             APP_CTRL_CMD_MAX_TURNS_S);
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
    out->vel_left_turns_s = out->cmd;
    out->vel_right_turns_s = out->cmd;
}
