/**
 * @file control_strategy_ff_cascade.c
 * @brief Orchestrator: position → velocity → balance → friction → motor
 *        correction → heading → ABS mix.
 */

#include "control_strategy.h"

#include "app_ctrl_params.h"
#include "ctrl_abs.h"
#include "ctrl_balance.h"
#include "ctrl_friction.h"
#include "ctrl_heading.h"
#include "ctrl_math.h"
#include "ctrl_motor_correction.h"
#include "ctrl_position.h"
#include "ctrl_velocity.h"
#include "wheel_contact.h"

#include <stddef.h>

static float s_tau_applied_l_nm;
static float s_tau_applied_r_nm;

void control_strategy_ff_cascade_reset(void)
{
    ctrl_position_reset();
    ctrl_velocity_reset();
    ctrl_balance_reset();
    ctrl_friction_reset();
    ctrl_motor_correction_reset();
    ctrl_heading_reset();
    wheel_contact_reset();
    s_tau_applied_l_nm = 0.0f;
    s_tau_applied_r_nm = 0.0f;
}

void control_strategy_ff_cascade_update(const control_strategy_input_t *in,
                                        control_strategy_output_t *out)
{
    if (in == NULL || out == NULL) {
        return;
    }

    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
    const wheel_contact_output_t *abs_prev = wheel_contact_last_output();

    ctrl_position_out_t pos;
    ctrl_position_step(in, abs_prev->integrator_trust, abs_prev->reanchor_pos,
                       abs_prev->pos_offset_turns_new, abs_prev->x_m_frozen_m, &pos);

    ctrl_velocity_out_t vel;
    ctrl_velocity_step(in, pos.vel_ref_turns_s, abs_prev->integrator_trust, abs_prev->recover_ramp,
                       abs_prev->reset_vel_integrators, abs_prev->vel_wheel_touch_turns_s,
                       abs_prev->pitch_trim_frozen_rad, &vel);

    ctrl_balance_pd_out_t bal;
    ctrl_balance_pd(in, vel.pitch_trim_rad, vel.vel_ref_turns_s, &bal);

    const float u_raw = bal.u_raw_nm
                      + ctrl_friction_comp(in->vel_wheel_turns_s, bal.u_raw_nm, in->pitch_rad,
                                           in->pitch_rate_rads, NULL);
    const float u = ctrl_balance_lpf(u_raw, abs_prev->integrator_trust, abs_prev->reset_u_lpf,
                                     abs_prev->u_lpf_seed_nm);

    const bool both_air_like = abs_prev->both_active || (abs_prev->mode == WC_MODE_BOTH_AIR);
    ctrl_motor_correction_out_t mc;
    ctrl_motor_correction_step(in, u, !both_air_like, &mc);

    const bool yaw_allowed = (abs_prev->mode == WC_MODE_NORMAL);
    const float u_yaw = ctrl_heading_step(in, yaw_allowed);

    wheel_contact_input_t wci = {
        .dt_s = in->dt_s,
        .u = u,
        .pitch_rad = in->pitch_rad,
        .pitch_rate_rads = in->pitch_rate_rads,
        .pitch_ref_rad = in->pitch_ref_rad,
        .pitch_trim_rad = vel.pitch_trim_rad,
        .vel_motor_l_turns_s = in->vel_motor_l_turns_s,
        .vel_motor_r_turns_s = in->vel_motor_r_turns_s,
        .vel_motor_l_valid = in->vel_motor_l_valid,
        .vel_motor_r_valid = in->vel_motor_r_valid,
        .alpha_l_rads2 = mc.alpha_l_rads2,
        .alpha_r_rads2 = mc.alpha_r_rads2,
        .yaw_rate_rads = in->yaw_rate_rads,
        .tau_l_applied_nm = s_tau_applied_l_nm,
        .tau_r_applied_nm = s_tau_applied_r_nm,
        .vel_wheel_turns_s = in->vel_wheel_turns_s,
        .pos_wheel_turns = in->pos_wheel_turns,
        .x_m = pos.x_m,
        .pos_wheel_valid = in->pos_wheel_valid,
        .vel_wheel_l_turns_s = in->vel_wheel_l_turns_s,
        .vel_wheel_r_turns_s = in->vel_wheel_r_turns_s,
        .acc_wheel_l_turns_s2 = in->acc_wheel_l_turns_s2,
        .acc_wheel_r_turns_s2 = in->acc_wheel_r_turns_s2,
        .wheel_lr_valid = in->wheel_lr_valid,
    };
    wheel_contact_output_t wco;
    wheel_contact_update(&wci, &wco);

    float cmd_l;
    float cmd_r;
    ctrl_abs_mix(&wco, mc.dtau_l_nm, mc.dtau_r_nm, u_yaw, &cmd_l, &cmd_r);
    cmd_l = ctrl_clampf(cmd_l, -p->cmd_max_torque_nm, p->cmd_max_torque_nm);
    cmd_r = ctrl_clampf(cmd_r, -p->cmd_max_torque_nm, p->cmd_max_torque_nm);
    s_tau_applied_l_nm = cmd_l;
    s_tau_applied_r_nm = cmd_r;

    out->u_vel = bal.u_vel_nm;
    out->u_meca = bal.u_meca_nm;
    out->u_err = bal.u_err_nm;
    out->u_balance = bal.u_meca_nm + bal.u_err_nm;
    out->cmd = 0.5f * (cmd_l + cmd_r);
    out->ok = true;
    out->estop = false;
    out->torque_left_nm = cmd_l;
    out->torque_right_nm = cmd_r;
}
