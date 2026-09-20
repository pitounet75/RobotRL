/**
 * @file ctrl_balance.c
 * @brief Pitch → couple (u_meca + u_err + Kv), then output LPF.
 */

#include "ctrl_balance.h"

#include "app_config.h"
#include "app_ctrl_params.h"
#include "ctrl_math.h"

#include <math.h>

static float s_u_prev;

static float grav_ff_sin(float pitch_rad)
{
    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
    const float pitch = ctrl_clampf(pitch_rad, -p->pitch_failsafe_rad, p->pitch_failsafe_rad);
    return sinf(pitch);
}

void ctrl_balance_reset(void)
{
    s_u_prev = 0.0f;
}

void ctrl_balance_pd(const control_strategy_input_t *in, float pitch_trim_rad, float vel_ref_turns_s,
                     ctrl_balance_pd_out_t *out)
{
    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
    const float pitch_ref_eff = ctrl_clampf(in->pitch_ref_rad + pitch_trim_rad,
                                            -p->cascade_pitch_ref_max_rad,
                                            p->cascade_pitch_ref_max_rad);
    const float theta_err = pitch_ref_eff - in->pitch_rad;

    float u_vel = p->err_k_vel * (vel_ref_turns_s - in->vel_wheel_turns_s);
    u_vel = ctrl_clampf(u_vel, -APP_CTRL_ERR_K_VEL_MAX_NM, APP_CTRL_ERR_K_VEL_MAX_NM);

    const float u_meca = -p->meca_k_grav * grav_ff_sin(in->pitch_rad)
                       - p->meca_k_pitch_damp * in->pitch_rate_rads;
    const float u_err = p->err_k_pitch * theta_err + u_vel;

    out->u_meca_nm = u_meca;
    out->u_err_nm = u_err;
    out->u_vel_nm = u_vel;
    out->u_raw_nm = u_meca + u_err;
}

float ctrl_balance_lpf(float u_raw_nm, bool integrator_trust, bool reset_lpf, float u_lpf_seed_nm)
{
    if (reset_lpf) {
        s_u_prev = u_lpf_seed_nm;
    }
    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
    const float a = p->balance_output_alpha;
    float u = a * s_u_prev + (1.0f - a) * u_raw_nm;
    if (integrator_trust) {
        s_u_prev = u;
    }
    return u;
}
