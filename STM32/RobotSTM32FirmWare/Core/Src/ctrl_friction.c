/**
 * @file ctrl_friction.c
 * @brief Gated Coulomb / two-level friction added to u_raw.
 */

#include "ctrl_friction.h"

#include "app_ctrl_params.h"

#include <math.h>

volatile float g_ctrl_friction_comp_nm;
volatile uint8_t g_ctrl_friction_regime;

static bool friction_gates_ok(const app_ctrl_params_snapshot_t *p, float pitch_rad, float pitch_rate_rads)
{
    if (p->torque_deadband_rate_max_rads > 0.0f && fabsf(pitch_rate_rads) > p->torque_deadband_rate_max_rads) {
        return false;
    }
    if (p->torque_deadband_pitch_max_rad > 0.0f && fabsf(pitch_rad) > p->torque_deadband_pitch_max_rad) {
        return false;
    }
    if (p->torque_deadband_pitch_max_rad <= 0.0f && fabsf(pitch_rad) > 0.0f) {
        return false;
    }
    return true;
}

static float friction_comp_nm(float omega_turns_s, float u_nm, const app_ctrl_params_snapshot_t *p,
                              uint8_t *regime_out)
{
    *regime_out = 0;
    if (p->friction_mode >= 0.5f) {
        const float eps = p->friction_vel_eps_turns_s;
        if (fabsf(omega_turns_s) <= eps) {
            if (fabsf(u_nm) < 1.0e-5f) {
                return 0.0f;
            }
            *regime_out = 1;
            return copysignf(p->friction_static_nm, u_nm);
        }
        if (p->friction_kinetic_nm > 0.0f) {
            *regime_out = 2;
            return copysignf(p->friction_kinetic_nm, omega_turns_s);
        }
        return 0.0f;
    }
    if (p->torque_deadband_nm > 0.0f && fabsf(u_nm) > 1.0e-5f) {
        *regime_out = 1;
        return copysignf(p->torque_deadband_nm, u_nm);
    }
    return 0.0f;
}

void ctrl_friction_reset(void)
{
    g_ctrl_friction_comp_nm = 0.0f;
    g_ctrl_friction_regime = 0;
}

float ctrl_friction_comp(float omega_turns_s, float u_raw_nm, float pitch_rad, float pitch_rate_rads,
                         uint8_t *regime_out)
{
    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
    uint8_t regime = 0;
    float comp = 0.0f;
    if (friction_gates_ok(p, pitch_rad, pitch_rate_rads)) {
        comp = friction_comp_nm(omega_turns_s, u_raw_nm, p, &regime);
    }
    g_ctrl_friction_comp_nm = comp;
    g_ctrl_friction_regime = regime;
    if (regime_out != NULL) {
        *regime_out = regime;
    }
    return comp;
}
