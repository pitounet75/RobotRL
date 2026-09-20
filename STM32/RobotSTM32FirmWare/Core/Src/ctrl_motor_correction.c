/**
 * @file ctrl_motor_correction.c
 * @brief dτ = Kp·(α_ref − α_meas) per wheel. Off when kp<=0.
 */

#include "ctrl_motor_correction.h"

#include "app_ctrl_params.h"
#include "ctrl_math.h"

#include <math.h>

static const float k_omega_sign_eps_turns = 0.05f;

typedef struct {
    float vel_prev_turns;
    float alpha_rads2;
    uint32_t last_update_ms;
    bool have_prev;
} alpha_est_t;

static alpha_est_t s_alpha_l;
static alpha_est_t s_alpha_r;

static void alpha_est_reset(alpha_est_t *e)
{
    e->vel_prev_turns = 0.0f;
    e->alpha_rads2 = 0.0f;
    e->last_update_ms = 0u;
    e->have_prev = false;
}

static float alpha_est_update(alpha_est_t *e, bool valid, float vel_turns_s, uint32_t update_ms, float lpf)
{
    if (!valid) {
        return e->alpha_rads2;
    }

    if (!e->have_prev || update_ms != e->last_update_ms) {
        if (e->have_prev) {
            uint32_t dt_ms = update_ms - e->last_update_ms;
            if (dt_ms > 0u && dt_ms < 200u) {
                const float dt_s = (float)dt_ms * 1.0e-3f;
                const float a_turns = (vel_turns_s - e->vel_prev_turns) / dt_s;
                const float a_rad = a_turns * ctrl_two_pi();
                const float a_clamped = ctrl_clampf(lpf, 0.0f, 0.999f);
                e->alpha_rads2 = a_clamped * e->alpha_rads2 + (1.0f - a_clamped) * a_rad;
            }
        }
        e->vel_prev_turns = vel_turns_s;
        e->last_update_ms = update_ms;
        e->have_prev = true;
    }
    return e->alpha_rads2;
}

static float friction_sign(float omega_turns_s, float u_nm)
{
    if (omega_turns_s > k_omega_sign_eps_turns) {
        return 1.0f;
    }
    if (omega_turns_s < -k_omega_sign_eps_turns) {
        return -1.0f;
    }
    if (u_nm > 1.0e-6f) {
        return 1.0f;
    }
    if (u_nm < -1.0e-6f) {
        return -1.0f;
    }
    return 0.0f;
}

static float alpha_torque_correction(float u_nm, float omega_turns_s, float alpha_meas_rads2,
                                     const app_ctrl_params_snapshot_t *p, float pitch_rad,
                                     float pitch_rate_rads)
{
    if (p->motor_torque_correction_kp <= 0.0f || p->motor_J <= 0.0f) {
        return 0.0f;
    }
    if (fabsf(pitch_rad) > p->motor_torque_correction_gate_pitch_max_rad || fabsf(pitch_rate_rads) > p->motor_torque_correction_gate_rate_max_rads) {
        return 0.0f;
    }
    if (p->motor_torque_correction_gate_vel_max_turns_s > 0.0f && fabsf(omega_turns_s) > p->motor_torque_correction_gate_vel_max_turns_s) {
        return 0.0f;
    }

    const float sgn = friction_sign(omega_turns_s, u_nm);
    const float alpha_ref = (u_nm - p->motor_friction_c * sgn) / p->motor_J;
    float dtau = p->motor_torque_correction_kp * (alpha_ref - alpha_meas_rads2);
    return ctrl_clampf(dtau, -p->motor_torque_correction_max_nm, p->motor_torque_correction_max_nm);
}

void ctrl_motor_correction_reset(void)
{
    alpha_est_reset(&s_alpha_l);
    alpha_est_reset(&s_alpha_r);
}

void ctrl_motor_correction_step(const control_strategy_input_t *in, float u_nm, bool allow,
                                ctrl_motor_correction_out_t *out)
{
    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
    const float alpha_l = alpha_est_update(&s_alpha_l, in->vel_motor_l_valid, in->vel_motor_l_turns_s,
                                           in->vel_motor_l_update_ms, p->motor_accel_lpf);
    const float alpha_r = alpha_est_update(&s_alpha_r, in->vel_motor_r_valid, in->vel_motor_r_turns_s,
                                           in->vel_motor_r_update_ms, p->motor_accel_lpf);

    float dtau_l = 0.0f;
    float dtau_r = 0.0f;
    if (allow && in->vel_motor_l_valid) {
        dtau_l = alpha_torque_correction(u_nm, in->vel_motor_l_turns_s, alpha_l, p, in->pitch_rad,
                                         in->pitch_rate_rads);
    }
    if (allow && in->vel_motor_r_valid) {
        dtau_r = alpha_torque_correction(u_nm, in->vel_motor_r_turns_s, alpha_r, p, in->pitch_rad,
                                         in->pitch_rate_rads);
    }

    out->dtau_l_nm = dtau_l;
    out->dtau_r_nm = dtau_r;
    out->alpha_l_rads2 = alpha_l;
    out->alpha_r_rads2 = alpha_r;
}
