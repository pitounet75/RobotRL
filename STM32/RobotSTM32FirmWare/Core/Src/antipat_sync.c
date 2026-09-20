/**
 * @file antipat_sync.c
 * @brief Unilateral SYNC_L / SYNC_R. Leaf — no call into common or both.
 */

#include "antipat_sync.h"

#include "app_config.h"
#include "ctrl_math.h"

#include <math.h>

static const float k_gear_motor_over_wheel = (float)APP_WHEEL_GEAR_MOTOR / (float)APP_WHEEL_GEAR_WHEEL;

static float maxf2(float a, float b)
{
    return (a > b) ? a : b;
}

bool antipat_sync_enabled(const app_ctrl_params_snapshot_t *p)
{
    return p->antipat_sync_enable > 0.5f;
}

bool antipat_sync_decorrelated(const app_ctrl_params_snapshot_t *p,
                               float omega_l_turns_s, float omega_r_turns_s,
                               float yaw_rate_rads, float *e_psi_out)
{
    if (p->antipat_sync_track_width_m <= 1.0e-4f) {
        *e_psi_out = 0.0f;
        return false;
    }

    const float r = p->wheel_radius_m;
    const float v_l = omega_l_turns_s * k_gear_motor_over_wheel * ctrl_two_pi() * r;
    const float v_r = omega_r_turns_s * k_gear_motor_over_wheel * ctrl_two_pi() * r;
    const float psi_kin = (v_r - v_l) / p->antipat_sync_track_width_m;
    const float e_psi = fabsf(yaw_rate_rads - psi_kin);
    const float thr = maxf2(p->antipat_sync_eps_abs_rads,
                            p->antipat_sync_k_rel * fabsf(yaw_rate_rads));

    *e_psi_out = e_psi;
    return e_psi > thr;
}

bool antipat_sync_candidate(const app_ctrl_params_snapshot_t *p,
                            float eta_this, float eta_other, bool decorrelated)
{
    if (!antipat_sync_enabled(p)) {
        return false;
    }
    const float other_floor = maxf2(eta_other, 1.0f);
    return decorrelated && (eta_this > p->antipat_eta_on) &&
           (eta_this > p->antipat_sync_k_dom * other_floor);
}

static bool sync_still_flying(const app_ctrl_params_snapshot_t *p,
                              float omega_ground_turns_s, float omega_air_turns_s)
{
    return fabsf(omega_air_turns_s - omega_ground_turns_s) > p->antipat_omega_air_min_turns_s;
}

bool antipat_sync_recontact(const app_ctrl_params_snapshot_t *p,
                            float e_psi, float yaw_rate_rads,
                            float omega_ground_turns_s, float omega_air_turns_s)
{
    if (sync_still_flying(p, omega_ground_turns_s, omega_air_turns_s)) {
        return false;
    }
    const float thr = maxf2(p->antipat_sync_eps_abs_rads,
                            p->antipat_sync_k_rel * fabsf(yaw_rate_rads));
    const float k_off = ctrl_clampf(p->antipat_sync_k_off, 0.0f, 1.0f);
    return e_psi < (k_off * thr);
}

float antipat_sync_tau(const app_ctrl_params_snapshot_t *p,
                       float omega_ground_turns_s, float omega_air_turns_s,
                       float alpha_ground_rads2, float alpha_air_rads2)
{
    const float e = omega_ground_turns_s - omega_air_turns_s;
    const float e_dot = (alpha_ground_rads2 - alpha_air_rads2) / ctrl_two_pi();
    const float tau = p->antipat_sync_k * e + p->antipat_sync_kd * e_dot;
    return ctrl_clampf(tau, -p->antipat_sync_tau_max_nm, p->antipat_sync_tau_max_nm);
}
