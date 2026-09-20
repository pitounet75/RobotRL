/**
 * @file antipat_common.c
 * @brief Shared antipatinage: η, FSM, u_scale, recovery. Calls sync and both.
 */

#include "antipat_common.h"

#include "antipat_both.h"
#include "antipat_sync.h"
#include "app_config.h"
#include "app_ctrl_params.h"
#include "ctrl_math.h"

#include <math.h>
#include <string.h>

static wheel_contact_output_t s_out;
static wheel_contact_mode_t s_mode;
static bool s_lift_l;
static bool s_lift_r;

static float s_x_m_frozen;
static float s_pitch_trim_frozen;
static uint32_t s_time_ms;

static float s_on_l_ms;
static float s_on_r_ms;
static float s_on_both_ms;
static float s_off_l_ms;
static float s_off_r_ms;
static float s_off_both_ms;
static float s_recover_ms;

static bool s_reanchor_pending;
static float s_tau_abs_ema_l;
static float s_tau_abs_ema_r;
static bool s_tau_ema_seeded;
static float s_u_scale_l;
static float s_u_scale_r;

volatile float g_wc_eta_l;
volatile float g_wc_eta_r;
volatile float g_wc_e_psi;
volatile uint8_t g_wc_pitch_mismatch;
volatile uint8_t g_wc_mode;
volatile float g_wc_v_good_l;
volatile float g_wc_v_good_r;
volatile float g_wc_integrator_trust;
volatile float g_wc_recover_ramp;

static float maxf2(float a, float b)
{
    return (a > b) ? a : b;
}

static bool antipat_enabled(const app_ctrl_params_snapshot_t *p)
{
    return p->antipat_enable > 0.5f;
}

static float slew_u_scale(float cur, float target, float dt_ms, float fade_ms)
{
    if (fade_ms <= 0.0f) {
        return target;
    }
    const float step = dt_ms / fade_ms;
    if (cur < target) {
        const float n = cur + step;
        return (n > target) ? target : n;
    }
    if (cur > target) {
        const float n = cur - step;
        return (n < target) ? target : n;
    }
    return cur;
}

static float smooth_tau_abs(float *state, float tau_nm, float alpha)
{
    const float x = fabsf(tau_nm);
    const float a = ctrl_clampf(alpha, 0.0f, 0.999f);
    if (!s_tau_ema_seeded) {
        *state = x;
        return x;
    }
    *state = a * (*state) + (1.0f - a) * x;
    return *state;
}

static float compute_eta(const app_ctrl_params_snapshot_t *p, float alpha_rads2, float tau_abs_filt)
{
    const float denom = maxf2(tau_abs_filt, p->antipat_tau_min_nm);
    return fabsf(alpha_rads2) / denom;
}

static void enter_lift_common(const wheel_contact_input_t *in)
{
    s_x_m_frozen = in->x_m;
    s_pitch_trim_frozen = in->pitch_trim_rad;
}

static void begin_recovery(void)
{
    s_mode = WC_MODE_RECOVERY;
    s_recover_ms = 0.0f;
    s_reanchor_pending = true;
    s_pitch_trim_frozen = 0.0f;
    s_on_l_ms = 0.0f;
    s_on_r_ms = 0.0f;
    s_on_both_ms = 0.0f;
}

void wheel_contact_init(void)
{
    wheel_contact_reset();
}

void wheel_contact_reset(void)
{
    memset(&s_out, 0, sizeof(s_out));
    s_mode = WC_MODE_NORMAL;
    s_lift_l = false;
    s_lift_r = false;
    s_x_m_frozen = 0.0f;
    s_pitch_trim_frozen = 0.0f;
    s_time_ms = 0u;
    s_on_l_ms = 0.0f;
    s_on_r_ms = 0.0f;
    s_on_both_ms = 0.0f;
    s_off_l_ms = 0.0f;
    s_off_r_ms = 0.0f;
    s_off_both_ms = 0.0f;
    s_recover_ms = 0.0f;
    s_reanchor_pending = false;
    s_tau_abs_ema_l = 0.0f;
    s_tau_abs_ema_r = 0.0f;
    s_tau_ema_seeded = false;
    s_u_scale_l = 1.0f;
    s_u_scale_r = 1.0f;
    s_out.mode = WC_MODE_NORMAL;
    s_out.integrator_trust = true;
    s_out.recover_ramp = 1.0f;
    s_out.u_scale_l = 1.0f;
    s_out.u_scale_r = 1.0f;
    g_wc_mode = 0u;
    g_wc_integrator_trust = 1.0f;
    g_wc_recover_ramp = 1.0f;
    antipat_both_reset();
}

const wheel_contact_output_t *wheel_contact_last_output(void)
{
    return &s_out;
}

void wheel_contact_update(const wheel_contact_input_t *in, wheel_contact_output_t *out)
{
    if (in == NULL || out == NULL) {
        return;
    }

    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
    const float dt_ms = in->dt_s * 1000.0f;
    s_time_ms += (uint32_t)dt_ms;

    if (!antipat_enabled(p)) {
        if (s_mode != WC_MODE_NORMAL) {
            wheel_contact_reset();
        }
        s_out.u_cmd_nm = in->u;
        s_out.u_yaw_cmd_nm = -1.0f;
        s_out.integrator_trust = true;
        s_out.recover_ramp = 1.0f;
        *out = s_out;
        return;
    }

    float tau_l_abs;
    float tau_r_abs;
    if (s_lift_l) {
        tau_l_abs = s_tau_abs_ema_l;
    } else {
        tau_l_abs = smooth_tau_abs(&s_tau_abs_ema_l, in->tau_l_applied_nm, p->antipat_tau_ema);
    }
    if (s_lift_r) {
        tau_r_abs = s_tau_abs_ema_r;
    } else {
        tau_r_abs = smooth_tau_abs(&s_tau_abs_ema_r, in->tau_r_applied_nm, p->antipat_tau_ema);
    }
    s_tau_ema_seeded = true;
    const float u_abs = fabsf(in->u);
    tau_l_abs = maxf2(tau_l_abs, u_abs);
    tau_r_abs = maxf2(tau_r_abs, u_abs);
    const float eta_l = compute_eta(p, in->alpha_l_rads2, tau_l_abs);
    const float eta_r = compute_eta(p, in->alpha_r_rads2, tau_r_abs);
    g_wc_eta_l = eta_l;
    g_wc_eta_r = eta_r;

    float e_psi = 0.0f;
    const bool decorrelated = antipat_sync_decorrelated(p, in->vel_motor_l_turns_s,
                                                        in->vel_motor_r_turns_s,
                                                        in->yaw_rate_rads,
                                                        &e_psi);
    g_wc_e_psi = e_psi;

    const bool pitch_mismatch = antipat_both_pitch_mismatch(p, in->u, in->pitch_rad,
                                                            in->pitch_rate_rads,
                                                            in->pitch_ref_rad);
    g_wc_pitch_mismatch = pitch_mismatch ? 1u : 0u;

    const bool cand_l = antipat_sync_candidate(p, eta_l, eta_r, decorrelated);
    const bool cand_r = antipat_sync_candidate(p, eta_r, eta_l, decorrelated);
    const bool unilateral = cand_l || cand_r;
    const bool both_cand = antipat_both_candidate(p, eta_l, eta_r, unilateral, pitch_mismatch);
    const bool both_overlay = (s_mode == WC_MODE_BOTH_AIR) ||
                              (s_mode == WC_MODE_NORMAL && both_cand);

    float tau_both_l = 0.0f;
    float tau_both_r = 0.0f;
    if (s_mode == WC_MODE_BOTH_AIR) {
        antipat_both_compute_tau(p, in->vel_motor_l_turns_s, in->vel_motor_r_turns_s,
                                 &tau_both_l, &tau_both_r);
    }

    if (s_mode == WC_MODE_NORMAL && in->vel_motor_l_valid && in->vel_motor_r_valid) {
        antipat_both_on_normal_sample(p, in->vel_motor_l_turns_s, in->vel_motor_r_turns_s,
                                      in->dt_s, s_time_ms);
    }

    if (s_mode == WC_MODE_NORMAL) {
        if (both_cand) {
            s_on_both_ms += dt_ms;
        } else {
            s_on_both_ms = 0.0f;
        }
        if (cand_l) {
            s_on_l_ms += dt_ms;
        } else {
            s_on_l_ms = 0.0f;
        }
        if (cand_r) {
            s_on_r_ms += dt_ms;
        } else {
            s_on_r_ms = 0.0f;
        }
    }

    if (s_mode == WC_MODE_NORMAL) {
        if (both_cand && s_on_both_ms >= p->antipat_both_t_on_ms) {
            antipat_both_snapshot_v_good(p, s_time_ms);
            enter_lift_common(in);
            s_mode = WC_MODE_BOTH_AIR;
        } else if (cand_l && s_on_l_ms >= p->antipat_sync_t_on_ms) {
            enter_lift_common(in);
            s_mode = WC_MODE_SYNC_L;
        } else if (cand_r && s_on_r_ms >= p->antipat_sync_t_on_ms) {
            enter_lift_common(in);
            s_mode = WC_MODE_SYNC_R;
        }
    } else if (s_mode == WC_MODE_SYNC_L) {
        if (both_cand) {
            antipat_both_snapshot_v_good(p, s_time_ms);
            s_mode = WC_MODE_BOTH_AIR;
        } else if (antipat_sync_recontact(p, e_psi, in->yaw_rate_rads,
                                          in->vel_motor_r_turns_s, in->vel_motor_l_turns_s)) {
            s_off_l_ms += dt_ms;
            if (s_off_l_ms >= p->antipat_sync_t_off_ms) {
                begin_recovery();
            }
        } else {
            s_off_l_ms = 0.0f;
        }
    } else if (s_mode == WC_MODE_SYNC_R) {
        if (both_cand) {
            antipat_both_snapshot_v_good(p, s_time_ms);
            s_mode = WC_MODE_BOTH_AIR;
        } else if (antipat_sync_recontact(p, e_psi, in->yaw_rate_rads,
                                          in->vel_motor_l_turns_s, in->vel_motor_r_turns_s)) {
            s_off_r_ms += dt_ms;
            if (s_off_r_ms >= p->antipat_sync_t_off_ms) {
                begin_recovery();
            }
        } else {
            s_off_r_ms = 0.0f;
        }
    } else if (s_mode == WC_MODE_BOTH_AIR) {
        if (!antipat_both_enabled(p)) {
            begin_recovery();
        } else if (antipat_both_contact_ok(p, eta_l, eta_r, in->alpha_l_rads2, in->alpha_r_rads2,
                                           tau_both_l, tau_both_r,
                                           in->vel_motor_l_turns_s, in->vel_motor_r_turns_s)) {
            s_off_both_ms += dt_ms;
            if (s_off_both_ms >= p->antipat_both_t_off_ms) {
                begin_recovery();
            }
        } else {
            s_off_both_ms = 0.0f;
        }
    } else if (s_mode == WC_MODE_RECOVERY) {
        s_recover_ms += dt_ms;
        if (s_recover_ms >= p->antipat_t_recover_ms) {
            s_mode = WC_MODE_NORMAL;
            s_reanchor_pending = false;
        }
    }

    g_wc_v_good_l = antipat_both_v_good_l();
    g_wc_v_good_r = antipat_both_v_good_r();
    g_wc_mode = (uint8_t)s_mode;

    const bool recovering = (s_mode == WC_MODE_RECOVERY);
    const float recover_ramp = recovering ?
        ctrl_clampf(s_recover_ms / maxf2(p->antipat_t_recover_ms, 1.0f), 0.0f, 1.0f) : 1.0f;
    const bool integrator_trust = (s_mode != WC_MODE_BOTH_AIR) && !recovering;

    g_wc_recover_ramp = recover_ramp;
    g_wc_integrator_trust = integrator_trust ? 1.0f : 0.0f;

    float u_cmd = in->u;
    if (both_overlay) {
        u_cmd = 0.0f;
    }

    float tau_sync_l = 0.0f;
    float tau_sync_r = 0.0f;
    if (!both_overlay) {
        tau_both_l = 0.0f;
        tau_both_r = 0.0f;
    }

    if (s_mode == WC_MODE_SYNC_L && in->vel_motor_l_valid && in->vel_motor_r_valid) {
        tau_sync_l = antipat_sync_tau(p, in->vel_motor_r_turns_s, in->vel_motor_l_turns_s,
                                      in->alpha_r_rads2, in->alpha_l_rads2);
    } else if (s_mode == WC_MODE_SYNC_R && in->vel_motor_l_valid && in->vel_motor_r_valid) {
        tau_sync_r = antipat_sync_tau(p, in->vel_motor_l_turns_s, in->vel_motor_r_turns_s,
                                      in->alpha_l_rads2, in->alpha_r_rads2);
    } else if (both_overlay) {
        if (s_mode == WC_MODE_BOTH_AIR && antipat_both_v_good_l() == 0.0f &&
            antipat_both_v_good_r() == 0.0f) {
            antipat_both_snapshot_v_good(p, s_time_ms);
        }
        if (both_cand && s_mode == WC_MODE_NORMAL) {
            antipat_both_snapshot_v_good(p, s_time_ms);
        }
        if (s_mode != WC_MODE_BOTH_AIR) {
            antipat_both_compute_tau(p, in->vel_motor_l_turns_s, in->vel_motor_r_turns_s,
                                     &tau_both_l, &tau_both_r);
        }
    }

    bool reanchor = false;
    float pos_offset_new = 0.0f;
    bool reset_vel = false;
    bool reset_u_lpf = false;

    if (s_reanchor_pending && recovering && s_recover_ms <= dt_ms * 2.0f && in->pos_wheel_valid &&
        p->wheel_radius_m > 0.0f) {
        const float turns_frozen = s_x_m_frozen / (ctrl_two_pi() * p->wheel_radius_m);
        pos_offset_new = in->pos_wheel_turns - turns_frozen;
        reanchor = true;
        reset_vel = true;
        reset_u_lpf = true;
        s_reanchor_pending = false;
    }

    s_lift_l = (s_mode == WC_MODE_SYNC_L) || (s_mode == WC_MODE_BOTH_AIR);
    s_lift_r = (s_mode == WC_MODE_SYNC_R) || (s_mode == WC_MODE_BOTH_AIR);

    s_u_scale_l = slew_u_scale(s_u_scale_l, s_lift_l ? 0.0f : 1.0f, dt_ms, p->antipat_u_fade_ms);
    s_u_scale_r = slew_u_scale(s_u_scale_r, s_lift_r ? 0.0f : 1.0f, dt_ms, p->antipat_u_fade_ms);

    s_out.mode = s_mode;
    s_out.integrator_trust = integrator_trust;
    s_out.recover_ramp = recover_ramp;
    s_out.lift_l = s_lift_l;
    s_out.lift_r = s_lift_r;
    s_out.pitch_trim_frozen_rad = s_pitch_trim_frozen;
    s_out.x_m_frozen_m = s_x_m_frozen;
    s_out.u_cmd_nm = u_cmd;
    s_out.u_yaw_cmd_nm = -1.0f;
    s_out.tau_sync_l_nm = tau_sync_l;
    s_out.tau_sync_r_nm = tau_sync_r;
    s_out.tau_both_l_nm = tau_both_l;
    s_out.tau_both_r_nm = tau_both_r;
    s_out.both_active = both_overlay;
    s_out.reanchor_pos = reanchor;
    s_out.pos_offset_turns_new = pos_offset_new;
    s_out.reset_vel_integrators = reset_vel;
    s_out.vel_wheel_touch_turns_s = in->vel_wheel_turns_s;
    s_out.reset_u_lpf = reset_u_lpf;
    s_out.u_lpf_seed_nm = u_cmd;
    s_out.u_scale_l = s_u_scale_l;
    s_out.u_scale_r = s_u_scale_r;

    *out = s_out;
}
