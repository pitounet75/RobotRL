/**
 * @file wheel_contact.c
 * @brief Antipatinage FSM, v_good lookback, torque overlays.
 */

#include "wheel_contact.h"

#include "app_config.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const float k_two_pi = (float)(2.0 * M_PI);
static const float k_gear_motor_over_wheel = (float)APP_WHEEL_GEAR_MOTOR / (float)APP_WHEEL_GEAR_WHEEL;

#define WC_RING_CAP 128u

typedef struct {
    float omega_l;
    float omega_r;
    uint32_t t_ms;
    uint8_t contact_ok;
} wc_ring_sample_t;

static wheel_contact_output_t s_out;
static wheel_contact_mode_t s_mode;
static bool s_lift_l;
static bool s_lift_r;

static float s_v_ma_l;
static float s_v_ma_r;
static float s_v_good_l;
static float s_v_good_r;
static float s_x_m_frozen;
static float s_pitch_trim_frozen;

static wc_ring_sample_t s_ring[WC_RING_CAP];
static uint16_t s_ring_head;
static uint16_t s_ring_count;
static uint32_t s_time_ms;

static float s_on_l_ms;
static float s_on_r_ms;
static float s_on_both_ms;
static float s_off_l_ms;
static float s_off_r_ms;
static float s_off_both_ms;
static float s_recover_ms;

static bool s_reanchor_pending;

volatile float g_wc_eta_l;
volatile float g_wc_eta_r;
volatile float g_wc_e_psi;
volatile uint8_t g_wc_pitch_mismatch;
volatile uint8_t g_wc_mode;
volatile float g_wc_v_good_l;
volatile float g_wc_v_good_r;
volatile float g_wc_integrator_trust;
volatile float g_wc_recover_ramp;

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

static float maxf2(float a, float b)
{
    return (a > b) ? a : b;
}

static bool antipat_enabled(void)
{
#if APP_CTRL_ANTIPATINAGE_ENABLE
    return true;
#else
    return false;
#endif
}

static bool sync_unilateral_enabled(void)
{
#if APP_ANTIPAT_SYNC_ENABLE
    return true;
#else
    return false;
#endif
}

static float ms_to_alpha(float dt_s, float tau_ms)
{
    if (tau_ms <= 1.0f) {
        return 0.0f;
    }
    return clampf((dt_s * 1000.0f) / tau_ms, 0.0f, 1.0f);
}

static void ring_push(float omega_l, float omega_r, uint8_t contact_ok)
{
    s_ring[s_ring_head].omega_l = omega_l;
    s_ring[s_ring_head].omega_r = omega_r;
    s_ring[s_ring_head].t_ms = s_time_ms;
    s_ring[s_ring_head].contact_ok = contact_ok;
    s_ring_head = (uint16_t)((s_ring_head + 1u) % WC_RING_CAP);
    if (s_ring_count < WC_RING_CAP) {
        s_ring_count++;
    }
}

static void ring_mean_omega(float t_end_ms, float t_start_ms, float *out_l, float *out_r)
{
    float sum_l = 0.0f;
    float sum_r = 0.0f;
    uint16_t n = 0u;

    for (uint16_t i = 0u; i < s_ring_count; i++) {
        const uint16_t idx = (uint16_t)((s_ring_head + WC_RING_CAP - 1u - i) % WC_RING_CAP);
        const wc_ring_sample_t *s = &s_ring[idx];
        if (s->t_ms > t_end_ms) {
            continue;
        }
        if (s->t_ms < t_start_ms) {
            break;
        }
        if (!s->contact_ok) {
            continue;
        }
        sum_l += s->omega_l;
        sum_r += s->omega_r;
        n++;
    }

    if (n > 0u) {
        *out_l = sum_l / (float)n;
        *out_r = sum_r / (float)n;
    } else {
        *out_l = s_v_ma_l;
        *out_r = s_v_ma_r;
    }
}

static void snapshot_v_good(void)
{
    const float t_excl_ms = 2.0f * (float)APP_ANTIPAT_T_ON_BOTH_MS;
    const float t_ma_ms = (float)APP_ANTIPAT_T_MA_MS;
    const float t_end = (float)s_time_ms - t_excl_ms;
    const float t_start = t_end - t_ma_ms;

    if (t_end <= t_start) {
        s_v_good_l = s_v_ma_l;
        s_v_good_r = s_v_ma_r;
        return;
    }

    ring_mean_omega(t_end, t_start, &s_v_good_l, &s_v_good_r);
}

static float compute_eta(float alpha_rads2, float tau_nm)
{
    const float denom = maxf2(fabsf(tau_nm), APP_ANTIPAT_TAU_MIN_NM);
    return fabsf(alpha_rads2) / denom;
}

static bool compute_decorrelated(float omega_l, float omega_r, float yaw_rate_rads, float *e_psi_out)
{
    if (APP_ANTIPAT_TRACK_WIDTH_M <= 1.0e-4f) {
        *e_psi_out = 0.0f;
        return false;
    }

    const float r = APP_WHEEL_RADIUS_M;
    const float v_l = omega_l * k_gear_motor_over_wheel * k_two_pi * r;
    const float v_r = omega_r * k_gear_motor_over_wheel * k_two_pi * r;
    const float psi_kin = (v_r - v_l) / APP_ANTIPAT_TRACK_WIDTH_M;
    const float e_psi = fabsf(yaw_rate_rads - psi_kin);
    const float thr = maxf2(APP_ANTIPAT_EPS_ABS_RADS, APP_ANTIPAT_K_REL * fabsf(yaw_rate_rads));

    *e_psi_out = e_psi;
    return e_psi > thr;
}

static bool compute_pitch_mismatch(float u, float pitch_rad, float pitch_rate_rads, float pitch_ref_rad)
{
    if (fabsf(u) <= APP_ANTIPAT_U_MIN_NM) {
        return false;
    }
    if (fabsf(pitch_rate_rads) <= APP_ANTIPAT_PITCH_RATE_MIN_RADS) {
        return false;
    }
    const float e_theta = pitch_ref_rad - pitch_rad;
    return (e_theta * pitch_rate_rads) < 0.0f;
}

static void enter_lift_common(const wheel_contact_input_t *in)
{
    s_x_m_frozen = in->x_m;
    s_pitch_trim_frozen = in->pitch_trim_rad;
}

static void compute_tau_both(const wheel_contact_input_t *in, float *tau_l, float *tau_r)
{
    float tl = APP_ANTIPAT_K_BOTH_V * (s_v_good_l - in->vel_motor_l_turns_s);
    float tr = APP_ANTIPAT_K_BOTH_V * (s_v_good_r - in->vel_motor_r_turns_s);
    *tau_l = clampf(tl, -APP_ANTIPAT_TAU_BOTH_MAX_NM, APP_ANTIPAT_TAU_BOTH_MAX_NM);
    *tau_r = clampf(tr, -APP_ANTIPAT_TAU_BOTH_MAX_NM, APP_ANTIPAT_TAU_BOTH_MAX_NM);
}

static bool both_air_still_flying(float omega_l, float omega_r, float tau_both_l, float tau_both_r)
{
    return (fabsf(tau_both_l) < APP_ANTIPAT_TAU_BOTH_STEADY_AIR_NM) &&
           (fabsf(tau_both_r) < APP_ANTIPAT_TAU_BOTH_STEADY_AIR_NM) &&
           (fabsf(omega_l) > APP_ANTIPAT_OMEGA_AIR_MIN_TURNS_S ||
            fabsf(omega_r) > APP_ANTIPAT_OMEGA_AIR_MIN_TURNS_S);
}

static bool both_air_contact_ok(float eta_l, float eta_r, float alpha_l, float alpha_r,
                                float tau_both_l, float tau_both_r,
                                float omega_l, float omega_r)
{
    if (both_air_still_flying(omega_l, omega_r, tau_both_l, tau_both_r)) {
        return false;
    }
    if (eta_l < APP_ANTIPAT_ETA_OFF && eta_r < APP_ANTIPAT_ETA_OFF) {
        return true;
    }
    if (APP_ANTIPAT_ALPHA_CONTACT_MAX_RADS2 > 0.0f) {
        return (fabsf(alpha_l) < APP_ANTIPAT_ALPHA_CONTACT_MAX_RADS2) &&
               (fabsf(alpha_r) < APP_ANTIPAT_ALPHA_CONTACT_MAX_RADS2);
    }
    return false;
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
    s_v_ma_l = 0.0f;
    s_v_ma_r = 0.0f;
    s_v_good_l = 0.0f;
    s_v_good_r = 0.0f;
    s_x_m_frozen = 0.0f;
    s_pitch_trim_frozen = 0.0f;
    s_ring_head = 0u;
    s_ring_count = 0u;
    s_time_ms = 0u;
    s_on_l_ms = 0.0f;
    s_on_r_ms = 0.0f;
    s_on_both_ms = 0.0f;
    s_off_l_ms = 0.0f;
    s_off_r_ms = 0.0f;
    s_off_both_ms = 0.0f;
    s_recover_ms = 0.0f;
    s_reanchor_pending = false;
    s_out.mode = WC_MODE_NORMAL;
    s_out.integrator_trust = true;
    s_out.recover_ramp = 1.0f;
    g_wc_mode = 0u;
    g_wc_integrator_trust = 1.0f;
    g_wc_recover_ramp = 1.0f;
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

    const float dt_ms = in->dt_s * 1000.0f;
    s_time_ms += (uint32_t)dt_ms;

    if (!antipat_enabled()) {
        s_out.mode = WC_MODE_NORMAL;
        s_out.integrator_trust = true;
        s_out.recover_ramp = 1.0f;
        s_out.lift_l = false;
        s_out.lift_r = false;
        s_out.u_cmd_nm = in->u;
        s_out.u_yaw_cmd_nm = -1.0f;
        s_out.tau_sync_l_nm = 0.0f;
        s_out.tau_sync_r_nm = 0.0f;
        s_out.tau_both_l_nm = 0.0f;
        s_out.tau_both_r_nm = 0.0f;
        s_out.both_active = false;
        s_out.reanchor_pos = false;
        s_out.reset_vel_integrators = false;
        s_out.reset_u_lpf = false;
        *out = s_out;
        return;
    }

    const float eta_l = compute_eta(in->alpha_l_rads2, in->tau_l_pre_nm);
    const float eta_r = compute_eta(in->alpha_r_rads2, in->tau_r_pre_nm);
    g_wc_eta_l = eta_l;
    g_wc_eta_r = eta_r;

    float e_psi = 0.0f;
    const bool decorrelated = compute_decorrelated(in->vel_motor_l_turns_s,
                                                   in->vel_motor_r_turns_s,
                                                   in->yaw_rate_rads,
                                                   &e_psi);
    g_wc_e_psi = e_psi;

    const bool pitch_mismatch = compute_pitch_mismatch(in->u,
                                                       in->pitch_rad,
                                                       in->pitch_rate_rads,
                                                       in->pitch_ref_rad);
    g_wc_pitch_mismatch = pitch_mismatch ? 1u : 0u;

    const float eta_r_floor = maxf2(eta_r, 1.0f);
    const float eta_l_floor = maxf2(eta_l, 1.0f);
    const bool cand_l_raw = decorrelated && (eta_l > APP_ANTIPAT_ETA_ON) &&
                            (eta_l > APP_ANTIPAT_K_DOM * eta_r_floor);
    const bool cand_r_raw = decorrelated && (eta_r > APP_ANTIPAT_ETA_ON) &&
                            (eta_r > APP_ANTIPAT_K_DOM * eta_l_floor);
    const bool cand_l = sync_unilateral_enabled() && cand_l_raw;
    const bool cand_r = sync_unilateral_enabled() && cand_r_raw;
    const bool unilateral = sync_unilateral_enabled() && (cand_l_raw || cand_r_raw);
    const bool both_cand = (eta_l > APP_ANTIPAT_ETA_ON) && (eta_r > APP_ANTIPAT_ETA_ON) &&
                           !unilateral && pitch_mismatch;
    const bool both_overlay = (s_mode == WC_MODE_BOTH_AIR) ||
                              (s_mode == WC_MODE_NORMAL && both_cand);

    float tau_both_l = 0.0f;
    float tau_both_r = 0.0f;
    if (s_mode == WC_MODE_BOTH_AIR) {
        compute_tau_both(in, &tau_both_l, &tau_both_r);
    }

    if (s_mode == WC_MODE_NORMAL && in->vel_motor_l_valid && in->vel_motor_r_valid) {
        const float a = ms_to_alpha(in->dt_s, APP_ANTIPAT_T_MA_MS);
        s_v_ma_l += a * (in->vel_motor_l_turns_s - s_v_ma_l);
        s_v_ma_r += a * (in->vel_motor_r_turns_s - s_v_ma_r);
        ring_push(in->vel_motor_l_turns_s, in->vel_motor_r_turns_s, 1u);
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
        if (both_cand && s_on_both_ms >= (float)APP_ANTIPAT_T_ON_BOTH_MS) {
            snapshot_v_good();
            enter_lift_common(in);
            s_mode = WC_MODE_BOTH_AIR;
        } else if (cand_l && s_on_l_ms >= (float)APP_ANTIPAT_T_ON_MS) {
            enter_lift_common(in);
            s_mode = WC_MODE_SYNC_L;
        } else if (cand_r && s_on_r_ms >= (float)APP_ANTIPAT_T_ON_MS) {
            enter_lift_common(in);
            s_mode = WC_MODE_SYNC_R;
        }
    } else if (s_mode == WC_MODE_SYNC_L) {
        if (both_cand) {
            snapshot_v_good();
            s_mode = WC_MODE_BOTH_AIR;
        } else if (eta_l < APP_ANTIPAT_ETA_OFF &&
                   (!decorrelated || e_psi < APP_ANTIPAT_EPS_ABS_RADS * APP_ANTIPAT_K_OFF)) {
            s_off_l_ms += dt_ms;
            if (s_off_l_ms >= (float)APP_ANTIPAT_T_OFF_MS) {
                begin_recovery();
            }
        } else {
            s_off_l_ms = 0.0f;
        }
    } else if (s_mode == WC_MODE_SYNC_R) {
        if (both_cand) {
            snapshot_v_good();
            s_mode = WC_MODE_BOTH_AIR;
        } else if (eta_r < APP_ANTIPAT_ETA_OFF &&
                   (!decorrelated || e_psi < APP_ANTIPAT_EPS_ABS_RADS * APP_ANTIPAT_K_OFF)) {
            s_off_r_ms += dt_ms;
            if (s_off_r_ms >= (float)APP_ANTIPAT_T_OFF_MS) {
                begin_recovery();
            }
        } else {
            s_off_r_ms = 0.0f;
        }
    } else if (s_mode == WC_MODE_BOTH_AIR) {
        /* Exit on η only: pitch_mismatch still uses balance u while u is cut → deadlock. */
        if (both_air_contact_ok(eta_l, eta_r, in->alpha_l_rads2, in->alpha_r_rads2,
                                tau_both_l, tau_both_r,
                                in->vel_motor_l_turns_s, in->vel_motor_r_turns_s)) {
            s_off_both_ms += dt_ms;
            if (s_off_both_ms >= (float)APP_ANTIPAT_T_OFF_BOTH_MS) {
                begin_recovery();
            }
        } else {
            s_off_both_ms = 0.0f;
        }
    } else if (s_mode == WC_MODE_RECOVERY) {
        s_recover_ms += dt_ms;
        if (s_recover_ms >= (float)APP_ANTIPAT_T_RECOVER_MS) {
            s_mode = WC_MODE_NORMAL;
            s_reanchor_pending = false;
        }
    }

    g_wc_v_good_l = s_v_good_l;
    g_wc_v_good_r = s_v_good_r;
    g_wc_mode = (uint8_t)s_mode;

    const bool recovering = (s_mode == WC_MODE_RECOVERY);
    const float recover_ramp = recovering ?
        clampf(s_recover_ms / (float)APP_ANTIPAT_T_RECOVER_MS, 0.0f, 1.0f) : 1.0f;
    const bool integrator_trust = (s_mode == WC_MODE_NORMAL) && !recovering;

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
        tau_sync_l = APP_ANTIPAT_K_SYNC * (in->vel_motor_r_turns_s - in->vel_motor_l_turns_s);
        tau_sync_l = clampf(tau_sync_l, -APP_ANTIPAT_TAU_SYNC_MAX_NM, APP_ANTIPAT_TAU_SYNC_MAX_NM);
    } else if (s_mode == WC_MODE_SYNC_R && in->vel_motor_l_valid && in->vel_motor_r_valid) {
        tau_sync_r = APP_ANTIPAT_K_SYNC * (in->vel_motor_l_turns_s - in->vel_motor_r_turns_s);
        tau_sync_r = clampf(tau_sync_r, -APP_ANTIPAT_TAU_SYNC_MAX_NM, APP_ANTIPAT_TAU_SYNC_MAX_NM);
    } else if (both_overlay) {
        if (s_mode == WC_MODE_BOTH_AIR && s_v_good_l == 0.0f && s_v_good_r == 0.0f) {
            snapshot_v_good();
        }
        if (both_cand && s_mode == WC_MODE_NORMAL) {
            snapshot_v_good();
        }
        if (s_mode != WC_MODE_BOTH_AIR) {
            compute_tau_both(in, &tau_both_l, &tau_both_r);
        }
    }

    bool reanchor = false;
    float pos_offset_new = 0.0f;
    bool reset_vel = false;
    bool reset_u_lpf = false;

    if (s_reanchor_pending && recovering && s_recover_ms <= dt_ms * 2.0f && in->pos_wheel_valid &&
        APP_WHEEL_RADIUS_M > 0.0f) {
        const float turns_frozen = s_x_m_frozen / (k_two_pi * APP_WHEEL_RADIUS_M);
        pos_offset_new = in->pos_wheel_turns - turns_frozen;
        reanchor = true;
        reset_vel = true;
        reset_u_lpf = true;
        s_reanchor_pending = false;
    }

    s_lift_l = (s_mode == WC_MODE_SYNC_L) || (s_mode == WC_MODE_BOTH_AIR);
    s_lift_r = (s_mode == WC_MODE_SYNC_R) || (s_mode == WC_MODE_BOTH_AIR);

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

    *out = s_out;
}
