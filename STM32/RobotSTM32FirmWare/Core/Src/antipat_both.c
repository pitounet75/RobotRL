/**
 * @file antipat_both.c
 * @brief BOTH_AIR. Leaf — no call into common or sync.
 */

#include "antipat_both.h"

#include "ctrl_math.h"

#include <math.h>
#include <string.h>

#define BOTH_RING_CAP 128u

typedef struct {
    float omega_l;
    float omega_r;
    uint32_t t_ms;
    uint8_t contact_ok;
} both_ring_sample_t;

static float s_v_ma_l;
static float s_v_ma_r;
static float s_v_good_l;
static float s_v_good_r;
static both_ring_sample_t s_ring[BOTH_RING_CAP];
static uint16_t s_ring_head;
static uint16_t s_ring_count;

static float maxf2(float a, float b)
{
    return (a > b) ? a : b;
}

static float ms_to_alpha(float dt_s, float tau_ms)
{
    if (tau_ms <= 1.0f) {
        return 0.0f;
    }
    return ctrl_clampf((dt_s * 1000.0f) / tau_ms, 0.0f, 1.0f);
}

static void ring_push(float omega_l, float omega_r, uint32_t time_ms)
{
    s_ring[s_ring_head].omega_l = omega_l;
    s_ring[s_ring_head].omega_r = omega_r;
    s_ring[s_ring_head].t_ms = time_ms;
    s_ring[s_ring_head].contact_ok = 1u;
    s_ring_head = (uint16_t)((s_ring_head + 1u) % BOTH_RING_CAP);
    if (s_ring_count < BOTH_RING_CAP) {
        s_ring_count++;
    }
}

static void ring_mean_omega(float t_end_ms, float t_start_ms, float *out_l, float *out_r)
{
    float sum_l = 0.0f;
    float sum_r = 0.0f;
    uint16_t n = 0u;

    for (uint16_t i = 0u; i < s_ring_count; i++) {
        const uint16_t idx = (uint16_t)((s_ring_head + BOTH_RING_CAP - 1u - i) % BOTH_RING_CAP);
        const both_ring_sample_t *s = &s_ring[idx];
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

void antipat_both_reset(void)
{
    s_v_ma_l = 0.0f;
    s_v_ma_r = 0.0f;
    s_v_good_l = 0.0f;
    s_v_good_r = 0.0f;
    memset(s_ring, 0, sizeof(s_ring));
    s_ring_head = 0u;
    s_ring_count = 0u;
}

bool antipat_both_enabled(const app_ctrl_params_snapshot_t *p)
{
    return p->antipat_both_enable > 0.5f;
}

bool antipat_both_pitch_mismatch(const app_ctrl_params_snapshot_t *p,
                                 float u, float pitch_rad, float pitch_rate_rads,
                                 float pitch_ref_rad)
{
    if (fabsf(u) <= p->antipat_both_u_min_nm) {
        return false;
    }
    if (fabsf(pitch_rate_rads) <= p->antipat_both_pitch_rate_min_rads) {
        return false;
    }
    const float e_theta = pitch_ref_rad - pitch_rad;
    return (e_theta * pitch_rate_rads) < 0.0f;
}

bool antipat_both_candidate(const app_ctrl_params_snapshot_t *p,
                            float eta_l, float eta_r, bool unilateral,
                            bool pitch_mismatch)
{
    return antipat_both_enabled(p) && (eta_l > p->antipat_eta_on) &&
           (eta_r > p->antipat_eta_on) && !unilateral && pitch_mismatch;
}

void antipat_both_on_normal_sample(const app_ctrl_params_snapshot_t *p,
                                   float omega_l_turns_s, float omega_r_turns_s,
                                   float dt_s, uint32_t time_ms)
{
    const float a = ms_to_alpha(dt_s, p->antipat_both_t_ma_ms);
    s_v_ma_l += a * (omega_l_turns_s - s_v_ma_l);
    s_v_ma_r += a * (omega_r_turns_s - s_v_ma_r);
    ring_push(omega_l_turns_s, omega_r_turns_s, time_ms);
}

void antipat_both_snapshot_v_good(const app_ctrl_params_snapshot_t *p, uint32_t time_ms)
{
    const float t_excl_ms = 2.0f * p->antipat_both_t_on_ms;
    const float t_ma_ms = p->antipat_both_t_ma_ms;
    const float t_end = (float)time_ms - t_excl_ms;
    const float t_start = t_end - t_ma_ms;

    if (t_end <= t_start) {
        s_v_good_l = s_v_ma_l;
        s_v_good_r = s_v_ma_r;
        return;
    }

    ring_mean_omega(t_end, t_start, &s_v_good_l, &s_v_good_r);
}

void antipat_both_compute_tau(const app_ctrl_params_snapshot_t *p,
                              float vel_motor_l_turns_s, float vel_motor_r_turns_s,
                              float *tau_l_nm, float *tau_r_nm)
{
    float tl = p->antipat_both_k_v * (s_v_good_l - vel_motor_l_turns_s);
    float tr = p->antipat_both_k_v * (s_v_good_r - vel_motor_r_turns_s);
    *tau_l_nm = ctrl_clampf(tl, -p->antipat_both_tau_max_nm, p->antipat_both_tau_max_nm);
    *tau_r_nm = ctrl_clampf(tr, -p->antipat_both_tau_max_nm, p->antipat_both_tau_max_nm);
}

static bool both_still_flying(const app_ctrl_params_snapshot_t *p,
                              float omega_l, float omega_r, float tau_both_l, float tau_both_r)
{
    return (fabsf(tau_both_l) < p->antipat_both_tau_steady_air_nm) &&
           (fabsf(tau_both_r) < p->antipat_both_tau_steady_air_nm) &&
           (fabsf(omega_l) > p->antipat_omega_air_min_turns_s ||
            fabsf(omega_r) > p->antipat_omega_air_min_turns_s);
}

bool antipat_both_contact_ok(const app_ctrl_params_snapshot_t *p,
                             float eta_l, float eta_r,
                             float alpha_l_rads2, float alpha_r_rads2,
                             float tau_both_l_nm, float tau_both_r_nm,
                             float omega_l_turns_s, float omega_r_turns_s)
{
    if (both_still_flying(p, omega_l_turns_s, omega_r_turns_s, tau_both_l_nm, tau_both_r_nm)) {
        return false;
    }
    if (eta_l < p->antipat_both_eta_off && eta_r < p->antipat_both_eta_off) {
        return true;
    }
    if (p->antipat_both_alpha_contact_max_rads2 > 0.0f) {
        return (fabsf(alpha_l_rads2) < p->antipat_both_alpha_contact_max_rads2) &&
               (fabsf(alpha_r_rads2) < p->antipat_both_alpha_contact_max_rads2);
    }
    return false;
}

float antipat_both_v_good_l(void)
{
    return s_v_good_l;
}

float antipat_both_v_good_r(void)
{
    return s_v_good_r;
}
