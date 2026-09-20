/**
 * @file antipat_both.h
 * @brief BOTH_AIR predicates, v_good lookback, and τ_both.
 *
 * Leaf: does not include antipat_common or antipat_sync.
 */
#ifndef ANTIPAT_BOTH_H
#define ANTIPAT_BOTH_H

#include "app_ctrl_params.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void antipat_both_reset(void);

bool antipat_both_enabled(const app_ctrl_params_snapshot_t *p);

bool antipat_both_pitch_mismatch(const app_ctrl_params_snapshot_t *p,
                                 float u, float pitch_rad, float pitch_rate_rads,
                                 float pitch_ref_rad);

bool antipat_both_candidate(const app_ctrl_params_snapshot_t *p,
                            float eta_l, float eta_r, bool unilateral,
                            bool pitch_mismatch);

void antipat_both_on_normal_sample(const app_ctrl_params_snapshot_t *p,
                                   float omega_l_turns_s, float omega_r_turns_s,
                                   float dt_s, uint32_t time_ms);

void antipat_both_snapshot_v_good(const app_ctrl_params_snapshot_t *p, uint32_t time_ms);

void antipat_both_compute_tau(const app_ctrl_params_snapshot_t *p,
                              float vel_motor_l_turns_s, float vel_motor_r_turns_s,
                              float *tau_l_nm, float *tau_r_nm);

bool antipat_both_contact_ok(const app_ctrl_params_snapshot_t *p,
                             float eta_l, float eta_r,
                             float alpha_l_rads2, float alpha_r_rads2,
                             float tau_both_l_nm, float tau_both_r_nm,
                             float omega_l_turns_s, float omega_r_turns_s);

float antipat_both_v_good_l(void);
float antipat_both_v_good_r(void);

#ifdef __cplusplus
}
#endif

#endif /* ANTIPAT_BOTH_H */
