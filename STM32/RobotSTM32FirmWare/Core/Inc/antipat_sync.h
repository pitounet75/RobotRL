/**
 * @file antipat_sync.h
 * @brief Unilateral SYNC_L / SYNC_R predicates and τ_sync.
 *
 * Leaf: does not include antipat_common or antipat_both.
 */
#ifndef ANTIPAT_SYNC_H
#define ANTIPAT_SYNC_H

#include "app_ctrl_params.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool antipat_sync_enabled(const app_ctrl_params_snapshot_t *p);

/** Yaw vs wheel kinematics. Writes e_psi even when track width is unusable. */
bool antipat_sync_decorrelated(const app_ctrl_params_snapshot_t *p,
                               float omega_l_turns_s, float omega_r_turns_s,
                               float yaw_rate_rads, float *e_psi_out);

bool antipat_sync_candidate(const app_ctrl_params_snapshot_t *p,
                            float eta_this, float eta_other, bool decorrelated);

bool antipat_sync_recontact(const app_ctrl_params_snapshot_t *p,
                            float e_psi, float yaw_rate_rads,
                            float omega_ground_turns_s, float omega_air_turns_s);

float antipat_sync_tau(const app_ctrl_params_snapshot_t *p,
                       float omega_ground_turns_s, float omega_air_turns_s,
                       float alpha_ground_rads2, float alpha_air_rads2);

#ifdef __cplusplus
}
#endif

#endif /* ANTIPAT_SYNC_H */
