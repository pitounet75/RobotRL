/**
 * @file ctrl_balance.h
 * @brief Pitch → couple: u_meca (gravity + viscous D) + u_err (P + Kv), then LPF.
 */
#ifndef CTRL_BALANCE_H
#define CTRL_BALANCE_H

#include "control_strategy.h"

#include <stdbool.h>

typedef struct {
    float u_meca_nm;
    float u_err_nm;
    float u_vel_nm;
    float u_raw_nm;
} ctrl_balance_pd_out_t;

void ctrl_balance_reset(void);
void ctrl_balance_pd(const control_strategy_input_t *in, float pitch_trim_rad, float vel_ref_turns_s,
                     ctrl_balance_pd_out_t *out);
float ctrl_balance_lpf(float u_raw_nm, bool integrator_trust, bool reset_lpf, float u_lpf_seed_nm);

#endif /* CTRL_BALANCE_H */
