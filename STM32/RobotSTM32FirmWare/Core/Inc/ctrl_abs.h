/**
 * @file ctrl_abs.h
 * @brief Antipatinage / ABS overlay: mix L/R after the other loops.
 *
 * Detection FSM: antipat_common.c (calls antipat_sync / antipat_both).
 */
#ifndef CTRL_ABS_H
#define CTRL_ABS_H

#include "wheel_contact.h"

void ctrl_abs_mix(const wheel_contact_output_t *wco, float dtau_l_nm, float dtau_r_nm, float u_yaw_nm,
                  float *cmd_l_nm, float *cmd_r_nm);

#endif /* CTRL_ABS_H */
