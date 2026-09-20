/**
 * @file ctrl_abs.c
 * @brief Mix L/R torques from last ABS / wheel-contact mode.
 */

#include "ctrl_abs.h"

void ctrl_abs_mix(const wheel_contact_output_t *wco, float dtau_l_nm, float dtau_r_nm, float u_yaw_nm,
                  float *cmd_l_nm, float *cmd_r_nm)
{
    const float u_l = wco->u_cmd_nm * wco->u_scale_l;
    const float u_r = wco->u_cmd_nm * wco->u_scale_r;
    float cmd_l;
    float cmd_r;

    if (wco->both_active) {
        cmd_l = wco->tau_both_l_nm;
        cmd_r = wco->tau_both_r_nm;
    } else if (wco->mode == WC_MODE_SYNC_L) {
        cmd_l = u_l + dtau_l_nm + wco->tau_sync_l_nm;
        cmd_r = u_r + dtau_r_nm + u_yaw_nm;
    } else if (wco->mode == WC_MODE_SYNC_R) {
        cmd_l = u_l + dtau_l_nm - u_yaw_nm;
        cmd_r = u_r + dtau_r_nm + wco->tau_sync_r_nm;
    } else {
        cmd_l = u_l + dtau_l_nm - u_yaw_nm;
        cmd_r = u_r + dtau_r_nm + u_yaw_nm;
    }

    *cmd_l_nm = cmd_l;
    *cmd_r_nm = cmd_r;
}
