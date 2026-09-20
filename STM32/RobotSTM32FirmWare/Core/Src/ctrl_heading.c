/**
 * @file ctrl_heading.c
 * @brief Yaw-rate P+D → u_yaw.
 */

#include "ctrl_heading.h"

#include "app_ctrl_params.h"
#include "ctrl_math.h"

static float s_heading_rad;
static bool s_heading_valid;
static float s_yaw_rate_lpf_rads;
static bool s_yaw_rate_lpf_valid;
static float s_yaw_raw_prev_rads;
static bool s_yaw_raw_prev_valid;
static float s_yaw_ddot_lpf;
static bool s_yaw_ddot_lpf_valid;
static bool s_yaw_was_allowed;

volatile float g_ctrl_heading_rad;
volatile float g_ctrl_heading_torque_nm;
volatile float g_ctrl_yaw_rate_lpf_rads;

void ctrl_heading_reset(void)
{
    s_heading_rad = 0.0f;
    s_heading_valid = false;
    s_yaw_rate_lpf_rads = 0.0f;
    s_yaw_rate_lpf_valid = false;
    s_yaw_raw_prev_rads = 0.0f;
    s_yaw_raw_prev_valid = false;
    s_yaw_ddot_lpf = 0.0f;
    s_yaw_ddot_lpf_valid = false;
    s_yaw_was_allowed = true;
    g_ctrl_heading_rad = 0.0f;
    g_ctrl_heading_torque_nm = 0.0f;
    g_ctrl_yaw_rate_lpf_rads = 0.0f;
}

float ctrl_heading_step(const control_strategy_input_t *in, bool yaw_allowed)
{
    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
    float u_yaw = 0.0f;
    const bool heading_reset = app_ctrl_params_consume_heading_reset();
    const float gyro = in->yaw_rate_rads;

    if (yaw_allowed && !s_yaw_was_allowed) {
        /* Leave SYNC/BOTH without a stale heading_ema from before the lift. */
        s_yaw_rate_lpf_rads = gyro;
        s_yaw_rate_lpf_valid = true;
        s_yaw_raw_prev_rads = gyro;
        s_yaw_raw_prev_valid = true;
        s_yaw_ddot_lpf = 0.0f;
        s_yaw_ddot_lpf_valid = true;
    }
    s_yaw_was_allowed = yaw_allowed;

    float yaw_rate = s_yaw_rate_lpf_valid ? s_yaw_rate_lpf_rads : gyro;
    if (yaw_allowed || heading_reset || !s_yaw_rate_lpf_valid) {
        const float a = ctrl_clampf(p->heading_ema, 0.0f, 0.999f);
        if (heading_reset || !s_yaw_rate_lpf_valid) {
            s_yaw_rate_lpf_rads = gyro;
            s_yaw_rate_lpf_valid = true;
        } else if (a > 0.0f) {
            s_yaw_rate_lpf_rads = a * s_yaw_rate_lpf_rads + (1.0f - a) * gyro;
        } else {
            s_yaw_rate_lpf_rads = gyro;
        }
        yaw_rate = s_yaw_rate_lpf_rads;
    }

    float rate_dot = s_yaw_ddot_lpf_valid ? s_yaw_ddot_lpf : 0.0f;
    if (heading_reset || !s_yaw_raw_prev_valid) {
        s_yaw_raw_prev_rads = gyro;
        s_yaw_raw_prev_valid = true;
        s_yaw_ddot_lpf = 0.0f;
        s_yaw_ddot_lpf_valid = false;
        rate_dot = 0.0f;
    } else if (yaw_allowed && in->dt_s > 1.0e-6f) {
        const float raw_dot = (gyro - s_yaw_raw_prev_rads) / in->dt_s;
        const float a_d = ctrl_clampf(p->heading_d_ema, 0.0f, 0.999f);
        if (!s_yaw_ddot_lpf_valid) {
            s_yaw_ddot_lpf = raw_dot;
            s_yaw_ddot_lpf_valid = true;
        } else if (a_d > 0.0f) {
            s_yaw_ddot_lpf = a_d * s_yaw_ddot_lpf + (1.0f - a_d) * raw_dot;
        } else {
            s_yaw_ddot_lpf = raw_dot;
        }
        rate_dot = s_yaw_ddot_lpf;
        s_yaw_raw_prev_rads = gyro;
    } else if (!yaw_allowed) {
        /* Keep prev = now so D does not spike on unfreeze. */
        s_yaw_raw_prev_rads = gyro;
    }

    if (heading_reset || !s_heading_valid) {
        s_heading_rad = 0.0f;
        s_heading_valid = true;
    }
    if (yaw_allowed && in->dt_s > 1.0e-6f) {
        s_heading_rad += yaw_rate * in->dt_s;
        s_heading_rad = ctrl_wrap_pi(s_heading_rad);
    }

    const bool yaw_rate_on = (p->heading_kp != 0.0f || p->heading_kd > 0.0f) &&
                             p->heading_torque_max_nm > 0.0f;
    if (yaw_allowed && yaw_rate_on) {
        const float rate_err = p->heading_ref_rad - yaw_rate;
        u_yaw = p->heading_kp * rate_err - p->heading_kd * rate_dot;
        u_yaw = ctrl_clampf(u_yaw, -p->heading_torque_max_nm, p->heading_torque_max_nm);
    }

    g_ctrl_yaw_rate_lpf_rads = yaw_rate;
    g_ctrl_heading_rad = s_heading_rad;
    g_ctrl_heading_torque_nm = u_yaw;
    return u_yaw;
}
