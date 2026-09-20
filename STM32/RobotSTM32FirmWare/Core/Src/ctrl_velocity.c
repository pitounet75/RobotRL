/**
 * @file ctrl_velocity.c
 * @brief v → pitch_trim.
 */

#include "ctrl_velocity.h"

#include "app_ctrl_params.h"
#include "ctrl_math.h"

static float s_vel_ref_slew;
static float s_vel_err_ema;
static float s_vel_ref_prev;
static bool s_vel_ref_prev_valid;
static float s_vel_prev_turns;
static float s_vel_dot_turns_s2;
static bool s_vel_dot_valid;

static const float k_vel_dot_lpf = 0.85f;

volatile float g_ctrl_pos_v_ref_turns_s;
volatile float g_ctrl_pos_pitch_trim_rad;

void ctrl_velocity_reset(void)
{
    s_vel_ref_slew = 0.0f;
    s_vel_err_ema = 0.0f;
    s_vel_ref_prev = 0.0f;
    s_vel_ref_prev_valid = false;
    s_vel_prev_turns = 0.0f;
    s_vel_dot_turns_s2 = 0.0f;
    s_vel_dot_valid = false;
    g_ctrl_pos_v_ref_turns_s = 0.0f;
    g_ctrl_pos_pitch_trim_rad = 0.0f;
}

void ctrl_velocity_step(const control_strategy_input_t *in, float vel_ref_cmd,
                        bool integrator_trust, float recover_ramp, bool reset_integrators,
                        float vel_wheel_touch_turns_s, float pitch_trim_frozen_rad,
                        ctrl_velocity_out_t *out)
{
    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();

    if (reset_integrators) {
        s_vel_err_ema = 0.0f;
        s_vel_dot_valid = false;
        s_vel_prev_turns = vel_wheel_touch_turns_s;
    }

    float vel_ref = vel_ref_cmd;
    float vel_ref_dot = 0.0f;
    if (integrator_trust) {
        const float slew = p->vel_ref_slew_turns_s2;
        if (slew > 0.0f && in->dt_s > 1.0e-6f) {
            const float max_step = slew * in->dt_s;
            float dv = vel_ref - s_vel_ref_slew;
            if (dv > max_step) {
                dv = max_step;
            } else if (dv < -max_step) {
                dv = -max_step;
            }
            s_vel_ref_slew += dv;
            vel_ref_dot = dv / in->dt_s;
        } else {
            if (s_vel_ref_prev_valid && in->dt_s > 1.0e-6f) {
                vel_ref_dot = (vel_ref - s_vel_ref_prev) / in->dt_s;
            }
            s_vel_ref_slew = vel_ref;
        }
        vel_ref = s_vel_ref_slew;
        s_vel_ref_prev = vel_ref;
        s_vel_ref_prev_valid = true;
    } else {
        vel_ref = s_vel_ref_slew * recover_ramp;
    }

    float vel_dot = 0.0f;
    if (integrator_trust && in->dt_s > 1.0e-6f) {
        const float raw_dot = (in->vel_wheel_turns_s - s_vel_prev_turns) / in->dt_s;
        if (s_vel_dot_valid) {
            s_vel_dot_turns_s2 = k_vel_dot_lpf * s_vel_dot_turns_s2 + (1.0f - k_vel_dot_lpf) * raw_dot;
        } else {
            s_vel_dot_turns_s2 = raw_dot;
            s_vel_dot_valid = true;
        }
        vel_dot = s_vel_dot_turns_s2;
        s_vel_prev_turns = in->vel_wheel_turns_s;
    }

    const float vel_err = vel_ref - in->vel_wheel_turns_s;
    if (integrator_trust) {
        const float a = ctrl_clampf(p->cascade_vel_err_ema_alpha, 0.0f, 0.9999f);
        s_vel_err_ema = a * s_vel_err_ema + (1.0f - a) * vel_err;
    }
    float pitch_cmd = p->cascade_vel_kp * vel_err + p->cascade_vel_ema_kp * s_vel_err_ema
                    + p->cascade_vel_kd * vel_dot - p->cascade_vel_accel_kp * vel_ref_dot;
    pitch_cmd = ctrl_clampf(pitch_cmd, -p->cascade_pitch_ref_max_rad, p->cascade_pitch_ref_max_rad);
    float pitch_trim = -pitch_cmd;
    if (!integrator_trust) {
        pitch_trim = pitch_trim_frozen_rad * (1.0f - recover_ramp) + pitch_trim * recover_ramp;
    }

    g_ctrl_pos_v_ref_turns_s = vel_ref;
    g_ctrl_pos_pitch_trim_rad = pitch_trim;

    out->vel_ref_turns_s = vel_ref;
    out->vel_ref_dot = vel_ref_dot;
    out->pitch_trim_rad = pitch_trim;
}
