/**
 * @file ctrl_position.c
 * @brief x → v_ref (outer_mode=1) or pass-through teleop v_ref.
 */

#include "ctrl_position.h"

#include "app_ctrl_params.h"
#include "ctrl_math.h"

static float s_pos_offset_turns;
static bool s_pos_offset_valid;
static float s_x_err_ema;

volatile float g_ctrl_pos_x_m;
volatile float g_ctrl_pos_err_ema_m;

void ctrl_position_reset(void)
{
    s_pos_offset_valid = false;
    s_pos_offset_turns = 0.0f;
    s_x_err_ema = 0.0f;
    g_ctrl_pos_x_m = 0.0f;
    g_ctrl_pos_err_ema_m = 0.0f;
}

void ctrl_position_step(const control_strategy_input_t *in, bool integrator_trust,
                        bool reanchor_pos, float pos_offset_turns_new, float x_m_frozen_m,
                        ctrl_position_out_t *out)
{
    const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
    const bool pos_mode = (p->outer_mode >= 0.5f);

    if (reanchor_pos && in->pos_wheel_valid) {
        s_pos_offset_turns = pos_offset_turns_new;
        s_pos_offset_valid = true;
        s_x_err_ema = 0.0f;
    }

    float vel_ref = 0.0f;
    float x_m = 0.0f;
    float x_err_ema = s_x_err_ema;

    if (app_ctrl_params_consume_pos_reset() || (pos_mode && !s_pos_offset_valid)) {
        if (in->pos_wheel_valid) {
            s_pos_offset_turns = in->pos_wheel_turns;
            s_pos_offset_valid = true;
            s_x_err_ema = 0.0f;
            x_err_ema = 0.0f;
        }
    }

    if (pos_mode) {
        if (integrator_trust && in->pos_wheel_valid && s_pos_offset_valid && p->wheel_radius_m > 0.0f) {
            const float turns = in->pos_wheel_turns - s_pos_offset_turns;
            x_m = turns * ctrl_two_pi() * p->wheel_radius_m;
            const float x_err = p->pos_x_ref_m - x_m;

            const float a = ctrl_clampf(p->pos_err_ema_alpha, 0.0f, 0.9999f);
            x_err_ema = a * s_x_err_ema + (1.0f - a) * x_err;
            s_x_err_ema = x_err_ema;

            vel_ref = (p->pos_kp * x_err + p->pos_ema_kp * x_err_ema) - p->pos_kd * in->vel_wheel_turns_s;
            vel_ref = ctrl_clampf(vel_ref, -p->pos_v_max_turns_s, p->pos_v_max_turns_s);
        } else if (!integrator_trust) {
            x_m = x_m_frozen_m;
            x_err_ema = s_x_err_ema;
        }
    } else {
        vel_ref = in->vel_ref_turns_s;
    }

    g_ctrl_pos_x_m = x_m;
    g_ctrl_pos_err_ema_m = x_err_ema;

    out->vel_ref_turns_s = vel_ref;
    out->x_m = x_m;
    out->x_err_ema_m = x_err_ema;
    out->pos_mode = pos_mode;
}
