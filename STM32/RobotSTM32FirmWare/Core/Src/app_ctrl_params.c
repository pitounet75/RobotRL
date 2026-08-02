/**
 * @file app_ctrl_params.c
 */

#include "app_ctrl_params.h"

#include "app_config.h"
#include "control_strategy.h"

#include <stddef.h>
#include <string.h>

static app_ctrl_params_snapshot_t s_params;

static void load_defaults(void)
{
    memset(&s_params, 0, sizeof(s_params));
    s_params.version = APP_CTRL_PARAMS_SNAPSHOT_VERSION;
    s_params.strategy_id = (uint32_t)APP_CTRL_STRATEGY_DEFAULT;
    s_params.pitch_ref_rad = APP_CTRL_PITCH_REF_RAD;
    s_params.vel_ref_turns_s = APP_CTRL_VEL_REF_TURNS_S;
    s_params.pitch_failsafe_rad = APP_CTRL_PITCH_FAILSAFE_RAD;
    s_params.pitch_kp = APP_CTRL_PITCH_KP;
    s_params.pitch_ki = APP_CTRL_PITCH_KI;
    s_params.pitch_kd = APP_CTRL_PITCH_KD;
    s_params.vel_kp = APP_CTRL_VEL_KP;
    s_params.vel_ki = APP_CTRL_VEL_KI;
    s_params.vel_kd = APP_CTRL_VEL_KD;
    s_params.cmd_max_torque_nm = APP_CTRL_CMD_MAX_TORQUE_NM;
    s_params.linear_theta_func = (uint32_t)APP_CTRL_LINEAR_THETA_FUNC;
    s_params.linear_k_pitch = APP_CTRL_LINEAR_K_PITCH;
    s_params.linear_k_pitch_rate = APP_CTRL_LINEAR_K_PITCH_RATE;
    s_params.linear_k_vel = APP_CTRL_LINEAR_K_VEL;
    s_params.linear_output_alpha = APP_CTRL_LINEAR_OUTPUT_ALPHA;
    s_params.cascade_vel_kp = APP_CTRL_CASCADE_VEL_KP;
    s_params.cascade_vel_ki = APP_CTRL_CASCADE_VEL_KI;
    s_params.cascade_vel_kd = APP_CTRL_CASCADE_VEL_KD;
    s_params.cascade_pitch_ref_max_rad = APP_CTRL_CASCADE_PITCH_REF_MAX_RAD;
    s_params.ff_grav_k = APP_CTRL_FF_GRAV_K;
    s_params.ff_fb_k_pitch = APP_CTRL_FF_FB_K_PITCH;
    s_params.ff_fb_k_rate = APP_CTRL_FF_FB_K_RATE;
    s_params.ff_output_alpha = APP_CTRL_FF_OUTPUT_ALPHA;
    s_params.wheel_encoder_vel_lpf_alpha = WHEEL_ENCODER_VEL_LPF_ALPHA;
}

static void reset_active_strategy(void)
{
    const control_strategy_id_t id = control_strategy_get();
    (void)control_strategy_set(id);
}

static void apply_side_effects(uint16_t param_id)
{
    switch ((app_ctrl_param_id_t)param_id) {
    case APP_CTRL_PARAM_STRATEGY:
        (void)control_strategy_set((control_strategy_id_t)s_params.strategy_id);
        return;
    case APP_CTRL_PARAM_PITCH_KP:
    case APP_CTRL_PARAM_PITCH_KI:
    case APP_CTRL_PARAM_PITCH_KD:
    case APP_CTRL_PARAM_VEL_KP:
    case APP_CTRL_PARAM_VEL_KI:
    case APP_CTRL_PARAM_VEL_KD:
    case APP_CTRL_PARAM_CMD_MAX_TORQUE_NM:
    case APP_CTRL_PARAM_CASCADE_VEL_KP:
    case APP_CTRL_PARAM_CASCADE_VEL_KI:
    case APP_CTRL_PARAM_CASCADE_VEL_KD:
    case APP_CTRL_PARAM_CASCADE_PITCH_REF_MAX_RAD:
        reset_active_strategy();
        return;
    default:
        return;
    }
}

void app_ctrl_params_init(void)
{
    load_defaults();
}

const app_ctrl_params_snapshot_t *app_ctrl_params_snapshot(void)
{
    return &s_params;
}

bool app_ctrl_params_get_value(uint16_t param_id, float *out_value)
{
    if (out_value == NULL || param_id >= (uint16_t)APP_CTRL_PARAM_COUNT) {
        return false;
    }

    switch ((app_ctrl_param_id_t)param_id) {
    case APP_CTRL_PARAM_STRATEGY:
        *out_value = (float)s_params.strategy_id;
        return true;
    case APP_CTRL_PARAM_PITCH_REF_RAD:
        *out_value = s_params.pitch_ref_rad;
        return true;
    case APP_CTRL_PARAM_VEL_REF_TURNS_S:
        *out_value = s_params.vel_ref_turns_s;
        return true;
    case APP_CTRL_PARAM_PITCH_FAILSAFE_RAD:
        *out_value = s_params.pitch_failsafe_rad;
        return true;
    case APP_CTRL_PARAM_PITCH_KP:
        *out_value = s_params.pitch_kp;
        return true;
    case APP_CTRL_PARAM_PITCH_KI:
        *out_value = s_params.pitch_ki;
        return true;
    case APP_CTRL_PARAM_PITCH_KD:
        *out_value = s_params.pitch_kd;
        return true;
    case APP_CTRL_PARAM_VEL_KP:
        *out_value = s_params.vel_kp;
        return true;
    case APP_CTRL_PARAM_VEL_KI:
        *out_value = s_params.vel_ki;
        return true;
    case APP_CTRL_PARAM_VEL_KD:
        *out_value = s_params.vel_kd;
        return true;
    case APP_CTRL_PARAM_CMD_MAX_TORQUE_NM:
        *out_value = s_params.cmd_max_torque_nm;
        return true;
    case APP_CTRL_PARAM_LINEAR_THETA_FUNC:
        *out_value = (float)s_params.linear_theta_func;
        return true;
    case APP_CTRL_PARAM_LINEAR_K_PITCH:
        *out_value = s_params.linear_k_pitch;
        return true;
    case APP_CTRL_PARAM_LINEAR_K_PITCH_RATE:
        *out_value = s_params.linear_k_pitch_rate;
        return true;
    case APP_CTRL_PARAM_LINEAR_K_VEL:
        *out_value = s_params.linear_k_vel;
        return true;
    case APP_CTRL_PARAM_LINEAR_OUTPUT_ALPHA:
        *out_value = s_params.linear_output_alpha;
        return true;
    case APP_CTRL_PARAM_CASCADE_VEL_KP:
        *out_value = s_params.cascade_vel_kp;
        return true;
    case APP_CTRL_PARAM_CASCADE_VEL_KI:
        *out_value = s_params.cascade_vel_ki;
        return true;
    case APP_CTRL_PARAM_CASCADE_VEL_KD:
        *out_value = s_params.cascade_vel_kd;
        return true;
    case APP_CTRL_PARAM_CASCADE_PITCH_REF_MAX_RAD:
        *out_value = s_params.cascade_pitch_ref_max_rad;
        return true;
    case APP_CTRL_PARAM_FF_GRAV_K:
        *out_value = s_params.ff_grav_k;
        return true;
    case APP_CTRL_PARAM_FF_FB_K_PITCH:
        *out_value = s_params.ff_fb_k_pitch;
        return true;
    case APP_CTRL_PARAM_FF_FB_K_RATE:
        *out_value = s_params.ff_fb_k_rate;
        return true;
    case APP_CTRL_PARAM_FF_OUTPUT_ALPHA:
        *out_value = s_params.ff_output_alpha;
        return true;
    case APP_CTRL_PARAM_WHEEL_ENCODER_VEL_LPF_ALPHA:
        *out_value = s_params.wheel_encoder_vel_lpf_alpha;
        return true;
    default:
        return false;
    }
}

bool app_ctrl_params_set(uint16_t param_id, float value, float *out_value)
{
    if (param_id >= (uint16_t)APP_CTRL_PARAM_COUNT) {
        return false;
    }

    switch ((app_ctrl_param_id_t)param_id) {
    case APP_CTRL_PARAM_STRATEGY: {
        const uint32_t sid = (uint32_t)value;
        if (sid >= (uint32_t)CTRL_STRATEGY_COUNT) {
            return false;
        }
        s_params.strategy_id = sid;
        break;
    }
    case APP_CTRL_PARAM_PITCH_REF_RAD:
        s_params.pitch_ref_rad = value;
        break;
    case APP_CTRL_PARAM_VEL_REF_TURNS_S:
        s_params.vel_ref_turns_s = value;
        break;
    case APP_CTRL_PARAM_PITCH_FAILSAFE_RAD:
        if (value <= 0.0f) {
            return false;
        }
        s_params.pitch_failsafe_rad = value;
        break;
    case APP_CTRL_PARAM_PITCH_KP:
        s_params.pitch_kp = value;
        break;
    case APP_CTRL_PARAM_PITCH_KI:
        s_params.pitch_ki = value;
        break;
    case APP_CTRL_PARAM_PITCH_KD:
        s_params.pitch_kd = value;
        break;
    case APP_CTRL_PARAM_VEL_KP:
        s_params.vel_kp = value;
        break;
    case APP_CTRL_PARAM_VEL_KI:
        s_params.vel_ki = value;
        break;
    case APP_CTRL_PARAM_VEL_KD:
        s_params.vel_kd = value;
        break;
    case APP_CTRL_PARAM_CMD_MAX_TORQUE_NM:
        if (value <= 0.0f) {
            return false;
        }
        s_params.cmd_max_torque_nm = value;
        break;
    case APP_CTRL_PARAM_LINEAR_THETA_FUNC:
        if (value < 0.0f || value > 1.0f) {
            return false;
        }
        s_params.linear_theta_func = (uint32_t)value;
        break;
    case APP_CTRL_PARAM_LINEAR_K_PITCH:
        s_params.linear_k_pitch = value;
        break;
    case APP_CTRL_PARAM_LINEAR_K_PITCH_RATE:
        s_params.linear_k_pitch_rate = value;
        break;
    case APP_CTRL_PARAM_LINEAR_K_VEL:
        s_params.linear_k_vel = value;
        break;
    case APP_CTRL_PARAM_LINEAR_OUTPUT_ALPHA:
        if (value < 0.0f || value > 1.0f) {
            return false;
        }
        s_params.linear_output_alpha = value;
        break;
    case APP_CTRL_PARAM_CASCADE_VEL_KP:
        s_params.cascade_vel_kp = value;
        break;
    case APP_CTRL_PARAM_CASCADE_VEL_KI:
        s_params.cascade_vel_ki = value;
        break;
    case APP_CTRL_PARAM_CASCADE_VEL_KD:
        s_params.cascade_vel_kd = value;
        break;
    case APP_CTRL_PARAM_CASCADE_PITCH_REF_MAX_RAD:
        if (value < 0.0f) {
            return false;
        }
        s_params.cascade_pitch_ref_max_rad = value;
        break;
    case APP_CTRL_PARAM_FF_GRAV_K:
        s_params.ff_grav_k = value;
        break;
    case APP_CTRL_PARAM_FF_FB_K_PITCH:
        s_params.ff_fb_k_pitch = value;
        break;
    case APP_CTRL_PARAM_FF_FB_K_RATE:
        s_params.ff_fb_k_rate = value;
        break;
    case APP_CTRL_PARAM_FF_OUTPUT_ALPHA:
        if (value < 0.0f || value > 1.0f) {
            return false;
        }
        s_params.ff_output_alpha = value;
        break;
    case APP_CTRL_PARAM_WHEEL_ENCODER_VEL_LPF_ALPHA:
        if (value < 0.0f || value >= 1.0f) {
            return false;
        }
        s_params.wheel_encoder_vel_lpf_alpha = value;
        break;
    default:
        return false;
    }

    apply_side_effects(param_id);

    if (out_value != NULL) {
        (void)app_ctrl_params_get_value(param_id, out_value);
    }
    return true;
}

const char *app_ctrl_params_name(uint16_t param_id)
{
    static const char *const names[APP_CTRL_PARAM_COUNT] = {
        [APP_CTRL_PARAM_STRATEGY] = "strategy",
        [APP_CTRL_PARAM_PITCH_REF_RAD] = "pitch_ref_rad",
        [APP_CTRL_PARAM_VEL_REF_TURNS_S] = "vel_ref_turns_s",
        [APP_CTRL_PARAM_PITCH_FAILSAFE_RAD] = "pitch_failsafe_rad",
        [APP_CTRL_PARAM_PITCH_KP] = "pitch_kp",
        [APP_CTRL_PARAM_PITCH_KI] = "pitch_ki",
        [APP_CTRL_PARAM_PITCH_KD] = "pitch_kd",
        [APP_CTRL_PARAM_VEL_KP] = "vel_kp",
        [APP_CTRL_PARAM_VEL_KI] = "vel_ki",
        [APP_CTRL_PARAM_VEL_KD] = "vel_kd",
        [APP_CTRL_PARAM_CMD_MAX_TORQUE_NM] = "cmd_max_torque_nm",
        [APP_CTRL_PARAM_LINEAR_THETA_FUNC] = "linear_theta_func",
        [APP_CTRL_PARAM_LINEAR_K_PITCH] = "linear_k_pitch",
        [APP_CTRL_PARAM_LINEAR_K_PITCH_RATE] = "linear_k_pitch_rate",
        [APP_CTRL_PARAM_LINEAR_K_VEL] = "linear_k_vel",
        [APP_CTRL_PARAM_LINEAR_OUTPUT_ALPHA] = "linear_output_alpha",
        [APP_CTRL_PARAM_CASCADE_VEL_KP] = "cascade_vel_kp",
        [APP_CTRL_PARAM_CASCADE_VEL_KI] = "cascade_vel_ki",
        [APP_CTRL_PARAM_CASCADE_VEL_KD] = "cascade_vel_kd",
        [APP_CTRL_PARAM_CASCADE_PITCH_REF_MAX_RAD] = "cascade_pitch_ref_max_rad",
        [APP_CTRL_PARAM_FF_GRAV_K] = "ff_grav_k",
        [APP_CTRL_PARAM_FF_FB_K_PITCH] = "ff_fb_k_pitch",
        [APP_CTRL_PARAM_FF_FB_K_RATE] = "ff_fb_k_rate",
        [APP_CTRL_PARAM_FF_OUTPUT_ALPHA] = "ff_output_alpha",
        [APP_CTRL_PARAM_WHEEL_ENCODER_VEL_LPF_ALPHA] = "wheel_encoder_vel_lpf_alpha",
    };

    if (param_id >= (uint16_t)APP_CTRL_PARAM_COUNT) {
        return "?";
    }
    return names[param_id];
}
