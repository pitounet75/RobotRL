/**
 * @file app_ctrl_params.c
 */

#include "app_ctrl_params.h"

#include "app_config.h"
#include "control_strategy.h"

#include <stddef.h>
#include <string.h>

static app_ctrl_params_snapshot_t s_params;
static volatile bool s_pos_reset_req;

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
    s_params.torque_deadband_nm = APP_CTRL_TORQUE_DEADBAND_NM;
    s_params.torque_deadband_pitch_max_rad = APP_CTRL_TORQUE_DEADBAND_PITCH_MAX_RAD;
    s_params.torque_deadband_rate_max_rads = APP_CTRL_TORQUE_DEADBAND_RATE_MAX_RADS;
    s_params.alpha_kp = APP_CTRL_ALPHA_KP;
    s_params.alpha_max_nm = APP_CTRL_ALPHA_MAX_NM;
    s_params.motor_J = APP_CTRL_MOTOR_J_KG_M2;
    s_params.motor_friction_c = APP_CTRL_MOTOR_FRICTION_C_NM;
    s_params.alpha_pitch_max_rad = APP_CTRL_ALPHA_PITCH_MAX_RAD;
    s_params.alpha_rate_max_rads = APP_CTRL_ALPHA_RATE_MAX_RADS;
    s_params.alpha_vel_max_turns_s = APP_CTRL_ALPHA_VEL_MAX_TURNS_S;
    s_params.alpha_lpf = APP_CTRL_ALPHA_LPF;
    s_params.pos_kp = APP_CTRL_POS_KP;
    s_params.pos_kd = APP_CTRL_POS_KD;
    s_params.pos_pitch_kp = APP_CTRL_POS_PITCH_KP;
    s_params.pos_x_ref_m = APP_CTRL_POS_X_REF_M;
    s_params.pos_v_max_turns_s = APP_CTRL_POS_V_MAX_TURNS_S;
    s_params.pos_pitch_max_rad = APP_CTRL_POS_PITCH_MAX_RAD;
    s_params.wheel_radius_m = APP_WHEEL_RADIUS_M;
    s_params.pos_reset = 0.0f;
    s_pos_reset_req = false;
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

bool app_ctrl_params_consume_pos_reset(void)
{
    if (!s_pos_reset_req) {
        return false;
    }
    s_pos_reset_req = false;
    return true;
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
    case APP_CTRL_PARAM_TORQUE_DEADBAND_NM:
        *out_value = s_params.torque_deadband_nm;
        return true;
    case APP_CTRL_PARAM_TORQUE_DEADBAND_PITCH_MAX_RAD:
        *out_value = s_params.torque_deadband_pitch_max_rad;
        return true;
    case APP_CTRL_PARAM_TORQUE_DEADBAND_RATE_MAX_RADS:
        *out_value = s_params.torque_deadband_rate_max_rads;
        return true;
    case APP_CTRL_PARAM_ALPHA_KP:
        *out_value = s_params.alpha_kp;
        return true;
    case APP_CTRL_PARAM_ALPHA_MAX_NM:
        *out_value = s_params.alpha_max_nm;
        return true;
    case APP_CTRL_PARAM_MOTOR_J:
        *out_value = s_params.motor_J;
        return true;
    case APP_CTRL_PARAM_MOTOR_FRICTION_C:
        *out_value = s_params.motor_friction_c;
        return true;
    case APP_CTRL_PARAM_ALPHA_PITCH_MAX_RAD:
        *out_value = s_params.alpha_pitch_max_rad;
        return true;
    case APP_CTRL_PARAM_ALPHA_RATE_MAX_RADS:
        *out_value = s_params.alpha_rate_max_rads;
        return true;
    case APP_CTRL_PARAM_ALPHA_VEL_MAX_TURNS_S:
        *out_value = s_params.alpha_vel_max_turns_s;
        return true;
    case APP_CTRL_PARAM_ALPHA_LPF:
        *out_value = s_params.alpha_lpf;
        return true;
    case APP_CTRL_PARAM_POS_KP:
        *out_value = s_params.pos_kp;
        return true;
    case APP_CTRL_PARAM_POS_KD:
        *out_value = s_params.pos_kd;
        return true;
    case APP_CTRL_PARAM_POS_PITCH_KP:
        *out_value = s_params.pos_pitch_kp;
        return true;
    case APP_CTRL_PARAM_POS_X_REF_M:
        *out_value = s_params.pos_x_ref_m;
        return true;
    case APP_CTRL_PARAM_POS_V_MAX_TURNS_S:
        *out_value = s_params.pos_v_max_turns_s;
        return true;
    case APP_CTRL_PARAM_POS_PITCH_MAX_RAD:
        *out_value = s_params.pos_pitch_max_rad;
        return true;
    case APP_CTRL_PARAM_WHEEL_RADIUS_M:
        *out_value = s_params.wheel_radius_m;
        return true;
    case APP_CTRL_PARAM_POS_RESET:
        *out_value = 0.0f;
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
    case APP_CTRL_PARAM_TORQUE_DEADBAND_NM:
        if (value < 0.0f) {
            return false;
        }
        s_params.torque_deadband_nm = value;
        break;
    case APP_CTRL_PARAM_TORQUE_DEADBAND_PITCH_MAX_RAD:
        if (value < 0.0f) {
            return false;
        }
        s_params.torque_deadband_pitch_max_rad = value;
        break;
    case APP_CTRL_PARAM_TORQUE_DEADBAND_RATE_MAX_RADS:
        if (value < 0.0f) {
            return false;
        }
        s_params.torque_deadband_rate_max_rads = value;
        break;
    case APP_CTRL_PARAM_ALPHA_KP:
        if (value < 0.0f) {
            return false;
        }
        s_params.alpha_kp = value;
        break;
    case APP_CTRL_PARAM_ALPHA_MAX_NM:
        if (value < 0.0f) {
            return false;
        }
        s_params.alpha_max_nm = value;
        break;
    case APP_CTRL_PARAM_MOTOR_J:
        if (value <= 0.0f) {
            return false;
        }
        s_params.motor_J = value;
        break;
    case APP_CTRL_PARAM_MOTOR_FRICTION_C:
        if (value < 0.0f) {
            return false;
        }
        s_params.motor_friction_c = value;
        break;
    case APP_CTRL_PARAM_ALPHA_PITCH_MAX_RAD:
        if (value < 0.0f) {
            return false;
        }
        s_params.alpha_pitch_max_rad = value;
        break;
    case APP_CTRL_PARAM_ALPHA_RATE_MAX_RADS:
        if (value < 0.0f) {
            return false;
        }
        s_params.alpha_rate_max_rads = value;
        break;
    case APP_CTRL_PARAM_ALPHA_VEL_MAX_TURNS_S:
        if (value < 0.0f) {
            return false;
        }
        s_params.alpha_vel_max_turns_s = value;
        break;
    case APP_CTRL_PARAM_ALPHA_LPF:
        if (value < 0.0f || value >= 1.0f) {
            return false;
        }
        s_params.alpha_lpf = value;
        break;
    case APP_CTRL_PARAM_POS_KP:
        if (value < 0.0f) {
            return false;
        }
        s_params.pos_kp = value;
        break;
    case APP_CTRL_PARAM_POS_KD:
        if (value < 0.0f) {
            return false;
        }
        s_params.pos_kd = value;
        break;
    case APP_CTRL_PARAM_POS_PITCH_KP:
        if (value < 0.0f) {
            return false;
        }
        s_params.pos_pitch_kp = value;
        break;
    case APP_CTRL_PARAM_POS_X_REF_M:
        s_params.pos_x_ref_m = value;
        break;
    case APP_CTRL_PARAM_POS_V_MAX_TURNS_S:
        if (value < 0.0f) {
            return false;
        }
        s_params.pos_v_max_turns_s = value;
        break;
    case APP_CTRL_PARAM_POS_PITCH_MAX_RAD:
        if (value < 0.0f) {
            return false;
        }
        s_params.pos_pitch_max_rad = value;
        break;
    case APP_CTRL_PARAM_WHEEL_RADIUS_M:
        if (value <= 0.0f) {
            return false;
        }
        s_params.wheel_radius_m = value;
        break;
    case APP_CTRL_PARAM_POS_RESET:
        s_pos_reset_req = true;
        s_params.pos_reset = 0.0f;
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
        [APP_CTRL_PARAM_TORQUE_DEADBAND_NM] = "torque_deadband_nm",
        [APP_CTRL_PARAM_TORQUE_DEADBAND_PITCH_MAX_RAD] = "torque_deadband_pitch_max_rad",
        [APP_CTRL_PARAM_TORQUE_DEADBAND_RATE_MAX_RADS] = "torque_deadband_rate_max_rads",
        [APP_CTRL_PARAM_ALPHA_KP] = "alpha_kp",
        [APP_CTRL_PARAM_ALPHA_MAX_NM] = "alpha_max_nm",
        [APP_CTRL_PARAM_MOTOR_J] = "motor_J",
        [APP_CTRL_PARAM_MOTOR_FRICTION_C] = "motor_friction_c",
        [APP_CTRL_PARAM_ALPHA_PITCH_MAX_RAD] = "alpha_pitch_max_rad",
        [APP_CTRL_PARAM_ALPHA_RATE_MAX_RADS] = "alpha_rate_max_rads",
        [APP_CTRL_PARAM_ALPHA_VEL_MAX_TURNS_S] = "alpha_vel_max_turns_s",
        [APP_CTRL_PARAM_ALPHA_LPF] = "alpha_lpf",
        [APP_CTRL_PARAM_POS_KP] = "pos_kp",
        [APP_CTRL_PARAM_POS_KD] = "pos_kd",
        [APP_CTRL_PARAM_POS_PITCH_KP] = "pos_pitch_kp",
        [APP_CTRL_PARAM_POS_X_REF_M] = "pos_x_ref_m",
        [APP_CTRL_PARAM_POS_V_MAX_TURNS_S] = "pos_v_max_turns_s",
        [APP_CTRL_PARAM_POS_PITCH_MAX_RAD] = "pos_pitch_max_rad",
        [APP_CTRL_PARAM_WHEEL_RADIUS_M] = "wheel_radius_m",
        [APP_CTRL_PARAM_POS_RESET] = "pos_reset",
    };

    if (param_id >= (uint16_t)APP_CTRL_PARAM_COUNT) {
        return "?";
    }
    return names[param_id];
}
