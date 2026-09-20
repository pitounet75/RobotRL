/**
 * @file app_ctrl_params.c
 */

#include "app_ctrl_params.h"

#include "app_config.h"
#include "control_strategy.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static app_ctrl_params_snapshot_t s_params;
static volatile bool s_pos_reset_req;
static volatile bool s_heading_reset_req;

/** ψ̇_ref = value (rad/s). heading_inc mirrors it for GET. */
static void set_yaw_rate_ref(float value)
{
    s_params.heading_ref_rad = clampf(value,
                                      -APP_CTRL_YAW_RATE_REF_MAX_RADS,
                                      APP_CTRL_YAW_RATE_REF_MAX_RADS);
    s_params.heading_inc = s_params.heading_ref_rad;
    s_params.heading_dec = 0.0f;
}

static void load_defaults(void)
{
    memset(&s_params, 0, sizeof(s_params));
    s_params.version = APP_CTRL_PARAMS_SNAPSHOT_VERSION;
    s_params.strategy_id = (uint32_t)APP_CTRL_STRATEGY_DEFAULT;
    s_params.pitch_ref_rad = APP_CTRL_PITCH_REF_RAD;
    s_params.vel_ref_turns_s = APP_CTRL_VEL_REF_TURNS_S;
    s_params.pitch_failsafe_rad = APP_CTRL_PITCH_FAILSAFE_RAD;
    s_params.cmd_max_torque_nm = APP_CTRL_CMD_MAX_TORQUE_NM;
    s_params.cascade_vel_kp = APP_CTRL_CASCADE_VEL_KP;
    s_params.cascade_vel_kd = APP_CTRL_CASCADE_VEL_KD;
    s_params.cascade_pitch_ref_max_rad = APP_CTRL_CASCADE_PITCH_REF_MAX_RAD;
    s_params.meca_k_grav = APP_CTRL_MECA_K_GRAV;
    s_params.err_k_pitch = APP_CTRL_ERR_K_PITCH;
    s_params.meca_k_pitch_damp = APP_CTRL_MECA_K_PITCH_DAMP;
    s_params.balance_output_alpha = APP_CTRL_BALANCE_OUTPUT_ALPHA;
    s_params.wheel_encoder_vel_lpf_alpha = WHEEL_ENCODER_VEL_LPF_ALPHA;
    s_params.torque_deadband_nm = APP_CTRL_TORQUE_DEADBAND_NM;
    s_params.torque_deadband_pitch_max_rad = APP_CTRL_TORQUE_DEADBAND_PITCH_MAX_RAD;
    s_params.torque_deadband_rate_max_rads = APP_CTRL_TORQUE_DEADBAND_RATE_MAX_RADS;
    s_params.motor_torque_correction_kp = APP_CTRL_MOTOR_TORQUE_CORRECTION_KP;
    s_params.motor_torque_correction_max_nm = APP_CTRL_MOTOR_TORQUE_CORRECTION_MAX_NM;
    s_params.motor_J = APP_CTRL_MOTOR_J_KG_M2;
    s_params.motor_friction_c = APP_CTRL_MOTOR_FRICTION_C_NM;
    s_params.motor_torque_correction_gate_pitch_max_rad = APP_CTRL_MOTOR_TORQUE_CORRECTION_GATE_PITCH_MAX_RAD;
    s_params.motor_torque_correction_gate_rate_max_rads = APP_CTRL_MOTOR_TORQUE_CORRECTION_GATE_RATE_MAX_RADS;
    s_params.motor_torque_correction_gate_vel_max_turns_s = APP_CTRL_MOTOR_TORQUE_CORRECTION_GATE_VEL_MAX_TURNS_S;
    s_params.motor_accel_lpf = APP_CTRL_MOTOR_ACCEL_LPF;
    s_params.pos_kp = APP_CTRL_POS_KP;
    s_params.pos_kd = APP_CTRL_POS_KD;
    s_params.pos_x_ref_m = APP_CTRL_POS_X_REF_M;
    s_params.pos_v_max_turns_s = APP_CTRL_POS_V_MAX_TURNS_S;
    s_params.wheel_radius_m = APP_WHEEL_RADIUS_M;
    s_params.pos_reset = 0.0f;
    s_params.pos_err_ema_alpha = APP_CTRL_POS_ERR_EMA_ALPHA;
    s_params.pos_ema_kp = APP_CTRL_POS_EMA_KP;
    s_params.outer_mode = (float)APP_CTRL_OUTER_MODE_DEFAULT;
    s_params.heading_kp = APP_CTRL_HEADING_KP;
    s_params.heading_kd = APP_CTRL_HEADING_KD;
    set_yaw_rate_ref(APP_CTRL_HEADING_REF_RAD);
    s_params.heading_torque_max_nm = APP_CTRL_HEADING_TORQUE_MAX_NM;
    s_params.heading_reset = 0.0f;
    s_params.cascade_vel_err_ema_alpha = APP_CTRL_CASCADE_VEL_ERR_EMA_ALPHA;
    s_params.cascade_vel_ema_kp = APP_CTRL_CASCADE_VEL_EMA_KP;
    s_params.vel_ref_slew_turns_s2 = APP_CTRL_VEL_REF_SLEW_TURNS_S2;
    s_params.cascade_vel_accel_kp = APP_CTRL_CASCADE_VEL_ACCEL_KP;
    s_params.friction_mode = (float)APP_CTRL_FRICTION_MODE;
    s_params.friction_static_nm = APP_CTRL_FRICTION_STATIC_NM;
    s_params.friction_kinetic_nm = APP_CTRL_FRICTION_KINETIC_NM;
    s_params.friction_vel_eps_turns_s = APP_CTRL_FRICTION_VEL_EPS_TURNS_S;
    s_params.err_k_vel = APP_CTRL_ERR_K_VEL;
    s_params.heading_ema = APP_CTRL_HEADING_EMA;
    s_params.heading_d_ema = APP_CTRL_HEADING_D_EMA;
    s_params.antipat_enable = (float)APP_CTRL_ANTIPATINAGE_ENABLE;
    s_params.antipat_sync_enable = (float)APP_ANTIPAT_SYNC_ENABLE;
    s_params.antipat_sync_track_width_m = APP_ANTIPAT_SYNC_TRACK_WIDTH_M;
    s_params.antipat_tau_min_nm = APP_ANTIPAT_TAU_MIN_NM;
    s_params.antipat_eta_on = APP_ANTIPAT_ETA_ON;
    s_params.antipat_both_eta_off = APP_ANTIPAT_BOTH_ETA_OFF;
    s_params.antipat_both_alpha_contact_max_rads2 = APP_ANTIPAT_BOTH_ALPHA_CONTACT_MAX_RADS2;
    s_params.antipat_both_tau_steady_air_nm = APP_ANTIPAT_BOTH_TAU_STEADY_AIR_NM;
    s_params.antipat_omega_air_min_turns_s = APP_ANTIPAT_OMEGA_AIR_MIN_TURNS_S;
    s_params.antipat_sync_k_dom = APP_ANTIPAT_SYNC_K_DOM;
    s_params.antipat_sync_eps_abs_rads = APP_ANTIPAT_SYNC_EPS_ABS_RADS;
    s_params.antipat_sync_k_rel = APP_ANTIPAT_SYNC_K_REL;
    s_params.antipat_sync_k_off = APP_ANTIPAT_SYNC_K_OFF;
    s_params.antipat_sync_t_on_ms = (float)APP_ANTIPAT_SYNC_T_ON_MS;
    s_params.antipat_sync_t_off_ms = (float)APP_ANTIPAT_SYNC_T_OFF_MS;
    s_params.antipat_both_t_on_ms = (float)APP_ANTIPAT_BOTH_T_ON_MS;
    s_params.antipat_both_t_off_ms = (float)APP_ANTIPAT_BOTH_T_OFF_MS;
    s_params.antipat_both_t_ma_ms = (float)APP_ANTIPAT_BOTH_T_MA_MS;
    s_params.antipat_t_recover_ms = (float)APP_ANTIPAT_T_RECOVER_MS;
    s_params.antipat_both_u_min_nm = APP_ANTIPAT_BOTH_U_MIN_NM;
    s_params.antipat_both_pitch_rate_min_rads = APP_ANTIPAT_BOTH_PITCH_RATE_MIN_RADS;
    s_params.antipat_sync_k = APP_ANTIPAT_SYNC_K;
    s_params.antipat_sync_tau_max_nm = APP_ANTIPAT_SYNC_TAU_MAX_NM;
    s_params.antipat_both_k_v = APP_ANTIPAT_BOTH_K_V;
    s_params.antipat_both_tau_max_nm = APP_ANTIPAT_BOTH_TAU_MAX_NM;
    s_params.antipat_both_enable = (float)APP_ANTIPAT_BOTH_ENABLE;
    s_params.antipat_tau_ema = APP_ANTIPAT_TAU_EMA;
    s_params.antipat_u_fade_ms = (float)APP_ANTIPAT_U_FADE_MS;
    s_params.antipat_sync_kd = APP_ANTIPAT_SYNC_KD;
    s_params.cascade_vel_dot_src = (float)APP_CTRL_CASCADE_VEL_DOT_SRC;
    s_params.cascade_vel_dot_lpf = APP_CTRL_CASCADE_VEL_DOT_LPF;
    s_pos_reset_req = false;
    s_heading_reset_req = false;
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
    case APP_CTRL_PARAM_CMD_MAX_TORQUE_NM:
    case APP_CTRL_PARAM_CASCADE_VEL_KP:
    case APP_CTRL_PARAM_CASCADE_VEL_KD:
    case APP_CTRL_PARAM_CASCADE_PITCH_REF_MAX_RAD:
        reset_active_strategy();
        return;
    case APP_CTRL_PARAM_OUTER_MODE:
        if (s_params.outer_mode >= 0.5f) {
            s_pos_reset_req = true;
        }
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

bool app_ctrl_params_consume_heading_reset(void)
{
    if (!s_heading_reset_req) {
        return false;
    }
    s_heading_reset_req = false;
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
    case APP_CTRL_PARAM_CMD_MAX_TORQUE_NM:
        *out_value = s_params.cmd_max_torque_nm;
        return true;
    case APP_CTRL_PARAM_CASCADE_VEL_KP:
        *out_value = s_params.cascade_vel_kp;
        return true;
    case APP_CTRL_PARAM_CASCADE_VEL_KD:
        *out_value = s_params.cascade_vel_kd;
        return true;
    case APP_CTRL_PARAM_CASCADE_PITCH_REF_MAX_RAD:
        *out_value = s_params.cascade_pitch_ref_max_rad;
        return true;
    case APP_CTRL_PARAM_MECA_K_GRAV:
        *out_value = s_params.meca_k_grav;
        return true;
    case APP_CTRL_PARAM_ERR_K_PITCH:
        *out_value = s_params.err_k_pitch;
        return true;
    case APP_CTRL_PARAM_MECA_K_PITCH_DAMP:
        *out_value = s_params.meca_k_pitch_damp;
        return true;
    case APP_CTRL_PARAM_BALANCE_OUTPUT_ALPHA:
        *out_value = s_params.balance_output_alpha;
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
    case APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_KP:
        *out_value = s_params.motor_torque_correction_kp;
        return true;
    case APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_MAX_NM:
        *out_value = s_params.motor_torque_correction_max_nm;
        return true;
    case APP_CTRL_PARAM_MOTOR_J:
        *out_value = s_params.motor_J;
        return true;
    case APP_CTRL_PARAM_MOTOR_FRICTION_C:
        *out_value = s_params.motor_friction_c;
        return true;
    case APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_GATE_PITCH_MAX_RAD:
        *out_value = s_params.motor_torque_correction_gate_pitch_max_rad;
        return true;
    case APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_GATE_RATE_MAX_RADS:
        *out_value = s_params.motor_torque_correction_gate_rate_max_rads;
        return true;
    case APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_GATE_VEL_MAX_TURNS_S:
        *out_value = s_params.motor_torque_correction_gate_vel_max_turns_s;
        return true;
    case APP_CTRL_PARAM_MOTOR_ACCEL_LPF:
        *out_value = s_params.motor_accel_lpf;
        return true;
    case APP_CTRL_PARAM_POS_KP:
        *out_value = s_params.pos_kp;
        return true;
    case APP_CTRL_PARAM_POS_KD:
        *out_value = s_params.pos_kd;
        return true;
    case APP_CTRL_PARAM_POS_X_REF_M:
        *out_value = s_params.pos_x_ref_m;
        return true;
    case APP_CTRL_PARAM_POS_V_MAX_TURNS_S:
        *out_value = s_params.pos_v_max_turns_s;
        return true;
    case APP_CTRL_PARAM_WHEEL_RADIUS_M:
        *out_value = s_params.wheel_radius_m;
        return true;
    case APP_CTRL_PARAM_POS_RESET:
        *out_value = 0.0f;
        return true;
    case APP_CTRL_PARAM_POS_ERR_EMA_ALPHA:
        *out_value = s_params.pos_err_ema_alpha;
        return true;
    case APP_CTRL_PARAM_POS_EMA_KP:
        *out_value = s_params.pos_ema_kp;
        return true;
    case APP_CTRL_PARAM_OUTER_MODE:
        *out_value = s_params.outer_mode;
        return true;
    case APP_CTRL_PARAM_HEADING_KP:
        *out_value = s_params.heading_kp;
        return true;
    case APP_CTRL_PARAM_HEADING_KD:
        *out_value = s_params.heading_kd;
        return true;
    case APP_CTRL_PARAM_HEADING_REF_RAD:
        *out_value = s_params.heading_ref_rad;
        return true;
    case APP_CTRL_PARAM_HEADING_TORQUE_MAX_NM:
        *out_value = s_params.heading_torque_max_nm;
        return true;
    case APP_CTRL_PARAM_HEADING_RESET:
        *out_value = 0.0f;
        return true;
    case APP_CTRL_PARAM_CASCADE_VEL_ERR_EMA_ALPHA:
        *out_value = s_params.cascade_vel_err_ema_alpha;
        return true;
    case APP_CTRL_PARAM_CASCADE_VEL_EMA_KP:
        *out_value = s_params.cascade_vel_ema_kp;
        return true;
    case APP_CTRL_PARAM_VEL_REF_SLEW_TURNS_S2:
        *out_value = s_params.vel_ref_slew_turns_s2;
        return true;
    case APP_CTRL_PARAM_CASCADE_VEL_ACCEL_KP:
        *out_value = s_params.cascade_vel_accel_kp;
        return true;
    case APP_CTRL_PARAM_HEADING_INC:
        *out_value = s_params.heading_ref_rad;
        return true;
    case APP_CTRL_PARAM_HEADING_DEC:
        *out_value = 0.0f;
        return true;
    case APP_CTRL_PARAM_FRICTION_MODE:
        *out_value = s_params.friction_mode;
        return true;
    case APP_CTRL_PARAM_FRICTION_STATIC_NM:
        *out_value = s_params.friction_static_nm;
        return true;
    case APP_CTRL_PARAM_FRICTION_KINETIC_NM:
        *out_value = s_params.friction_kinetic_nm;
        return true;
    case APP_CTRL_PARAM_FRICTION_VEL_EPS_TURNS_S:
        *out_value = s_params.friction_vel_eps_turns_s;
        return true;
    case APP_CTRL_PARAM_ERR_K_VEL:
        *out_value = s_params.err_k_vel;
        return true;
    case APP_CTRL_PARAM_HEADING_EMA:
        *out_value = s_params.heading_ema;
        return true;
    case APP_CTRL_PARAM_HEADING_D_EMA:
        *out_value = s_params.heading_d_ema;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_ENABLE:
        *out_value = s_params.antipat_enable;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_ENABLE:
        *out_value = s_params.antipat_sync_enable;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_TRACK_WIDTH_M:
        *out_value = s_params.antipat_sync_track_width_m;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_TAU_MIN_NM:
        *out_value = s_params.antipat_tau_min_nm;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_ETA_ON:
        *out_value = s_params.antipat_eta_on;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_ETA_OFF:
        *out_value = s_params.antipat_both_eta_off;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_ALPHA_CONTACT_MAX_RADS2:
        *out_value = s_params.antipat_both_alpha_contact_max_rads2;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_TAU_STEADY_AIR_NM:
        *out_value = s_params.antipat_both_tau_steady_air_nm;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_OMEGA_AIR_MIN_TURNS_S:
        *out_value = s_params.antipat_omega_air_min_turns_s;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_K_DOM:
        *out_value = s_params.antipat_sync_k_dom;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_EPS_ABS_RADS:
        *out_value = s_params.antipat_sync_eps_abs_rads;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_K_REL:
        *out_value = s_params.antipat_sync_k_rel;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_K_OFF:
        *out_value = s_params.antipat_sync_k_off;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_T_ON_MS:
        *out_value = s_params.antipat_sync_t_on_ms;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_T_OFF_MS:
        *out_value = s_params.antipat_sync_t_off_ms;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_T_ON_MS:
        *out_value = s_params.antipat_both_t_on_ms;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_T_OFF_MS:
        *out_value = s_params.antipat_both_t_off_ms;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_T_MA_MS:
        *out_value = s_params.antipat_both_t_ma_ms;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_T_RECOVER_MS:
        *out_value = s_params.antipat_t_recover_ms;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_U_MIN_NM:
        *out_value = s_params.antipat_both_u_min_nm;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_PITCH_RATE_MIN_RADS:
        *out_value = s_params.antipat_both_pitch_rate_min_rads;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_K:
        *out_value = s_params.antipat_sync_k;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_TAU_MAX_NM:
        *out_value = s_params.antipat_sync_tau_max_nm;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_K_V:
        *out_value = s_params.antipat_both_k_v;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_TAU_MAX_NM:
        *out_value = s_params.antipat_both_tau_max_nm;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_ENABLE:
        *out_value = s_params.antipat_both_enable;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_TAU_EMA:
        *out_value = s_params.antipat_tau_ema;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_U_FADE_MS:
        *out_value = s_params.antipat_u_fade_ms;
        return true;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_KD:
        *out_value = s_params.antipat_sync_kd;
        return true;
    case APP_CTRL_PARAM_CASCADE_VEL_DOT_SRC:
        *out_value = s_params.cascade_vel_dot_src;
        return true;
    case APP_CTRL_PARAM_CASCADE_VEL_DOT_LPF:
        *out_value = s_params.cascade_vel_dot_lpf;
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
    case APP_CTRL_PARAM_CMD_MAX_TORQUE_NM:
        if (value <= 0.0f) {
            return false;
        }
        s_params.cmd_max_torque_nm = value;
        break;
    case APP_CTRL_PARAM_CASCADE_VEL_KP:
        s_params.cascade_vel_kp = value;
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
    case APP_CTRL_PARAM_MECA_K_GRAV:
        s_params.meca_k_grav = value;
        break;
    case APP_CTRL_PARAM_ERR_K_PITCH:
        s_params.err_k_pitch = value;
        break;
    case APP_CTRL_PARAM_MECA_K_PITCH_DAMP:
        s_params.meca_k_pitch_damp = value;
        break;
    case APP_CTRL_PARAM_BALANCE_OUTPUT_ALPHA:
        if (value < 0.0f || value > 1.0f) {
            return false;
        }
        s_params.balance_output_alpha = value;
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
    case APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_KP:
        if (value < 0.0f) {
            return false;
        }
        s_params.motor_torque_correction_kp = value;
        break;
    case APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_MAX_NM:
        if (value < 0.0f) {
            return false;
        }
        s_params.motor_torque_correction_max_nm = value;
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
    case APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_GATE_PITCH_MAX_RAD:
        if (value < 0.0f) {
            return false;
        }
        s_params.motor_torque_correction_gate_pitch_max_rad = value;
        break;
    case APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_GATE_RATE_MAX_RADS:
        if (value < 0.0f) {
            return false;
        }
        s_params.motor_torque_correction_gate_rate_max_rads = value;
        break;
    case APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_GATE_VEL_MAX_TURNS_S:
        if (value < 0.0f) {
            return false;
        }
        s_params.motor_torque_correction_gate_vel_max_turns_s = value;
        break;
    case APP_CTRL_PARAM_MOTOR_ACCEL_LPF:
        if (value < 0.0f || value >= 1.0f) {
            return false;
        }
        s_params.motor_accel_lpf = value;
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
    case APP_CTRL_PARAM_POS_X_REF_M:
        s_params.pos_x_ref_m = value;
        break;
    case APP_CTRL_PARAM_POS_V_MAX_TURNS_S:
        if (value < 0.0f) {
            return false;
        }
        s_params.pos_v_max_turns_s = value;
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
    case APP_CTRL_PARAM_POS_ERR_EMA_ALPHA:
        if (value < 0.0f || value >= 1.0f) {
            return false;
        }
        s_params.pos_err_ema_alpha = value;
        break;
    case APP_CTRL_PARAM_POS_EMA_KP:
        if (value < 0.0f) {
            return false;
        }
        s_params.pos_ema_kp = value;
        break;
    case APP_CTRL_PARAM_OUTER_MODE: {
        const uint32_t mode = (value >= 0.5f) ? APP_CTRL_OUTER_MODE_POS
                                              : APP_CTRL_OUTER_MODE_VEL;
        s_params.outer_mode = (float)mode;
        break;
    }
    case APP_CTRL_PARAM_HEADING_KP:
        s_params.heading_kp = value;
        break;
    case APP_CTRL_PARAM_HEADING_KD:
        if (value < 0.0f) {
            return false;
        }
        s_params.heading_kd = value;
        break;
    case APP_CTRL_PARAM_HEADING_REF_RAD:
        set_yaw_rate_ref(value);
        break;
    case APP_CTRL_PARAM_HEADING_TORQUE_MAX_NM:
        if (value < 0.0f) {
            return false;
        }
        s_params.heading_torque_max_nm = value;
        break;
    case APP_CTRL_PARAM_HEADING_RESET:
        s_heading_reset_req = true;
        set_yaw_rate_ref(0.0f);
        s_params.heading_reset = 0.0f;
        break;
    case APP_CTRL_PARAM_CASCADE_VEL_ERR_EMA_ALPHA:
        if (value < 0.0f || value >= 1.0f) {
            return false;
        }
        s_params.cascade_vel_err_ema_alpha = value;
        break;
    case APP_CTRL_PARAM_CASCADE_VEL_EMA_KP:
        if (value < 0.0f) {
            return false;
        }
        s_params.cascade_vel_ema_kp = value;
        break;
    case APP_CTRL_PARAM_VEL_REF_SLEW_TURNS_S2:
        if (value < 0.0f) {
            return false;
        }
        s_params.vel_ref_slew_turns_s2 = value;
        break;
    case APP_CTRL_PARAM_CASCADE_VEL_ACCEL_KP:
        if (value < 0.0f) {
            return false;
        }
        s_params.cascade_vel_accel_kp = value;
        break;
    case APP_CTRL_PARAM_HEADING_INC:
        set_yaw_rate_ref(value);
        break;
    case APP_CTRL_PARAM_HEADING_DEC:
        /* Alias: ψ̇_ref = −value (old UI). */
        set_yaw_rate_ref(-value);
        break;
    case APP_CTRL_PARAM_FRICTION_MODE:
        if (value < 0.0f || value > 1.0f) {
            return false;
        }
        s_params.friction_mode = value;
        break;
    case APP_CTRL_PARAM_FRICTION_STATIC_NM:
        if (value < 0.0f) {
            return false;
        }
        s_params.friction_static_nm = value;
        break;
    case APP_CTRL_PARAM_FRICTION_KINETIC_NM:
        if (value < 0.0f) {
            return false;
        }
        s_params.friction_kinetic_nm = value;
        break;
    case APP_CTRL_PARAM_FRICTION_VEL_EPS_TURNS_S:
        if (value <= 0.0f) {
            return false;
        }
        s_params.friction_vel_eps_turns_s = value;
        break;
    case APP_CTRL_PARAM_ERR_K_VEL:
        s_params.err_k_vel = value;
        break;
    case APP_CTRL_PARAM_HEADING_EMA:
        if (value < 0.0f || value >= 1.0f) {
            return false;
        }
        s_params.heading_ema = value;
        break;
    case APP_CTRL_PARAM_HEADING_D_EMA:
        if (value < 0.0f || value >= 1.0f) {
            return false;
        }
        s_params.heading_d_ema = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_ENABLE:
        if (value < 0.0f || value > 1.0f) {
            return false;
        }
        s_params.antipat_enable = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_ENABLE:
        if (value < 0.0f || value > 1.0f) {
            return false;
        }
        s_params.antipat_sync_enable = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_TRACK_WIDTH_M:
        if (value <= 0.0f) {
            return false;
        }
        s_params.antipat_sync_track_width_m = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_TAU_MIN_NM:
        if (value <= 0.0f) {
            return false;
        }
        s_params.antipat_tau_min_nm = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_ETA_ON:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_eta_on = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_ETA_OFF:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_both_eta_off = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_ALPHA_CONTACT_MAX_RADS2:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_both_alpha_contact_max_rads2 = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_TAU_STEADY_AIR_NM:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_both_tau_steady_air_nm = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_OMEGA_AIR_MIN_TURNS_S:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_omega_air_min_turns_s = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_K_DOM:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_sync_k_dom = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_EPS_ABS_RADS:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_sync_eps_abs_rads = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_K_REL:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_sync_k_rel = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_K_OFF:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_sync_k_off = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_T_ON_MS:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_sync_t_on_ms = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_T_OFF_MS:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_sync_t_off_ms = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_T_ON_MS:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_both_t_on_ms = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_T_OFF_MS:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_both_t_off_ms = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_T_MA_MS:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_both_t_ma_ms = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_T_RECOVER_MS:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_t_recover_ms = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_U_MIN_NM:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_both_u_min_nm = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_PITCH_RATE_MIN_RADS:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_both_pitch_rate_min_rads = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_K:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_sync_k = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_TAU_MAX_NM:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_sync_tau_max_nm = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_K_V:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_both_k_v = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_TAU_MAX_NM:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_both_tau_max_nm = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_BOTH_ENABLE:
        if (value < 0.0f || value > 1.0f) {
            return false;
        }
        s_params.antipat_both_enable = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_TAU_EMA:
        if (value < 0.0f || value >= 1.0f) {
            return false;
        }
        s_params.antipat_tau_ema = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_U_FADE_MS:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_u_fade_ms = value;
        break;
    case APP_CTRL_PARAM_ANTIPAT_SYNC_KD:
        if (value < 0.0f) {
            return false;
        }
        s_params.antipat_sync_kd = value;
        break;
    case APP_CTRL_PARAM_CASCADE_VEL_DOT_SRC:
        if (value < 0.0f || value > 1.0f) {
            return false;
        }
        s_params.cascade_vel_dot_src = (value >= 0.5f) ? 1.0f : 0.0f;
        break;
    case APP_CTRL_PARAM_CASCADE_VEL_DOT_LPF:
        if (value < 0.0f || value >= 1.0f) {
            return false;
        }
        s_params.cascade_vel_dot_lpf = value;
        break;
    default:
        return false;
    }

    apply_side_effects(param_id);

    if (out_value != NULL) {
        if (param_id == (uint16_t)APP_CTRL_PARAM_HEADING_INC ||
            param_id == (uint16_t)APP_CTRL_PARAM_HEADING_DEC) {
            *out_value = s_params.heading_ref_rad;
        } else {
            (void)app_ctrl_params_get_value(param_id, out_value);
        }
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
        [APP_CTRL_PARAM_CMD_MAX_TORQUE_NM] = "cmd_max_torque_nm",
        [APP_CTRL_PARAM_CASCADE_VEL_KP] = "cascade_vel_kp",
        [APP_CTRL_PARAM_CASCADE_VEL_KD] = "cascade_vel_kd",
        [APP_CTRL_PARAM_CASCADE_PITCH_REF_MAX_RAD] = "cascade_pitch_ref_max_rad",
        [APP_CTRL_PARAM_MECA_K_GRAV] = "meca_k_grav",
        [APP_CTRL_PARAM_ERR_K_PITCH] = "err_k_pitch",
        [APP_CTRL_PARAM_MECA_K_PITCH_DAMP] = "meca_k_pitch_damp",
        [APP_CTRL_PARAM_BALANCE_OUTPUT_ALPHA] = "balance_output_alpha",
        [APP_CTRL_PARAM_WHEEL_ENCODER_VEL_LPF_ALPHA] = "wheel_encoder_vel_lpf_alpha",
        [APP_CTRL_PARAM_TORQUE_DEADBAND_NM] = "torque_deadband_nm",
        [APP_CTRL_PARAM_TORQUE_DEADBAND_PITCH_MAX_RAD] = "torque_deadband_pitch_max_rad",
        [APP_CTRL_PARAM_TORQUE_DEADBAND_RATE_MAX_RADS] = "torque_deadband_rate_max_rads",
        [APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_KP] = "motor_torque_correction_kp",
        [APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_MAX_NM] = "motor_torque_correction_max_nm",
        [APP_CTRL_PARAM_MOTOR_J] = "motor_J",
        [APP_CTRL_PARAM_MOTOR_FRICTION_C] = "motor_friction_c",
        [APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_GATE_PITCH_MAX_RAD] = "motor_torque_correction_gate_pitch_max_rad",
        [APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_GATE_RATE_MAX_RADS] = "motor_torque_correction_gate_rate_max_rads",
        [APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_GATE_VEL_MAX_TURNS_S] = "motor_torque_correction_gate_vel_max_turns_s",
        [APP_CTRL_PARAM_MOTOR_ACCEL_LPF] = "motor_accel_lpf",
        [APP_CTRL_PARAM_POS_KP] = "pos_kp",
        [APP_CTRL_PARAM_POS_KD] = "pos_kd",
        [APP_CTRL_PARAM_POS_X_REF_M] = "pos_x_ref_m",
        [APP_CTRL_PARAM_POS_V_MAX_TURNS_S] = "pos_v_max_turns_s",
        [APP_CTRL_PARAM_WHEEL_RADIUS_M] = "wheel_radius_m",
        [APP_CTRL_PARAM_POS_RESET] = "pos_reset",
        [APP_CTRL_PARAM_POS_ERR_EMA_ALPHA] = "pos_err_ema_alpha",
        [APP_CTRL_PARAM_POS_EMA_KP] = "pos_ema_kp",
        [APP_CTRL_PARAM_OUTER_MODE] = "outer_mode",
        [APP_CTRL_PARAM_HEADING_KP] = "heading_kp",
        [APP_CTRL_PARAM_HEADING_KD] = "heading_kd",
        [APP_CTRL_PARAM_HEADING_REF_RAD] = "heading_ref_rad",
        [APP_CTRL_PARAM_HEADING_TORQUE_MAX_NM] = "heading_torque_max_nm",
        [APP_CTRL_PARAM_HEADING_RESET] = "heading_reset",
        [APP_CTRL_PARAM_CASCADE_VEL_ERR_EMA_ALPHA] = "cascade_vel_err_ema_alpha",
        [APP_CTRL_PARAM_CASCADE_VEL_EMA_KP] = "cascade_vel_ema_kp",
        [APP_CTRL_PARAM_VEL_REF_SLEW_TURNS_S2] = "vel_ref_slew_turns_s2",
        [APP_CTRL_PARAM_CASCADE_VEL_ACCEL_KP] = "cascade_vel_accel_kp",
        [APP_CTRL_PARAM_HEADING_INC] = "heading_inc",
        [APP_CTRL_PARAM_HEADING_DEC] = "heading_dec",
        [APP_CTRL_PARAM_FRICTION_MODE] = "friction_mode",
        [APP_CTRL_PARAM_FRICTION_STATIC_NM] = "friction_static_nm",
        [APP_CTRL_PARAM_FRICTION_KINETIC_NM] = "friction_kinetic_nm",
        [APP_CTRL_PARAM_FRICTION_VEL_EPS_TURNS_S] = "friction_vel_eps_turns_s",
        [APP_CTRL_PARAM_ERR_K_VEL] = "err_k_vel",
        [APP_CTRL_PARAM_HEADING_EMA] = "heading_ema",
        [APP_CTRL_PARAM_HEADING_D_EMA] = "heading_d_ema",
        [APP_CTRL_PARAM_ANTIPAT_ENABLE] = "antipat_enable",
        [APP_CTRL_PARAM_ANTIPAT_SYNC_ENABLE] = "antipat_sync_enable",
        [APP_CTRL_PARAM_ANTIPAT_SYNC_TRACK_WIDTH_M] = "antipat_sync_track_width_m",
        [APP_CTRL_PARAM_ANTIPAT_TAU_MIN_NM] = "antipat_tau_min_nm",
        [APP_CTRL_PARAM_ANTIPAT_ETA_ON] = "antipat_eta_on",
        [APP_CTRL_PARAM_ANTIPAT_BOTH_ETA_OFF] = "antipat_both_eta_off",
        [APP_CTRL_PARAM_ANTIPAT_BOTH_ALPHA_CONTACT_MAX_RADS2] = "antipat_both_alpha_contact_max_rads2",
        [APP_CTRL_PARAM_ANTIPAT_BOTH_TAU_STEADY_AIR_NM] = "antipat_both_tau_steady_air_nm",
        [APP_CTRL_PARAM_ANTIPAT_OMEGA_AIR_MIN_TURNS_S] = "antipat_omega_air_min_turns_s",
        [APP_CTRL_PARAM_ANTIPAT_SYNC_K_DOM] = "antipat_sync_k_dom",
        [APP_CTRL_PARAM_ANTIPAT_SYNC_EPS_ABS_RADS] = "antipat_sync_eps_abs_rads",
        [APP_CTRL_PARAM_ANTIPAT_SYNC_K_REL] = "antipat_sync_k_rel",
        [APP_CTRL_PARAM_ANTIPAT_SYNC_K_OFF] = "antipat_sync_k_off",
        [APP_CTRL_PARAM_ANTIPAT_SYNC_T_ON_MS] = "antipat_sync_t_on_ms",
        [APP_CTRL_PARAM_ANTIPAT_SYNC_T_OFF_MS] = "antipat_sync_t_off_ms",
        [APP_CTRL_PARAM_ANTIPAT_BOTH_T_ON_MS] = "antipat_both_t_on_ms",
        [APP_CTRL_PARAM_ANTIPAT_BOTH_T_OFF_MS] = "antipat_both_t_off_ms",
        [APP_CTRL_PARAM_ANTIPAT_BOTH_T_MA_MS] = "antipat_both_t_ma_ms",
        [APP_CTRL_PARAM_ANTIPAT_T_RECOVER_MS] = "antipat_t_recover_ms",
        [APP_CTRL_PARAM_ANTIPAT_BOTH_U_MIN_NM] = "antipat_both_u_min_nm",
        [APP_CTRL_PARAM_ANTIPAT_BOTH_PITCH_RATE_MIN_RADS] = "antipat_both_pitch_rate_min_rads",
        [APP_CTRL_PARAM_ANTIPAT_SYNC_K] = "antipat_sync_k",
        [APP_CTRL_PARAM_ANTIPAT_SYNC_TAU_MAX_NM] = "antipat_sync_tau_max_nm",
        [APP_CTRL_PARAM_ANTIPAT_BOTH_K_V] = "antipat_both_k_v",
        [APP_CTRL_PARAM_ANTIPAT_BOTH_TAU_MAX_NM] = "antipat_both_tau_max_nm",
        [APP_CTRL_PARAM_ANTIPAT_BOTH_ENABLE] = "antipat_both_enable",
        [APP_CTRL_PARAM_ANTIPAT_TAU_EMA] = "antipat_tau_ema",
        [APP_CTRL_PARAM_ANTIPAT_U_FADE_MS] = "antipat_u_fade_ms",
        [APP_CTRL_PARAM_ANTIPAT_SYNC_KD] = "antipat_sync_kd",
        [APP_CTRL_PARAM_CASCADE_VEL_DOT_SRC] = "cascade_vel_dot_src",
        [APP_CTRL_PARAM_CASCADE_VEL_DOT_LPF] = "cascade_vel_dot_lpf",
    };

    if (param_id >= (uint16_t)APP_CTRL_PARAM_COUNT) {
        return "?";
    }
    return names[param_id];
}
