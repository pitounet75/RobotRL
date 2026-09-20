/**
 * @file app_ctrl_params.h
 * @brief Mutable control gains/refs (defaults from app_config.h, tunable over telemetry).
 */
#ifndef APP_CTRL_PARAMS_H
#define APP_CTRL_PARAMS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* GET payload starts with this u32. Bump together with SNAPSHOT_VERSION
 * (TelemetryServer/telemetry/ctrl_params.py) when appending a packed float.
 * SET is param_id+f32 only — no version on the wire. */
#define APP_CTRL_PARAMS_SNAPSHOT_VERSION 19u

/** Outer loop: 0 = velocity (vel_ref), 1 = position (x → v_ref). Mutually exclusive. */
#define APP_CTRL_OUTER_MODE_VEL 0u
#define APP_CTRL_OUTER_MODE_POS 1u

typedef enum {
    APP_CTRL_PARAM_STRATEGY = 0,
    APP_CTRL_PARAM_PITCH_REF_RAD,
    APP_CTRL_PARAM_VEL_REF_TURNS_S,
    APP_CTRL_PARAM_PITCH_FAILSAFE_RAD,
    APP_CTRL_PARAM_CMD_MAX_TORQUE_NM,
    APP_CTRL_PARAM_CASCADE_VEL_KP,
    APP_CTRL_PARAM_CASCADE_VEL_KD,
    APP_CTRL_PARAM_CASCADE_PITCH_REF_MAX_RAD,
    APP_CTRL_PARAM_MECA_K_GRAV,
    APP_CTRL_PARAM_ERR_K_PITCH,
    APP_CTRL_PARAM_MECA_K_PITCH_DAMP,
    APP_CTRL_PARAM_BALANCE_OUTPUT_ALPHA,
    APP_CTRL_PARAM_WHEEL_ENCODER_VEL_LPF_ALPHA,
    APP_CTRL_PARAM_TORQUE_DEADBAND_NM,
    APP_CTRL_PARAM_TORQUE_DEADBAND_PITCH_MAX_RAD,
    APP_CTRL_PARAM_TORQUE_DEADBAND_RATE_MAX_RADS,
    APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_KP,
    APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_MAX_NM,
    APP_CTRL_PARAM_MOTOR_J,
    APP_CTRL_PARAM_MOTOR_FRICTION_C,
    APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_GATE_PITCH_MAX_RAD,
    APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_GATE_RATE_MAX_RADS,
    APP_CTRL_PARAM_MOTOR_TORQUE_CORRECTION_GATE_VEL_MAX_TURNS_S,
    APP_CTRL_PARAM_MOTOR_ACCEL_LPF,
    APP_CTRL_PARAM_POS_KP,
    APP_CTRL_PARAM_POS_KD,
    APP_CTRL_PARAM_POS_X_REF_M,
    APP_CTRL_PARAM_POS_V_MAX_TURNS_S,
    APP_CTRL_PARAM_WHEEL_RADIUS_M,
    APP_CTRL_PARAM_POS_RESET,
    APP_CTRL_PARAM_POS_ERR_EMA_ALPHA,
    APP_CTRL_PARAM_POS_EMA_KP,
    APP_CTRL_PARAM_OUTER_MODE,
    APP_CTRL_PARAM_HEADING_KP,
    APP_CTRL_PARAM_HEADING_KD,
    APP_CTRL_PARAM_HEADING_REF_RAD,
    APP_CTRL_PARAM_HEADING_TORQUE_MAX_NM,
    APP_CTRL_PARAM_HEADING_RESET,
    APP_CTRL_PARAM_CASCADE_VEL_ERR_EMA_ALPHA,
    APP_CTRL_PARAM_CASCADE_VEL_EMA_KP,
    APP_CTRL_PARAM_VEL_REF_SLEW_TURNS_S2,
    APP_CTRL_PARAM_CASCADE_VEL_ACCEL_KP,
    APP_CTRL_PARAM_HEADING_INC,
    APP_CTRL_PARAM_HEADING_DEC,
    APP_CTRL_PARAM_FRICTION_MODE,
    APP_CTRL_PARAM_FRICTION_STATIC_NM,
    APP_CTRL_PARAM_FRICTION_KINETIC_NM,
    APP_CTRL_PARAM_FRICTION_VEL_EPS_TURNS_S,
    APP_CTRL_PARAM_ERR_K_VEL,
    APP_CTRL_PARAM_HEADING_EMA,
    APP_CTRL_PARAM_HEADING_D_EMA,
    APP_CTRL_PARAM_ANTIPAT_ENABLE,
    APP_CTRL_PARAM_ANTIPAT_SYNC_ENABLE,
    APP_CTRL_PARAM_ANTIPAT_SYNC_TRACK_WIDTH_M,
    APP_CTRL_PARAM_ANTIPAT_TAU_MIN_NM,
    APP_CTRL_PARAM_ANTIPAT_ETA_ON,
    APP_CTRL_PARAM_ANTIPAT_BOTH_ETA_OFF,
    APP_CTRL_PARAM_ANTIPAT_BOTH_ALPHA_CONTACT_MAX_RADS2,
    APP_CTRL_PARAM_ANTIPAT_BOTH_TAU_STEADY_AIR_NM,
    APP_CTRL_PARAM_ANTIPAT_OMEGA_AIR_MIN_TURNS_S,
    APP_CTRL_PARAM_ANTIPAT_SYNC_K_DOM,
    APP_CTRL_PARAM_ANTIPAT_SYNC_EPS_ABS_RADS,
    APP_CTRL_PARAM_ANTIPAT_SYNC_K_REL,
    APP_CTRL_PARAM_ANTIPAT_SYNC_K_OFF,
    APP_CTRL_PARAM_ANTIPAT_SYNC_T_ON_MS,
    APP_CTRL_PARAM_ANTIPAT_SYNC_T_OFF_MS,
    APP_CTRL_PARAM_ANTIPAT_BOTH_T_ON_MS,
    APP_CTRL_PARAM_ANTIPAT_BOTH_T_OFF_MS,
    APP_CTRL_PARAM_ANTIPAT_BOTH_T_MA_MS,
    APP_CTRL_PARAM_ANTIPAT_T_RECOVER_MS,
    APP_CTRL_PARAM_ANTIPAT_BOTH_U_MIN_NM,
    APP_CTRL_PARAM_ANTIPAT_BOTH_PITCH_RATE_MIN_RADS,
    APP_CTRL_PARAM_ANTIPAT_SYNC_K,
    APP_CTRL_PARAM_ANTIPAT_SYNC_TAU_MAX_NM,
    APP_CTRL_PARAM_ANTIPAT_BOTH_K_V,
    APP_CTRL_PARAM_ANTIPAT_BOTH_TAU_MAX_NM,
    APP_CTRL_PARAM_ANTIPAT_BOTH_ENABLE,
    APP_CTRL_PARAM_ANTIPAT_TAU_EMA,
    APP_CTRL_PARAM_ANTIPAT_U_FADE_MS,
    APP_CTRL_PARAM_ANTIPAT_SYNC_KD,
    APP_CTRL_PARAM_COUNT
} app_ctrl_param_id_t;

/** Wire snapshot for GET (little-endian, packed). */
typedef struct __attribute__((packed)) {
    uint32_t version;
    uint32_t strategy_id;
    float pitch_ref_rad;
    float vel_ref_turns_s;
    float pitch_failsafe_rad;
    float cmd_max_torque_nm;
    float cascade_vel_kp;
    float cascade_vel_kd;
    float cascade_pitch_ref_max_rad;
    float meca_k_grav;
    float err_k_pitch;
    float meca_k_pitch_damp;
    float balance_output_alpha;
    float wheel_encoder_vel_lpf_alpha;
    float torque_deadband_nm;
    float torque_deadband_pitch_max_rad;
    float torque_deadband_rate_max_rads;
    float motor_torque_correction_kp;
    float motor_torque_correction_max_nm;
    float motor_J;
    float motor_friction_c;
    float motor_torque_correction_gate_pitch_max_rad;
    float motor_torque_correction_gate_rate_max_rads;
    float motor_torque_correction_gate_vel_max_turns_s;
    float motor_accel_lpf;
    float pos_kp;
    float pos_kd;
    float pos_x_ref_m;
    float pos_v_max_turns_s;
    float wheel_radius_m;
    float pos_reset;
    float pos_err_ema_alpha;
    float pos_ema_kp;
    float outer_mode;
    float heading_kp;
    float heading_kd;
    float heading_ref_rad;
    float heading_torque_max_nm;
    float heading_reset;
    float cascade_vel_err_ema_alpha;
    float cascade_vel_ema_kp;
    float vel_ref_slew_turns_s2;
    float cascade_vel_accel_kp;
    float heading_inc;
    float heading_dec;
    float friction_mode;
    float friction_static_nm;
    float friction_kinetic_nm;
    float friction_vel_eps_turns_s;
    float err_k_vel;
    float heading_ema;
    float heading_d_ema;
    float antipat_enable;
    float antipat_sync_enable;
    float antipat_sync_track_width_m;
    float antipat_tau_min_nm;
    float antipat_eta_on;
    float antipat_both_eta_off;
    float antipat_both_alpha_contact_max_rads2;
    float antipat_both_tau_steady_air_nm;
    float antipat_omega_air_min_turns_s;
    float antipat_sync_k_dom;
    float antipat_sync_eps_abs_rads;
    float antipat_sync_k_rel;
    float antipat_sync_k_off;
    float antipat_sync_t_on_ms;
    float antipat_sync_t_off_ms;
    float antipat_both_t_on_ms;
    float antipat_both_t_off_ms;
    float antipat_both_t_ma_ms;
    float antipat_t_recover_ms;
    float antipat_both_u_min_nm;
    float antipat_both_pitch_rate_min_rads;
    float antipat_sync_k;
    float antipat_sync_tau_max_nm;
    float antipat_both_k_v;
    float antipat_both_tau_max_nm;
    float antipat_both_enable;
    float antipat_tau_ema;
    float antipat_u_fade_ms;
    float antipat_sync_kd;
} app_ctrl_params_snapshot_t;

_Static_assert(sizeof(app_ctrl_params_snapshot_t) ==
                   8u + 4u * ((unsigned)APP_CTRL_PARAM_COUNT - 1u),
               "snapshot size must match version + strategy_id + floats id 1..COUNT-1");

void app_ctrl_params_init(void);
const app_ctrl_params_snapshot_t *app_ctrl_params_snapshot(void);
bool app_ctrl_params_set(uint16_t param_id, float value, float *out_value);
bool app_ctrl_params_get_value(uint16_t param_id, float *out_value);
const char *app_ctrl_params_name(uint16_t param_id);

bool app_ctrl_params_consume_pos_reset(void);
bool app_ctrl_params_consume_heading_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CTRL_PARAMS_H */
