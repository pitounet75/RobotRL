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

#define APP_CTRL_PARAMS_SNAPSHOT_VERSION 11u

/** Outer loop: 0 = velocity (vel_ref), 1 = position (x → v_ref). Mutually exclusive. */
#define APP_CTRL_OUTER_MODE_VEL 0u
#define APP_CTRL_OUTER_MODE_POS 1u

typedef enum {
    APP_CTRL_PARAM_STRATEGY = 0,
    APP_CTRL_PARAM_PITCH_REF_RAD,
    APP_CTRL_PARAM_VEL_REF_TURNS_S,
    APP_CTRL_PARAM_PITCH_FAILSAFE_RAD,
    APP_CTRL_PARAM_PITCH_KP,
    APP_CTRL_PARAM_PITCH_KI,
    APP_CTRL_PARAM_PITCH_KD,
    APP_CTRL_PARAM_VEL_KP,
    APP_CTRL_PARAM_VEL_KI,
    APP_CTRL_PARAM_VEL_KD,
    APP_CTRL_PARAM_CMD_MAX_TORQUE_NM,
    APP_CTRL_PARAM_LINEAR_THETA_FUNC,
    APP_CTRL_PARAM_LINEAR_K_PITCH,
    APP_CTRL_PARAM_LINEAR_K_PITCH_RATE,
    APP_CTRL_PARAM_LINEAR_K_VEL,
    APP_CTRL_PARAM_LINEAR_OUTPUT_ALPHA,
    APP_CTRL_PARAM_CASCADE_VEL_KP,
    APP_CTRL_PARAM_CASCADE_VEL_KI,
    APP_CTRL_PARAM_CASCADE_VEL_KD,
    APP_CTRL_PARAM_CASCADE_PITCH_REF_MAX_RAD,
    APP_CTRL_PARAM_FF_GRAV_K,
    APP_CTRL_PARAM_FF_FB_K_PITCH,
    APP_CTRL_PARAM_FF_FB_K_RATE,
    APP_CTRL_PARAM_FF_OUTPUT_ALPHA,
    APP_CTRL_PARAM_WHEEL_ENCODER_VEL_LPF_ALPHA,
    APP_CTRL_PARAM_TORQUE_DEADBAND_NM,
    APP_CTRL_PARAM_TORQUE_DEADBAND_PITCH_MAX_RAD,
    APP_CTRL_PARAM_TORQUE_DEADBAND_RATE_MAX_RADS,
    APP_CTRL_PARAM_ALPHA_KP,
    APP_CTRL_PARAM_ALPHA_MAX_NM,
    APP_CTRL_PARAM_MOTOR_J,
    APP_CTRL_PARAM_MOTOR_FRICTION_C,
    APP_CTRL_PARAM_ALPHA_PITCH_MAX_RAD,
    APP_CTRL_PARAM_ALPHA_RATE_MAX_RADS,
    APP_CTRL_PARAM_ALPHA_VEL_MAX_TURNS_S,
    APP_CTRL_PARAM_ALPHA_LPF,
    APP_CTRL_PARAM_POS_KP,
    APP_CTRL_PARAM_POS_KD,
    APP_CTRL_PARAM_POS_PITCH_KP,
    APP_CTRL_PARAM_POS_X_REF_M,
    APP_CTRL_PARAM_POS_V_MAX_TURNS_S,
    APP_CTRL_PARAM_POS_PITCH_MAX_RAD,
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
    APP_CTRL_PARAM_COUNT
} app_ctrl_param_id_t;

/** Wire snapshot for GET (little-endian, packed). Keep layout stable across firmware versions. */
typedef struct __attribute__((packed)) {
    uint32_t version;
    uint32_t strategy_id;
    float pitch_ref_rad;
    float vel_ref_turns_s;
    float pitch_failsafe_rad;
    float pitch_kp;
    float pitch_ki;
    float pitch_kd;
    float vel_kp;
    float vel_ki;
    float vel_kd;
    float cmd_max_torque_nm;
    uint32_t linear_theta_func;
    float linear_k_pitch;
    float linear_k_pitch_rate;
    float linear_k_vel;
    float linear_output_alpha;
    float cascade_vel_kp;
    float cascade_vel_ki;
    float cascade_vel_kd;
    float cascade_pitch_ref_max_rad;
    float ff_grav_k;
    float ff_fb_k_pitch;
    float ff_fb_k_rate;
    float ff_output_alpha;
    float wheel_encoder_vel_lpf_alpha;
    float torque_deadband_nm;
    float torque_deadband_pitch_max_rad;
    float torque_deadband_rate_max_rads;
    float alpha_kp;
    float alpha_max_nm;
    float motor_J;
    float motor_friction_c;
    float alpha_pitch_max_rad;
    float alpha_rate_max_rads;
    float alpha_vel_max_turns_s;
    float alpha_lpf;
    float pos_kp;
    float pos_kd;
    float pos_pitch_kp;
    float pos_x_ref_m;
    float pos_v_max_turns_s;
    float pos_pitch_max_rad;
    float wheel_radius_m;
    float pos_reset; /* GET always 0; SET any value requests x zero */
    float pos_err_ema_alpha;
    float pos_ema_kp;
    float outer_mode; /* 0=vel, 1=pos (stored float for SET/GET) */
    float heading_kp;
    float heading_kd;
    float heading_ref_rad;
    float heading_torque_max_nm;
    float heading_reset; /* GET always 0; SET pulses ψ zero */
    float cascade_vel_err_ema_alpha; /* leaky vel-error filter (v8) */
    float cascade_vel_ema_kp;        /* rad / (turn/s of e_f) */
    float vel_ref_slew_turns_s2;     /* |d v_ref/dt| limit; 0=off (v9) */
    float cascade_vel_accel_kp;      /* lean FF on v̇_ref (v9) */
    float heading_inc;               /* GET 0; SET += |value| to heading_ref (v10) */
    float heading_dec;               /* GET 0; SET -= |value| to heading_ref (v10) */
    float friction_mode;             /* 0=legacy deadband, 1=two-level (v11) */
    float friction_static_nm;        /* |ω|≤ε: +sign(u)*static */
    float friction_kinetic_nm;       /* |ω|>ε: +sign(ω)*kinetic */
    float friction_vel_eps_turns_s;  /* static/kinetic boundary (wheel ω) */
} app_ctrl_params_snapshot_t;

void app_ctrl_params_init(void);
const app_ctrl_params_snapshot_t *app_ctrl_params_snapshot(void);
bool app_ctrl_params_set(uint16_t param_id, float value, float *out_value);
bool app_ctrl_params_get_value(uint16_t param_id, float *out_value);
const char *app_ctrl_params_name(uint16_t param_id);

/** True once per SET pos_reset; cleared when consumed by the position loop. */
bool app_ctrl_params_consume_pos_reset(void);
/** True once per SET heading_reset; cleared when consumed by heading hold. */
bool app_ctrl_params_consume_heading_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CTRL_PARAMS_H */
