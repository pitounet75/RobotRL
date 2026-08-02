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

#define APP_CTRL_PARAMS_SNAPSHOT_VERSION 1u

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
} app_ctrl_params_snapshot_t;

void app_ctrl_params_init(void);
const app_ctrl_params_snapshot_t *app_ctrl_params_snapshot(void);
bool app_ctrl_params_set(uint16_t param_id, float value, float *out_value);
bool app_ctrl_params_get_value(uint16_t param_id, float *out_value);
const char *app_ctrl_params_name(uint16_t param_id);

#ifdef __cplusplus
}
#endif

#endif /* APP_CTRL_PARAMS_H */
