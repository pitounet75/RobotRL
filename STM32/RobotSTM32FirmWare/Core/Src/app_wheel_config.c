/**
 * @file app_wheel_config.c
 */

#include "app_wheel_config.h"

#include "app_config.h"
#include "fdcan.h"

const app_wheel_config_t g_app_wheel_config[APP_WHEEL_COUNT] = {
    [APP_WHEEL_LEFT] = {
        .wheel = APP_WHEEL_LEFT,
        .odrive_can = &ODRIVE_CAN_LEFT_HANDLE,
        .odrive_node_id = APP_ODRIVE_LEFT_NODE_ID,
        .odrive_drive_idx = 0u,
        .local_encoder = WHEEL_ENCODER_TIM2,
        .odrive_cmd_sign = APP_WHEEL_LEFT_ODRIVE_CMD_SIGN,
        .odrive_feedback_sign = APP_WHEEL_LEFT_ODRIVE_FEEDBACK_SIGN,
        .local_encoder_sign = APP_WHEEL_LEFT_ENCODER_SIGN,
    },
    [APP_WHEEL_RIGHT] = {
        .wheel = APP_WHEEL_RIGHT,
        .odrive_can = &ODRIVE_CAN_RIGHT_HANDLE,
        .odrive_node_id = APP_ODRIVE_RIGHT_NODE_ID,
        .odrive_drive_idx = 1u,
        .local_encoder = WHEEL_ENCODER_TIM4,
        .odrive_cmd_sign = APP_WHEEL_RIGHT_ODRIVE_CMD_SIGN,
        .odrive_feedback_sign = APP_WHEEL_RIGHT_ODRIVE_FEEDBACK_SIGN,
        .local_encoder_sign = APP_WHEEL_RIGHT_ENCODER_SIGN,
    },
};

const app_wheel_config_t *app_wheel_config_get(app_wheel_id_t wheel)
{
    if ((uint32_t)wheel >= APP_WHEEL_COUNT) {
        return 0;
    }
    return &g_app_wheel_config[wheel];
}

const app_wheel_config_t *app_wheel_config_for_odrive_drive(uint32_t drive_idx)
{
    for (uint32_t i = 0u; i < APP_WHEEL_COUNT; i++) {
        if (g_app_wheel_config[i].odrive_drive_idx == drive_idx) {
            return &g_app_wheel_config[i];
        }
    }
    return 0;
}

const app_wheel_config_t *app_wheel_config_for_local_encoder(wheel_encoder_id_t encoder)
{
    for (uint32_t i = 0u; i < APP_WHEEL_COUNT; i++) {
        if (g_app_wheel_config[i].local_encoder == encoder) {
            return &g_app_wheel_config[i];
        }
    }
    return 0;
}

int8_t app_wheel_encoder_sign(wheel_encoder_id_t encoder)
{
    const app_wheel_config_t *cfg = app_wheel_config_for_local_encoder(encoder);
    return cfg != 0 ? cfg->local_encoder_sign : 1;
}

int8_t app_wheel_odrive_feedback_sign(uint32_t drive_idx)
{
    const app_wheel_config_t *cfg = app_wheel_config_for_odrive_drive(drive_idx);
    return cfg != 0 ? cfg->odrive_feedback_sign : 1;
}
