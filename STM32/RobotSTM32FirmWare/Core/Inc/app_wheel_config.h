/**
 * @file app_wheel_config.h
 * @brief Hardware mapping and orientation signs for each wheel.
 */
#ifndef APP_WHEEL_CONFIG_H
#define APP_WHEEL_CONFIG_H

#include "odrive_can_hal.h"
#include "wheel_encoder_abz.h"

#include <stdint.h>

typedef enum {
    APP_WHEEL_LEFT = 0,
    APP_WHEEL_RIGHT = 1,
    APP_WHEEL_COUNT = 2,
} app_wheel_id_t;

typedef struct {
    app_wheel_id_t wheel;
    ODriveCanHalHandle *odrive_can;
    uint32_t odrive_node_id;
    uint32_t odrive_drive_idx;
    wheel_encoder_id_t local_encoder;
    int8_t odrive_cmd_sign;
    int8_t odrive_feedback_sign;
    int8_t local_encoder_sign;
} app_wheel_config_t;

extern const app_wheel_config_t g_app_wheel_config[APP_WHEEL_COUNT];

const app_wheel_config_t *app_wheel_config_get(app_wheel_id_t wheel);
const app_wheel_config_t *app_wheel_config_for_odrive_drive(uint32_t drive_idx);
const app_wheel_config_t *app_wheel_config_for_local_encoder(wheel_encoder_id_t encoder);

int8_t app_wheel_encoder_sign(wheel_encoder_id_t encoder);
int8_t app_wheel_odrive_feedback_sign(uint32_t drive_idx);

#endif /* APP_WHEEL_CONFIG_H */
