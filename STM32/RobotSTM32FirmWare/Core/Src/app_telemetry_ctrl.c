/**
 * @file app_telemetry_ctrl.c
 * @brief Descending RPC: read/write runtime control parameters.
 */

#include "app_telemetry.h"
extern volatile uint32_t g_telemetry_rpc_dispatch_count;

#include "app_ctrl_params.h"
#include "telemetry.h"
#include "telemetry_ctrl_params.h"
#include "telemetry_protocol.h"

#include <stddef.h>
#include <string.h>

static int on_get_control_params(telemetry_t *tel, uint16_t sequence_id, const uint8_t *payload,
                                 uint16_t payload_len, void *user_data)
{
    (void)user_data;

    if (payload_len != 0u) {
        return app_telemetry_send_frame_immediate(TELEM_MSG_GET_CONTROL_PARAMS, sequence_id,
                                                  TELEMETRY_ERR_INVALID_PAYLOAD, NULL, 0);
    }

    const app_ctrl_params_snapshot_t *snap = app_ctrl_params_snapshot();
    g_telemetry_rpc_dispatch_count++;
    return app_telemetry_send_frame_immediate(TELEM_MSG_GET_CONTROL_PARAMS, sequence_id, TELEMETRY_ERR_NONE,
                                              (const uint8_t *)snap,
                                              (uint16_t)sizeof(app_ctrl_params_snapshot_t));
}

static int on_set_control_param(telemetry_t *tel, uint16_t sequence_id, const uint8_t *payload,
                                uint16_t payload_len, void *user_data)
{
    (void)user_data;

    if (payload_len != TELEMETRY_SET_CONTROL_PARAM_PAYLOAD_LEN) {
        return app_telemetry_send_frame_immediate(TELEM_MSG_SET_CONTROL_PARAM, sequence_id,
                                                  TELEMETRY_ERR_INVALID_PAYLOAD, NULL, 0);
    }

    telemetry_set_control_param_t req;
    memcpy(&req, payload, sizeof(req));

    float applied = 0.0f;
    if (!app_ctrl_params_set(req.param_id, req.value, &applied)) {
        return app_telemetry_send_frame_immediate(TELEM_MSG_SET_CONTROL_PARAM, sequence_id,
                                                  TELEMETRY_ERR_INVALID_PAYLOAD, NULL, 0);
    }

    telemetry_set_control_param_t ack = {
        .param_id = req.param_id,
        .value = applied,
    };
    return app_telemetry_send_frame_immediate(TELEM_MSG_SET_CONTROL_PARAM, sequence_id, TELEMETRY_ERR_NONE,
                                              (const uint8_t *)&ack, (uint16_t)sizeof(ack));
}

bool app_telemetry_ctrl_register(telemetry_t *tel)
{
    static const telemetry_field_def_t get_response_fields[] = {
        {"version", TELEMETRY_TYPE_UINT32},
        {"strategy_id", TELEMETRY_TYPE_UINT32},
        {"pitch_ref_rad", TELEMETRY_TYPE_FLOAT},
        {"vel_ref_turns_s", TELEMETRY_TYPE_FLOAT},
        {"pitch_failsafe_rad", TELEMETRY_TYPE_FLOAT},
        {"pitch_kp", TELEMETRY_TYPE_FLOAT},
        {"pitch_ki", TELEMETRY_TYPE_FLOAT},
        {"pitch_kd", TELEMETRY_TYPE_FLOAT},
        {"vel_kp", TELEMETRY_TYPE_FLOAT},
        {"vel_ki", TELEMETRY_TYPE_FLOAT},
        {"vel_kd", TELEMETRY_TYPE_FLOAT},
        {"cmd_max_torque_nm", TELEMETRY_TYPE_FLOAT},
        {"linear_theta_func", TELEMETRY_TYPE_UINT32},
        {"linear_k_pitch", TELEMETRY_TYPE_FLOAT},
        {"linear_k_pitch_rate", TELEMETRY_TYPE_FLOAT},
        {"linear_k_vel", TELEMETRY_TYPE_FLOAT},
        {"linear_output_alpha", TELEMETRY_TYPE_FLOAT},
        {"cascade_vel_kp", TELEMETRY_TYPE_FLOAT},
        {"cascade_vel_ki", TELEMETRY_TYPE_FLOAT},
        {"cascade_vel_kd", TELEMETRY_TYPE_FLOAT},
        {"cascade_pitch_ref_max_rad", TELEMETRY_TYPE_FLOAT},
        {"ff_grav_k", TELEMETRY_TYPE_FLOAT},
        {"ff_fb_k_pitch", TELEMETRY_TYPE_FLOAT},
        {"ff_fb_k_rate", TELEMETRY_TYPE_FLOAT},
        {"ff_output_alpha", TELEMETRY_TYPE_FLOAT},
        {"wheel_encoder_vel_lpf_alpha", TELEMETRY_TYPE_FLOAT},
        {"torque_deadband_nm", TELEMETRY_TYPE_FLOAT},
        {"torque_deadband_pitch_max_rad", TELEMETRY_TYPE_FLOAT},
        {"torque_deadband_rate_max_rads", TELEMETRY_TYPE_FLOAT},
        {"alpha_kp", TELEMETRY_TYPE_FLOAT},
        {"alpha_max_nm", TELEMETRY_TYPE_FLOAT},
        {"motor_J", TELEMETRY_TYPE_FLOAT},
        {"motor_friction_c", TELEMETRY_TYPE_FLOAT},
        {"alpha_pitch_max_rad", TELEMETRY_TYPE_FLOAT},
        {"alpha_rate_max_rads", TELEMETRY_TYPE_FLOAT},
        {"alpha_vel_max_turns_s", TELEMETRY_TYPE_FLOAT},
        {"alpha_lpf", TELEMETRY_TYPE_FLOAT},
        {"pos_kp", TELEMETRY_TYPE_FLOAT},
        {"pos_kd", TELEMETRY_TYPE_FLOAT},
        {"pos_pitch_kp", TELEMETRY_TYPE_FLOAT},
        {"pos_x_ref_m", TELEMETRY_TYPE_FLOAT},
        {"pos_v_max_turns_s", TELEMETRY_TYPE_FLOAT},
        {"pos_pitch_max_rad", TELEMETRY_TYPE_FLOAT},
        {"wheel_radius_m", TELEMETRY_TYPE_FLOAT},
        {"pos_reset", TELEMETRY_TYPE_FLOAT},
        {"pos_err_ema_alpha", TELEMETRY_TYPE_FLOAT},
        {"pos_ema_kp", TELEMETRY_TYPE_FLOAT},
        {"outer_mode", TELEMETRY_TYPE_FLOAT},
        {"heading_kp", TELEMETRY_TYPE_FLOAT},
        {"heading_kd", TELEMETRY_TYPE_FLOAT},
        {"heading_ref_rad", TELEMETRY_TYPE_FLOAT},
        {"heading_torque_max_nm", TELEMETRY_TYPE_FLOAT},
        {"heading_reset", TELEMETRY_TYPE_FLOAT},
        {"cascade_vel_err_ema_alpha", TELEMETRY_TYPE_FLOAT},
        {"cascade_vel_ema_kp", TELEMETRY_TYPE_FLOAT},
        {"vel_ref_slew_turns_s2", TELEMETRY_TYPE_FLOAT},
        {"cascade_vel_accel_kp", TELEMETRY_TYPE_FLOAT},
        {"heading_inc", TELEMETRY_TYPE_FLOAT},
        {"heading_dec", TELEMETRY_TYPE_FLOAT},
        {"friction_mode", TELEMETRY_TYPE_FLOAT},
        {"friction_static_nm", TELEMETRY_TYPE_FLOAT},
        {"friction_kinetic_nm", TELEMETRY_TYPE_FLOAT},
        {"friction_vel_eps_turns_s", TELEMETRY_TYPE_FLOAT},
    };

    static const telemetry_field_def_t set_request_fields[] = {
        {"param_id", "UInt16"},
        {"value", TELEMETRY_TYPE_FLOAT},
    };

    static const telemetry_message_def_t get_def = {
        .key = TELEMETRY_KEY_GET_CONTROL_PARAMS,
        .message_type = TELEM_MSG_GET_CONTROL_PARAMS,
        .response_fields = get_response_fields,
        .response_field_count = (uint16_t)(sizeof(get_response_fields) / sizeof(get_response_fields[0])),
        .on_descend = on_get_control_params,
    };

    static const telemetry_message_def_t set_def = {
        .key = TELEMETRY_KEY_SET_CONTROL_PARAM,
        .message_type = TELEM_MSG_SET_CONTROL_PARAM,
        .request_fields = set_request_fields,
        .request_field_count = (uint16_t)(sizeof(set_request_fields) / sizeof(set_request_fields[0])),
        .response_fields = set_request_fields,
        .response_field_count = (uint16_t)(sizeof(set_request_fields) / sizeof(set_request_fields[0])),
        .on_descend = on_set_control_param,
        .descend_payload_len = TELEMETRY_SET_CONTROL_PARAM_PAYLOAD_LEN,
    };

    /* Register independently: a bloated GET schema must not block SET RPC. */
    const bool get_ok = (telemetry_register(tel, &get_def) == 0);
    const bool set_ok = (telemetry_register(tel, &set_def) == 0);
    return get_ok && set_ok;
}
