/**
 * @file telemetry_ctrl_params.h
 * @brief Runtime control-parameter RPC over telemetry (host <-> STM32).
 *
 * TELEM_MSG_GET_CONTROL_PARAMS (0x0101): empty request -> packed snapshot
 *   [APP_CTRL_PARAMS_SNAPSHOT_VERSION u32][strategy_id u32][floats in enum order].
 * TELEM_MSG_SET_CONTROL_PARAM   (0x0102): param_id u16 + value f32 -> echo ack
 *   (no snapshot version in the request).
 */
#ifndef TELEMETRY_CTRL_PARAMS_H
#define TELEMETRY_CTRL_PARAMS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TELEM_MSG_GET_CONTROL_PARAMS   0x0101u
#define TELEM_MSG_SET_CONTROL_PARAM    0x0102u
#define TELEMETRY_KEY_GET_CONTROL_PARAMS "GetControlParams"
#define TELEMETRY_KEY_SET_CONTROL_PARAM  "SetControlParam"

/* Unused leftover. Snapshot version is APP_CTRL_PARAMS_SNAPSHOT_VERSION
 * (GET payload u32) / SNAPSHOT_VERSION on the PC — not this define. */
#define TELEMETRY_CTRL_PARAMS_VERSION    9u

/** SET request / ack payload length. */
#define TELEMETRY_SET_CONTROL_PARAM_PAYLOAD_LEN 6u

typedef struct __attribute__((packed)) {
    uint16_t param_id;
    float value;
} telemetry_set_control_param_t;

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_CTRL_PARAMS_H */
