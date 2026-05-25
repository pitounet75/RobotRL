/**
 * @file odrive_can_protocol.h
 * @brief ODrive CAN simple protocol — matches ODrive v0.5.x `can_simple.hpp` / `can_simple.cpp`.
 *
 * Arbitration ID layout (standard 11-bit):
 *   id = (can_node_id << ODRIVE_CAN_CMD_ID_BITS) | message_cmd
 * Node ID uses the upper 6 bits; command uses the lower 5 bits (mask 0x1F).
 *
 * Reference: ODrive_S-fw-v0.5.1/Firmware/communication/can_simple.cpp
 */
#ifndef ODRIVE_CAN_PROTOCOL_H
#define ODRIVE_CAN_PROTOCOL_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Bits used for command index in 11-bit standard ID (must match firmware). */
#define ODRIVE_CAN_NODE_ID_BITS 6u
#define ODRIVE_CAN_CMD_ID_BITS  (11u - ODRIVE_CAN_NODE_ID_BITS)

/** Maximum node id (inclusive) when using standard simple-CAN layout. */
#define ODRIVE_CAN_NODE_ID_MAX  ((1u << ODRIVE_CAN_NODE_ID_BITS) - 1u)

/**
 * Command indices — values MUST stay aligned with:
 *   `Firmware/communication/can_simple.hpp` (CANSimple enum).
 */
typedef enum {
    ODRIVE_MSG_CO_NMT_CTRL = 0x000u,
    ODRIVE_MSG_ODRIVE_HEARTBEAT,
    ODRIVE_MSG_ODRIVE_ESTOP,
    ODRIVE_MSG_GET_MOTOR_ERROR,
    ODRIVE_MSG_GET_ENCODER_ERROR,
    ODRIVE_MSG_GET_SENSORLESS_ERROR,
    ODRIVE_MSG_SET_AXIS_NODE_ID,
    ODRIVE_MSG_SET_AXIS_REQUESTED_STATE,
    ODRIVE_MSG_SET_AXIS_STARTUP_CONFIG,
    ODRIVE_MSG_GET_ENCODER_ESTIMATES,
    ODRIVE_MSG_GET_ENCODER_COUNT,
    ODRIVE_MSG_SET_CONTROLLER_MODES,
    ODRIVE_MSG_SET_INPUT_POS,
    ODRIVE_MSG_SET_INPUT_VEL,
    ODRIVE_MSG_SET_INPUT_TORQUE,
    ODRIVE_MSG_SET_VEL_LIMIT,
    ODRIVE_MSG_START_ANTICOGGING,
    ODRIVE_MSG_SET_TRAJ_VEL_LIMIT,
    ODRIVE_MSG_SET_TRAJ_ACCEL_LIMITS,
    ODRIVE_MSG_SET_TRAJ_INERTIA,
    ODRIVE_MSG_GET_IQ,
    ODRIVE_MSG_GET_SENSORLESS_ESTIMATES,
    ODRIVE_MSG_RESET_ODRIVE,
    ODRIVE_MSG_GET_VBUS_VOLTAGE,
    ODRIVE_MSG_CLEAR_ERRORS,
    /** CANopen heartbeat COB-ID style constant; low 5 bits are 0 — same wire cmd as `CO_NMT_CTRL`. */
    ODRIVE_MSG_CO_HEARTBEAT_CMD = 0x700u,
} ODriveCanMsg;

/** Number of sequential commands (excludes explicit CO_HEARTBEAT value used in enum). */
#define ODRIVE_MSG_COUNT 25u

static inline uint32_t odrive_can_std_id(uint32_t node_id, ODriveCanMsg cmd)
{
    uint32_t c = (uint32_t)cmd & 0x1Fu;
    return ((node_id & ODRIVE_CAN_NODE_ID_MAX) << ODRIVE_CAN_CMD_ID_BITS) | c;
}

static inline uint32_t odrive_can_node_from_id(uint32_t std_id)
{
    return std_id >> ODRIVE_CAN_CMD_ID_BITS;
}

static inline uint8_t odrive_can_cmd_from_id(uint32_t std_id)
{
    return (uint8_t)(std_id & 0x1Fu);
}

/* -------------------------------------------------------------------------- */
/* Payload pack helpers (intel / little-endian, matches `can_helpers.hpp`)    */
/* -------------------------------------------------------------------------- */

static inline void odrive_can_pack_set_controller_modes(uint8_t buf[8], int32_t control_mode,
                                                      int32_t input_mode)
{
    memcpy(buf, &control_mode, sizeof(control_mode));
    memcpy(buf + 4, &input_mode, sizeof(input_mode));
}

static inline void odrive_can_pack_set_input_pos(uint8_t buf[8], float input_pos,
                                                 float input_vel_ff, float torque_ff)
{
    memcpy(buf, &input_pos, sizeof(input_pos));
    int16_t v = (int16_t)(input_vel_ff / 0.001f);
    int16_t t = (int16_t)(torque_ff / 0.001f);
    memcpy(buf + 4, &v, sizeof(v));
    memcpy(buf + 6, &t, sizeof(t));
}

static inline void odrive_can_pack_set_input_vel(uint8_t buf[8], float input_vel,
                                                 float input_torque_ff)
{
    memcpy(buf, &input_vel, sizeof(input_vel));
    memcpy(buf + 4, &input_torque_ff, sizeof(input_torque_ff));
}

static inline void odrive_can_pack_set_input_torque(uint8_t buf[8], float torque)
{
    memcpy(buf, &torque, sizeof(torque));
}

static inline void odrive_can_pack_float(uint8_t buf[8], float value)
{
    memcpy(buf, &value, sizeof(value));
}

static inline void odrive_can_pack_two_float(uint8_t buf[8], float a, float b)
{
    memcpy(buf, &a, sizeof(a));
    memcpy(buf + 4, &b, sizeof(b));
}

static inline void odrive_can_pack_set_axis_node_id(uint8_t buf[8], uint32_t new_node_id)
{
    memcpy(buf, &new_node_id, sizeof(new_node_id));
    memset(buf + 4, 0, 4);
}

static inline void odrive_can_pack_set_axis_requested_state(uint8_t buf[8], int16_t axis_state)
{
    memcpy(buf, &axis_state, sizeof(axis_state));
    memset(buf + 2, 0, 6);
}

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_PROTOCOL_H */
