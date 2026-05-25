/**
 * @file odrive_velocity_mode_startup.c
 * @see ODrive/OdriveTool/Commands/velocity_mode.txt
 * @see ODrive/OdriveTool/Commands/Configs/283_anticogging_calibrated.txt
 */

#include "odrive_velocity_mode_startup.h"
#include "odrive_can_protocol.h"
#include "odrive_can_stm32.h"

#include <string.h>

volatile ODriveStartupError g_odrive_startup_last_error = ODRIVE_STARTUP_OK;

typedef enum {
    ODRIVE_AXIS_STATE_UNDEFINED = 0,
    ODRIVE_AXIS_STATE_IDLE = 1,
    ODRIVE_AXIS_STATE_STARTUP_SEQUENCE = 2,
    ODRIVE_AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
    ODRIVE_AXIS_STATE_MOTOR_CALIBRATION = 4,
    ODRIVE_AXIS_STATE_SENSORLESS_CONTROL = 5,
    ODRIVE_AXIS_STATE_ENCODER_INDEX_SEARCH = 6,
    ODRIVE_AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
    ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
} ODriveAxisState;

typedef enum {
    ODRIVE_CONTROL_MODE_VELOCITY = 2,
    ODRIVE_INPUT_MODE_PASSTHROUGH = 1,
} ODriveControlModeInput;

static bool tx_set_state(ODriveCanHalHandle *hcan, uint32_t node_id, int16_t state)
{
    uint8_t buf[8];
    odrive_can_pack_set_axis_requested_state(buf, state);
    return odrive_can_send_data(hcan, node_id, ODRIVE_MSG_SET_AXIS_REQUESTED_STATE, buf, 8);
}

static bool tx_clear_errors(ODriveCanHalHandle *hcan, uint32_t node_id)
{
    uint8_t buf[8] = {0};
    return odrive_can_send_data(hcan, node_id, ODRIVE_MSG_CLEAR_ERRORS, buf, 8);
}

static bool tx_controller_modes(ODriveCanHalHandle *hcan, uint32_t node_id, int32_t control_mode,
                                int32_t input_mode)
{
    uint8_t buf[8];
    odrive_can_pack_set_controller_modes(buf, control_mode, input_mode);
    return odrive_can_send_data(hcan, node_id, ODRIVE_MSG_SET_CONTROLLER_MODES, buf, 8);
}

static bool tx_input_vel(ODriveCanHalHandle *hcan, uint32_t node_id, float vel_turns_s,
                         float torque_ff_nm)
{
    uint8_t buf[8];
    odrive_can_pack_set_input_vel(buf, vel_turns_s, torque_ff_nm);
    return odrive_can_send_data(hcan, node_id, ODRIVE_MSG_SET_INPUT_VEL, buf, 8);
}

static bool tx_input_torque(ODriveCanHalHandle *hcan, uint32_t node_id, float torque_nm)
{
    uint8_t buf[8];
    odrive_can_pack_set_input_torque(buf, torque_nm);
    return odrive_can_send_data(hcan, node_id, ODRIVE_MSG_SET_INPUT_TORQUE, buf, 8);
}

static bool heartbeat_for_node(uint32_t std_id, uint32_t node_id)
{
    return odrive_can_node_from_id(std_id) == (node_id & ODRIVE_CAN_NODE_ID_MAX) &&
           odrive_can_cmd_from_id(std_id) == (uint8_t)(ODRIVE_MSG_ODRIVE_HEARTBEAT & 0x1Fu);
}

static bool heartbeat_parse_state(const uint8_t data[8], uint8_t dlc, int32_t *out_state)
{
    if (dlc < 5u || out_state == NULL) {
        return false;
    }
    memcpy(out_state, data + 4, sizeof(int32_t));
    return true;
}

static void drain_rx(ODriveCanHalHandle *hcan)
{
    uint32_t id;
    bool ext, rtr;
    uint8_t data[8];
    uint8_t dlc;
    while (odrive_can_hal_rx(hcan, &id, &ext, &rtr, data, &dlc)) {
    }
}

static bool poll_axis_state(ODriveCanHalHandle *hcan, uint32_t node_id, uint32_t timeout_ms,
                            bool (*accept)(int32_t state, void *ctx), void *ctx)
{
    const uint32_t t0 = HAL_GetTick();

    for (;;) {
        uint32_t id;
        bool ext, rtr;
        uint8_t data[8];
        uint8_t dlc;

        while (odrive_can_hal_rx(hcan, &id, &ext, &rtr, data, &dlc)) {
            int32_t st = 0;
            if (ext || rtr || !heartbeat_for_node(id, node_id)) {
                continue;
            }
            if (!heartbeat_parse_state(data, dlc, &st)) {
                continue;
            }
            if (accept(st, ctx)) {
                return true;
            }
        }

        if ((HAL_GetTick() - t0) >= timeout_ms) {
            return false;
        }
            HAL_Delay(1);
    }
}

static bool state_not_cal_busy(int32_t state, void *ctx)
{
    (void)ctx;
    return !(state == ODRIVE_AXIS_STATE_FULL_CALIBRATION_SEQUENCE ||
             state == ODRIVE_AXIS_STATE_MOTOR_CALIBRATION ||
             state == ODRIVE_AXIS_STATE_ENCODER_OFFSET_CALIBRATION ||
             state == ODRIVE_AXIS_STATE_ENCODER_INDEX_SEARCH);
}

static bool state_is_closed_loop(int32_t state, void *ctx)
{
    (void)ctx;
    return state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL;
}

/** Wait for any ODrive heartbeat (axis0 or axis1) before we transmit on the bus. */
static bool wait_any_odrive_heartbeat(ODriveCanHalHandle *hcan, uint32_t timeout_ms, bool *saw_std_data)
{
    const uint32_t t0 = HAL_GetTick();
    bool saw_other = false;

    if (saw_std_data != NULL) {
        *saw_std_data = false;
    }

    for (;;) {
        uint32_t id;
        bool ext, rtr;
        uint8_t data[8];
        uint8_t dlc;

        while (odrive_can_hal_rx(hcan, &id, &ext, &rtr, data, &dlc)) {
            if (ext || rtr) {
                continue;
            }
            saw_other = true;
            if (odrive_can_cmd_from_id(id) != (uint8_t)(ODRIVE_MSG_ODRIVE_HEARTBEAT & 0x1Fu)) {
                continue;
            }
            int32_t st = 0;
            if (heartbeat_parse_state(data, dlc, &st)) {
                (void)st;
                return true;
            }
        }

        if ((HAL_GetTick() - t0) >= timeout_ms) {
            if (saw_std_data != NULL) {
                *saw_std_data = saw_other;
            }
            return false;
        }
        HAL_Delay(1);
    }
}

bool odrive_velocity_mode_startup(ODriveCanHalHandle *hcan)
{
    g_odrive_startup_last_error = ODRIVE_STARTUP_OK;

    if (hcan == NULL) {
        g_odrive_startup_last_error = ODRIVE_STARTUP_ERR_NULL;
        return false;
    }

    const uint32_t axis0 = ODRIVE_VELOCITY_MODE_AXIS0_NODE_ID;
    const uint32_t axis1 = ODRIVE_VELOCITY_MODE_AXIS1_NODE_ID;

    drain_rx(hcan);

    /* Listen before TX: failed TX without bus ACK can put FDCAN in bus-off and block RX. */
    {
        bool saw_std_data = false;
        if (!wait_any_odrive_heartbeat(hcan, ODRIVE_VELOCITY_MODE_HEARTBEAT_TIMEOUT_MS,
                                       &saw_std_data)) {
            g_odrive_startup_last_error = saw_std_data ? ODRIVE_STARTUP_ERR_CAN_RX_OTHER
                                                       : ODRIVE_STARTUP_ERR_NO_HEARTBEAT;
            return false;
        }
    }
    drain_rx(hcan);

#if !ODRIVE_VELOCITY_MODE_SKIP_CALIBRATION
    if (!tx_set_state(hcan, axis0, ODRIVE_AXIS_STATE_IDLE)) {
        g_odrive_startup_last_error = ODRIVE_STARTUP_ERR_TX;
        return false;
    }
    HAL_Delay(50);
#endif

    if (!tx_clear_errors(hcan, axis0) || !tx_clear_errors(hcan, axis1)) {
        g_odrive_startup_last_error = ODRIVE_STARTUP_ERR_TX;
        return false;
    }
    HAL_Delay(100);
    drain_rx(hcan);

#if !ODRIVE_VELOCITY_MODE_SKIP_CALIBRATION
    if (!tx_set_state(hcan, axis0, ODRIVE_AXIS_STATE_FULL_CALIBRATION_SEQUENCE)) {
        g_odrive_startup_last_error = ODRIVE_STARTUP_ERR_TX;
        return false;
    }

    if (!poll_axis_state(hcan, axis0, ODRIVE_VELOCITY_MODE_CALIB_TIMEOUT_MS, state_not_cal_busy,
                         NULL)) {
        g_odrive_startup_last_error = ODRIVE_STARTUP_ERR_CALIB_TIMEOUT;
        return false;
    }
#else
    if (!poll_axis_state(hcan, axis0, ODRIVE_VELOCITY_MODE_BOOT_CLOSED_LOOP_WAIT_MS,
                         state_is_closed_loop, NULL)) {
        if (!tx_set_state(hcan, axis0, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL)) {
            g_odrive_startup_last_error = ODRIVE_STARTUP_ERR_TX;
            return false;
        }
        if (!poll_axis_state(hcan, axis0, ODRIVE_VELOCITY_MODE_STATE_TIMEOUT_MS,
                             state_is_closed_loop, NULL)) {
            g_odrive_startup_last_error = ODRIVE_STARTUP_ERR_CLOSED_LOOP_TIMEOUT;
            return false;
        }
    }
#endif

    if (!tx_controller_modes(hcan, axis0, ODRIVE_CONTROL_MODE_VELOCITY, ODRIVE_INPUT_MODE_PASSTHROUGH)) {
        g_odrive_startup_last_error = ODRIVE_STARTUP_ERR_TX;
        return false;
    }

    if (!tx_input_vel(hcan, axis0, 0.0f, 0.0f) || !tx_input_torque(hcan, axis0, 0.0f)) {
        g_odrive_startup_last_error = ODRIVE_STARTUP_ERR_TX;
        return false;
    }

    return true;
}
