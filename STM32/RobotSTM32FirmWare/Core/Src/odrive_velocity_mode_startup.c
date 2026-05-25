/**
 * @file odrive_velocity_mode_startup.c
 * @see ODrive/OdriveTool/Commands/velocity_mode.txt
 * @see ODrive/OdriveTool/Commands/Configs/283_anticogging_calibrated.txt
 */

#include "odrive_velocity_mode_startup.h"
#include "odrive_can_protocol.h"
#include "odrive_can_stm32.h"

#include <string.h>

volatile uint32_t g_odrive_startup_last_error = (uint32_t)ODRIVE_STARTUP_IN_PROGRESS;
volatile uint32_t g_odrive_startup_fail_line = 0u;

#define ODRIVE_STARTUP_FAIL(err)                     \
    do {                                             \
        g_odrive_startup_last_error = (uint32_t)(err); \
        g_odrive_startup_fail_line = (uint32_t)__LINE__; \
    } while (0)
volatile uint32_t g_odrive_startup_rx_std_frames = 0u;
volatile uint32_t g_odrive_startup_rx_fifo0_peak = 0u;
volatile uint32_t g_odrive_startup_fdcan_psr = 0u;
volatile uint32_t g_odrive_startup_last_rx_std_id = 0u;
volatile uint32_t g_odrive_startup_fdcan_ecr = 0u;
volatile uint32_t g_odrive_startup_fdcan_cccr = 0u;
volatile uint32_t g_odrive_startup_fdcan_rxf0s = 0u;

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

#ifndef ODRIVE_STARTUP_TX_FRAME_TIMEOUT_MS
#define ODRIVE_STARTUP_TX_FRAME_TIMEOUT_MS 200u
#endif

#define ODRIVE_TX_OP_SET_IDLE 1u
#define ODRIVE_TX_OP_CLEAR_ERR0 2u
#define ODRIVE_TX_OP_CLEAR_ERR1 3u
#define ODRIVE_TX_OP_SET_CAL 4u
#define ODRIVE_TX_OP_SET_CLOSED_LOOP 5u
#define ODRIVE_TX_OP_CTRL_MODES 6u
#define ODRIVE_TX_OP_INPUT_VEL 7u
#define ODRIVE_TX_OP_INPUT_TORQUE 8u

volatile uint32_t g_odrive_startup_tx_fail_op = 0u;
volatile uint32_t g_odrive_startup_tx_fifo_free = 0u;

static void odrive_startup_record_tx_fail(ODriveCanHalHandle *hcan, uint32_t op)
{
    g_odrive_startup_tx_fail_op = op;
#if ODRIVE_CAN_HAL_FDCAN
    if (hcan != NULL && hcan->Instance != NULL) {
        g_odrive_startup_tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(hcan);
        g_odrive_startup_fdcan_psr = READ_REG(hcan->Instance->PSR);
        g_odrive_startup_fdcan_ecr = READ_REG(hcan->Instance->ECR);
        g_odrive_startup_fdcan_cccr = READ_REG(hcan->Instance->CCCR);
    }
#else
    (void)hcan;
#endif
    (void)op;
}

/** Queue one frame and wait until that buffer leaves TXBRP (not "all 8 FIFO slots free"). */
static bool tx_startup_frame(ODriveCanHalHandle *hcan, uint32_t node_id, ODriveCanMsg cmd,
                             const uint8_t *data, uint8_t dlc, uint32_t op)
{
#if ODRIVE_CAN_HAL_FDCAN
    FDCAN_ProtocolStatusTypeDef ps = {0};
    if (HAL_FDCAN_GetProtocolStatus(hcan, &ps) == HAL_OK && ps.BusOff != 0u) {
        odrive_can_fdcan_recover_bus_off(hcan);
    }
#endif
    if (!odrive_can_send_data(hcan, node_id, cmd, data, dlc)) {
        odrive_startup_record_tx_fail(hcan, op);
        return false;
    }
#if ODRIVE_CAN_HAL_FDCAN
    if (!odrive_can_hal_wait_tx_fifo_empty(hcan, ODRIVE_STARTUP_TX_FRAME_TIMEOUT_MS)) {
        odrive_startup_record_tx_fail(hcan, op);
        return false;
    }
    return true;
#else
    (void)op;
    return true;
#endif
}

static bool tx_set_state(ODriveCanHalHandle *hcan, uint32_t node_id, int16_t state, uint32_t op)
{
    uint8_t buf[8];
    odrive_can_pack_set_axis_requested_state(buf, state);
    return tx_startup_frame(hcan, node_id, ODRIVE_MSG_SET_AXIS_REQUESTED_STATE, buf, 8, op);
}

static bool tx_clear_errors(ODriveCanHalHandle *hcan, uint32_t node_id, uint32_t op)
{
    uint8_t buf[8] = {0};
    return tx_startup_frame(hcan, node_id, ODRIVE_MSG_CLEAR_ERRORS, buf, 8, op);
}

static bool tx_controller_modes(ODriveCanHalHandle *hcan, uint32_t node_id, int32_t control_mode,
                                int32_t input_mode)
{
    uint8_t buf[8];
    odrive_can_pack_set_controller_modes(buf, control_mode, input_mode);
    return tx_startup_frame(hcan, node_id, ODRIVE_MSG_SET_CONTROLLER_MODES, buf, 8,
                            ODRIVE_TX_OP_CTRL_MODES);
}

static bool tx_input_vel(ODriveCanHalHandle *hcan, uint32_t node_id, float vel_turns_s,
                         float torque_ff_nm)
{
    uint8_t buf[8];
    odrive_can_pack_set_input_vel(buf, vel_turns_s, torque_ff_nm);
    return tx_startup_frame(hcan, node_id, ODRIVE_MSG_SET_INPUT_VEL, buf, 8, ODRIVE_TX_OP_INPUT_VEL);
}

static bool tx_input_torque(ODriveCanHalHandle *hcan, uint32_t node_id, float torque_nm)
{
    uint8_t buf[8];
    odrive_can_pack_set_input_torque(buf, torque_nm);
    return tx_startup_frame(hcan, node_id, ODRIVE_MSG_SET_INPUT_TORQUE, buf, 8,
                            ODRIVE_TX_OP_INPUT_TORQUE);
}

static bool heartbeat_for_node(uint32_t std_id, uint32_t node_id)
{
    return odrive_can_node_from_id(std_id) == (node_id & ODRIVE_CAN_NODE_ID_MAX) &&
           odrive_can_cmd_from_id(std_id) == (uint8_t)(ODRIVE_MSG_ODRIVE_HEARTBEAT & 0x1Fu);
}

/** Axis0 heartbeat id=1, axis1 id=33 (node<<5 | ODRIVE_HEARTBEAT). */
static bool is_odrive_heartbeat_std_id(uint32_t std_id)
{
    return std_id == odrive_can_std_id(ODRIVE_VELOCITY_MODE_AXIS0_NODE_ID, ODRIVE_MSG_ODRIVE_HEARTBEAT) ||
           std_id == odrive_can_std_id(ODRIVE_VELOCITY_MODE_AXIS1_NODE_ID, ODRIVE_MSG_ODRIVE_HEARTBEAT) ||
           odrive_can_cmd_from_id(std_id) == (uint8_t)(ODRIVE_MSG_ODRIVE_HEARTBEAT & 0x1Fu);
}

static bool is_configured_axis_std_id(uint32_t std_id)
{
    const uint32_t node = odrive_can_node_from_id(std_id);
    return node == (ODRIVE_VELOCITY_MODE_AXIS0_NODE_ID & ODRIVE_CAN_NODE_ID_MAX) ||
           node == (ODRIVE_VELOCITY_MODE_AXIS1_NODE_ID & ODRIVE_CAN_NODE_ID_MAX);
}

/** ODrive is on the bus: heartbeat or any frame from configured node IDs. */
static bool is_bus_alive_std_id(uint32_t std_id)
{
    return is_odrive_heartbeat_std_id(std_id) || is_configured_axis_std_id(std_id);
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
        bool drained_fifo = false;
        uint32_t id;
        bool ext, rtr;
        uint8_t data[8];
        uint8_t dlc;

        while (odrive_can_hal_rx(hcan, &id, &ext, &rtr, data, &dlc)) {
            drained_fifo = true;
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
        if (!drained_fifo) {
            HAL_Delay(1);
        }
    }
}

#if !ODRIVE_VELOCITY_MODE_SKIP_CALIBRATION
static bool state_not_cal_busy(int32_t state, void *ctx)
{
    (void)ctx;
    return !(state == ODRIVE_AXIS_STATE_FULL_CALIBRATION_SEQUENCE ||
             state == ODRIVE_AXIS_STATE_MOTOR_CALIBRATION ||
             state == ODRIVE_AXIS_STATE_ENCODER_OFFSET_CALIBRATION ||
             state == ODRIVE_AXIS_STATE_ENCODER_INDEX_SEARCH);
}
#endif

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

    g_odrive_startup_rx_std_frames = 0u;
    g_odrive_startup_rx_fifo0_peak = 0u;
    g_odrive_startup_last_rx_std_id = 0u;

    if (saw_std_data != NULL) {
        *saw_std_data = false;
    }

    for (;;) {
        bool drained_fifo = false;

#if ODRIVE_CAN_HAL_FDCAN
        const uint32_t fifo0 = HAL_FDCAN_GetRxFifoFillLevel(hcan, FDCAN_RX_FIFO0);
        if (fifo0 > g_odrive_startup_rx_fifo0_peak) {
            g_odrive_startup_rx_fifo0_peak = fifo0;
        }
#endif
        uint32_t id;
        bool ext, rtr;
        uint8_t data[8];
        uint8_t dlc;

        while (odrive_can_hal_rx(hcan, &id, &ext, &rtr, data, &dlc)) {
            drained_fifo = true;
            if (ext || rtr) {
                continue;
            }
            g_odrive_startup_rx_std_frames++;
            g_odrive_startup_last_rx_std_id = id;
            saw_other = true;
            if (is_bus_alive_std_id(id)) {
                return true;
            }
        }

        if ((HAL_GetTick() - t0) >= timeout_ms) {
#if ODRIVE_CAN_HAL_FDCAN
            g_odrive_startup_fdcan_psr = READ_REG(hcan->Instance->PSR);
            g_odrive_startup_fdcan_ecr = READ_REG(hcan->Instance->ECR);
            g_odrive_startup_fdcan_cccr = READ_REG(hcan->Instance->CCCR);
            g_odrive_startup_fdcan_rxf0s = READ_REG(hcan->Instance->RXF0S);
#endif
            if (saw_std_data != NULL) {
                *saw_std_data = saw_other;
            }
            return false;
        }

        /* Do not sleep while FIFO may still hold frames (16-deep FIFO overflows at ~20 frames/s). */
        if (!drained_fifo) {
            HAL_Delay(1);
        }
    }
}

bool odrive_velocity_mode_startup(ODriveCanHalHandle *hcan)
{
    ODRIVE_STARTUP_FAIL(ODRIVE_STARTUP_IN_PROGRESS);

    if (hcan == NULL) {
        ODRIVE_STARTUP_FAIL(ODRIVE_STARTUP_ERR_NULL);
        return false;
    }

    const uint32_t axis0 = ODRIVE_VELOCITY_MODE_AXIS0_NODE_ID;
    const uint32_t axis1 = ODRIVE_VELOCITY_MODE_AXIS1_NODE_ID;

#if ODRIVE_CAN_HAL_FDCAN
    odrive_can_fdcan_recover_bus_off(hcan);
#endif
    HAL_Delay(100);

    drain_rx(hcan);

    /* Listen before TX: failed TX without bus ACK can put FDCAN in bus-off and block RX. */
    {
        bool saw_std_data = false;
        if (!wait_any_odrive_heartbeat(hcan, ODRIVE_VELOCITY_MODE_HEARTBEAT_TIMEOUT_MS,
                                       &saw_std_data)) {
            if (g_odrive_startup_rx_fifo0_peak > 0u && g_odrive_startup_rx_std_frames == 0u) {
                ODRIVE_STARTUP_FAIL(ODRIVE_STARTUP_ERR_CAN_RX_NON_STD);
            } else if (saw_std_data) {
                ODRIVE_STARTUP_FAIL(ODRIVE_STARTUP_ERR_CAN_RX_OTHER);
            } else {
                ODRIVE_STARTUP_FAIL(ODRIVE_STARTUP_ERR_NO_HEARTBEAT);
            }
            return false;
        }
    }
    drain_rx(hcan);

#if !ODRIVE_VELOCITY_MODE_SKIP_CALIBRATION
    /* Full cal path (velocity_mode.txt): idle before clear + calibration. */
    if (!tx_set_state(hcan, axis0, ODRIVE_AXIS_STATE_IDLE, ODRIVE_TX_OP_SET_IDLE)) {
        ODRIVE_STARTUP_FAIL(ODRIVE_STARTUP_ERR_TX);
        return false;
    }
    HAL_Delay(50);
#endif

    if (!tx_clear_errors(hcan, axis0, ODRIVE_TX_OP_CLEAR_ERR0)) {
        ODRIVE_STARTUP_FAIL(ODRIVE_STARTUP_ERR_TX);
        return false;
    }
    (void)tx_clear_errors(hcan, axis1, ODRIVE_TX_OP_CLEAR_ERR1);
    HAL_Delay(100);
    drain_rx(hcan);

#if !ODRIVE_VELOCITY_MODE_SKIP_CALIBRATION
    if (!tx_set_state(hcan, axis0, ODRIVE_AXIS_STATE_FULL_CALIBRATION_SEQUENCE, ODRIVE_TX_OP_SET_CAL)) {
        ODRIVE_STARTUP_FAIL(ODRIVE_STARTUP_ERR_TX);
        return false;
    }

    if (!poll_axis_state(hcan, axis0, ODRIVE_VELOCITY_MODE_CALIB_TIMEOUT_MS, state_not_cal_busy,
                         NULL)) {
        ODRIVE_STARTUP_FAIL(ODRIVE_STARTUP_ERR_CALIB_TIMEOUT);
        return false;
    }
#else
    /* 283 config: startup_closed_loop_control=True — axis may already be in closed loop.
     * Do not force IDLE first (that cancels auto startup and wastes STATE_TIMEOUT_MS).
     * Short listen, then explicitly request closed loop only if needed. */
    if (!poll_axis_state(hcan, axis0, ODRIVE_VELOCITY_MODE_BOOT_CLOSED_LOOP_WAIT_MS,
                         state_is_closed_loop, NULL)) {
        if (!tx_set_state(hcan, axis0, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL,
                          ODRIVE_TX_OP_SET_CLOSED_LOOP)) {
            ODRIVE_STARTUP_FAIL(ODRIVE_STARTUP_ERR_TX);
            return false;
        }
        if (!poll_axis_state(hcan, axis0, ODRIVE_VELOCITY_MODE_STATE_TIMEOUT_MS,
                             state_is_closed_loop, NULL)) {
            ODRIVE_STARTUP_FAIL(ODRIVE_STARTUP_ERR_CLOSED_LOOP_TIMEOUT);
            return false;
        }
    }
#endif

    if (!tx_controller_modes(hcan, axis0, ODRIVE_CONTROL_MODE_VELOCITY, ODRIVE_INPUT_MODE_PASSTHROUGH)) {
        ODRIVE_STARTUP_FAIL(ODRIVE_STARTUP_ERR_TX);
        return false;
    }

    if (!tx_input_vel(hcan, axis0, 0.2f, 0.0f) || !tx_input_torque(hcan, axis0, 0.0f)) {
        ODRIVE_STARTUP_FAIL(ODRIVE_STARTUP_ERR_TX);
        return false;
    }

    g_odrive_startup_last_error = (uint32_t)ODRIVE_STARTUP_OK;
    g_odrive_startup_fail_line = 0u;
    g_odrive_startup_tx_fail_op = 0u;
    return true;
}
