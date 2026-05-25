/**
 * @file odrive_velocity_mode_startup.h
 * @brief CAN startup for velocity mode — tuned for 283_anticogging_calibrated.txt.
 *
 * Your ODrive dump has startup_closed_loop_control=True and pre_calibrated encoders,
 * so the default path skips FULL_CALIBRATION_SEQUENCE and waits for closed loop.
 */
#ifndef ODRIVE_VELOCITY_MODE_STARTUP_H
#define ODRIVE_VELOCITY_MODE_STARTUP_H

#include "odrive_can_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef ODRIVE_VELOCITY_MODE_AXIS0_NODE_ID
#define ODRIVE_VELOCITY_MODE_AXIS0_NODE_ID 0u
#endif

#ifndef ODRIVE_VELOCITY_MODE_AXIS1_NODE_ID
#define ODRIVE_VELOCITY_MODE_AXIS1_NODE_ID 1u
#endif

/** 1 = skip cal (matches saved config with pre_calibrated / startup_closed_loop). */
#ifndef ODRIVE_VELOCITY_MODE_SKIP_CALIBRATION
#define ODRIVE_VELOCITY_MODE_SKIP_CALIBRATION 1
#endif

#ifndef ODRIVE_VELOCITY_MODE_HEARTBEAT_TIMEOUT_MS
#define ODRIVE_VELOCITY_MODE_HEARTBEAT_TIMEOUT_MS 5000u
#endif

#ifndef ODRIVE_VELOCITY_MODE_CALIB_TIMEOUT_MS
#define ODRIVE_VELOCITY_MODE_CALIB_TIMEOUT_MS 120000u
#endif

/** After requesting closed loop: wait for heartbeat state (100 ms rate on 283 config). */
#ifndef ODRIVE_VELOCITY_MODE_STATE_TIMEOUT_MS
#define ODRIVE_VELOCITY_MODE_STATE_TIMEOUT_MS 5000u
#endif

/** Skip-cal path: listen briefly if axis is already in closed loop (startup_closed_loop_control). */
#ifndef ODRIVE_VELOCITY_MODE_BOOT_CLOSED_LOOP_WAIT_MS
#define ODRIVE_VELOCITY_MODE_BOOT_CLOSED_LOOP_WAIT_MS 300u
#endif

typedef enum {
    ODRIVE_STARTUP_OK = 0,
    /** Set at odrive_velocity_mode_startup() entry — not a successful result. */
    ODRIVE_STARTUP_IN_PROGRESS = 0xFFu,
    ODRIVE_STARTUP_ERR_NULL = 1,
    ODRIVE_STARTUP_ERR_TX = 2,
    ODRIVE_STARTUP_ERR_NO_HEARTBEAT = 3,
    ODRIVE_STARTUP_ERR_CALIB_TIMEOUT = 4,
    ODRIVE_STARTUP_ERR_CLOSED_LOOP_TIMEOUT = 5,
    /** CAN RX saw std frames but not from node 0/1 and not heartbeat cmd. */
    ODRIVE_STARTUP_ERR_CAN_RX_OTHER = 6,
    /** RX FIFO had traffic but only extended/RTR (check can_node_id_extended). */
    ODRIVE_STARTUP_ERR_CAN_RX_NON_STD = 7,
} ODriveStartupError;

#ifdef __cplusplus
extern "C" {
#endif

/** Last error code (uint32_t storage — use numeric values in debugger). */
extern volatile uint32_t g_odrive_startup_last_error;
/** Source line of last ODRIVE_STARTUP_FAIL() — compare with odrive_velocity_mode_startup.c. */
extern volatile uint32_t g_odrive_startup_fail_line;

/** Standard data frames received during heartbeat wait (0 = nothing on bus at MCU). */
extern volatile uint32_t g_odrive_startup_rx_std_frames;
/** Max RX FIFO0 fill level seen during wait. */
extern volatile uint32_t g_odrive_startup_rx_fifo0_peak;
/** FDCAN PSR register at failure (bit2=bus-off). */
extern volatile uint32_t g_odrive_startup_fdcan_psr;
/** Last standard ID seen during heartbeat wait (0 if none). */
extern volatile uint32_t g_odrive_startup_last_rx_std_id;
/** FDCAN ECR / CCCR / RXF0S at heartbeat-wait failure (debug). */
extern volatile uint32_t g_odrive_startup_fdcan_ecr;
extern volatile uint32_t g_odrive_startup_fdcan_cccr;
extern volatile uint32_t g_odrive_startup_fdcan_rxf0s;
/** Which TX step failed (see ODRIVE_TX_OP_* in .c). 0 if not a TX error. */
extern volatile uint32_t g_odrive_startup_tx_fail_op;
/** FDCAN TX FIFO free level when last TX wait timed out. */
extern volatile uint32_t g_odrive_startup_tx_fifo_free;

bool odrive_velocity_mode_startup(ODriveCanHalHandle *hcan);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_VELOCITY_MODE_STARTUP_H */
