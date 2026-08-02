/**
 * @file odrive_torque_mode_startup.h
 * @brief CAN startup for torque mode — tuned for 283_anticogging_calibrated.txt.
 *
 * @see ODrive/OdriveTool/Commands/torque_mode.txt
 */
#ifndef ODRIVE_TORQUE_MODE_STARTUP_H
#define ODRIVE_TORQUE_MODE_STARTUP_H

#include "odrive_can_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef ODRIVE_TORQUE_MODE_AXIS0_NODE_ID
#define ODRIVE_TORQUE_MODE_AXIS0_NODE_ID 0u
#endif

#ifndef ODRIVE_TORQUE_MODE_AXIS1_NODE_ID
#define ODRIVE_TORQUE_MODE_AXIS1_NODE_ID 1u
#endif

/** 1 = skip cal (matches saved config with pre_calibrated / startup_closed_loop). */
#ifndef ODRIVE_TORQUE_MODE_SKIP_CALIBRATION
#define ODRIVE_TORQUE_MODE_SKIP_CALIBRATION 1
#endif

#ifndef ODRIVE_TORQUE_MODE_HEARTBEAT_TIMEOUT_MS
#define ODRIVE_TORQUE_MODE_HEARTBEAT_TIMEOUT_MS 5000u
#endif

#ifndef ODRIVE_TORQUE_MODE_CALIB_TIMEOUT_MS
#define ODRIVE_TORQUE_MODE_CALIB_TIMEOUT_MS 120000u
#endif

#ifndef ODRIVE_TORQUE_MODE_STATE_TIMEOUT_MS
#define ODRIVE_TORQUE_MODE_STATE_TIMEOUT_MS 5000u
#endif

#ifndef ODRIVE_TORQUE_MODE_BOOT_CLOSED_LOOP_WAIT_MS
#define ODRIVE_TORQUE_MODE_BOOT_CLOSED_LOOP_WAIT_MS 300u
#endif

typedef enum {
    ODRIVE_TORQUE_STARTUP_OK = 0,
    ODRIVE_TORQUE_STARTUP_IN_PROGRESS = 0xFFu,
    ODRIVE_TORQUE_STARTUP_ERR_NULL = 1,
    ODRIVE_TORQUE_STARTUP_ERR_TX = 2,
    ODRIVE_TORQUE_STARTUP_ERR_NO_HEARTBEAT = 3,
    ODRIVE_TORQUE_STARTUP_ERR_CALIB_TIMEOUT = 4,
    ODRIVE_TORQUE_STARTUP_ERR_CLOSED_LOOP_TIMEOUT = 5,
    ODRIVE_TORQUE_STARTUP_ERR_CAN_RX_OTHER = 6,
    ODRIVE_TORQUE_STARTUP_ERR_CAN_RX_NON_STD = 7,
} ODriveTorqueStartupError;

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t g_odrive_startup_last_error;
extern volatile uint32_t g_odrive_startup_fail_line;
extern volatile uint32_t g_odrive_startup_rx_std_frames;
extern volatile uint32_t g_odrive_startup_rx_fifo0_peak;
extern volatile uint32_t g_odrive_startup_fdcan_psr;
extern volatile uint32_t g_odrive_startup_last_rx_std_id;
extern volatile uint32_t g_odrive_startup_fdcan_ecr;
extern volatile uint32_t g_odrive_startup_fdcan_cccr;
extern volatile uint32_t g_odrive_startup_fdcan_rxf0s;
extern volatile uint32_t g_odrive_startup_tx_fail_op;
extern volatile uint32_t g_odrive_startup_tx_fifo_free;
/** Mirrors APP_ODRIVE_ANTICOGGING_ENABLED after startup (ODrive flash must match). */
extern volatile uint32_t g_odrive_anticogging_enabled_cfg;

bool odrive_torque_mode_startup(ODriveCanHalHandle *hcan);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_TORQUE_MODE_STARTUP_H */
