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

#ifndef ODRIVE_VELOCITY_MODE_STATE_TIMEOUT_MS
#define ODRIVE_VELOCITY_MODE_STATE_TIMEOUT_MS 5000u
#endif

#ifndef ODRIVE_VELOCITY_MODE_BOOT_CLOSED_LOOP_WAIT_MS
#define ODRIVE_VELOCITY_MODE_BOOT_CLOSED_LOOP_WAIT_MS 300u
#endif

typedef enum {
    ODRIVE_STARTUP_OK = 0,
    ODRIVE_STARTUP_ERR_NULL = 1,
    ODRIVE_STARTUP_ERR_TX = 2,
    ODRIVE_STARTUP_ERR_NO_HEARTBEAT = 3,
    ODRIVE_STARTUP_ERR_CALIB_TIMEOUT = 4,
    ODRIVE_STARTUP_ERR_CLOSED_LOOP_TIMEOUT = 5,
    /** CAN RX saw standard data frames but no ODrive heartbeat (wrong ID / bitrate). */
    ODRIVE_STARTUP_ERR_CAN_RX_OTHER = 6,
} ODriveStartupError;

#ifdef __cplusplus
extern "C" {
#endif

/** Last failure from odrive_velocity_mode_startup() (for debugger). */
extern volatile ODriveStartupError g_odrive_startup_last_error;

bool odrive_velocity_mode_startup(ODriveCanHalHandle *hcan);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_VELOCITY_MODE_STARTUP_H */
