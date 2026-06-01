/**
 * @file odrive_can_dma.h
 * @brief Non-blocking ODrive CAN simple driver (TX queue + IRQ RX).
 *
 * **STM32H7**: `odrive_can_fdcan_start()` then `odrive_can_dma_init(&hfdcan1, node_id)`.
 * In `HAL_FDCAN_RxFifo0Callback`: `odrive_can_dma_on_rx_fifo0(hfdcan)`.
 *
 * **bxCAN**: `HAL_CAN_Start()` then same API with `CAN_HandleTypeDef`.
 * In `HAL_CAN_RxFifo0MsgPendingCallback`: `odrive_can_dma_on_rx_fifo0(hcan)`.
 *
 * Call `odrive_can_dma_process_tx()` from the main loop.
 */
#ifndef ODRIVE_CAN_DMA_H
#define ODRIVE_CAN_DMA_H

#include "../odrive_can/odrive_can_protocol.h"
#include "../odrive_can/odrive_can_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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
    ODRIVE_AXIS_STATE_LOCKIN_SPIN = 9,
    ODRIVE_AXIS_STATE_ENCODER_DIR_FIND = 10,
    ODRIVE_AXIS_STATE_HOMING = 11,
} ODriveAxisState;

typedef enum {
    ODRIVE_CONTROL_MODE_VOLTAGE = 0,
    ODRIVE_CONTROL_MODE_TORQUE = 1,
    ODRIVE_CONTROL_MODE_VELOCITY = 2,
    ODRIVE_CONTROL_MODE_POSITION = 3,
} ODriveControlMode;

typedef enum {
    ODRIVE_INPUT_MODE_INACTIVE = 0,
    ODRIVE_INPUT_MODE_PASSTHROUGH = 1,
    ODRIVE_INPUT_MODE_VEL_RAMP = 2,
    ODRIVE_INPUT_MODE_POS_FILTER = 3,
    ODRIVE_INPUT_MODE_MIX_CHANNELS = 4,
    ODRIVE_INPUT_MODE_TRAP_TRAJ = 5,
    ODRIVE_INPUT_MODE_TORQUE_RAMP = 6,
    ODRIVE_INPUT_MODE_MIRROR = 7,
} ODriveInputMode;

typedef struct {
    float encoder_pos_turns;
    float encoder_vel_turns_s;
    uint32_t last_update_ms;
    bool valid;
} ODriveCanDmaEncoderSnapshot;

bool odrive_can_dma_init(ODriveCanHalHandle *hcan, uint32_t default_node_id);

void odrive_can_dma_set_node_id(uint32_t node_id);
uint32_t odrive_can_dma_get_node_id(void);

void odrive_can_dma_process_tx(ODriveCanHalHandle *hcan);
void odrive_can_dma_on_rx_fifo0(ODriveCanHalHandle *hcan);

bool odrive_can_dma_set_input_vel(float vel_turns_s, float torque_ff_nm);
bool odrive_can_dma_set_input_pos(float pos_turns, float vel_ff_turns_s, float torque_ff_nm);
bool odrive_can_dma_set_input_torque(float torque_nm);
bool odrive_can_dma_set_controller_modes(ODriveControlMode control_mode, ODriveInputMode input_mode);
bool odrive_can_dma_set_requested_state(ODriveAxisState state);
bool odrive_can_dma_clear_errors(void);
bool odrive_can_dma_request_encoder_estimates(void);
bool odrive_can_dma_get_encoder_snapshot(ODriveCanDmaEncoderSnapshot *out);
void odrive_can_dma_get_last_controller_modes_written(ODriveControlMode *out_cm, ODriveInputMode *out_im);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_DMA_H */
