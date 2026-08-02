/**

 * @file odrive_can_dma.h

 * @brief Non-blocking ODrive CAN simple driver (TX queue + IRQ RX snapshots).

 */

#ifndef ODRIVE_CAN_DMA_H

#define ODRIVE_CAN_DMA_H



#include "odrive_can_protocol.h"

#include "odrive_can_hal.h"

#include <stdbool.h>

#include <stdint.h>



#ifdef __cplusplus

extern "C" {

#endif



#ifndef ODRIVE_CAN_DMA_DRIVE_COUNT
#define ODRIVE_CAN_DMA_DRIVE_COUNT 2u
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

    int32_t encoder_pos_counts;

    uint32_t last_update_ms;

    bool valid;

} ODriveCanDmaEncoderSnapshot;



bool odrive_can_dma_init(ODriveCanHalHandle *hcan);

void odrive_can_dma_set_node_id(uint32_t node_id);

uint32_t odrive_can_dma_get_node_id(void);



void odrive_can_dma_process_tx(ODriveCanHalHandle *hcan);



/** Drain RX FIFO0 (standalone sniffer path). */

void odrive_can_dma_on_rx_fifo0(ODriveCanHalHandle *hcan);



/**

 * Parse one classic-CAN data frame (call from shared RX ISR after HAL read).

 * Updates encoder snapshots for monitored node IDs; ISR-safe.

 */

void odrive_can_dma_on_rx_frame(uint32_t std_id, const uint8_t *data, uint8_t dlc);
void odrive_can_dma_on_rx_frame_for_bus(ODriveCanHalHandle *hcan, uint32_t std_id,
                                        const uint8_t *data, uint8_t dlc);



bool odrive_can_dma_set_input_vel(uint32_t node_id, float vel_turns_s, float torque_ff_nm);
bool odrive_can_dma_set_input_vel_on_bus(ODriveCanHalHandle *hcan, uint32_t node_id,
                                         float vel_turns_s, float torque_ff_nm);

bool odrive_can_dma_set_input_pos(uint32_t node_id, float pos_turns, float vel_ff_turns_s, float torque_ff_nm);

bool odrive_can_dma_set_input_torque_on_bus(ODriveCanHalHandle *hcan, uint32_t node_id,
                                            float torque_nm);

bool odrive_can_dma_set_input_torque(uint32_t node_id, float torque_nm);

bool odrive_can_dma_set_controller_modes(uint32_t node_id, ODriveControlMode control_mode,

                                         ODriveInputMode input_mode);

bool odrive_can_dma_set_requested_state(uint32_t node_id, ODriveAxisState state);

bool odrive_can_dma_clear_errors(uint32_t node_id);

bool odrive_can_dma_request_encoder_estimates(uint32_t node_id);
bool odrive_can_dma_request_encoder_estimates_on_bus(ODriveCanHalHandle *hcan, uint32_t node_id);



bool odrive_can_dma_get_encoder_snapshot(uint32_t node_id, ODriveCanDmaEncoderSnapshot *out);
bool odrive_can_dma_get_encoder_snapshot_for_drive(uint32_t drive_idx, ODriveCanDmaEncoderSnapshot *out);

bool odrive_can_dma_is_encoder_fresh(uint32_t node_id, uint32_t max_age_ms);
bool odrive_can_dma_is_encoder_fresh_for_drive(uint32_t drive_idx, uint32_t max_age_ms);



void odrive_can_dma_get_last_controller_modes_written(ODriveControlMode *out_cm, ODriveInputMode *out_im);

/** Live Expressions: TX queue full (enqueue dropped) / HAL_FDCAN_AddMessageToTxFifoQ failed. */
extern volatile uint32_t g_odrive_can_tx_queue_full;
extern volatile uint32_t g_odrive_can_tx_hal_fail;

#ifdef __cplusplus

}

#endif



#endif /* ODRIVE_CAN_DMA_H */


