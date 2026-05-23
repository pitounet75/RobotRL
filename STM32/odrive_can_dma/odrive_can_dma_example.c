/**
 * @file odrive_can_dma_example.c
 * @brief Example wiring for `odrive_can_dma` — set velocity and track `control_mode`.
 *
 * This file is **reference code**: copy the callbacks into `stm32f4xx_it.c` / your
 * `main.c`, or call the `odrive_can_dma_example_*` helpers from your app.
 *
 * **CAN simple limitation:** ODrive does not expose a read-back CAN frame for
 * `controller.config.control_mode`. This example uses
 * `odrive_can_dma_get_last_controller_modes_written()` which returns the last
 * values **successfully queued** by `odrive_can_dma_set_controller_modes()`.
 * To read the true value from the drive, use USB/Fibre or extend firmware.
 */

#include "odrive_can_dma.h"

#include "stm32f4xx_hal.h"

/**
 * CAN node ID of the ODrive axis you talk to (`odrv0.axis0.config.can_node_id`).
 * Must be 0 .. ODRIVE_CAN_NODE_ID_MAX (6 bits in the simple protocol).
 */
#define ODRIVE_EXAMPLE_CAN_NODE_ID 0u

/* CubeMX: `extern CAN_HandleTypeDef hcan1;` (adjust name to match your project). */
extern CAN_HandleTypeDef hcan1;

/**
 * Call from `HAL_CAN_RxFifo0MsgPendingCallback` in `stm32f4xx_it.c`:
 *
 *   void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
 *       odrive_can_dma_example_on_rx_fifo0(hcan);
 *   }
 */
void odrive_can_dma_example_on_rx_fifo0(CAN_HandleTypeDef *hcan)
{
    odrive_can_dma_on_rx_fifo0(hcan);
}

/**
 * One-time setup after `MX_CAN1_Init()` / `HAL_CAN_Start()` (use your CAN instance).
 * Binds this driver to `ODRIVE_EXAMPLE_CAN_NODE_ID` (same as ODrive `can_node_id`).
 */
void odrive_can_dma_example_init(void)
{
    (void)odrive_can_dma_init(&hcan1, ODRIVE_EXAMPLE_CAN_NODE_ID);

    /* At runtime you can retarget another drive, e.g. odrive_can_dma_set_node_id(3u); */

    (void)odrive_can_dma_clear_errors();

    /* Velocity control + passthrough inputs (matches typical `velocity_mode.txt`). */
    (void)odrive_can_dma_set_controller_modes(ODRIVE_CONTROL_MODE_VELOCITY, ODRIVE_INPUT_MODE_PASSTHROUGH);

    /* Request closed loop before sending `input_vel`. */
    (void)odrive_can_dma_set_requested_state(ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);

    /* Drain TX queue so the above reach the bus (call periodically in real code). */
    odrive_can_dma_process_tx(&hcan1);
}

/**
 * Periodic tick (e.g. every 1–10 ms from `HAL_GetTick()` or a timer).
 * - Flushes the TX queue
 * - Sends a velocity setpoint (turns/s)
 * - Reads back the **last written** control_mode / input_mode (see file header)
 * - Optionally polls encoder position via CAN RTR
 */
void odrive_can_dma_example_run_periodic(void)
{
    odrive_can_dma_process_tx(&hcan1);

    /* All commands below go to node `odrive_can_dma_get_node_id()` (here: ODRIVE_EXAMPLE_CAN_NODE_ID). */
    (void)odrive_can_dma_set_input_vel(0.25f, 0.0f);

    ODriveControlMode cm;
    ODriveInputMode im;
    odrive_can_dma_get_last_controller_modes_written(&cm, &im);
    (void)cm;
    (void)im;
    /* Here `cm` should be `ODRIVE_CONTROL_MODE_VELOCITY` if the set_modes call
     * succeeded and nothing else overwrote the cache. */

    /* Optional: request encoder estimates; reply handled in IRQ → snapshot. */
    (void)odrive_can_dma_request_encoder_estimates();
    ODriveCanDmaEncoderSnapshot snap;
    if (odrive_can_dma_get_encoder_snapshot(&snap)) {
        (void)snap.encoder_pos_turns;
        (void)snap.encoder_vel_turns_s;
    }
}
