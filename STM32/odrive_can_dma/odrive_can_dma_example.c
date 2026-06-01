/**
 * @file odrive_can_dma_example.c
 *
 * **STM32H7**: `HAL_FDCAN_RxFifo0Callback` → `odrive_can_dma_example_on_rx_fifo0(hfdcan)`.
 * **STM32F4**: `HAL_CAN_RxFifo0MsgPendingCallback` → same with `hcan`.
 */

#include "odrive_can_dma_example.h"

#if defined(STM32H7xx)
#include "stm32h7xx_hal.h"
extern FDCAN_HandleTypeDef hfdcan1;
#define ODRIVE_EXAMPLE_CAN_HANDLE (&hfdcan1)
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
extern CAN_HandleTypeDef hcan1;
#define ODRIVE_EXAMPLE_CAN_HANDLE (&hcan1)
#else
#error "odrive_can_dma_example: add handle extern for your MCU family"
#endif

#define ODRIVE_EXAMPLE_CAN_NODE_ID 0u

void odrive_can_dma_example_on_rx_fifo0(ODriveCanHalHandle *hcan)
{
    odrive_can_dma_on_rx_fifo0(hcan);
}

void odrive_can_dma_example_init(void)
{
#if ODRIVE_CAN_HAL_FDCAN
    (void)odrive_can_fdcan_start(ODRIVE_EXAMPLE_CAN_HANDLE);
#endif
    (void)odrive_can_dma_init(ODRIVE_EXAMPLE_CAN_HANDLE, ODRIVE_EXAMPLE_CAN_NODE_ID);
    (void)odrive_can_dma_clear_errors();
    (void)odrive_can_dma_set_controller_modes(ODRIVE_CONTROL_MODE_VELOCITY, ODRIVE_INPUT_MODE_PASSTHROUGH);
    (void)odrive_can_dma_set_requested_state(ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrive_can_dma_process_tx(ODRIVE_EXAMPLE_CAN_HANDLE);
}

void odrive_can_dma_example_run_periodic(void)
{
    odrive_can_dma_process_tx(ODRIVE_EXAMPLE_CAN_HANDLE);
    (void)odrive_can_dma_set_input_vel(0.25f, 0.0f);

    ODriveControlMode cm;
    ODriveInputMode im;
    odrive_can_dma_get_last_controller_modes_written(&cm, &im);
    (void)cm;
    (void)im;

    (void)odrive_can_dma_request_encoder_estimates();
    odrive_can_dma_process_tx(ODRIVE_EXAMPLE_CAN_HANDLE);

    ODriveCanDmaEncoderSnapshot snap;
    if (odrive_can_dma_get_encoder_snapshot(&snap)) {
        (void)snap.encoder_pos_turns;
        (void)snap.encoder_vel_turns_s;
    }
}
