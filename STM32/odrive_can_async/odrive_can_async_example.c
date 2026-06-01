/**
 * @file odrive_can_async_example.c
 *
 * **STM32H7**: `odrive_can_fdcan_start(&hfdcan1)` then init. In `HAL_FDCAN_RxFifo0Callback`:
 *   `odrive_can_async_example_on_rx_fifo0(hfdcan);`
 *
 * **STM32F4**: `HAL_CAN_Start(&hcan1)` + filters. In `HAL_CAN_RxFifo0MsgPendingCallback`:
 *   `odrive_can_async_example_on_rx_fifo0(hcan);`
 */

#include "odrive_can_async_example.h"
#include "odrive_can_async.h"

#include <string.h>

#if defined(STM32H7xx)
#include "stm32h7xx_hal.h"
extern FDCAN_HandleTypeDef hfdcan1;
#define ODRIVE_EXAMPLE_CAN_HANDLE (&hfdcan1)
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
extern CAN_HandleTypeDef hcan1;
#define ODRIVE_EXAMPLE_CAN_HANDLE (&hcan1)
#else
#error "odrive_can_async_example: add handle extern for your MCU family"
#endif

#define ODRIVE_EXAMPLE_CAN_NODE_ID 0u

static void example_on_encoder_reply(void *user_ctx, ODriveCanAsyncStatus status, const uint8_t *payload,
                                     uint8_t dlc, uint32_t std_id)
{
    (void)user_ctx;
    (void)std_id;
    if (status != ODRIVE_CAN_ASYNC_OK || payload == NULL || dlc < 8U) {
        return;
    }
    float pos_turns;
    float vel_turns_s;
    memcpy(&pos_turns, payload + 0, sizeof(float));
    memcpy(&vel_turns_s, payload + 4, sizeof(float));
    (void)pos_turns;
    (void)vel_turns_s;
}

void odrive_can_async_example_on_rx_fifo0(ODriveCanHalHandle *hcan)
{
    odrive_can_async_on_rx_fifo0_isr(hcan);
}

void odrive_can_async_example_init(void)
{
#if ODRIVE_CAN_HAL_FDCAN
    (void)odrive_can_fdcan_start(ODRIVE_EXAMPLE_CAN_HANDLE);
#endif
    (void)odrive_can_async_init(ODRIVE_EXAMPLE_CAN_HANDLE);
    (void)odrive_can_async_start();
}

void odrive_can_async_example_run_periodic(void)
{
    (void)odrive_can_async_request(ODRIVE_EXAMPLE_CAN_NODE_ID, ODRIVE_MSG_GET_ENCODER_ESTIMATES, true, NULL, 8U,
                                   example_on_encoder_reply, NULL, 50U);
}
