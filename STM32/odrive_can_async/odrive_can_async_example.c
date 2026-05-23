/**
 * @file odrive_can_async_example.c
 * @brief Reference wiring: IRQ → ISR queue → worker task → **user callback**.
 *
 * Copy patterns into your `stm32f4xx_it.c` / `freertos.c` / `main.c`, or compile this file.
 *
 * **Note:** bxCAN on STM32F4 has **no** CAN DMA; this stack is **async** (non-blocking + callbacks).
 */

#include "odrive_can_async_example.h"

#include "odrive_can_async.h"

#include "stm32f4xx_hal.h"

#include <string.h>

/** CAN node ID of the ODrive axis (`odrv0.axis0.config.can_node_id`). */
#define ODRIVE_EXAMPLE_CAN_NODE_ID 0u

extern CAN_HandleTypeDef hcan1;

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

void odrive_can_async_example_on_rx_fifo0(CAN_HandleTypeDef *hcan)
{
    odrive_can_async_on_rx_fifo0_isr(hcan);
}

void odrive_can_async_example_init(void)
{
    (void)odrive_can_async_init(&hcan1);
    (void)odrive_can_async_start();
}

void odrive_can_async_example_run_periodic(void)
{
    /* RTR GET_ENCODER_ESTIMATES — reply matched by same std_id in worker task. */
    (void)odrive_can_async_request(ODRIVE_EXAMPLE_CAN_NODE_ID, ODRIVE_MSG_GET_ENCODER_ESTIMATES, true, NULL, 8U,
                                   example_on_encoder_reply, NULL, 50U);
}
