/**
 * @file odrive_can_async.h
 * @brief ODrive simple CAN with async request/response under FreeRTOS.
 */
#ifndef ODRIVE_CAN_ASYNC_H
#define ODRIVE_CAN_ASYNC_H

#include "odrive_can_protocol.h"
#include "odrive_can_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ODRIVE_CAN_ASYNC_OK = 0,
    ODRIVE_CAN_ASYNC_TIMEOUT = 1,
    ODRIVE_CAN_ASYNC_CANCELLED = 2,
} ODriveCanAsyncStatus;

typedef void (*ODriveCanAsyncReplyCb)(void *user_ctx, ODriveCanAsyncStatus status, const uint8_t *payload,
                                      uint8_t dlc, uint32_t std_id);

bool odrive_can_async_init(ODriveCanHalHandle *hcan);
void odrive_can_async_deinit(void);

bool odrive_can_async_start(void);
void odrive_can_async_stop(void);

/** Call from RX interrupt callback (FDCAN FIFO0 or bxCAN FIFO0). */
void odrive_can_async_on_rx_fifo0_isr(ODriveCanHalHandle *hcan);

bool odrive_can_async_send_data(uint32_t node_id, ODriveCanMsg cmd, const uint8_t *data, uint8_t dlc);

bool odrive_can_async_request(uint32_t node_id, ODriveCanMsg cmd, bool rtr, const uint8_t *tx_data, uint8_t tx_dlc,
                              ODriveCanAsyncReplyCb on_done, void *user_ctx, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_ASYNC_H */
