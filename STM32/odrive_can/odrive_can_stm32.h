/**
 * @file odrive_can_stm32.h
 * @brief STM32 HAL CAN (classic bxCAN) transport for ODrive simple protocol.
 *
 * Include your MCU HAL before this header, e.g. `#include "stm32f4xx_hal.h"`
 * (same stack as ODrive v3 / STM32F405).
 *
 * For STM32 FDCAN targets, use `odrive_can_protocol.h` with your own HAL calls.
 */
#ifndef ODRIVE_CAN_STM32_H
#define ODRIVE_CAN_STM32_H

#include "odrive_can_protocol.h"
#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F4xx)
#include "stm32f4xx_hal_can.h"
#elif defined(STM32F7xx)
#include "stm32f7xx_hal_can.h"
#elif defined(STM32L4xx)
#include "stm32l4xx_hal_can.h"
#else
#include "stm32f4xx_hal_can.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t id;
    bool is_extended;
    bool is_rtr;
    uint8_t dlc;
    uint8_t data[8];
} ODriveCanFrame;

/**
 * Transmit one frame to a specific ODrive (by CAN node id).
 * @param node_id 0 .. ODRIVE_CAN_NODE_ID_MAX (configure each ODrive’s `can_node_id`).
 */
bool odrive_can_send(CAN_HandleTypeDef *hcan, uint32_t node_id, ODriveCanMsg cmd, bool remote,
                     const uint8_t *data, uint8_t dlc);

/** Convenience: data frame, DLC clamped to 8. */
bool odrive_can_send_data(CAN_HandleTypeDef *hcan, uint32_t node_id, ODriveCanMsg cmd,
                          const uint8_t *data, uint8_t dlc);

/** RTR request (used for all MSG_GET_* that respond with data on ODrive v0.5.1). */
bool odrive_can_send_rtr(CAN_HandleTypeDef *hcan, uint32_t node_id, ODriveCanMsg cmd);

/**
 * Pop one frame from RX FIFO0 then FIFO1 (same order as typical polling).
 * @return true if a frame was returned.
 */
bool odrive_can_receive(CAN_HandleTypeDef *hcan, ODriveCanFrame *out);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_STM32_H */
