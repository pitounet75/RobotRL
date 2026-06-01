/**
 * @file odrive_can_stm32.h
 * @brief STM32 HAL transport for ODrive simple CAN protocol.
 *
 * - **STM32H7**: FDCAN in classic CAN mode (`FDCAN_FRAME_CLASSIC`). Call
 *   `odrive_can_fdcan_start(&hfdcan1)` after CubeMX init.
 * - **STM32F4/F7/L4**: legacy bxCAN (`HAL_CAN_Start` + filters as usual).
 *
 * Include your MCU HAL first, e.g. `#include "stm32h7xx_hal.h"`.
 */
#ifndef ODRIVE_CAN_STM32_H
#define ODRIVE_CAN_STM32_H

#include "odrive_can_protocol.h"
#include "odrive_can_hal.h"
#include <stdbool.h>
#include <stdint.h>

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

bool odrive_can_send(ODriveCanHalHandle *hcan, uint32_t node_id, ODriveCanMsg cmd, bool remote,
                     const uint8_t *data, uint8_t dlc);

bool odrive_can_send_data(ODriveCanHalHandle *hcan, uint32_t node_id, ODriveCanMsg cmd,
                          const uint8_t *data, uint8_t dlc);

bool odrive_can_send_rtr(ODriveCanHalHandle *hcan, uint32_t node_id, ODriveCanMsg cmd);

bool odrive_can_receive(ODriveCanHalHandle *hcan, ODriveCanFrame *out);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_STM32_H */
