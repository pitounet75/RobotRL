/**
 * @file odrive_can_hal.h
 * @brief MCU-specific CAN transport: STM32H7 FDCAN (classic CAN) or legacy bxCAN.
 */
#ifndef ODRIVE_CAN_HAL_H
#define ODRIVE_CAN_HAL_H

#include <stdbool.h>
#include <stdint.h>
#ifndef STM32H7xx
#define STM32H7xx
#endif
#if defined(STM32H7xx)
/* Full HAL first — do not include stm32h7xx_hal_fdcan.h alone (breaks HAL_StatusTypeDef). */
#include "stm32h7xx_hal.h"
typedef FDCAN_HandleTypeDef ODriveCanHalHandle;
#define ODRIVE_CAN_HAL_FDCAN 1
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
typedef CAN_HandleTypeDef ODriveCanHalHandle;
#define ODRIVE_CAN_HAL_FDCAN 0
#elif defined(STM32F7xx)
#include "stm32f7xx_hal.h"
typedef CAN_HandleTypeDef ODriveCanHalHandle;
#define ODRIVE_CAN_HAL_FDCAN 0
#elif defined(STM32L4xx)
#include "stm32l4xx_hal.h"
typedef CAN_HandleTypeDef ODriveCanHalHandle;
#define ODRIVE_CAN_HAL_FDCAN 0
#else
#include "stm32f4xx_hal.h"
typedef CAN_HandleTypeDef ODriveCanHalHandle;
#define ODRIVE_CAN_HAL_FDCAN 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if ODRIVE_CAN_HAL_FDCAN

/**
 * After `MX_FDCANx_Init()`: fix CubeMX FIFO depths if zero, apply accept-all filter,
 * start peripheral, enable RX FIFO0 new-message interrupt.
 */
bool odrive_can_fdcan_start(ODriveCanHalHandle *hfdcan);

/** Start FDCAN if not already BUSY (safe to call from tasks). */
bool odrive_can_fdcan_ensure_started(ODriveCanHalHandle *hfdcan);

/** Last hfdcan->State seen by ensure_started (HAL enum: 0 RESET, 1 READY, 2 BUSY). */
extern volatile uint32_t g_odrive_can_fdcan_hal_state;

/** Accept all 11-bit frames into RX FIFO0 (call after HAL_FDCAN_Init, state READY). */
bool odrive_can_fdcan_apply_global_accept_std(ODriveCanHalHandle *hfdcan);

void odrive_can_fdcan_recover_bus_off(ODriveCanHalHandle *hfdcan);

/** Non-zero if odrive_can_fdcan_start() failed (see values in odrive_can_hal.c). */
extern volatile uint32_t g_fdcan_start_fail_step;
/** hfdcan->State when start failed: 1=READY, 2=BUSY, 3=ERROR, 0=RESET. */
extern volatile uint32_t g_fdcan_hcan_state_at_fail;

#endif

/** Transmit standard 11-bit ID frame (classic CAN data or remote). */
bool odrive_can_hal_tx(ODriveCanHalHandle *hcan, uint32_t std_id, bool remote, const uint8_t *data,
                       uint8_t dlc);

/** Pop one RX frame from FIFO0, then FIFO1. @return false if both empty. */
bool odrive_can_hal_rx(ODriveCanHalHandle *hcan, uint32_t *std_id, bool *is_extended, bool *is_rtr,
                       uint8_t *data, uint8_t *dlc);

/** True if at least one TX slot is free (Tx FIFO queue on H7, mailbox on bxCAN). */
bool odrive_can_hal_tx_ready(ODriveCanHalHandle *hcan);

/** Block until the latest Tx-FIFO frame completes (TXBRP clear) or timeout. */
bool odrive_can_hal_wait_tx_fifo_empty(ODriveCanHalHandle *hcan, uint32_t timeout_ms);

void odrive_can_hal_rx_irq_enable(ODriveCanHalHandle *hcan);
void odrive_can_hal_rx_irq_disable(ODriveCanHalHandle *hcan);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_HAL_H */
