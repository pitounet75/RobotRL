/**
 * @file odrive_can_hal.h
 * @brief MCU-specific CAN transport: STM32H7 FDCAN (classic CAN) or legacy bxCAN.
 */
#ifndef ODRIVE_CAN_HAL_H
#define ODRIVE_CAN_HAL_H

#include <stdbool.h>
#include <stdint.h>
#define STM32H7xx
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

bool odrive_can_fdcan_apply_global_accept_std(ODriveCanHalHandle *hfdcan);

/** 1 = scope test only: one TX/s, no auto-retry on missing ACK (set 0 for normal ODrive). */
#ifndef ODRIVE_CAN_DIAG_SCOPE_PING
#define ODRIVE_CAN_DIAG_SCOPE_PING 0
#endif

#if ODRIVE_CAN_DIAG_SCOPE_PING
extern volatile uint32_t g_fdcan_scope_tx_attempts;
extern volatile uint32_t g_fdcan_scope_tx_ok;
extern volatile uint32_t g_fdcan_scope_tx_fail;
extern volatile uint32_t g_fdcan_scope_fdcan_psr;
#endif

#ifndef ODRIVE_CAN_SCOPE_PING_PERIOD_MS
#define ODRIVE_CAN_SCOPE_PING_PERIOD_MS 1000u
#endif

/** Scope test: AutoRetransmission off, then filter + start (call after MX_FDCAN1_Init). */
bool odrive_can_fdcan_start_scope_ping(ODriveCanHalHandle *hfdcan);

void odrive_can_fdcan_recover_bus_off(ODriveCanHalHandle *hfdcan);

/** Before MX_FDCAN1_Init: PA12 high → VP230 recessive (avoids bus clamp until AF init). */
void odrive_can_hal_gpio_safe_recessive(void);

/** FDCAN internal loopback (no transceiver). 1=pass 0=fail — set by odrive_can_fdcan_run_mcu_diagnostics(). */
extern volatile uint8_t g_fdcan_diag_internal_loopback_ok;
/** External loopback: jumper CANH–CANL on VP230 module first. */
extern volatile uint8_t g_fdcan_diag_external_loopback_ok;

/** Run internal (+ optional external) loopback; restores normal FDCAN init for odrive_can_fdcan_start(). */
bool odrive_can_fdcan_run_mcu_diagnostics(ODriveCanHalHandle *hfdcan, bool run_external_loopback);

/** 1 = DeInit/loopback before bus start (debug only; keep 0 when ODrive is on the bus). */
#ifndef ODRIVE_CAN_RUN_BOOT_DIAGNOSTICS
#define ODRIVE_CAN_RUN_BOOT_DIAGNOSTICS 0
#endif

extern volatile uint32_t g_fdcan_start_fail_step;

#endif

/** Transmit standard 11-bit ID frame (classic CAN data or remote). */
bool odrive_can_hal_tx(ODriveCanHalHandle *hcan, uint32_t std_id, bool remote, const uint8_t *data,
                       uint8_t dlc);

/** Pop one RX frame from FIFO0, then FIFO1. @return false if both empty. */
bool odrive_can_hal_rx(ODriveCanHalHandle *hcan, uint32_t *std_id, bool *is_extended, bool *is_rtr,
                       uint8_t *data, uint8_t *dlc);

/** True if at least one TX slot is free (Tx FIFO queue on H7, mailbox on bxCAN). */
bool odrive_can_hal_tx_ready(ODriveCanHalHandle *hcan);

void odrive_can_hal_rx_irq_enable(ODriveCanHalHandle *hcan);
void odrive_can_hal_rx_irq_disable(ODriveCanHalHandle *hcan);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_HAL_H */
