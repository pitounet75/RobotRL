/**
 * @file odrive_can_async.h
 * @brief CAN ODrive (protocole simple) avec **requêtes / réponses asynchrones** sous **FreeRTOS**.
 *
 * - **ISR** : lecture FIFO0 → `xQueueSendFromISR` (léger).
 * - **Tâche interne** : associe la trame reçue à une requête en attente, appelle le **callback**
 *   dans un **contexte de tâche** (pas dans l’IRQ).
 * - **TX** : `HAL_CAN_AddTxMessage` sous mutex (bxCAN F4 : pas de DMA matériel CAN).
 *
 * Dépendances : **FreeRTOS**, `stm32*_hal_can.h`, `../odrive_can/odrive_can_protocol.h`.
 *
 * Intégration :
 * 1. `HAL_CAN_Start()` + filtres / masques (comme d’habitude, **avant** les requêtes).
 * 2. `odrive_can_async_init(&hcan1);`
 * 3. `odrive_can_async_start();`  (crée la tâche worker FreeRTOS)
 * 4. Dans `HAL_CAN_RxFifo0MsgPendingCallback` : `odrive_can_async_on_rx_fifo0_isr(hcan);`
 * 5. `HAL_CAN_ActivateNotification(..., CAN_IT_RX_FIFO0_MSG_PENDING);` (fait aussi par `init`).
 */
#ifndef ODRIVE_CAN_ASYNC_H
#define ODRIVE_CAN_ASYNC_H

#include "../odrive_can/odrive_can_protocol.h"
#include <stdbool.h>
#include <stdint.h>

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

/** Statut passé au callback de réponse. */
typedef enum {
    ODRIVE_CAN_ASYNC_OK = 0,
    ODRIVE_CAN_ASYNC_TIMEOUT = 1,
    ODRIVE_CAN_ASYNC_CANCELLED = 2,
} ODriveCanAsyncStatus;

/**
 * Callback de complétion (appelé depuis la **tâche worker**, pas l’ISR).
 * Restez **léger** : pas de printf long, pas d’attente bloquante.
 */
typedef void (*ODriveCanAsyncReplyCb)(void *user_ctx, ODriveCanAsyncStatus status, const uint8_t *payload,
                                      uint8_t dlc, uint32_t std_id);

bool odrive_can_async_init(CAN_HandleTypeDef *hcan);
void odrive_can_async_deinit(void);

/** Crée la tâche FreeRTOS interne (priorité modérée). */
bool odrive_can_async_start(void);
void odrive_can_async_stop(void);

/** À appeler depuis `HAL_CAN_RxFifo0MsgPendingCallback` uniquement. */
void odrive_can_async_on_rx_fifo0_isr(CAN_HandleTypeDef *hcan);

/**
 * Envoie une trame **sans** attente de réponse (pas de callback).
 */
bool odrive_can_async_send_data(uint32_t node_id, ODriveCanMsg cmd, const uint8_t *data, uint8_t dlc);

/**
 * Envoie une trame (souvent **RTR** pour les `MSG_GET_*`) et enregistre un callback
 * pour la **prochaine trame données** avec le **même** `std_id = odrive_can_std_id(node, cmd)`.
 *
 * @param timeout_ms 0 = défaut interne (~100 ms).
 * @return false si file d’attente pleine ou mutex / bus indisponible.
 *
 * @note Une seule requête en attente par **(node_id, cmd)** à la fois ; sinon `false`.
 */
bool odrive_can_async_request(uint32_t node_id, ODriveCanMsg cmd, bool rtr, const uint8_t *tx_data, uint8_t tx_dlc,
                              ODriveCanAsyncReplyCb on_done, void *user_ctx, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_ASYNC_H */
