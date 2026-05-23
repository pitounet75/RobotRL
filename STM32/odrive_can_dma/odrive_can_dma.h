/**
 * @file odrive_can_dma.h
 * @brief Pilote CAN « non bloquant » vers ODrive (protocole simple v0.5.x).
 *
 * @note DMA matériel : le **bxCAN** (STM32F405 / ODrive v3) **n’a pas** de
 *       canal DMA RX/TX pour le CAN comme l’UART. Ce module utilise une **file
 *       d’envoi** + un **tampon circulaire RX** rempli depuis l’**IRQ FIFO0**
 *       (`HAL_CAN_RxFifo0MsgPendingCallback`), pour libérer la CPU comme un
 *       transfert piloté par événements.
 *
 * Inclure `../odrive_can/odrive_can_protocol.h` (ODriveCanMsg, IDs, packers).
 *
 * À brancher : dans `HAL_CAN_RxFifo0MsgPendingCallback`, appeler
 * `odrive_can_dma_on_rx_fifo0(hcan)` ; dans la boucle principale, appeler
 * `odrive_can_dma_process_tx(hcan)`.
 *
 * **Cible CAN (node ID) :** toutes les commandes d’émission utilisent le même
 * identifiant d’axe **`odrv0.axis0.config.can_node_id`** côté ODrive, fixé par
 * `odrive_can_dma_init(..., default_node_id)` puis modifiable avec
 * `odrive_can_dma_set_node_id()`. L’ID de trame est
 * `(node_id << ODRIVE_CAN_CMD_ID_BITS) | commande` — voir `odrive_can_std_id()`.
 */
#ifndef ODRIVE_CAN_DMA_H
#define ODRIVE_CAN_DMA_H

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

/** États d’axe (ODrive.Axis.AxisState) — alignés sur `tools/odrive/enums.py`. */
typedef enum {
    ODRIVE_AXIS_STATE_UNDEFINED = 0,
    ODRIVE_AXIS_STATE_IDLE = 1,
    ODRIVE_AXIS_STATE_STARTUP_SEQUENCE = 2,
    ODRIVE_AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
    ODRIVE_AXIS_STATE_MOTOR_CALIBRATION = 4,
    ODRIVE_AXIS_STATE_SENSORLESS_CONTROL = 5,
    ODRIVE_AXIS_STATE_ENCODER_INDEX_SEARCH = 6,
    ODRIVE_AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
    ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
    ODRIVE_AXIS_STATE_LOCKIN_SPIN = 9,
    ODRIVE_AXIS_STATE_ENCODER_DIR_FIND = 10,
    ODRIVE_AXIS_STATE_HOMING = 11,
} ODriveAxisState;

/** ODrive.Controller.ControlMode */
typedef enum {
    ODRIVE_CONTROL_MODE_VOLTAGE = 0,
    ODRIVE_CONTROL_MODE_TORQUE = 1,
    ODRIVE_CONTROL_MODE_VELOCITY = 2,
    ODRIVE_CONTROL_MODE_POSITION = 3,
} ODriveControlMode;

/** ODrive.Controller.InputMode */
typedef enum {
    ODRIVE_INPUT_MODE_INACTIVE = 0,
    ODRIVE_INPUT_MODE_PASSTHROUGH = 1,
    ODRIVE_INPUT_MODE_VEL_RAMP = 2,
    ODRIVE_INPUT_MODE_POS_FILTER = 3,
    ODRIVE_INPUT_MODE_MIX_CHANNELS = 4,
    ODRIVE_INPUT_MODE_TRAP_TRAJ = 5,
    ODRIVE_INPUT_MODE_TORQUE_RAMP = 6,
    ODRIVE_INPUT_MODE_MIRROR = 7,
} ODriveInputMode;

typedef struct {
    float encoder_pos_turns;
    float encoder_vel_turns_s;
    uint32_t last_update_ms;
    bool valid;
} ODriveCanDmaEncoderSnapshot;

/**
 * Initialise le driver (enregistre le `CAN_HandleTypeDef` et le **node_id CAN**
 * de l’axe cible, identique à `odrv0.axis0.config.can_node_id` sur l’ODrive).
 * N’active pas les filtres CAN : configurez-les dans CubeMX (souvent masque large).
 */
bool odrive_can_dma_init(CAN_HandleTypeDef *hcan, uint32_t default_node_id);

void odrive_can_dma_set_node_id(uint32_t node_id);
uint32_t odrive_can_dma_get_node_id(void);

/**
 * Toutes les fonctions `odrive_can_dma_set_*` / `*_clear_errors` / `*_request_*`
 * ci‑dessous envoient vers le **même** nœud : `odrive_can_dma_get_node_id()`.
 * L’arbitration ID standard est `odrive_can_std_id(node_id, ODRIVE_MSG_…)`.
 */

/** Vide la file TX (appeler périodiquement depuis la boucle principale). */
void odrive_can_dma_process_tx(CAN_HandleTypeDef *hcan);

/** À appeler depuis `HAL_CAN_RxFifo0MsgPendingCallback`. */
void odrive_can_dma_on_rx_fifo0(CAN_HandleTypeDef *hcan);

/* --- Commandes vers l’ODrive (file TX, non bloquant si la file n’est pas pleine) --- */

bool odrive_can_dma_set_input_vel(float vel_turns_s, float torque_ff_nm);
bool odrive_can_dma_set_input_pos(float pos_turns, float vel_ff_turns_s, float torque_ff_nm);
bool odrive_can_dma_set_input_torque(float torque_nm);
bool odrive_can_dma_set_controller_modes(ODriveControlMode control_mode, ODriveInputMode input_mode);
bool odrive_can_dma_set_requested_state(ODriveAxisState state);
bool odrive_can_dma_clear_errors(void);

/**
 * Demande les estimations encodeur (position / vitesse en tours) : envoie un **RTR**
 * `MSG_GET_ENCODER_ESTIMATES`. La réponse arrive en trame **données** même ID.
 */
bool odrive_can_dma_request_encoder_estimates(void);

/** Dernière position/vitesse **encodeur** (réponse RTR `GET_ENCODER_ESTIMATES`, champs `pos_estimate` / `vel_estimate` côté ODrive). Ce n’est pas `controller.pos_setpoint` (non exposé sur le CAN simple). */
bool odrive_can_dma_get_encoder_snapshot(ODriveCanDmaEncoderSnapshot *out);

/**
 * Derniers `control_mode` / `input_mode` **acceptés en file TX** après
 * `odrive_can_dma_set_controller_modes` (succès de la mise en file).
 *
 * @note Le protocole CAN **simple** d’ODrive **ne fournit pas** de trame de
 *       **lecture** de `controller.config.control_mode`. Pour connaître la
 *       valeur réelle côté ODrive, il faut USB/Fibre ou étendre le firmware.
 */
void odrive_can_dma_get_last_controller_modes_written(ODriveControlMode *out_cm, ODriveInputMode *out_im);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_DMA_H */
