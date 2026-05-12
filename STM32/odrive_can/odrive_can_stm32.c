/**
 * @file odrive_can_stm32.c
 * Classic STM32 bxCAN implementation (HAL_CAN).
 */

#include "odrive_can_stm32.h"
#include <string.h>

static HAL_StatusTypeDef odrive_hal_add_tx(CAN_HandleTypeDef *hcan, uint32_t std_id, bool use_ext,
                                           uint32_t ext_id, bool remote, const uint8_t *data,
                                           uint8_t dlc)
{
    CAN_TxHeaderTypeDef header = {0};

    if (use_ext) {
        header.IDE = CAN_ID_EXT;
        header.ExtId = ext_id & 0x1FFFFFFFu;
    } else {
        header.IDE = CAN_ID_STD;
        header.StdId = std_id & 0x7FFu;
    }

    header.RTR = remote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    header.DLC = dlc > 8u ? 8u : dlc;
    header.TransmitGlobalTime = DISABLE;

    uint8_t payload[8];
    if (!remote && data != NULL) {
        memcpy(payload, data, header.DLC);
    } else {
        memset(payload, 0, sizeof(payload));
    }

    uint32_t mailbox = 0;
    return HAL_CAN_AddTxMessage(hcan, &header, payload, &mailbox);
}

bool odrive_can_send(CAN_HandleTypeDef *hcan, uint32_t node_id, ODriveCanMsg cmd, bool remote,
                     const uint8_t *data, uint8_t dlc)
{
    if (hcan == NULL) {
        return false;
    }

    uint32_t id = odrive_can_std_id(node_id, cmd);
    /* Standard simple protocol uses 11-bit IDs only (matches ODrive v3 `interface_can.cpp`). */
    if (odrive_hal_add_tx(hcan, id, false, 0, remote, data, dlc) != HAL_OK) {
        return false;
    }
    return true;
}

bool odrive_can_send_data(CAN_HandleTypeDef *hcan, uint32_t node_id, ODriveCanMsg cmd,
                          const uint8_t *data, uint8_t dlc)
{
    return odrive_can_send(hcan, node_id, cmd, false, data, dlc);
}

bool odrive_can_send_rtr(CAN_HandleTypeDef *hcan, uint32_t node_id, ODriveCanMsg cmd)
{
    /* Remote frame: DLC 8 matches ODrive reply length for GET callbacks */
    return odrive_can_send(hcan, node_id, cmd, true, NULL, 8);
}

bool odrive_can_receive(CAN_HandleTypeDef *hcan, ODriveCanFrame *out)
{
    if (hcan == NULL || out == NULL) {
        return false;
    }

    CAN_RxHeaderTypeDef header;
    uint32_t fifo = CAN_RX_FIFO0;

    if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0u) {
        if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) == 0u) {
            return false;
        }
        fifo = CAN_RX_FIFO1;
    }

    if (HAL_CAN_GetRxMessage(hcan, fifo, &header, out->data) != HAL_OK) {
        return false;
    }

    out->is_extended = (header.IDE == CAN_ID_EXT);
    out->id = out->is_extended ? header.ExtId : header.StdId;
    out->is_rtr = (header.RTR == CAN_RTR_REMOTE);
    out->dlc = header.DLC > 8u ? 8u : header.DLC;
    return true;
}
