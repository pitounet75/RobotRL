/**
 * @file odrive_can_dma.c
 * @brief File TX + IRQ RX pour ODrive CAN simple (bxCAN HAL).
 */

#include "odrive_can_dma.h"
#include <string.h>

#ifndef ODRIVE_CAN_DMA_TX_QUEUE_DEPTH
#define ODRIVE_CAN_DMA_TX_QUEUE_DEPTH 16u
#endif

typedef struct {
    uint32_t std_id;
    uint8_t rtr;
    uint8_t dlc;
    uint8_t data[8];
} ODriveCanDmaTxSlot;

static CAN_HandleTypeDef *s_hcan;
static uint32_t s_node_id;

static ODriveCanDmaTxSlot s_tx_q[ODRIVE_CAN_DMA_TX_QUEUE_DEPTH];
static volatile uint32_t s_tx_head;
static volatile uint32_t s_tx_tail;

static ODriveCanDmaEncoderSnapshot s_enc_snap;

static ODriveControlMode s_last_cm_written = ODRIVE_CONTROL_MODE_POSITION;
static ODriveInputMode s_last_im_written = ODRIVE_INPUT_MODE_PASSTHROUGH;

static uint32_t s_hal_ticks_ms(void)
{
    return HAL_GetTick();
}

static bool tx_queue_push(const ODriveCanDmaTxSlot *item)
{
    uint32_t next = (s_tx_head + 1u) % ODRIVE_CAN_DMA_TX_QUEUE_DEPTH;
    if (next == s_tx_tail) {
        return false;
    }
    s_tx_q[s_tx_head] = *item;
    s_tx_head = next;
    return true;
}

static const ODriveCanDmaTxSlot *tx_queue_peek(void)
{
    if (s_tx_head == s_tx_tail) {
        return NULL;
    }
    return &s_tx_q[s_tx_tail];
}

static void tx_queue_drop_head(void)
{
    if (s_tx_head != s_tx_tail) {
        s_tx_tail = (s_tx_tail + 1u) % ODRIVE_CAN_DMA_TX_QUEUE_DEPTH;
    }
}

static bool hal_tx_one(CAN_HandleTypeDef *hcan, const ODriveCanDmaTxSlot *slot)
{
    CAN_TxHeaderTypeDef hdr = {0};
    hdr.StdId = slot->std_id & 0x7FFu;
    hdr.IDE = CAN_ID_STD;
    hdr.RTR = slot->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    hdr.DLC = slot->dlc > 8u ? 8u : slot->dlc;
    hdr.TransmitGlobalTime = DISABLE;

    uint8_t payload[8];
    memset(payload, 0, sizeof(payload));
    if (slot->rtr == 0u) {
        memcpy(payload, slot->data, hdr.DLC);
    }

    uint32_t mb = 0;
    return HAL_CAN_AddTxMessage(hcan, &hdr, payload, &mb) == HAL_OK;
}

bool odrive_can_dma_init(CAN_HandleTypeDef *hcan, uint32_t default_node_id)
{
    if (hcan == NULL) {
        return false;
    }
    s_hcan = hcan;
    s_node_id = default_node_id & ODRIVE_CAN_NODE_ID_MAX;
    s_tx_head = s_tx_tail = 0u;
    memset(&s_enc_snap, 0, sizeof(s_enc_snap));

    (void)HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    return true;
}

void odrive_can_dma_set_node_id(uint32_t node_id)
{
    s_node_id = node_id & ODRIVE_CAN_NODE_ID_MAX;
}

uint32_t odrive_can_dma_get_node_id(void)
{
    return s_node_id;
}

void odrive_can_dma_process_tx(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL || hcan != s_hcan) {
        return;
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0u) {
        const ODriveCanDmaTxSlot *slot = tx_queue_peek();
        if (slot == NULL) {
            break;
        }
        if (!hal_tx_one(hcan, slot)) {
            break;
        }
        tx_queue_drop_head();
    }
}

void odrive_can_dma_on_rx_fifo0(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL || hcan != s_hcan) {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0u) {
        CAN_RxHeaderTypeDef hdr;
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK) {
            break;
        }

        if (hdr.IDE != CAN_ID_STD || hdr.RTR == CAN_RTR_REMOTE) {
            continue;
        }
        uint8_t dlc = hdr.DLC > 8u ? 8u : hdr.DLC;
        if (dlc < 8u) {
            continue;
        }

        uint32_t std_id = hdr.StdId;
        uint32_t nid = odrive_can_node_from_id(std_id);
        uint8_t cmd = odrive_can_cmd_from_id(std_id);
        if (nid != s_node_id) {
            continue;
        }
        if (cmd != (uint8_t)(ODRIVE_MSG_GET_ENCODER_ESTIMATES & 0x1Fu)) {
            continue;
        }

        float pos, vel;
        memcpy(&pos, data, sizeof(pos));
        memcpy(&vel, data + 4, sizeof(vel));
        s_enc_snap.encoder_pos_turns = pos;
        s_enc_snap.encoder_vel_turns_s = vel;
        s_enc_snap.last_update_ms = s_hal_ticks_ms();
        s_enc_snap.valid = true;
    }
}

static bool enqueue_std(uint32_t std_id, bool rtr, const uint8_t *data, uint8_t dlc)
{
    ODriveCanDmaTxSlot slot;
    slot.std_id = std_id;
    slot.rtr = rtr ? 1u : 0u;
    slot.dlc = dlc > 8u ? 8u : dlc;
    if (data != NULL) {
        memcpy(slot.data, data, slot.dlc);
    } else {
        memset(slot.data, 0, sizeof(slot.data));
    }
    return tx_queue_push(&slot);
}

bool odrive_can_dma_set_input_vel(float vel_turns_s, float torque_ff_nm)
{
    uint8_t buf[8];
    memcpy(buf, &vel_turns_s, sizeof(float));
    memcpy(buf + 4, &torque_ff_nm, sizeof(float));
    return enqueue_std(odrive_can_std_id(s_node_id, ODRIVE_MSG_SET_INPUT_VEL), false, buf, 8);
}

bool odrive_can_dma_set_input_pos(float pos_turns, float vel_ff_turns_s, float torque_ff_nm)
{
    uint8_t buf[8];
    odrive_can_pack_set_input_pos(buf, pos_turns, vel_ff_turns_s, torque_ff_nm);
    return enqueue_std(odrive_can_std_id(s_node_id, ODRIVE_MSG_SET_INPUT_POS), false, buf, 8);
}

bool odrive_can_dma_set_input_torque(float torque_nm)
{
    uint8_t buf[8];
    odrive_can_pack_set_input_torque(buf, torque_nm);
    return enqueue_std(odrive_can_std_id(s_node_id, ODRIVE_MSG_SET_INPUT_TORQUE), false, buf, 8);
}

bool odrive_can_dma_set_controller_modes(ODriveControlMode control_mode, ODriveInputMode input_mode)
{
    uint8_t buf[8];
    odrive_can_pack_set_controller_modes(buf, (int32_t)control_mode, (int32_t)input_mode);
    if (!enqueue_std(odrive_can_std_id(s_node_id, ODRIVE_MSG_SET_CONTROLLER_MODES), false, buf, 8)) {
        return false;
    }
    s_last_cm_written = control_mode;
    s_last_im_written = input_mode;
    return true;
}

bool odrive_can_dma_set_requested_state(ODriveAxisState state)
{
    uint8_t buf[8];
    int16_t st = (int16_t)state;
    memcpy(buf, &st, sizeof(st));
    memset(buf + 2, 0, 6);
    return enqueue_std(odrive_can_std_id(s_node_id, ODRIVE_MSG_SET_AXIS_REQUESTED_STATE), false, buf, 8);
}

bool odrive_can_dma_clear_errors(void)
{
    uint8_t buf[8];
    memset(buf, 0, sizeof(buf));
    return enqueue_std(odrive_can_std_id(s_node_id, ODRIVE_MSG_CLEAR_ERRORS), false, buf, 8);
}

bool odrive_can_dma_request_encoder_estimates(void)
{
    return enqueue_std(odrive_can_std_id(s_node_id, ODRIVE_MSG_GET_ENCODER_ESTIMATES), true, NULL, 8);
}

bool odrive_can_dma_get_encoder_snapshot(ODriveCanDmaEncoderSnapshot *out)
{
    if (out == NULL) {
        return false;
    }
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    *out = s_enc_snap;
    if (!prim) {
        __enable_irq();
    }
    return out->valid;
}

void odrive_can_dma_get_last_controller_modes_written(ODriveControlMode *out_cm, ODriveInputMode *out_im)
{
    if (out_cm != NULL) {
        *out_cm = s_last_cm_written;
    }
    if (out_im != NULL) {
        *out_im = s_last_im_written;
    }
}
