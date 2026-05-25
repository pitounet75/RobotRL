/**

 * @file odrive_can_dma.c

 */



#include "odrive_can_dma.h"

#include "app_config.h"

#include "FreeRTOS.h"
#include "semphr.h"

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



static ODriveCanHalHandle *s_hcan;

static uint32_t s_default_node_id;



static ODriveCanDmaTxSlot s_tx_q[ODRIVE_CAN_DMA_TX_QUEUE_DEPTH];

static volatile uint32_t s_tx_head;

static volatile uint32_t s_tx_tail;



static ODriveCanDmaEncoderSnapshot s_enc_snap[ODRIVE_CAN_DMA_DRIVE_COUNT];

static SemaphoreHandle_t s_tx_mtx;

volatile uint32_t g_odrive_can_tx_queue_full;
volatile uint32_t g_odrive_can_tx_hal_fail;

static ODriveControlMode s_last_cm_written = ODRIVE_CONTROL_MODE_POSITION;

static ODriveInputMode s_last_im_written = ODRIVE_INPUT_MODE_PASSTHROUGH;



static uint32_t s_hal_ticks_ms(void)

{

    return HAL_GetTick();

}



static int odrive_can_dma_node_index(uint32_t node_id)
{
    if (node_id == APP_ODRIVE_NODE0_ID) {
        return 0;
    }
    if (node_id == APP_ODRIVE_NODE1_ID) {
        return 1;
    }
    return -1;
}



static bool tx_queue_push_locked(const ODriveCanDmaTxSlot *item)
{
    uint32_t next = (s_tx_head + 1u) % ODRIVE_CAN_DMA_TX_QUEUE_DEPTH;
    if (next == s_tx_tail) {
        return false;
    }
    s_tx_q[s_tx_head] = *item;
    s_tx_head = next;
    return true;
}

static bool tx_queue_push(const ODriveCanDmaTxSlot *item)
{
    if (s_tx_mtx == NULL) {
        return tx_queue_push_locked(item);
    }
    if (xSemaphoreTake(s_tx_mtx, pdMS_TO_TICKS(2u)) != pdTRUE) {
        return false;
    }
    const bool ok = tx_queue_push_locked(item);
    (void)xSemaphoreGive(s_tx_mtx);
    return ok;
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



bool odrive_can_dma_init(ODriveCanHalHandle *hcan)

{

    if (hcan == NULL) {

        return false;

    }

    s_hcan = hcan;

    s_default_node_id = APP_ODRIVE_NODE0_ID;

    s_tx_head = s_tx_tail = 0u;
    g_odrive_can_tx_queue_full = 0u;
    g_odrive_can_tx_hal_fail = 0u;

    memset(s_enc_snap, 0, sizeof(s_enc_snap));

    if (s_tx_mtx == NULL) {
        s_tx_mtx = xSemaphoreCreateMutex();
        if (s_tx_mtx == NULL) {
            return false;
        }
    }

    return true;

}



void odrive_can_dma_set_node_id(uint32_t node_id)

{

    s_default_node_id = node_id & ODRIVE_CAN_NODE_ID_MAX;

}



uint32_t odrive_can_dma_get_node_id(void)

{

    return s_default_node_id;

}



void odrive_can_dma_process_tx(ODriveCanHalHandle *hcan)
{
    if (hcan == NULL || hcan != s_hcan) {
        return;
    }

#if ODRIVE_CAN_HAL_FDCAN
    if (!odrive_can_fdcan_ensure_started(hcan)) {
        return;
    }
#endif

    if (s_tx_mtx != NULL) {
        if (xSemaphoreTake(s_tx_mtx, pdMS_TO_TICKS(5u)) != pdTRUE) {
            return;
        }
    }

    while (odrive_can_hal_tx_ready(hcan)) {
        const ODriveCanDmaTxSlot *slot = tx_queue_peek();
        if (slot == NULL) {
            break;
        }
        if (!odrive_can_hal_tx(hcan, slot->std_id, slot->rtr != 0u, slot->data, slot->dlc)) {
            g_odrive_can_tx_hal_fail++;
            break;
        }
        tx_queue_drop_head();
    }

    if (s_tx_mtx != NULL) {
        (void)xSemaphoreGive(s_tx_mtx);
    }
}



void odrive_can_dma_on_rx_frame(uint32_t std_id, const uint8_t *data, uint8_t dlc)

{

    if (data == NULL || dlc < 8u) {

        return;

    }



    const uint32_t nid = odrive_can_node_from_id(std_id);

    const int idx = odrive_can_dma_node_index(nid);

    if (idx < 0) {

        return;

    }

    if (odrive_can_cmd_from_id(std_id) != (uint8_t)(ODRIVE_MSG_GET_ENCODER_ESTIMATES & 0x1Fu)) {

        return;

    }



    float pos;

    float vel;

    memcpy(&pos, data, sizeof(pos));

    memcpy(&vel, data + 4, sizeof(vel));



    ODriveCanDmaEncoderSnapshot snap;

    snap.encoder_pos_turns = pos;

    snap.encoder_vel_turns_s = vel;

    snap.encoder_pos_counts = (int32_t)(pos * (float)APP_ODRIVE_ENCODER_CPR);

    snap.last_update_ms = s_hal_ticks_ms();

    snap.valid = true;



    uint32_t prim = __get_PRIMASK();

    __disable_irq();

    s_enc_snap[idx] = snap;

    if (!prim) {

        __enable_irq();

    }

}



void odrive_can_dma_on_rx_fifo0(ODriveCanHalHandle *hcan)

{

    if (hcan == NULL || hcan != s_hcan) {

        return;

    }



    for (;;) {

        uint32_t std_id = 0u;

        bool ext = false;

        bool rtr = false;

        uint8_t data[8];

        uint8_t dlc = 0u;



        if (!odrive_can_hal_rx(hcan, &std_id, &ext, &rtr, data, &dlc)) {

            break;

        }

        if (ext || rtr) {

            continue;

        }

        odrive_can_dma_on_rx_frame(std_id, data, dlc);

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

    const bool ok = tx_queue_push(&slot);
    if (!ok) {
        g_odrive_can_tx_queue_full++;
    }
    return ok;

}



bool odrive_can_dma_set_input_vel(uint32_t node_id, float vel_turns_s, float torque_ff_nm)

{

    uint8_t buf[8];

    memcpy(buf, &vel_turns_s, sizeof(float));

    memcpy(buf + 4, &torque_ff_nm, sizeof(float));

    return enqueue_std(odrive_can_std_id(node_id, ODRIVE_MSG_SET_INPUT_VEL), false, buf, 8);

}



bool odrive_can_dma_set_input_pos(uint32_t node_id, float pos_turns, float vel_ff_turns_s, float torque_ff_nm)

{

    uint8_t buf[8];

    odrive_can_pack_set_input_pos(buf, pos_turns, vel_ff_turns_s, torque_ff_nm);

    return enqueue_std(odrive_can_std_id(node_id, ODRIVE_MSG_SET_INPUT_POS), false, buf, 8);

}



bool odrive_can_dma_set_input_torque(uint32_t node_id, float torque_nm)

{

    uint8_t buf[8];

    odrive_can_pack_set_input_torque(buf, torque_nm);

    return enqueue_std(odrive_can_std_id(node_id, ODRIVE_MSG_SET_INPUT_TORQUE), false, buf, 8);

}



bool odrive_can_dma_set_controller_modes(uint32_t node_id, ODriveControlMode control_mode, ODriveInputMode input_mode)

{

    uint8_t buf[8];

    odrive_can_pack_set_controller_modes(buf, (int32_t)control_mode, (int32_t)input_mode);

    if (!enqueue_std(odrive_can_std_id(node_id, ODRIVE_MSG_SET_CONTROLLER_MODES), false, buf, 8)) {

        return false;

    }

    s_last_cm_written = control_mode;

    s_last_im_written = input_mode;

    return true;

}



bool odrive_can_dma_set_requested_state(uint32_t node_id, ODriveAxisState state)

{

    uint8_t buf[8];

    int16_t st = (int16_t)state;

    memcpy(buf, &st, sizeof(st));

    memset(buf + 2, 0, 6);

    return enqueue_std(odrive_can_std_id(node_id, ODRIVE_MSG_SET_AXIS_REQUESTED_STATE), false, buf, 8);

}



bool odrive_can_dma_clear_errors(uint32_t node_id)

{

    uint8_t buf[8];

    memset(buf, 0, sizeof(buf));

    return enqueue_std(odrive_can_std_id(node_id, ODRIVE_MSG_CLEAR_ERRORS), false, buf, 8);

}



bool odrive_can_dma_request_encoder_estimates(uint32_t node_id)

{

    return enqueue_std(odrive_can_std_id(node_id, ODRIVE_MSG_GET_ENCODER_ESTIMATES), true, NULL, 8);

}



bool odrive_can_dma_get_encoder_snapshot(uint32_t node_id, ODriveCanDmaEncoderSnapshot *out)

{

    if (out == NULL) {

        return false;

    }

    const int idx = odrive_can_dma_node_index(node_id);

    if (idx < 0) {

        return false;

    }



    uint32_t prim = __get_PRIMASK();

    __disable_irq();

    *out = s_enc_snap[idx];

    if (!prim) {

        __enable_irq();

    }

    return out->valid;

}



bool odrive_can_dma_is_encoder_fresh(uint32_t node_id, uint32_t max_age_ms)

{

    ODriveCanDmaEncoderSnapshot snap;

    if (!odrive_can_dma_get_encoder_snapshot(node_id, &snap)) {

        return false;

    }

    const uint32_t now = s_hal_ticks_ms();

    return (now - snap.last_update_ms) <= max_age_ms;

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


