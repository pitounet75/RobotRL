/**
 * @file odrive_can_hal.c
 */

#include "odrive_can_hal.h"
#include <string.h>

#if defined(STM32H7xx) && defined(__has_include)
#if __has_include("main.h")
#include "main.h"
#endif
#endif

#if ODRIVE_CAN_HAL_FDCAN

static uint8_t fdcan_dlc_bytes(uint32_t fdcan_dlc)
{
    if (fdcan_dlc <= FDCAN_DLC_BYTES_8) {
        return (uint8_t)fdcan_dlc;
    }
    static const uint8_t map[] = {12, 16, 20, 24, 32, 48, 64};
    if (fdcan_dlc >= FDCAN_DLC_BYTES_12 && fdcan_dlc <= FDCAN_DLC_BYTES_64) {
        return map[fdcan_dlc - FDCAN_DLC_BYTES_12];
    }
    return 8u;
}

static uint32_t fdcan_dlc_from_bytes(uint8_t nbytes)
{
    static const uint32_t hal_dlc[] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8,
    };
    if (nbytes > 8u) {
        nbytes = 8u;
    }
    return hal_dlc[nbytes];
}

static void fdcan_fill_tx_header(FDCAN_TxHeaderTypeDef *hdr, uint32_t std_id, bool remote, uint8_t dlc)
{
    memset(hdr, 0, sizeof(*hdr));
    hdr->Identifier = std_id & 0x7FFu;
    hdr->IdType = FDCAN_STANDARD_ID;
    hdr->TxFrameType = remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    hdr->DataLength = fdcan_dlc_from_bytes(dlc);
    hdr->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr->BitRateSwitch = FDCAN_BRS_OFF;
    hdr->FDFormat = FDCAN_CLASSIC_CAN;
    hdr->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    hdr->MessageMarker = 0u;
}

static bool fdcan_needs_ram_fixup(const FDCAN_HandleTypeDef *hfdcan)
{
    return (hfdcan->Init.RxFifo0ElmtsNbr == 0u) || (hfdcan->Init.TxFifoQueueElmtsNbr == 0u);
}

volatile uint32_t g_fdcan_start_fail_step = 0u;
volatile uint32_t g_fdcan_hcan_state_at_fail = 0u;
volatile uint32_t g_odrive_can_fdcan_hal_state = 0u;

/**
 * No standard filter elements: every 11-bit frame uses the global filter → RX FIFO0.
 * Avoids CubeMX allocating filter slot 0 with uninitialized RAM (drops all traffic).
 */
bool odrive_can_fdcan_apply_global_accept_std(ODriveCanHalHandle *hfdcan)
{
    if (hfdcan == NULL || hfdcan->State != HAL_FDCAN_STATE_READY) {
        return false;
    }
    return HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) == HAL_OK;
}

static bool fdcan_prepare_accept_all_std(ODriveCanHalHandle *hfdcan)
{
    /* HAL_FDCAN_Stop() only works in BUSY state — never call it when State==READY. */
    if (hfdcan->State == HAL_FDCAN_STATE_BUSY) {
        if (HAL_FDCAN_Stop(hfdcan) != HAL_OK) {
            return false;
        }
    } else if (hfdcan->State != HAL_FDCAN_STATE_READY) {
        return false;
    }

    if (hfdcan->Init.StdFiltersNbr != 0u || hfdcan->Init.ExtFiltersNbr != 0u) {
        hfdcan->Init.StdFiltersNbr = 0u;
        hfdcan->Init.ExtFiltersNbr = 0u;
        if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
            return false;
        }
    }

    return odrive_can_fdcan_apply_global_accept_std(hfdcan);
}

/** Leave init mode when HAL state is wrong but CCCR.INIT is still set. */
static void fdcan_force_leave_init(ODriveCanHalHandle *hfdcan)
{
    if ((hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) == 0u) {
        return;
    }
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    while ((hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) != 0u) {
    }
    hfdcan->State = HAL_FDCAN_STATE_BUSY;
    hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;
}

void odrive_can_fdcan_recover_bus_off(ODriveCanHalHandle *hfdcan)
{
    FDCAN_ProtocolStatusTypeDef ps = {0};
    if (hfdcan == NULL) {
        return;
    }
    if (HAL_FDCAN_GetProtocolStatus(hfdcan, &ps) != HAL_OK) {
        return;
    }
    if (ps.BusOff == 0u && ps.ErrorPassive == 0u) {
        return;
    }
    (void)HAL_FDCAN_Stop(hfdcan);
    (void)fdcan_prepare_accept_all_std(hfdcan);
    (void)HAL_FDCAN_Start(hfdcan);
}

bool odrive_can_fdcan_start(ODriveCanHalHandle *hfdcan)
{
    g_fdcan_start_fail_step = 0u;
    g_fdcan_hcan_state_at_fail = 0u;

    if (hfdcan == NULL) {
        g_fdcan_start_fail_step = 1u;
        return false;
    }

    /* Already on the bus (e.g. debugger continued without reset). */
    if (hfdcan->State == HAL_FDCAN_STATE_BUSY) {
        (void)HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        return true;
    }

    if (fdcan_needs_ram_fixup(hfdcan)) {
        (void)HAL_FDCAN_DeInit(hfdcan);
        hfdcan->Init.StdFiltersNbr = 0u;
        hfdcan->Init.RxFifo0ElmtsNbr = 32u;
        hfdcan->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
        hfdcan->Init.RxFifo1ElmtsNbr = 0u;
        hfdcan->Init.TxFifoQueueElmtsNbr = 8u;
        hfdcan->Init.TxElmtSize = FDCAN_DATA_BYTES_8;
        if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
            g_fdcan_start_fail_step = 3u;
            return false;
        }
        if (!odrive_can_fdcan_apply_global_accept_std(hfdcan)) {
            g_fdcan_hcan_state_at_fail = (uint32_t)hfdcan->State;
            g_fdcan_start_fail_step = 4u;
            return false;
        }
    } else if (hfdcan->State == HAL_FDCAN_STATE_RESET) {
        if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
            g_fdcan_start_fail_step = 2u;
            return false;
        }
        if (!odrive_can_fdcan_apply_global_accept_std(hfdcan)) {
            g_fdcan_hcan_state_at_fail = (uint32_t)hfdcan->State;
            g_fdcan_start_fail_step = 4u;
            return false;
        }
    } else if (hfdcan->State != HAL_FDCAN_STATE_READY) {
        /* Desynced HAL state — full re-init from CubeMX Init struct. */
        g_fdcan_hcan_state_at_fail = (uint32_t)hfdcan->State;
        (void)HAL_FDCAN_DeInit(hfdcan);
        if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
            g_fdcan_start_fail_step = 2u;
            return false;
        }
        if (!odrive_can_fdcan_apply_global_accept_std(hfdcan)) {
            g_fdcan_start_fail_step = 4u;
            return false;
        }
    }
    /* else: MX_FDCANx_Init + apply_global_accept_std already ran — only Start needed. */

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        g_fdcan_hcan_state_at_fail = (uint32_t)hfdcan->State;
        /* HAL only fails when State!=READY; try direct leave-init if still in config mode. */
        if ((hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) != 0u) {
            fdcan_force_leave_init(hfdcan);
        } else {
            g_fdcan_start_fail_step = 5u;
            return false;
        }
    }

    (void)HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    return true;
}

bool odrive_can_fdcan_ensure_started(ODriveCanHalHandle *hfdcan)
{
    if (hfdcan == NULL) {
        return false;
    }
    g_odrive_can_fdcan_hal_state = (uint32_t)hfdcan->State;
    if (hfdcan->State == HAL_FDCAN_STATE_BUSY) {
        return true;
    }
    return odrive_can_fdcan_start(hfdcan);
}

bool odrive_can_hal_tx(ODriveCanHalHandle *hcan, uint32_t std_id, bool remote, const uint8_t *data,
                       uint8_t dlc)
{
    if (hcan == NULL) {
        return false;
    }
    if (dlc > 8u) {
        dlc = 8u;
    }

    FDCAN_TxHeaderTypeDef hdr;
    fdcan_fill_tx_header(&hdr, std_id, remote, dlc);

    uint8_t payload[8];
    memset(payload, 0, sizeof(payload));
    if (!remote && data != NULL && dlc > 0u) {
        memcpy(payload, data, dlc);
    }

    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &hdr, payload) == HAL_OK;
}

bool odrive_can_hal_rx(ODriveCanHalHandle *hcan, uint32_t *std_id, bool *is_extended, bool *is_rtr,
                       uint8_t *data, uint8_t *dlc)
{
    if (hcan == NULL) {
        return false;
    }

    uint32_t location = FDCAN_RX_FIFO0;
    if (HAL_FDCAN_GetRxFifoFillLevel(hcan, FDCAN_RX_FIFO0) == 0u) {
        if (HAL_FDCAN_GetRxFifoFillLevel(hcan, FDCAN_RX_FIFO1) == 0u) {
            return false;
        }
        location = FDCAN_RX_FIFO1;
    }

    FDCAN_RxHeaderTypeDef hdr;
    uint8_t buf[8];
    if (HAL_FDCAN_GetRxMessage(hcan, location, &hdr, buf) != HAL_OK) {
        return false;
    }

    const bool ext = (hdr.IdType == FDCAN_EXTENDED_ID);
    if (std_id != NULL) {
        *std_id = ext ? (hdr.Identifier & 0x1FFFFFFFu) : (hdr.Identifier & 0x7FFu);
    }
    if (is_extended != NULL) {
        *is_extended = ext;
    }
    if (is_rtr != NULL) {
        *is_rtr = (hdr.RxFrameType == FDCAN_REMOTE_FRAME);
    }

    const uint8_t nbytes = fdcan_dlc_bytes(hdr.DataLength);
    if (dlc != NULL) {
        *dlc = nbytes;
    }
    if (data != NULL && nbytes > 0u) {
        memcpy(data, buf, nbytes);
    }
    return true;
}

bool odrive_can_hal_tx_ready(ODriveCanHalHandle *hcan)
{
    if (hcan == NULL) {
        return false;
    }
    /* TFQF=1 means Tx FIFO/queue full (HAL checks this before enqueue). */
    return (hcan->Instance->TXFQS & FDCAN_TXFQS_TFQF) == 0u;
}

bool odrive_can_hal_wait_tx_fifo_empty(ODriveCanHalHandle *hcan, uint32_t timeout_ms)
{
    if (hcan == NULL) {
        return false;
    }

    /* TFFL is "consecutive free from GetIndex", not "all slots empty" — never wait for depth.
     * Wait until the latest TX buffer clears TXBRP (ST HAL-recommended polling path). */
    const uint32_t buf_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(hcan);
    if (buf_mask == 0u) {
        return true;
    }

    const uint32_t t0 = HAL_GetTick();
    while (HAL_FDCAN_IsTxBufferMessagePending(hcan, buf_mask) != 0u) {
        if ((HAL_GetTick() - t0) >= timeout_ms) {
            return false;
        }
        FDCAN_ProtocolStatusTypeDef ps = {0};
        if (HAL_FDCAN_GetProtocolStatus(hcan, &ps) == HAL_OK && ps.BusOff != 0u) {
            odrive_can_fdcan_recover_bus_off(hcan);
        }
        HAL_Delay(1);
    }
    return true;
}

static void fdcan_nvic_enable(ODriveCanHalHandle *hfdcan)
{
    if (hfdcan->Instance == FDCAN1) {
        /* Must be > configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5) for FromISR. */
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
        HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
        HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
    } else if (hfdcan->Instance == FDCAN2) {
        HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 6, 0);
        HAL_NVIC_SetPriority(FDCAN2_IT1_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
        HAL_NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
    }
}

void odrive_can_hal_rx_irq_enable(ODriveCanHalHandle *hcan)
{
    if (hcan == NULL) {
        return;
    }
    (void)HAL_FDCAN_ConfigInterruptLines(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                         FDCAN_INTERRUPT_LINE0);
    fdcan_nvic_enable(hcan);
    (void)HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0u);
}

void odrive_can_hal_rx_irq_disable(ODriveCanHalHandle *hcan)
{
    if (hcan != NULL) {
        (void)HAL_FDCAN_DeactivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    }
}

#else /* bxCAN */

bool odrive_can_hal_tx(ODriveCanHalHandle *hcan, uint32_t std_id, bool remote, const uint8_t *data,
                       uint8_t dlc)
{
    if (hcan == NULL) {
        return false;
    }
    if (dlc > 8u) {
        dlc = 8u;
    }

    CAN_TxHeaderTypeDef hdr = {0};
    hdr.StdId = std_id & 0x7FFu;
    hdr.IDE = CAN_ID_STD;
    hdr.RTR = remote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    hdr.DLC = dlc;
    hdr.TransmitGlobalTime = DISABLE;

    uint8_t payload[8];
    memset(payload, 0, sizeof(payload));
    if (!remote && data != NULL && dlc > 0u) {
        memcpy(payload, data, dlc);
    }

    uint32_t mailbox = 0u;
    return HAL_CAN_AddTxMessage(hcan, &hdr, payload, &mailbox) == HAL_OK;
}

bool odrive_can_hal_rx(ODriveCanHalHandle *hcan, uint32_t *std_id, bool *is_extended, bool *is_rtr,
                       uint8_t *data, uint8_t *dlc)
{
    if (hcan == NULL) {
        return false;
    }

    uint32_t fifo = CAN_RX_FIFO0;
    if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0u) {
        if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) == 0u) {
            return false;
        }
        fifo = CAN_RX_FIFO1;
    }

    CAN_RxHeaderTypeDef hdr;
    uint8_t buf[8];
    if (HAL_CAN_GetRxMessage(hcan, fifo, &hdr, buf) != HAL_OK) {
        return false;
    }

    const bool ext = (hdr.IDE == CAN_ID_EXT);
    if (std_id != NULL) {
        *std_id = ext ? hdr.ExtId : hdr.StdId;
    }
    if (is_extended != NULL) {
        *is_extended = ext;
    }
    if (is_rtr != NULL) {
        *is_rtr = (hdr.RTR == CAN_RTR_REMOTE);
    }

    const uint8_t nbytes = hdr.DLC > 8u ? 8u : hdr.DLC;
    if (dlc != NULL) {
        *dlc = nbytes;
    }
    if (data != NULL && nbytes > 0u) {
        memcpy(data, buf, nbytes);
    }
    return true;
}

bool odrive_can_hal_tx_ready(ODriveCanHalHandle *hcan)
{
    if (hcan == NULL) {
        return false;
    }
    return HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0u;
}

void odrive_can_hal_rx_irq_enable(ODriveCanHalHandle *hcan)
{
    if (hcan != NULL) {
        (void)HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}

void odrive_can_hal_rx_irq_disable(ODriveCanHalHandle *hcan)
{
    if (hcan != NULL) {
        (void)HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}

#endif
