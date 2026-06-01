/**
 * @file odrive_can_async.c
 */

#include "odrive_can_async.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include <string.h>

#ifndef ODRIVE_CAN_ASYNC_RX_QUEUE_LEN
#define ODRIVE_CAN_ASYNC_RX_QUEUE_LEN 24U
#endif

#ifndef ODRIVE_CAN_ASYNC_MAX_PENDING
#define ODRIVE_CAN_ASYNC_MAX_PENDING 6U
#endif

#ifndef ODRIVE_CAN_ASYNC_DEFAULT_TIMEOUT_MS
#define ODRIVE_CAN_ASYNC_DEFAULT_TIMEOUT_MS 100U
#endif

#ifndef ODRIVE_CAN_ASYNC_WORKER_STACK_WORDS
#define ODRIVE_CAN_ASYNC_WORKER_STACK_WORDS 384U
#endif

#ifndef ODRIVE_CAN_ASYNC_WORKER_PRIORITY
#define ODRIVE_CAN_ASYNC_WORKER_PRIORITY (tskIDLE_PRIORITY + 2U)
#endif

typedef struct {
    uint32_t std_id;
    uint8_t dlc;
    uint8_t data[8];
} can_rx_item_t;

typedef struct {
    bool active;
    uint32_t expected_std_id;
    ODriveCanAsyncReplyCb cb;
    void *user_ctx;
    TickType_t deadline_ticks;
} pending_slot_t;

static ODriveCanHalHandle *s_hcan;
static QueueHandle_t s_rx_queue;
static SemaphoreHandle_t s_mtx;
static TaskHandle_t s_worker;
static pending_slot_t s_pending[ODRIVE_CAN_ASYNC_MAX_PENDING];

static void odrive_can_async_worker(void *arg);

static bool tx_send_locked(uint32_t std_id, bool rtr, const uint8_t *data, uint8_t dlc)
{
    if (s_hcan == NULL) {
        return false;
    }
    if (dlc > 8U) {
        dlc = 8U;
    }

    for (int spin = 0; spin < 2000; ++spin) {
        if (odrive_can_hal_tx_ready(s_hcan)) {
            break;
        }
        taskYIELD();
    }
    if (!odrive_can_hal_tx_ready(s_hcan)) {
        return false;
    }

    return odrive_can_hal_tx(s_hcan, std_id, rtr, data, dlc);
}

static int pending_find_by_std_id(uint32_t std_id)
{
    for (unsigned i = 0; i < ODRIVE_CAN_ASYNC_MAX_PENDING; ++i) {
        if (s_pending[i].active && s_pending[i].expected_std_id == std_id) {
            return (int)i;
        }
    }
    return -1;
}

static int pending_alloc_slot(uint32_t expected_std_id)
{
    if (pending_find_by_std_id(expected_std_id) >= 0) {
        return -1;
    }
    for (unsigned i = 0; i < ODRIVE_CAN_ASYNC_MAX_PENDING; ++i) {
        if (!s_pending[i].active) {
            s_pending[i].active = true;
            s_pending[i].expected_std_id = expected_std_id;
            return (int)i;
        }
    }
    return -1;
}

static void pending_clear(int idx)
{
    if (idx >= 0 && (unsigned)idx < ODRIVE_CAN_ASYNC_MAX_PENDING) {
        memset(&s_pending[idx], 0, sizeof(s_pending[idx]));
    }
}

static void dispatch_timeouts_locked(void)
{
    const TickType_t now = xTaskGetTickCount();
    for (unsigned i = 0; i < ODRIVE_CAN_ASYNC_MAX_PENDING; ++i) {
        if (!s_pending[i].active) {
            continue;
        }
        if ((TickType_t)(now - s_pending[i].deadline_ticks) >= (TickType_t)0x80000000UL) {
            continue;
        }
        ODriveCanAsyncReplyCb cb = s_pending[i].cb;
        void *ctx = s_pending[i].user_ctx;
        uint32_t sid = s_pending[i].expected_std_id;
        pending_clear((int)i);
        if (cb != NULL) {
            cb(ctx, ODRIVE_CAN_ASYNC_TIMEOUT, NULL, 0U, sid);
        }
    }
}

static void try_dispatch_rx_locked(const can_rx_item_t *rx)
{
    const int idx = pending_find_by_std_id(rx->std_id);
    if (idx < 0) {
        return;
    }
    ODriveCanAsyncReplyCb cb = s_pending[idx].cb;
    void *ctx = s_pending[idx].user_ctx;
    pending_clear(idx);
    if (cb != NULL) {
        cb(ctx, ODRIVE_CAN_ASYNC_OK, rx->data, rx->dlc, rx->std_id);
    }
}

static void odrive_can_async_worker(void *arg)
{
    (void)arg;
    can_rx_item_t rx;

    for (;;) {
        const TickType_t poll = pdMS_TO_TICKS(10);
        if (xQueueReceive(s_rx_queue, &rx, poll) == pdTRUE) {
            if (xSemaphoreTake(s_mtx, portMAX_DELAY) == pdTRUE) {
                try_dispatch_rx_locked(&rx);
                xSemaphoreGive(s_mtx);
            }
        }

        if (xSemaphoreTake(s_mtx, portMAX_DELAY) == pdTRUE) {
            dispatch_timeouts_locked();
            xSemaphoreGive(s_mtx);
        }
    }
}

bool odrive_can_async_init(ODriveCanHalHandle *hcan)
{
    if (hcan == NULL) {
        return false;
    }
    odrive_can_async_deinit();

    s_hcan = hcan;
    memset(s_pending, 0, sizeof(s_pending));

    s_rx_queue = xQueueCreate(ODRIVE_CAN_ASYNC_RX_QUEUE_LEN, sizeof(can_rx_item_t));
    if (s_rx_queue == NULL) {
        s_hcan = NULL;
        return false;
    }

    s_mtx = xSemaphoreCreateMutex();
    if (s_mtx == NULL) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
        s_hcan = NULL;
        return false;
    }

    odrive_can_hal_rx_irq_enable(hcan);
    return true;
}

void odrive_can_async_deinit(void)
{
    odrive_can_async_stop();

    if (s_hcan != NULL) {
        odrive_can_hal_rx_irq_disable(s_hcan);
    }

    if (s_mtx != NULL) {
        vSemaphoreDelete(s_mtx);
        s_mtx = NULL;
    }
    if (s_rx_queue != NULL) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
    }

    memset(s_pending, 0, sizeof(s_pending));
    s_hcan = NULL;
}

bool odrive_can_async_start(void)
{
    if (s_hcan == NULL || s_rx_queue == NULL || s_mtx == NULL) {
        return false;
    }
    if (s_worker != NULL) {
        return true;
    }

    BaseType_t ok = xTaskCreate(odrive_can_async_worker, "odCANasync", ODRIVE_CAN_ASYNC_WORKER_STACK_WORDS, NULL,
                                ODRIVE_CAN_ASYNC_WORKER_PRIORITY, &s_worker);
    if (ok != pdPASS) {
        s_worker = NULL;
        return false;
    }
    return true;
}

void odrive_can_async_stop(void)
{
    if (s_worker != NULL) {
        vTaskDelete(s_worker);
        s_worker = NULL;
    }
}

void odrive_can_async_on_rx_fifo0_isr(ODriveCanHalHandle *hcan)
{
    if (hcan != s_hcan || s_rx_queue == NULL) {
        return;
    }

    for (;;) {
        uint32_t std_id = 0u;
        bool ext = false;
        bool rtr = false;
        uint8_t data[8] = {0};
        uint8_t dlc = 0u;

        if (!odrive_can_hal_rx(hcan, &std_id, &ext, &rtr, data, &dlc)) {
            break;
        }
        if (ext || rtr) {
            continue;
        }

        can_rx_item_t item = {0};
        item.std_id = std_id & 0x7FFU;
        item.dlc = dlc;
        if (dlc > 0u) {
            memcpy(item.data, data, dlc);
        }

        BaseType_t hpw = pdFALSE;
        (void)xQueueSendFromISR(s_rx_queue, &item, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

bool odrive_can_async_send_data(uint32_t node_id, ODriveCanMsg cmd, const uint8_t *data, uint8_t dlc)
{
    if (s_hcan == NULL || s_mtx == NULL) {
        return false;
    }
    const uint32_t std_id = odrive_can_std_id(node_id, cmd);

    if (xSemaphoreTake(s_mtx, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    const bool ok = tx_send_locked(std_id, false, data, dlc);
    xSemaphoreGive(s_mtx);
    return ok;
}

bool odrive_can_async_request(uint32_t node_id, ODriveCanMsg cmd, bool rtr, const uint8_t *tx_data, uint8_t tx_dlc,
                              ODriveCanAsyncReplyCb on_done, void *user_ctx, uint32_t timeout_ms)
{
    if (s_hcan == NULL || s_mtx == NULL || on_done == NULL) {
        return false;
    }

    if (timeout_ms == 0U) {
        timeout_ms = ODRIVE_CAN_ASYNC_DEFAULT_TIMEOUT_MS;
    }

    const uint32_t std_id = odrive_can_std_id(node_id, cmd);

    if (xSemaphoreTake(s_mtx, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    const int slot = pending_alloc_slot(std_id);
    if (slot < 0) {
        xSemaphoreGive(s_mtx);
        return false;
    }

    s_pending[slot].cb = on_done;
    s_pending[slot].user_ctx = user_ctx;
    s_pending[slot].deadline_ticks = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    const bool sent = tx_send_locked(std_id, rtr, tx_data, tx_dlc);
    if (!sent) {
        pending_clear(slot);
        xSemaphoreGive(s_mtx);
        return false;
    }

    xSemaphoreGive(s_mtx);
    return true;
}
