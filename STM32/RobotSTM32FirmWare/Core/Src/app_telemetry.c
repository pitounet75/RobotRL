/**
 * @file app_telemetry.c
 * @brief UART4 binary telemetry (BalanceFrame @ 500 Hz, push always).
 */

#include "app_telemetry.h"

#include "app_config.h"
#include "app_ctrl_params.h"
#include "app_motor_command.h"
#include "app_samples.h"
#include "app_time_us.h"
#include "control_strategy.h"
#include "dma.h"
#include "telemetry_balance_frame.h"
#include "usart.h"
#include "wheel_encoder_abz.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "stm32h7xx_hal.h"

#include <string.h>

extern volatile float g_ctrl_pitch_rad;
extern volatile float g_ctrl_pitch_rate;
extern volatile float g_ctrl_vel_wheel;
extern volatile float g_ctrl_cmd_torque_nm;
extern volatile float g_ctrl_u_ff;
extern volatile float g_ctrl_u_fb;
extern volatile control_strategy_id_t g_ctrl_strategy;

#define APP_TELEMETRY_TX_QUEUE_DEPTH 16u
#define APP_TELEMETRY_TX_MAX_BYTES   TELEMETRY_MAX_FRAME_SIZE

typedef struct {
    uint16_t len;
    uint16_t message_type;
    uint8_t data[APP_TELEMETRY_TX_MAX_BYTES];
} app_telemetry_tx_slot_t;

static telemetry_t s_telemetry = {0};
static telemetry_balance_frame_t s_balance_frame;

static app_telemetry_tx_slot_t s_tx_queue[APP_TELEMETRY_TX_QUEUE_DEPTH];
static volatile uint8_t s_tx_head;
static volatile uint8_t s_tx_tail;
static volatile bool s_tx_dma_busy;
static volatile uint32_t s_tx_dma_start_tick;
static volatile bool s_tx_start_retry_pending;

static SemaphoreHandle_t s_tx_mutex;
static volatile bool s_telemetry_ready;

/** RPC replies are built here (not on defaultTask's 512 B stack). Guarded by s_tx_mutex. */
static uint8_t s_immediate_tx_frame[TELEMETRY_MAX_FRAME_SIZE];

volatile uint32_t g_telemetry_uart_rx_bytes;
volatile uint32_t g_telemetry_uart_tx_bytes;
volatile uint32_t g_telemetry_rpc_dispatch_count;
volatile uint32_t g_telemetry_rx_ring_drop;
volatile uint32_t g_telemetry_tx_queue_drop;
volatile uint32_t g_telemetry_tx_fail_count;
volatile uint32_t g_telemetry_tx_dma_stuck_recover;
volatile uint32_t g_telemetry_tx_dma_busy;
volatile uint32_t g_telemetry_balance_frame_number;
volatile uint32_t g_telemetry_balance_frames_generated;
volatile uint32_t g_telemetry_balance_frames_queued;
volatile uint32_t g_telemetry_balance_frames_dma_completed;
volatile uint32_t g_telemetry_balance_frames_send_failed;
volatile uint32_t g_telemetry_tx_mutex_fail;
volatile uint32_t g_telemetry_tx_write_fail;
volatile uint32_t g_telemetry_tx_recovery_count;

#define APP_TELEMETRY_RX_RING_SIZE 256u

static volatile uint8_t s_rx_ring[APP_TELEMETRY_RX_RING_SIZE];
static volatile uint16_t s_rx_ring_head;
static volatile uint16_t s_rx_ring_tail;

#define APP_TELEMETRY_RX_DMA_BUF_SIZE APP_TELEMETRY_RX_RING_SIZE

static uint8_t s_uart4_rx_dma_buf[APP_TELEMETRY_RX_DMA_BUF_SIZE] __attribute__((aligned(32)));
static volatile bool s_uart4_rx_dma_active;

static bool tx_queue_empty(void)
{
    return s_tx_head == s_tx_tail;
}

static bool tx_queue_full(void)
{
    return (uint8_t)((s_tx_head + 1u) % APP_TELEMETRY_TX_QUEUE_DEPTH) == s_tx_tail;
}

static bool tx_queue_push(const uint8_t *data, uint16_t len)
{
    if (len == 0u || len > APP_TELEMETRY_TX_MAX_BYTES || tx_queue_full()) {
        return false;
    }

    app_telemetry_tx_slot_t *slot = &s_tx_queue[s_tx_head];
    slot->len = len;
    slot->message_type =
        (len > (TELEMETRY_OFF_MESSAGE_TYPE + 1u))
            ? (uint16_t)data[TELEMETRY_OFF_MESSAGE_TYPE] |
                  ((uint16_t)data[TELEMETRY_OFF_MESSAGE_TYPE + 1u] << 8)
            : 0u;
    memcpy(slot->data, data, len);
    s_tx_head = (uint8_t)((s_tx_head + 1u) % APP_TELEMETRY_TX_QUEUE_DEPTH);
    if (slot->message_type == TELEM_MSG_BALANCE_FRAME) {
        g_telemetry_balance_frames_queued++;
        g_telemetry_balance_frame_number = g_telemetry_balance_frames_queued;
    }
    return true;
}

static bool tx_queue_pop(void)
{
    if (tx_queue_empty()) {
        return false;
    }

    s_tx_tail = (uint8_t)((s_tx_tail + 1u) % APP_TELEMETRY_TX_QUEUE_DEPTH);
    return true;
}

static void telemetry_dma_prepare_tx(const uint8_t *data, uint16_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        const uint32_t start = (uint32_t)(uintptr_t)data & ~31U;
        const uint32_t end = ((uint32_t)(uintptr_t)data + (uint32_t)len + 31U) & ~31U;
        SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
    }
#else
    (void)data;
    (void)len;
#endif
}

static void telemetry_dma_invalidate_rx(const uint8_t *data, uint16_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        const uint32_t start = (uint32_t)(uintptr_t)data & ~31U;
        const uint32_t end = ((uint32_t)(uintptr_t)data + (uint32_t)len + 31U) & ~31U;
        SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
    }
#else
    (void)data;
    (void)len;
#endif
}

static void telemetry_uart4_rx_ring_push_byte(uint8_t byte)
{
    const uint16_t next = (uint16_t)((s_rx_ring_head + 1u) % APP_TELEMETRY_RX_RING_SIZE);
    if (next != s_rx_ring_tail) {
        s_rx_ring[s_rx_ring_head] = byte;
        s_rx_ring_head = next;
    } else {
        g_telemetry_rx_ring_drop++;
    }
}

static void telemetry_uart4_rx_ring_push_block(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0u; i < len; i++) {
        telemetry_uart4_rx_ring_push_byte(data[i]);
    }
}

static HAL_StatusTypeDef telemetry_uart4_rx_dma_start(void)
{
    s_uart4_rx_dma_active = true;
    return HAL_UARTEx_ReceiveToIdle_DMA(&huart4, s_uart4_rx_dma_buf, APP_TELEMETRY_RX_DMA_BUF_SIZE);
}

static bool tx_queue_peek(app_telemetry_tx_slot_t **out_slot)
{
    if (tx_queue_empty() || out_slot == NULL) {
        return false;
    }

    *out_slot = &s_tx_queue[s_tx_tail];
    return true;
}

static void tx_kick(void)
{
    if (s_tx_dma_busy || tx_queue_empty()) {
        return;
    }

    app_telemetry_tx_slot_t *slot = NULL;
    if (!tx_queue_peek(&slot)) {
        return;
    }

    telemetry_dma_prepare_tx(slot->data, slot->len);
    s_tx_dma_busy = true;
    g_telemetry_tx_dma_busy = 1u;
    s_tx_dma_start_tick = HAL_GetTick();
    if (HAL_UART_Transmit_DMA(&huart4, slot->data, slot->len) == HAL_OK) {
        g_telemetry_uart_tx_bytes += slot->len;
        if (s_tx_start_retry_pending) {
            s_tx_start_retry_pending = false;
            g_telemetry_tx_recovery_count++;
        }
    } else {
        s_tx_dma_busy = false;
        g_telemetry_tx_dma_busy = 0u;
        s_tx_dma_start_tick = 0u;
        s_tx_start_retry_pending = true;
        g_telemetry_tx_write_fail++;
        g_telemetry_tx_fail_count++;
    }
}

static int app_telemetry_uart_write(void *ctx, const uint8_t *data, uint16_t len)
{
    (void)ctx;
    if (data == NULL || len == 0u) {
        return -1;
    }

    if (s_tx_mutex == NULL) {
        g_telemetry_tx_mutex_fail++;
        return -1;
    }

    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
        g_telemetry_tx_mutex_fail++;
        return -1;
    }

    const bool ok = tx_queue_push(data, len);
    if (ok) {
        tx_kick();
    } else {
        g_telemetry_tx_queue_drop++;
    }

    xSemaphoreGive(s_tx_mutex);
    return ok ? 0 : -1;
}

static int balance_frame_encode(void *user, uint8_t *payload, uint16_t capacity, uint16_t *out_len)
{
    (void)user;
    if (capacity < TELEMETRY_BALANCE_FRAME_PAYLOAD_LEN || out_len == NULL) {
        return -1;
    }

    telemetry_balance_frame_encode(&s_balance_frame, payload);
    *out_len = TELEMETRY_BALANCE_FRAME_PAYLOAD_LEN;
    return 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4) {
        app_telemetry_tx_slot_t *completed = NULL;
        if (tx_queue_peek(&completed) && completed->message_type == TELEM_MSG_BALANCE_FRAME) {
            g_telemetry_balance_frames_dma_completed++;
        }
        (void)tx_queue_pop();
        s_tx_dma_busy = false;
        g_telemetry_tx_dma_busy = 0u;
        s_tx_dma_start_tick = 0u;
        tx_kick();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != &huart4 || Size == 0u) {
        if (huart == &huart4) {
            (void)telemetry_uart4_rx_dma_start();
        }
        return;
    }

    telemetry_dma_invalidate_rx(s_uart4_rx_dma_buf, Size);
    telemetry_uart4_rx_ring_push_block(s_uart4_rx_dma_buf, Size);
    g_telemetry_uart_rx_bytes += Size;
    (void)telemetry_uart4_rx_dma_start();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart4 || !s_uart4_rx_dma_active) {
        return;
    }
    (void)telemetry_uart4_rx_dma_start();
}

bool app_telemetry_init(void)
{
    static bool initialized;

    if (initialized) {
        return s_telemetry_ready;
    }

    s_tx_mutex = xSemaphoreCreateMutex();
    if (s_tx_mutex == NULL) {
        return false;
    }

    telemetry_init(&s_telemetry, app_telemetry_uart_write, NULL);
    telemetry_set_error(&s_telemetry, "");

    static const telemetry_field_def_t balance_response_fields[] = {
        {"frame_number", TELEMETRY_TYPE_UINT32},
        {"time_us", TELEMETRY_TYPE_UINT32},
        {"pitch_rad", TELEMETRY_TYPE_FLOAT},
        {"pitch_rate_rads", TELEMETRY_TYPE_FLOAT},
        {"vel_wheel_turns_s", TELEMETRY_TYPE_FLOAT},
        {"vel_wheel_l_turns_s", TELEMETRY_TYPE_FLOAT},
        {"vel_wheel_r_turns_s", TELEMETRY_TYPE_FLOAT},
        {"cmd_torque_nm", TELEMETRY_TYPE_FLOAT},
        {"cmd_torque_left_nm", TELEMETRY_TYPE_FLOAT},
        {"cmd_torque_right_nm", TELEMETRY_TYPE_FLOAT},
        {"u_ff_nm", TELEMETRY_TYPE_FLOAT},
        {"u_fb_nm", TELEMETRY_TYPE_FLOAT},
        {"pitch_ref_rad", TELEMETRY_TYPE_FLOAT},
        {"imu_valid", TELEMETRY_TYPE_UINT32},
        {"estop", TELEMETRY_TYPE_UINT32},
        {"strategy_id", TELEMETRY_TYPE_UINT32},
        {"source_drop_count_mod256", TELEMETRY_TYPE_UINT32},
    };

    static const telemetry_message_def_t balance_def = {
        .key = TELEMETRY_KEY_BALANCE_FRAME,
        .message_type = TELEM_MSG_BALANCE_FRAME,
        .response_fields = balance_response_fields,
        .response_field_count = (uint16_t)(sizeof(balance_response_fields) / sizeof(balance_response_fields[0])),
        .encode = balance_frame_encode,
    };

    if (telemetry_register(&s_telemetry, &balance_def) != 0) {
        return false;
    }

    /* Runtime gain RPC — failure must not block BalanceFrame streaming. */
    (void)app_telemetry_ctrl_register(&s_telemetry);

    /* Keep FIFO off — CubeMX disables it; re-enabling breaks UART4 DMA TX completion. */
    (void)HAL_UARTEx_DisableFifoMode(&huart4);

    app_dma_nvic_apply();
    HAL_NVIC_EnableIRQ(UART4_IRQn);

    if (telemetry_uart4_rx_dma_start() != HAL_OK) {
        return false;
    }

    s_telemetry_ready = true;
    initialized = true;

    return true;
}

telemetry_t *app_telemetry_handle(void)
{
    return &s_telemetry;
}

void app_telemetry_tick_1ms(void)
{
    telemetry_tick_1ms(&s_telemetry);
}

void app_telemetry_tx_poll(void)
{
    if (!s_tx_dma_busy) {
        if (s_tx_mutex != NULL && xSemaphoreTake(s_tx_mutex, 0) == pdTRUE) {
            tx_kick();
            xSemaphoreGive(s_tx_mutex);
        }
        return;
    }
    if (s_tx_dma_start_tick == 0u) {
        return;
    }
    if ((HAL_GetTick() - s_tx_dma_start_tick) < 5u) {
        return;
    }
    if (s_tx_mutex == NULL || xSemaphoreTake(s_tx_mutex, 0) != pdTRUE) {
        return;
    }

    g_telemetry_tx_dma_stuck_recover++;
    g_telemetry_tx_recovery_count++;
    (void)HAL_UART_AbortTransmit(&huart4);
    s_tx_dma_busy = false;
    g_telemetry_tx_dma_busy = 0u;
    s_tx_dma_start_tick = 0u;
    tx_kick();
    xSemaphoreGive(s_tx_mutex);
}

void app_telemetry_poll_rx(void)
{
    if (!s_telemetry_ready) {
        return;
    }

    while (s_rx_ring_tail != s_rx_ring_head) {
        const uint8_t byte = s_rx_ring[s_rx_ring_tail];
        s_rx_ring_tail = (uint16_t)((s_rx_ring_tail + 1u) % APP_TELEMETRY_RX_RING_SIZE);
        telemetry_rx_feed(&s_telemetry, &byte, 1u);
    }
}

bool app_telemetry_wait_bridge_ready(uint32_t timeout_ms)
{
    static const char k_ready[] = "READY\n";
    size_t matched = 0u;
    const uint32_t start_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_ms) < timeout_ms) {
        while (s_rx_ring_tail != s_rx_ring_head) {
            const uint8_t byte = s_rx_ring[s_rx_ring_tail];
            s_rx_ring_tail = (uint16_t)((s_rx_ring_tail + 1u) % APP_TELEMETRY_RX_RING_SIZE);

            if (byte == (uint8_t)k_ready[matched]) {
                matched++;
                if (k_ready[matched] == '\0') {
                    return true;
                }
            } else {
                matched = (byte == (uint8_t)k_ready[0]) ? 1u : 0u;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return false;
}

int app_telemetry_send_frame_immediate(uint16_t message_type, uint16_t sequence_id,
                                       telemetry_error_code_t error_code, const uint8_t *payload,
                                       uint16_t payload_len)
{
    if (!s_telemetry_ready) {
        return -1;
    }

    const uint16_t body_len = TELEMETRY_BODY_LEN(payload_len);
    const uint16_t frame_len = TELEMETRY_FRAME_LEN(payload_len);
    if (frame_len > TELEMETRY_MAX_FRAME_SIZE) {
        return -2;
    }

    if (s_tx_mutex == NULL) {
        g_telemetry_tx_mutex_fail++;
        return -4;
    }
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        g_telemetry_tx_mutex_fail++;
        return -5;
    }

    uint8_t *frame = s_immediate_tx_frame;
    frame[0] = (uint8_t)(TELEMETRY_MAGIC & 0xFFu);
    frame[1] = (uint8_t)((TELEMETRY_MAGIC >> 8) & 0xFFu);
    frame[2] = (uint8_t)(body_len & 0xFFu);
    frame[3] = (uint8_t)((body_len >> 8) & 0xFFu);
    frame[TELEMETRY_OFF_VERSION] = TELEMETRY_PROTOCOL_VERSION;
    frame[TELEMETRY_OFF_SEQUENCE] = (uint8_t)(sequence_id & 0xFFu);
    frame[TELEMETRY_OFF_SEQUENCE + 1u] = (uint8_t)((sequence_id >> 8) & 0xFFu);
    frame[TELEMETRY_OFF_MESSAGE_TYPE] = (uint8_t)(message_type & 0xFFu);
    frame[TELEMETRY_OFF_MESSAGE_TYPE + 1u] = (uint8_t)((message_type >> 8) & 0xFFu);
    frame[TELEMETRY_OFF_ERROR_CODE] = (uint8_t)error_code;
    if (payload_len > 0u && payload != NULL) {
        memcpy(&frame[TELEMETRY_PAYLOAD_OFFSET], payload, payload_len);
    }
    frame[TELEMETRY_PAYLOAD_OFFSET + payload_len] =
        telemetry_crc8(frame, (uint16_t)(TELEMETRY_PAYLOAD_OFFSET + payload_len));

    const bool queued = tx_queue_push(frame, frame_len);
    if (queued) {
        tx_kick();
    } else {
        g_telemetry_tx_queue_drop++;
    }
    xSemaphoreGive(s_tx_mutex);
    return queued ? 0 : -3;
}

void app_telemetry_publish_balance_frame(void)
{
    g_telemetry_balance_frames_generated++;

    app_imu_sample_t imu;
    const bool imu_ok = app_samples_imu_read(&imu) && imu.valid;

    app_encoder_bank_sample_t bank;
    float vel_l = 0.0f;
    float vel_r = 0.0f;
    if (app_samples_encoder_bank_read(&bank)) {
        const app_encoder_sample_t *left = &bank.encoder[WHEEL_ENCODER_TIM2];
        const app_encoder_sample_t *right = &bank.encoder[WHEEL_ENCODER_TIM4];
        if (left->valid) {
            vel_l = left->vel_turns_s;
        }
        if (right->valid) {
            vel_r = right->vel_turns_s;
        }
    }

    app_motor_command_t motor_cmd;
    bool estop = false;
    float torque_left = 0.0f;
    float torque_right = 0.0f;
    if (app_motor_command_read(&motor_cmd) && motor_cmd.valid) {
        estop = motor_cmd.estop;
        if (!estop) {
            torque_left = motor_cmd.torque_left_nm;
            torque_right = motor_cmd.torque_right_nm;
        }
    }

    /*
     * frame_number is the next queue-admission number, not the generation
     * attempt number. A failed enqueue reuses this number on the next sample,
     * so host-visible gaps represent loss after successful queue admission.
     */
    s_balance_frame.frame_number = g_telemetry_balance_frames_queued + 1u;
    s_balance_frame.time_us = app_time_us_now();
    s_balance_frame.pitch_rad = imu_ok ? imu.pitch_rad : g_ctrl_pitch_rad;
    s_balance_frame.pitch_rate_rads = imu_ok ? imu.pitch_rate_rads : g_ctrl_pitch_rate;
    s_balance_frame.vel_wheel_turns_s = g_ctrl_vel_wheel;
    s_balance_frame.vel_wheel_l_turns_s = vel_l;
    s_balance_frame.vel_wheel_r_turns_s = vel_r;
    s_balance_frame.cmd_torque_nm = g_ctrl_cmd_torque_nm;
    s_balance_frame.cmd_torque_left_nm = torque_left;
    s_balance_frame.cmd_torque_right_nm = torque_right;
    s_balance_frame.u_ff_nm = g_ctrl_u_ff;
    s_balance_frame.u_fb_nm = g_ctrl_u_fb;
    s_balance_frame.pitch_ref_rad = app_ctrl_params_snapshot()->pitch_ref_rad;
    s_balance_frame.imu_valid = imu_ok ? 1u : 0u;
    s_balance_frame.estop = estop ? 1u : 0u;
    s_balance_frame.strategy_id = (uint8_t)g_ctrl_strategy;
    s_balance_frame.source_drop_count_mod256 =
        (uint8_t)(g_telemetry_balance_frames_send_failed & 0xFFu);

    if (telemetry_send(&s_telemetry, TELEM_MSG_BALANCE_FRAME) != 0) {
        g_telemetry_balance_frames_send_failed++;
    }
}
