/**
 * @file app_telemetry.h
 * @brief UART4 telemetry transport + BalanceFrame streaming.
 */
#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "telemetry.h"
#include "telemetry_protocol.h"

bool app_telemetry_init(void);
/** Block until ESP32 sends "READY\n" on UART4 RX (or timeout). Returns true if READY seen. */
bool app_telemetry_wait_bridge_ready(uint32_t timeout_ms);
bool app_telemetry_ctrl_register(telemetry_t *tel);
telemetry_t *app_telemetry_handle(void);
void app_telemetry_tick_1ms(void);
void app_telemetry_tx_poll(void);
void app_telemetry_poll_rx(void);
void app_telemetry_publish_balance_frame(void);
/** Build an RPC reply in static storage and append it to the shared UART4 TX queue. */
int app_telemetry_send_frame_immediate(uint16_t message_type, uint16_t sequence_id,
                                       telemetry_error_code_t error_code, const uint8_t *payload,
                                       uint16_t payload_len);

extern volatile uint32_t g_telemetry_uart_rx_bytes;
extern volatile uint32_t g_telemetry_uart_tx_bytes;
extern volatile uint32_t g_telemetry_rpc_dispatch_count;
extern volatile uint32_t g_telemetry_rx_ring_drop;
extern volatile uint32_t g_telemetry_tx_queue_drop;
/** HAL UART DMA start failures; the queued head remains pending for retry. */
extern volatile uint32_t g_telemetry_tx_fail_count;
extern volatile uint32_t g_telemetry_tx_dma_stuck_recover;
/** 1 while a UART4 DMA TX is in flight (debug: stuck=1 means tx_bytes is frozen). */
extern volatile uint32_t g_telemetry_tx_dma_busy;
/** Compatibility alias for g_telemetry_balance_frames_queued. */
extern volatile uint32_t g_telemetry_balance_frame_number;
/** BalanceFrame sample generation attempts, including samples later rejected by TX. */
extern volatile uint32_t g_telemetry_balance_frames_generated;
/** BalanceFrames accepted into the shared UART4 TX queue; matches wire frame_number. */
extern volatile uint32_t g_telemetry_balance_frames_queued;
/** Queued BalanceFrames whose UART4 DMA completion callback ran. */
extern volatile uint32_t g_telemetry_balance_frames_dma_completed;
/** BalanceFrame telemetry_send() calls that failed before queue admission. */
extern volatile uint32_t g_telemetry_balance_frames_send_failed;
/** TX write callback failures caused by a missing/contended mutex. */
extern volatile uint32_t g_telemetry_tx_mutex_fail;
/** HAL UART4 DMA start failures. Frames remain queued and are retried. */
extern volatile uint32_t g_telemetry_tx_write_fail;
/** Successful retry starts plus stuck-DMA abort/restart recovery attempts. */
extern volatile uint32_t g_telemetry_tx_recovery_count;

#endif /* APP_TELEMETRY_H */
