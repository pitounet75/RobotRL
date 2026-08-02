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
bool app_telemetry_ctrl_register(telemetry_t *tel);
telemetry_t *app_telemetry_handle(void);
void app_telemetry_tick_1ms(void);
void app_telemetry_tx_poll(void);
void app_telemetry_poll_rx(void);
void app_telemetry_publish_balance_frame(void);
void app_telemetry_uart4_rx_isr(void);
int app_telemetry_send_frame_immediate(uint16_t message_type, uint16_t sequence_id,
                                       telemetry_error_code_t error_code, const uint8_t *payload,
                                       uint16_t payload_len);

extern volatile uint32_t g_telemetry_uart_rx_bytes;
extern volatile uint32_t g_telemetry_uart_tx_bytes;
extern volatile uint32_t g_telemetry_rpc_dispatch_count;
extern volatile uint32_t g_telemetry_rx_ring_drop;
extern volatile uint32_t g_telemetry_tx_queue_drop;
extern volatile uint32_t g_telemetry_tx_fail_count;
extern volatile uint32_t g_telemetry_tx_dma_stuck_recover;
/** Incremented each time task_telemetry publishes (200 Hz); matches BalanceFrame frame_number. */
extern volatile uint32_t g_telemetry_balance_frame_number;

#endif /* APP_TELEMETRY_H */
