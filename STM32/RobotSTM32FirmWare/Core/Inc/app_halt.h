/**
 * @file app_halt.h
 * @brief Flat uint32_t halt diagnostics (no struct — avoids GDB field misalignment).
 *
 * After halt, watch these symbols (decimal):
 *   g_halt_magic == 1213412420 (0x48414C54)
 *   g_halt_step  == 2 (ODrive startup)
 *   g_halt_startup_error == 3 (no HB), 2 (TX fail), ...
 *   g_halt_startup_fail_line == source line in odrive_torque_mode_startup.c
 *
 * Also compare raw globals: g_odrive_startup_last_error, g_odrive_startup_fail_line,
 * g_odrive_startup_rx_std_frames.
 */
#ifndef APP_HALT_H
#define APP_HALT_H

#include <stdint.h>

#define APP_HALT_MAGIC 0x48414C54u

#define APP_HALT_NONE 0u
#define APP_HALT_FDCAN_START 1u
#define APP_HALT_ODRIVE_STARTUP 2u
#define APP_HALT_SPI1_INIT 3u
#define APP_HALT_SPI1_MSP 4u
#define APP_HALT_IMU_INIT 5u
#define APP_HALT_UNKNOWN 0xFFFFFFFFu

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t g_halt_magic;
extern volatile uint32_t g_halt_step;
extern volatile uint32_t g_halt_startup_error;
extern volatile uint32_t g_halt_startup_fail_line;
extern volatile uint32_t g_halt_fdcan_start_fail_step;
extern volatile uint32_t g_halt_rx_std_frames;
extern volatile uint32_t g_halt_rx_fifo0_peak;
extern volatile uint32_t g_halt_last_rx_std_id;
extern volatile uint32_t g_halt_fdcan_psr;
extern volatile uint32_t g_halt_fdcan_ecr;
extern volatile uint32_t g_halt_fdcan_cccr;
extern volatile uint32_t g_halt_fdcan_rxf0s;
extern volatile uint32_t g_halt_tx_fail_op;
extern volatile uint32_t g_halt_tx_fifo_free;

/** Legacy alias; also set by app_halt_record(). */
extern volatile uint32_t g_app_halt_step;

void app_halt_record(uint32_t step);

#ifdef __cplusplus
}
#endif

#endif /* APP_HALT_H */
