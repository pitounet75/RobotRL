/**
 * @file telemetry_example.h
 * @brief Optional HAL integration helpers (see telemetry_example.c).
 */

#ifndef TELEMETRY_EXAMPLE_H
#define TELEMETRY_EXAMPLE_H

#include "telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Call once after HAL UART is up. */
void telemetry_example_init(telemetry_t *tel);

/** Call from main loop or UART RX path (feeds protocol parser). */
void telemetry_example_poll_rx(telemetry_t *tel);

/** Call from control loop — updates GetTelemetryFrame snapshot. */
void telemetry_example_update_frame(telemetry_t *tel);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_EXAMPLE_H */
