/**
 * @file telemetry_config.h
 * @brief GetConfig response (skeleton — fields TBD).
 */

#ifndef TELEMETRY_CONFIG_H
#define TELEMETRY_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Placeholder until ODrive config fields are defined. */
typedef struct {
    uint32_t config_version;
} telemetry_config_t;

#define TELEMETRY_CONFIG_PAYLOAD_LEN ((uint16_t)sizeof(telemetry_config_t))

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_CONFIG_H */
