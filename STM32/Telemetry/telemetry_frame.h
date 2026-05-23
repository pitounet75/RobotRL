/**
 * @file telemetry_frame.h
 * @brief GetTelemetryFrame response payload (little-endian).
 */

#ifndef TELEMETRY_FRAME_H
#define TELEMETRY_FRAME_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TELEMETRY_FRAME_REQUEST_PAYLOAD_LEN 4u
#define TELEMETRY_FRAME_PAYLOAD_LEN 40u /* frame_number u32 + 9 x float */

typedef struct {
    uint32_t frame_number;   /**< Increments each telemetry_set_frame(); detect stale data */
    float encoder_speed_l;   /**< rad/s */
    float encoder_speed_r;   /**< rad/s */
    float omega;             /**< rad/s (yaw rate) */
    float angle;             /**< rad (tilt) */
    float distance_x;        /**< m */
    float distance_y;        /**< m */
    float setpoint_l;        /**< ODrive-dependent command (TBD) */
    float setpoint_r;        /**< ODrive-dependent command (TBD) */
    float angle_setpoint;    /**< rad */
} telemetry_frame_t;

static inline void telemetry_frame_encode(const telemetry_frame_t *frame, uint8_t *out)
{
    memcpy(out, frame, TELEMETRY_FRAME_PAYLOAD_LEN);
}

static inline void telemetry_frame_decode(telemetry_frame_t *frame, const uint8_t *in)
{
    memcpy(frame, in, TELEMETRY_FRAME_PAYLOAD_LEN);
}

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_FRAME_H */
