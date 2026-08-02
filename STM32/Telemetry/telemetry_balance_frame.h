/**
 * @file telemetry_balance_frame.h
 * @brief Extended balance control telemetry payload (little-endian).
 *
 * Message type TELEM_MSG_BALANCE_FRAME (0x0100), unsolicited ascending @ 100 Hz.
 */
#ifndef TELEMETRY_BALANCE_FRAME_H
#define TELEMETRY_BALANCE_FRAME_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TELEM_MSG_BALANCE_FRAME      0x0100u
#define TELEMETRY_KEY_BALANCE_FRAME  "BalanceFrame"
#define TELEMETRY_BALANCE_FRAME_PAYLOAD_LEN 56u

typedef struct __attribute__((packed)) {
    uint32_t frame_number;
    uint32_t time_us;
    float pitch_rad;
    float pitch_rate_rads;
    float vel_wheel_turns_s;
    float vel_wheel_l_turns_s;
    float vel_wheel_r_turns_s;
    float cmd_torque_nm;
    float cmd_torque_left_nm;
    float cmd_torque_right_nm;
    float u_ff_nm;
    float u_fb_nm;
    float pitch_ref_rad;
    uint8_t imu_valid;
    uint8_t estop;
    uint8_t strategy_id;
    uint8_t reserved;
} telemetry_balance_frame_t;

static inline void telemetry_balance_frame_encode(const telemetry_balance_frame_t *frame, uint8_t *out)
{
    memcpy(out, frame, TELEMETRY_BALANCE_FRAME_PAYLOAD_LEN);
}

static inline void telemetry_balance_frame_decode(telemetry_balance_frame_t *frame, const uint8_t *in)
{
    memcpy(frame, in, TELEMETRY_BALANCE_FRAME_PAYLOAD_LEN);
}

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_BALANCE_FRAME_H */
