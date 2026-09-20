/**
 * @file telemetry_balance_frame.h
 * @brief Extended balance control telemetry payload (little-endian).
 *
 * Message type TELEM_MSG_BALANCE_FRAME (0x0100), unsolicited ascending @ 500 Hz.
 * frame_number counts frames accepted by the STM32 TX queue, so source-side
 * generation drops do not appear as wire-loss gaps.
 * V4 payload is 68 B: V3 (64 B) plus wc_mode / sync_l / sync_r / pad.
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
#define TELEMETRY_BALANCE_FRAME_PAYLOAD_LEN 68u

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
    float u_meca_nm;
    float u_err_nm;
    float pitch_ref_rad;
    uint8_t imu_valid;
    uint8_t estop;
    uint8_t strategy_id;
    /** Low 8 bits of source samples rejected before STM32 TX queue admission. */
    uint8_t source_drop_count_mod256;
    float vbus_l_v;
    float vbus_r_v;
    /** 0=NORMAL, 1=SYNC_L, 2=SYNC_R, 3=BOTH_AIR, 4=RECOVERY */
    uint8_t wc_mode;
    uint8_t sync_l;
    uint8_t sync_r;
    uint8_t wc_reserved;
} telemetry_balance_frame_t;

_Static_assert(sizeof(telemetry_balance_frame_t) == TELEMETRY_BALANCE_FRAME_PAYLOAD_LEN,
               "BalanceFrame V4 must be 68 bytes");

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
