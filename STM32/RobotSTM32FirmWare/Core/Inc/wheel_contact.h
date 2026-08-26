/**
 * @file wheel_contact.h
 * @brief Antipatinage: wheel lift detection, SYNC / BOTH_AIR / RECOVERY modes.
 *
 * Spec: STM32/RobotSTM32FirmWare/docs/ANTIPATINAGE.md
 */
#ifndef WHEEL_CONTACT_H
#define WHEEL_CONTACT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WC_MODE_NORMAL = 0,
    WC_MODE_SYNC_L = 1,
    WC_MODE_SYNC_R = 2,
    WC_MODE_BOTH_AIR = 3,
    WC_MODE_RECOVERY = 4,
} wheel_contact_mode_t;

typedef struct {
    float dt_s;
    float u;
    float pitch_rad;
    float pitch_rate_rads;
    float pitch_ref_rad;
    float pitch_trim_rad;
    float vel_motor_l_turns_s;
    float vel_motor_r_turns_s;
    bool vel_motor_l_valid;
    bool vel_motor_r_valid;
    float alpha_l_rads2;
    float alpha_r_rads2;
    float yaw_rate_rads;
    float tau_l_pre_nm;
    float tau_r_pre_nm;
    float vel_wheel_turns_s;
    float pos_wheel_turns;
    float x_m;
    bool pos_wheel_valid;
} wheel_contact_input_t;

typedef struct {
    wheel_contact_mode_t mode;
    bool integrator_trust;
    float recover_ramp;
    bool lift_l;
    bool lift_r;
    float pitch_trim_frozen_rad;
    float x_m_frozen_m;
    float u_cmd_nm;
    float u_yaw_cmd_nm;
    float tau_sync_l_nm;
    float tau_sync_r_nm;
    float tau_both_l_nm;
    float tau_both_r_nm;
    bool both_active;
    bool reanchor_pos;
    float pos_offset_turns_new;
    bool reset_vel_integrators;
    float vel_wheel_touch_turns_s;
    bool reset_u_lpf;
    float u_lpf_seed_nm;
} wheel_contact_output_t;

void wheel_contact_init(void);
void wheel_contact_reset(void);

/** Apply detection + mode logic; fills torque overlays and integrator gates. */
void wheel_contact_update(const wheel_contact_input_t *in, wheel_contact_output_t *out);

/** Last output (valid after wheel_contact_update). */
const wheel_contact_output_t *wheel_contact_last_output(void);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_CONTACT_H */
