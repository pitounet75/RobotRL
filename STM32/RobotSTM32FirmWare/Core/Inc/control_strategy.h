/**
 * @file control_strategy.h
 * @brief Swappable balance / velocity control laws (500 Hz).
 *
 * Change at runtime via control_strategy_set() or Live Expression g_ctrl_strategy.
 */
#ifndef CONTROL_STRATEGY_H
#define CONTROL_STRATEGY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CTRL_STRATEGY_DUAL_PID = 0,
    /** State feedback: u = Kθ·f(θ_err) − Kω·θ̇ + Kv·ẋ; f=linear|atan (see APP_CTRL_LINEAR_THETA_FUNC). */
    CTRL_STRATEGY_LINEAR = 1,
    /** Segway-style: velocity error shifts pitch_ref, then pitch PID. */
    CTRL_STRATEGY_CASCADE = 2,
    /** Cascade + u_ff = -K_ff*sin(pitch) + FB on (pitch_ref_eff - pitch, pitch_rate). */
    CTRL_STRATEGY_FF_CASCADE = 3,
    CTRL_STRATEGY_COUNT
} control_strategy_id_t;

typedef struct {
    float pitch_rad;
    float pitch_rate_rads;
    float vel_wheel_turns_s;
    float pitch_ref_rad;
    float vel_ref_turns_s;
    float dt_s;
    /** ODrive motor-shaft vel in robot frame (turn/s); from CAN encoder estimates. */
    float vel_motor_l_turns_s;
    float vel_motor_r_turns_s;
    bool vel_motor_l_valid;
    bool vel_motor_r_valid;
    uint32_t vel_motor_l_update_ms;
    uint32_t vel_motor_r_update_ms;
    /** Mean local ABZ wheel position (robot-frame turn); for x station-keeping. */
    float pos_wheel_turns;
    bool pos_wheel_valid;
} control_strategy_input_t;

typedef struct {
    bool ok;
    bool estop;
    float torque_left_nm;
    float torque_right_nm;
    /** Debug taps (strategy-dependent; zero if unused). */
    float u_balance;
    float u_vel;
    float u_ff;
    float u_fb;
    float cmd;
} control_strategy_output_t;

/** Active strategy (writable from debugger for quick A/B). */
extern volatile control_strategy_id_t g_ctrl_strategy;

void control_strategy_init(void);
control_strategy_id_t control_strategy_get(void);
bool control_strategy_set(control_strategy_id_t id);
const char *control_strategy_name(control_strategy_id_t id);

void control_strategy_update(const control_strategy_input_t *in, control_strategy_output_t *out);

#endif /* CONTROL_STRATEGY_H */
