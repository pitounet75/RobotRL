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
    /** Cascade vel → pitch_ref + u_meca (gravity + D on θ̇) + u_err (P on θ_err + Kv). */
    CTRL_STRATEGY_FF_CASCADE = 0,
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
    /** Yaw rate about vertical (rad/s), from IMU gyro. */
    float yaw_rate_rads;
    /** Per-wheel local ABZ, robot frame, order-2 fit at 500 Hz (vel_fit.h).
     *  For the antipatinage: it compares the two wheels and needs both sides
     *  sampled at the control rate. The ODrive CAN estimates below refresh
     *  only every ~80 ms (RTR poll), far too slow to debounce in ms.
     *  Wheel side, NOT motor shaft: multiply by APP_WHEEL_GEAR_WHEEL /
     *  APP_WHEEL_GEAR_MOTOR to compare with vel_motor_*. */
    float vel_wheel_l_turns_s;
    float vel_wheel_r_turns_s;
    float acc_wheel_l_turns_s2;
    float acc_wheel_r_turns_s2;
    bool wheel_lr_valid;
    /** Wheel acceleration matching vel_wheel_turns_s: same wheel
     *  selection (grounded wheel in SYNC, else the mean), from the
     *  order-2 fit. Lets the cascade D term use a 10 ms estimate
     *  instead of differentiating the EMA at ~17 ms. */
    float acc_wheel_turns_s2;
    bool acc_wheel_valid;
} control_strategy_input_t;

typedef struct {
    bool ok;
    bool estop;
    float torque_left_nm;
    float torque_right_nm;
    /** Debug taps (strategy-dependent; zero if unused). */
    float u_balance;
    float u_vel;
    float u_meca;
    float u_err;
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
