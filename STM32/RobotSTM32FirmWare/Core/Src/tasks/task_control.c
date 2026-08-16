/**
 * @file task_control.c
 * @brief 500 Hz control: read snapshots, run active strategy, publish motor command.
 */

#include "tasks/tasks.h"

#include "app_config.h"
#include "app_ctrl_params.h"
#include "app_motor_command.h"
#include "app_samples.h"
#include "app_time_us.h"
#include "app_wheel_config.h"
#include "control_strategy.h"
#include "wheel_encoder_abz.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>

volatile uint32_t g_control_loop_count;
volatile float g_ctrl_pitch_rad;
volatile float g_ctrl_pitch_rate;
volatile float g_ctrl_vel_wheel;
volatile float g_ctrl_u_balance;
volatile float g_ctrl_u_vel;
volatile float g_ctrl_u_ff;
volatile float g_ctrl_u_fb;
volatile float g_ctrl_cmd_torque_nm;

static bool wheel_odometry(float *vel_turns_s, float *pos_turns, bool *pos_valid)
{
    *vel_turns_s = 0.0f;
    *pos_turns = 0.0f;
    *pos_valid = false;

    app_encoder_bank_sample_t bank;
    if (!app_samples_encoder_bank_read(&bank)) {
        return false;
    }

    const app_encoder_sample_t *left = &bank.encoder[WHEEL_ENCODER_TIM2];
    const app_encoder_sample_t *right = &bank.encoder[WHEEL_ENCODER_TIM4];

    if (left->valid && right->valid) {
        *vel_turns_s = 0.5f * (left->vel_turns_s + right->vel_turns_s);
        *pos_turns = 0.5f * (left->pos_turns + right->pos_turns);
        *pos_valid = true;
        return true;
    }
    if (left->valid) {
        *vel_turns_s = left->vel_turns_s;
        *pos_turns = left->pos_turns;
        *pos_valid = true;
        return true;
    }
    if (right->valid) {
        *vel_turns_s = right->vel_turns_s;
        *pos_turns = right->pos_turns;
        *pos_valid = true;
        return true;
    }
    return false;
}

/** ODrive motor-shaft velocities in robot frame (drive[0]=left, drive[1]=right). */
static void odrive_motor_vel_robot(float *vel_l, float *vel_r,
                                   bool *ok_l, bool *ok_r,
                                   uint32_t *t_l_ms, uint32_t *t_r_ms)
{
    *vel_l = 0.0f;
    *vel_r = 0.0f;
    *ok_l = false;
    *ok_r = false;
    *t_l_ms = 0u;
    *t_r_ms = 0u;

    app_odrive_sample_t od;
    if (!app_samples_odrive_read(&od)) {
        return;
    }
    if (od.drive[0].valid) {
        *vel_l = (float)app_wheel_odrive_feedback_sign(0u) * od.drive[0].vel_turns_s;
        *ok_l = true;
        *t_l_ms = od.drive[0].last_update_ms;
    }
    if (od.drive[1].valid) {
        *vel_r = (float)app_wheel_odrive_feedback_sign(1u) * od.drive[1].vel_turns_s;
        *ok_r = true;
        *t_r_ms = od.drive[1].last_update_ms;
    }
}

static bool control_failsafe_angle(float pitch_rad)
{
    const float limit = app_ctrl_params_snapshot()->pitch_failsafe_rad;
    return (fabsf(pitch_rad) > limit);
}

void task_control(void *argument)
{
    (void)argument;

    control_strategy_init();

    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS);
    const float dt_s = (float)APP_CONTROL_PERIOD_MS * 1e-3f;

    for (;;) {
        vTaskDelayUntil(&wake, period);
        g_control_loop_count++;

        app_motor_command_t cmd = {0};
        cmd.t_us = app_time_us_now();

        app_imu_sample_t imu;
        const bool imu_ok = app_samples_imu_read(&imu) && imu.valid;

        if (!imu_ok) {
            cmd.valid = true;
            cmd.estop = true;
            cmd.torque_left_nm = 0.0f;
            cmd.torque_right_nm = 0.0f;
            app_motor_command_publish(&cmd);
            continue;
        }

        g_ctrl_pitch_rad = imu.pitch_rad;
        g_ctrl_pitch_rate = imu.pitch_rate_rads;

        float vel_wheel = 0.0f;
        float pos_wheel = 0.0f;
        bool pos_ok = false;
        (void)wheel_odometry(&vel_wheel, &pos_wheel, &pos_ok);
        g_ctrl_vel_wheel = vel_wheel;

        float vel_m_l = 0.0f;
        float vel_m_r = 0.0f;
        bool vel_m_l_ok = false;
        bool vel_m_r_ok = false;
        uint32_t vel_m_l_ms = 0u;
        uint32_t vel_m_r_ms = 0u;
        odrive_motor_vel_robot(&vel_m_l, &vel_m_r, &vel_m_l_ok, &vel_m_r_ok,
                               &vel_m_l_ms, &vel_m_r_ms);

        float yaw_rate = 0.0f;
        if (APP_IMU_YAW_GYRO_AXIS >= 0 && APP_IMU_YAW_GYRO_AXIS <= 2) {
            yaw_rate = (float)APP_IMU_YAW_GYRO_SIGN *
                       imu.gyro_rads[APP_IMU_YAW_GYRO_AXIS];
        }

        const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
        control_strategy_input_t in = {
            .pitch_rad = imu.pitch_rad,
            .pitch_rate_rads = imu.pitch_rate_rads,
            .vel_wheel_turns_s = g_ctrl_vel_wheel,
            .pitch_ref_rad = p->pitch_ref_rad,
            .vel_ref_turns_s = p->vel_ref_turns_s,
            .dt_s = dt_s,
            .vel_motor_l_turns_s = vel_m_l,
            .vel_motor_r_turns_s = vel_m_r,
            .vel_motor_l_valid = vel_m_l_ok,
            .vel_motor_r_valid = vel_m_r_ok,
            .vel_motor_l_update_ms = vel_m_l_ms,
            .vel_motor_r_update_ms = vel_m_r_ms,
            .pos_wheel_turns = pos_wheel,
            .pos_wheel_valid = pos_ok,
            .yaw_rate_rads = yaw_rate,
        };

        control_strategy_output_t out = {0};
        control_strategy_update(&in, &out);

        g_ctrl_u_balance = out.u_balance;
        g_ctrl_u_vel = out.u_vel;
        g_ctrl_u_ff = out.u_ff;
        g_ctrl_u_fb = out.u_fb;
        g_ctrl_cmd_torque_nm = out.cmd;

        const bool estop = !out.ok || out.estop || control_failsafe_angle(imu.pitch_rad);

        cmd.valid = true;
        cmd.estop = estop;
        cmd.torque_left_nm = estop ? 0.0f : out.torque_left_nm;
        cmd.torque_right_nm = estop ? 0.0f : out.torque_right_nm;
        app_motor_command_publish(&cmd);
    }
}

