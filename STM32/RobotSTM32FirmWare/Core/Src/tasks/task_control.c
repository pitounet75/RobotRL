/**
 * @file task_control.c
 * @brief 500 Hz control: read snapshots, run active strategy, publish motor command.
 */

#include "tasks/tasks.h"

#include "app_config.h"
#include "app_motor_command.h"
#include "app_samples.h"
#include "app_time_us.h"
#include "control_strategy.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

volatile uint32_t g_control_loop_count;
volatile float g_ctrl_pitch_rad;
volatile float g_ctrl_pitch_rate;
volatile float g_ctrl_vel_wheel;
volatile float g_ctrl_u_balance;
volatile float g_ctrl_u_vel;
volatile float g_ctrl_cmd_turns_s;

static float wheel_velocity_turns_s(void)
{
    app_encoder_sample_t enc;
    if (app_samples_encoder_read(&enc) && enc.valid) {
        return enc.vel_turns_s;
    }
    return 0.0f;
}

static bool control_failsafe_angle(float pitch_rad)
{
    const float limit = APP_CTRL_PITCH_FAILSAFE_RAD;
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
            cmd.vel_left_turns_s = 0.0f;
            cmd.vel_right_turns_s = 0.0f;
            app_motor_command_publish(&cmd);
            continue;
        }

        g_ctrl_pitch_rad = imu.pitch_rad;
        g_ctrl_pitch_rate = imu.pitch_rate_rads;
        g_ctrl_vel_wheel = wheel_velocity_turns_s();

        control_strategy_input_t in = {
            .pitch_rad = imu.pitch_rad,
            .pitch_rate_rads = imu.pitch_rate_rads,
            .vel_wheel_turns_s = g_ctrl_vel_wheel,
            .pitch_ref_rad = APP_CTRL_PITCH_REF_RAD,
            .vel_ref_turns_s = APP_CTRL_VEL_REF_TURNS_S,
            .dt_s = dt_s,
        };

        control_strategy_output_t out = {0};
        control_strategy_update(&in, &out);

        g_ctrl_u_balance = out.u_balance;
        g_ctrl_u_vel = out.u_vel;
        g_ctrl_cmd_turns_s = out.cmd;

        const bool estop = !out.ok || out.estop || control_failsafe_angle(imu.pitch_rad);

        cmd.valid = true;
        cmd.estop = estop;
        cmd.vel_left_turns_s = estop ? 0.0f : out.vel_left_turns_s;
        cmd.vel_right_turns_s = estop ? 0.0f : out.vel_right_turns_s;
        app_motor_command_publish(&cmd);
    }
}
