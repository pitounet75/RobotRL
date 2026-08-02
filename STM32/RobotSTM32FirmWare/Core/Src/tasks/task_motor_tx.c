/**
 * @file task_motor_tx.c
 * @brief Send SET_INPUT_TORQUE from control commands to both ODrive boards.
 */

#include "tasks/tasks.h"

#include "app_config.h"
#include "app_motor_command.h"
#include "app_wheel_config.h"
#include "fdcan.h"
#include "odrive_can_dma.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

volatile uint32_t g_motor_tx_count;
volatile uint32_t g_motor_tx_enqueue_fail;
volatile uint32_t g_motor_tx_cmd_miss;
volatile float g_motor_tx_torque_left_nm;
volatile float g_motor_tx_torque_right_nm;
volatile float g_motor_tx_odrive_torque_left_nm;
volatile float g_motor_tx_odrive_torque_right_nm;
volatile bool g_app_odrive_torque_tx_enabled = (bool)APP_ODRIVE_TORQUE_TX_ENABLED;

void task_motor_tx(void *argument)
{
    (void)argument;

    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(APP_MOTOR_TX_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&wake, period);
        g_motor_tx_count++;

        float torque_left = 0.0f;
        float torque_right = 0.0f;

        app_motor_command_t cmd;
        if (app_motor_command_read(&cmd)) {
            if (cmd.valid && !cmd.estop) {
                torque_left = cmd.torque_left_nm;
                torque_right = cmd.torque_right_nm;
            }
        } else {
            g_motor_tx_cmd_miss++;
        }

        g_motor_tx_torque_left_nm = torque_left;
        g_motor_tx_torque_right_nm = torque_right;

        const app_wheel_config_t *left = app_wheel_config_get(APP_WHEEL_LEFT);
        const app_wheel_config_t *right = app_wheel_config_get(APP_WHEEL_RIGHT);

        const float odrive_torque_left =
            (left != 0) ? ((float)left->odrive_cmd_sign * torque_left) : torque_left;
        const float odrive_torque_right =
            (right != 0) ? ((float)right->odrive_cmd_sign * torque_right) : torque_right;
        g_motor_tx_odrive_torque_left_nm = odrive_torque_left;
        g_motor_tx_odrive_torque_right_nm = odrive_torque_right;

        const float tx_left = g_app_odrive_torque_tx_enabled ? odrive_torque_left : 0.0f;
        const float tx_right = g_app_odrive_torque_tx_enabled ? odrive_torque_right : 0.0f;

        if (left != 0 &&
            !odrive_can_dma_set_input_torque_on_bus(left->odrive_can, left->odrive_node_id, tx_left)) {
            g_motor_tx_enqueue_fail++;
        }
        if (right != 0 &&
            !odrive_can_dma_set_input_torque_on_bus(right->odrive_can, right->odrive_node_id,
                                                    tx_right)) {
            g_motor_tx_enqueue_fail++;
        }
        if (left != 0) {
            odrive_can_dma_process_tx(left->odrive_can);
        }
        if (right != 0) {
            odrive_can_dma_process_tx(right->odrive_can);
        }
    }
}
