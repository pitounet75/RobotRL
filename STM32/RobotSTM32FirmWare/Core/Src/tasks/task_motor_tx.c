/**
 * @file task_motor_tx.c
 * @brief Send SET_INPUT_VEL from control commands to both ODrive boards.
 */

#include "tasks/tasks.h"

#include "app_config.h"
#include "app_motor_command.h"
#include "fdcan.h"
#include "odrive_can_dma.h"

#include "FreeRTOS.h"
#include "task.h"

volatile uint32_t g_motor_tx_count;
volatile uint32_t g_motor_tx_enqueue_fail;
volatile uint32_t g_motor_tx_cmd_miss;
volatile float g_motor_tx_vel_left;
volatile float g_motor_tx_vel_right;

void task_motor_tx(void *argument)
{
    (void)argument;

    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(APP_MOTOR_TX_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&wake, period);
        g_motor_tx_count++;

        float vel_left = 0.0f;
        float vel_right = 0.0f;

        app_motor_command_t cmd;
        if (app_motor_command_read(&cmd)) {
            if (cmd.valid && !cmd.estop) {
                vel_left = cmd.vel_left_turns_s;
                vel_right = cmd.vel_right_turns_s;
            }
        } else {
            g_motor_tx_cmd_miss++;
        }

        g_motor_tx_vel_left = vel_left;
        g_motor_tx_vel_right = vel_right;

        if (!odrive_can_dma_set_input_vel(APP_ODRIVE_DRIVE0_NODE_ID, vel_left, 0.0f)) {
            g_motor_tx_enqueue_fail++;
        }
        if (!odrive_can_dma_set_input_vel(APP_ODRIVE_DRIVE1_NODE_ID, vel_right, 0.0f)) {
            g_motor_tx_enqueue_fail++;
        }
        /* Sole drain point for the CAN TX queue (encoder RTRs are queued in task_odrive). */
        odrive_can_dma_process_tx(&ODRIVE_CAN_HANDLE);
    }
}
