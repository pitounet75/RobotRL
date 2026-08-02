/**
 * @file tasks.h
 * @brief FreeRTOS task entry points.
 */
#ifndef TASKS_H
#define TASKS_H

#include <stdbool.h>

void task_control(void *argument);
void task_motor_tx(void *argument);
void task_imu(void *argument);
void task_encoder(void *argument);
void task_odrive(void *argument);
void task_bias(void *argument);
void task_watchdog(void *argument);
void task_jetson(void *argument);
void task_telemetry(void *argument);

/** When false, motor TX sends 0 Nm to ODrives (see APP_ODRIVE_TORQUE_TX_ENABLED). */
extern volatile bool g_app_odrive_torque_tx_enabled;

#endif /* TASKS_H */
