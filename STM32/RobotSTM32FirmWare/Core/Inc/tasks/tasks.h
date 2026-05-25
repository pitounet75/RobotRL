/**
 * @file tasks.h
 * @brief FreeRTOS task entry points.
 */
#ifndef TASKS_H
#define TASKS_H

void task_control(void *argument);
void task_motor_tx(void *argument);
void task_imu(void *argument);
void task_encoder(void *argument);
void task_odrive(void *argument);
void task_bias(void *argument);
void task_watchdog(void *argument);
void task_jetson(void *argument);
void task_telemetry(void *argument);

#endif /* TASKS_H */
