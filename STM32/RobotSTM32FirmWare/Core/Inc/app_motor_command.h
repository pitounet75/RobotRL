/**
 * @file app_motor_command.h
 * @brief Wheel torque commands from control task to motor TX (seqlock).
 */
#ifndef APP_MOTOR_COMMAND_H
#define APP_MOTOR_COMMAND_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t t_us;
    uint32_t seq;
    bool valid;
    bool estop;
    float torque_left_nm;
    float torque_right_nm;
} app_motor_command_t;

void app_motor_command_publish(const app_motor_command_t *cmd);
bool app_motor_command_read(app_motor_command_t *out);

#endif /* APP_MOTOR_COMMAND_H */
