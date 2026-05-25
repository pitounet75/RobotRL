/**
 * @file pid_controller.h
 * @brief Simple PID controller (HAL-free, host-testable).
 */
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float out_min;
    float out_max;
    float i_min;
    float i_max;
} pid_controller_t;

void pid_reset(pid_controller_t *pid);
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float out_min, float out_max);

/**
 * @param setpoint          Target for measured value.
 * @param measurement       Plant output (e.g. pitch rad).
 * @param measurement_dot   Time derivative of measurement (e.g. pitch rate rad/s); pass 0 if unused.
 * @param dt_s              Sample period in seconds (> 0).
 * @return                  Clamped control output.
 */
float pid_update(pid_controller_t *pid, float setpoint, float measurement, float measurement_dot, float dt_s);

#endif /* PID_CONTROLLER_H */
