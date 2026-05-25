/**
 * @file pid_controller.c
 */

#include "pid_controller.h"

#include <stddef.h>

void pid_reset(pid_controller_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integral = 0.0f;
}

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    if (pid == NULL) {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->i_min = out_min;
    pid->i_max = out_max;
    pid->integral = 0.0f;
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement, float measurement_dot, float dt_s)
{
    if (pid == NULL || dt_s <= 0.0f) {
        return 0.0f;
    }

    const float error = setpoint - measurement;
    pid->integral += pid->ki * error * dt_s;
    pid->integral = clampf(pid->integral, pid->i_min, pid->i_max);

    float out = pid->kp * error - pid->kd * measurement_dot + pid->integral;
    return clampf(out, pid->out_min, pid->out_max);
}
