/**
 * @file ctrl_heading.h
 * @brief Yaw-rate hold → differential torque u_yaw.
 */
#ifndef CTRL_HEADING_H
#define CTRL_HEADING_H

#include "control_strategy.h"

#include <stdbool.h>

void ctrl_heading_reset(void);
float ctrl_heading_step(const control_strategy_input_t *in, bool yaw_allowed);

#endif /* CTRL_HEADING_H */
