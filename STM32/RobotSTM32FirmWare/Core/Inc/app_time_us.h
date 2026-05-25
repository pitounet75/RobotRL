/**
 * @file app_time_us.h
 * @brief Monotonic microsecond timestamp (TIM4 free-running @ 1 MHz).
 */
#ifndef APP_TIME_US_H
#define APP_TIME_US_H

#include <stdint.h>

void app_time_us_init(void);
uint32_t app_time_us_now(void);

#endif /* APP_TIME_US_H */
