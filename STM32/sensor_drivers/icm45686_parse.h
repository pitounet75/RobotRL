/**
 * @file icm45686_parse.h
 * @brief HAL-free ICM45686 raw sample parsing (host- and target-testable).
 */
#ifndef ICM45686_PARSE_H
#define ICM45686_PARSE_H

#include "icm45686.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
	float accel_scale;
	float gyro_scale;
	float temp_scale;
} icm45686_scales_t;

void icm45686_scales_for_config(icm45686_accel_scale_t accel_fs,
                                icm45686_gyro_scale_t gyro_fs,
                                icm45686_scales_t *scales);

bool icm45686_parse_raw14(const uint8_t raw14[14],
                          const icm45686_scales_t *scales,
                          imu_data_t *data);

#endif /* ICM45686_PARSE_H */
