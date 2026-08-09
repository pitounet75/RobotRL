/**
 * @file imu_fusion.h
 * @brief Complementary pitch fusion (HAL-free, host-testable).
 *
 * Canonical source: Core/Inc/imu_fusion.h (keep in sync with STM32/common/ for host tests).
 */
#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	float pitch_rad;
	uint32_t sample_count;
	uint32_t last_t_us;
} imu_fusion_state_t;

typedef struct {
	float pitch_rad;
	float pitch_rate_rads;
} imu_fusion_out_t;

void imu_fusion_reset(imu_fusion_state_t *state);

/**
 * @param alpha                 Complementary filter gyro weight (e.g. 0.994 @ 1 kHz).
 * @param accel_norm_tol_mps2   Reject accel tilt when |‖a‖−g| exceeds this; <=0 disables.
 */
bool imu_fusion_update(imu_fusion_state_t *state,
                       const float accel_mps2[3],
                       const float gyro_rads[3],
                       uint32_t t_us,
                       float dt_default_s,
                       float alpha,
                       int pitch_accel_forward_axis,
                       int pitch_accel_up_axis,
                       int pitch_gyro_axis,
                       float pitch_gyro_sign,
                       float accel_norm_tol_mps2,
                       imu_fusion_out_t *out);

/** pitch = atan2f(-accel[forward_axis], accel[up_axis]) when chip axes match robot frame. */
float imu_fusion_pitch_from_accel(const float accel_mps2[3]);
float imu_fusion_pitch_from_accel_axes(const float accel_mps2[3],
                                       int forward_axis,
                                       int up_axis);

#endif /* IMU_FUSION_H */
