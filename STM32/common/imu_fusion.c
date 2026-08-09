/**
 * @file imu_fusion.c
 *
 * Keep in sync with RobotSTM32FirmWare/Core/Src/imu_fusion.c.
 */

#include "imu_fusion.h"

#include <math.h>
#include <stddef.h>

#ifndef IMU_FUSION_G_MPS2
#define IMU_FUSION_G_MPS2 9.80665f
#endif

void imu_fusion_reset(imu_fusion_state_t *state)
{
	if (state == NULL) {
		return;
	}
	state->pitch_rad = 0.0f;
	state->sample_count = 0u;
	state->last_t_us = 0u;
}

static bool axis_valid(int axis)
{
	return axis >= 0 && axis <= 2;
}

static float accel_norm_mps2(const float accel_mps2[3])
{
	return sqrtf(accel_mps2[0] * accel_mps2[0] +
	             accel_mps2[1] * accel_mps2[1] +
	             accel_mps2[2] * accel_mps2[2]);
}

float imu_fusion_pitch_from_accel(const float accel_mps2[3])
{
	if (accel_mps2 == NULL) {
		return 0.0f;
	}
	/* Robot balances about Y: X forward, Z up when level. */
	return imu_fusion_pitch_from_accel_axes(accel_mps2, 0, 2);
}

float imu_fusion_pitch_from_accel_axes(const float accel_mps2[3],
                                       int forward_axis,
                                       int up_axis)
{
	if (accel_mps2 == NULL || !axis_valid(forward_axis) || !axis_valid(up_axis)) {
		return 0.0f;
	}

	return atan2f(-accel_mps2[forward_axis], accel_mps2[up_axis]);
}

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
                       imu_fusion_out_t *out)
{
	if (state == NULL || accel_mps2 == NULL || gyro_rads == NULL || out == NULL) {
		return false;
	}
	if (!axis_valid(pitch_accel_forward_axis) || !axis_valid(pitch_accel_up_axis) ||
	    !axis_valid(pitch_gyro_axis)) {
		return false;
	}
	if (alpha < 0.0f || alpha > 1.0f) {
		return false;
	}

	float dt_s = (float)(t_us - state->last_t_us) * 1e-6f;
	if (state->last_t_us == 0u || dt_s <= 0.0f || dt_s > 0.05f) {
		dt_s = dt_default_s;
	}
	state->last_t_us = t_us;

	const float pitch_accel = imu_fusion_pitch_from_accel_axes(accel_mps2,
	                                                          pitch_accel_forward_axis,
	                                                          pitch_accel_up_axis);
	const float pitch_rate = pitch_gyro_sign * gyro_rads[pitch_gyro_axis];
	const bool accel_ok = (accel_norm_tol_mps2 <= 0.0f) ||
	                      (fabsf(accel_norm_mps2(accel_mps2) - IMU_FUSION_G_MPS2) <=
	                       accel_norm_tol_mps2);

	if (state->sample_count == 0u) {
		state->pitch_rad = accel_ok ? pitch_accel : 0.0f;
	} else if (accel_ok) {
		state->pitch_rad = alpha * (state->pitch_rad + pitch_rate * dt_s)
		                 + (1.0f - alpha) * pitch_accel;
	} else {
		state->pitch_rad = state->pitch_rad + pitch_rate * dt_s;
	}
	state->sample_count++;

	out->pitch_rad = state->pitch_rad;
	out->pitch_rate_rads = pitch_rate;
	return true;
}
