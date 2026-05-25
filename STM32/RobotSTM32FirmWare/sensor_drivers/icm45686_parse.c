/**
 * @file icm45686_parse.c
 */

#include "icm45686_parse.h"

#include <stddef.h>

#define ICM45686_TEMP_LSB_PER_C  132.48f
#define ICM45686_TEMP_OFFSET_C   25.0f

static float accel_lsb_per_g(icm45686_accel_scale_t fs)
{
	static const float lsb[] = {
		[ICM45686_ACCEL_32G] = 512.0f,
		[ICM45686_ACCEL_16G] = 2048.0f,
		[ICM45686_ACCEL_8G]  = 4096.0f,
		[ICM45686_ACCEL_4G]  = 8192.0f,
		[ICM45686_ACCEL_2G]  = 16384.0f,
	};
	return lsb[fs];
}

static float gyro_lsb_per_dps(icm45686_gyro_scale_t fs)
{
	static const float lsb[] = {
		[ICM45686_GYRO_4000_DPS]   = 8.0f,
		[ICM45686_GYRO_2000_DPS]   = 16.0f,
		[ICM45686_GYRO_1000_DPS]   = 32.0f,
		[ICM45686_GYRO_500_DPS]    = 65.0f,
		[ICM45686_GYRO_250_DPS]    = 131.0f,
		[ICM45686_GYRO_125_DPS]    = 262.0f,
		[ICM45686_GYRO_62_5_DPS]   = 524.0f,
		[ICM45686_GYRO_31_25_DPS]  = 1048.0f,
		[ICM45686_GYRO_15_625_DPS] = 2096.0f,
	};
	return lsb[fs];
}

static int16_t read16_le(const uint8_t *p)
{
	return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

void icm45686_scales_for_config(icm45686_accel_scale_t accel_fs,
                                icm45686_gyro_scale_t gyro_fs,
                                icm45686_scales_t *scales)
{
	if (scales == NULL) {
		return;
	}
	scales->accel_scale = 9.80665f / accel_lsb_per_g(accel_fs);
	scales->gyro_scale = (3.14159265f / 180.0f) / gyro_lsb_per_dps(gyro_fs);
	scales->temp_scale = 1.0f / ICM45686_TEMP_LSB_PER_C;
}

bool icm45686_parse_raw14(const uint8_t raw14[14],
                          const icm45686_scales_t *scales,
                          imu_data_t *data)
{
	if (raw14 == NULL || scales == NULL || data == NULL) {
		return false;
	}

	data->accel_mps2[0] = (float)read16_le(raw14 + 0) * scales->accel_scale;
	data->accel_mps2[1] = (float)read16_le(raw14 + 2) * scales->accel_scale;
	data->accel_mps2[2] = (float)read16_le(raw14 + 4) * scales->accel_scale;
	data->gyro_rads[0] = (float)read16_le(raw14 + 6) * scales->gyro_scale;
	data->gyro_rads[1] = (float)read16_le(raw14 + 8) * scales->gyro_scale;
	data->gyro_rads[2] = (float)read16_le(raw14 + 10) * scales->gyro_scale;
	data->temp_celsius = ICM45686_TEMP_OFFSET_C + (float)read16_le(raw14 + 12) * scales->temp_scale;
	return true;
}
