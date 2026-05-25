/**
 * @file imu_noise_test.c
 * @brief IMU noise and bias test implementation.
 *
 * Uses Welford's online algorithm for numerically stable mean and variance.
 */

#include "imu_noise_test.h"
#include "imu_driver.h"
#include "icm45686.h"
#include "bmi323.h"
#include "mpu6050.h"
#include <math.h>
#include <string.h>

/* Welford accumulators for one axis: mean, M2 (sum of squared differences from current mean). */
typedef struct {
	double n;
	double mean;
	double M2;
} welford_t;

static void welford_update(welford_t *w, double x)
{
	w->n += 1.0;
	double delta = x - w->mean;
	w->mean += delta / w->n;
	double delta2 = x - w->mean;
	w->M2 += delta * delta2;
}

static double welford_variance(const welford_t *w)
{
	if (w->n < 2.0)
		return 0.0;
	return w->M2 / (w->n - 1.0);
}

static double welford_stddev(const welford_t *w)
{
	return sqrt(welford_variance(w));
}

static int read_sample(void *dev, imu_test_type_t type, imu_data_t *data)
{
	if (type == IMU_TEST_ICM45686)
		return icm45686_read((icm45686_t *)dev, data);
	if (type == IMU_TEST_BMI323)
		return bmi323_read((bmi323_t *)dev, data);
	if (type == IMU_TEST_MPU6050)
		return mpu6050_read((mpu6050_t *)dev, data);
	return -1;
}

int imu_noise_test_run(void *dev, imu_test_type_t type,
                       float duration_s, float samples_per_sec,
                       imu_test_delay_ms_fn delay_ms,
                       imu_noise_test_result_t *out)
{
	if (!dev || !out || duration_s <= 0.0f || samples_per_sec <= 0.0f || !delay_ms)
		return -1;

	uint32_t n_target = (uint32_t)(duration_s * samples_per_sec);
	if (n_target < 2)
		return -1;

	float interval_ms = 1000.0f / samples_per_sec;
	uint32_t interval_ms_u = (uint32_t)(interval_ms + 0.5f);
	if (interval_ms_u == 0)
		interval_ms_u = 1;

	welford_t accel[3], gyro[3], temp;
	memset(accel, 0, sizeof(accel));
	memset(gyro, 0, sizeof(gyro));
	memset(&temp, 0, sizeof(temp));

	imu_data_t sample;
	uint32_t n = 0;

	for (uint32_t i = 0; i < n_target; i++) {
		int r = read_sample(dev, type, &sample);
		if (r != 0)
			return -2;

		for (int j = 0; j < 3; j++) {
			welford_update(&accel[j], (double)sample.accel_mps2[j]);
			welford_update(&gyro[j], (double)sample.gyro_rads[j]);
		}
		welford_update(&temp, (double)sample.temp_celsius);
		n++;

		if (i + 1 < n_target)
			delay_ms(interval_ms_u);
	}

	/* Fill output: per-axis bias (mean) and stddev */
	for (int j = 0; j < 3; j++) {
		out->accel_bias[j] = (float)accel[j].mean;
		out->accel_stddev[j] = (float)welford_stddev(&accel[j]);
		out->gyro_bias[j] = (float)gyro[j].mean;
		out->gyro_stddev[j] = (float)welford_stddev(&gyro[j]);
	}
	out->temp_mean = (float)temp.mean;

	/* Combined stddev = sqrt(sum of variances) */
	double accel_var_sum = 0.0, gyro_var_sum = 0.0;
	for (int j = 0; j < 3; j++) {
		accel_var_sum += welford_variance(&accel[j]);
		gyro_var_sum += welford_variance(&gyro[j]);
	}
	out->accel_combined_stddev = (float)sqrt(accel_var_sum);
	out->gyro_combined_stddev = (float)sqrt(gyro_var_sum);

	/* Combined bias = sqrt(sum of squared bias) */
	double accel_bias_sq = 0.0, gyro_bias_sq = 0.0;
	for (int j = 0; j < 3; j++) {
		accel_bias_sq += out->accel_bias[j] * out->accel_bias[j];
		gyro_bias_sq += out->gyro_bias[j] * out->gyro_bias[j];
	}
	out->accel_combined_bias = (float)sqrt(accel_bias_sq);
	out->gyro_combined_bias = (float)sqrt(gyro_bias_sq);

	out->n_samples = n;
	out->duration_s = (float)n / samples_per_sec;

	return 0;
}
