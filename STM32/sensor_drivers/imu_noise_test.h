/**
 * @file imu_noise_test.h
 * @brief IMU noise and bias characterization test.
 *
 * Collects samples from ICM45686, BMI323, or MPU6050 while stationary, then computes
 * per-axis and combined standard deviation (noise) and bias (mean).
 */

#ifndef IMU_NOISE_TEST_H
#define IMU_NOISE_TEST_H

#include <stdint.h>

/** IMU type for the test. */
typedef enum {
	IMU_TEST_ICM45686,
	IMU_TEST_BMI323,
	IMU_TEST_MPU6050,
} imu_test_type_t;

/** Delay callback: sleep for `ms` milliseconds. Pass HAL_Delay on STM32. */
typedef void (*imu_test_delay_ms_fn)(uint32_t ms);

/** Result: per-axis and combined stddev and bias for accel and gyro. */
typedef struct {
	/* Accelerometer (m/s²): bias = mean, noise = stddev */
	float accel_bias[3];          /**< Per-axis bias (mean) */
	float accel_stddev[3];        /**< Per-axis standard deviation */
	float accel_combined_stddev;  /**< sqrt(var_x + var_y + var_z) */
	float accel_combined_bias;    /**< sqrt(bias_x² + bias_y² + bias_z²) */

	/* Gyroscope (rad/s) */
	float gyro_bias[3];
	float gyro_stddev[3];
	float gyro_combined_stddev;
	float gyro_combined_bias;

	float temp_mean;              /**< Mean temperature (°C) */
	uint32_t n_samples;           /**< Number of samples collected */
	float duration_s;             /**< Actual test duration (s) */
} imu_noise_test_result_t;

/**
 * Run noise/bias test on an IMU.
 *
 * Place the sensor stationary (e.g. on a table). The function collects samples
 * at the given rate for the given duration, then computes statistics.
 *
 * @param dev           IMU device pointer (type must match `type`).
 * @param type          ICM45686, BMI323, or MPU6050.
 * @param duration_s    Test duration in seconds.
 * @param samples_per_sec  Target sample rate (Hz).
 * @param delay_ms      Callback to delay (e.g. HAL_Delay). Must block for ms.
 * @param out           Output: bias, stddev per axis and combined.
 * @return 0 on success, negative on error (e.g. read failure).
 */
int imu_noise_test_run(void *dev, imu_test_type_t type,
                       float duration_s, float samples_per_sec,
                       imu_test_delay_ms_fn delay_ms,
                       imu_noise_test_result_t *out);

#endif /* IMU_NOISE_TEST_H */
