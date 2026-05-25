/**
 * @file imu_driver.h
 * @brief Common IMU interface for STM32 sensor drivers.
 *
 * ICM45686 and BMI323 (and other IMUs) implement this interface so the
 * balance loop can switch between them via config or a single compile-time
 * change. Same init/read API and data layout for all drivers.
 */

#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>

/** IMU sample: accelerometer (m/s²), gyro (rad/s), optional temp (°C). */
typedef struct {
	float accel_mps2[3];  /**< X, Y, Z in m/s² */
	float gyro_rads[3];   /**< X, Y, Z in rad/s */
	float temp_celsius;   /**< Sensor temperature, °C */
} imu_data_t;

/**
 * Transport: read one register. Optional for drivers that only need block read.
 * @param user_data  Opaque context (e.g. SPI handle)
 * @param reg        Register address
 * @param buf        Output buffer
 * @param len        Number of bytes to read
 * @return 0 on success, negative on error
 */
typedef int (*imu_reg_read_fn)(void *user_data, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * Transport: write one register.
 * @param user_data  Opaque context (e.g. SPI handle)
 * @param reg        Register address
 * @param buf        Data to write
 * @param len        Number of bytes
 * @return 0 on success, negative on error
 */
typedef int (*imu_reg_write_fn)(void *user_data, uint8_t reg, const uint8_t *buf, uint16_t len);

#endif /* IMU_DRIVER_H */
