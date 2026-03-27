/**
 * @file mpu6050.h
 * @brief MPU6050 6-axis IMU driver (TDK/InvenSense) — public API.
 *
 * I2C only (no SPI). Same usage pattern as other sensor_drivers IMUs:
 * init_i2c, then read into imu_data_t.
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "imu_driver.h"

struct _I2C_HandleTypeDef;

typedef enum {
	MPU6050_IFACE_NONE = 0,
	MPU6050_IFACE_I2C,
} mpu6050_iface_t;

/** Accelerometer full scale (ACCEL_CONFIG AFS_SEL). */
typedef enum {
	MPU6050_ACCEL_2G  = 0x00,
	MPU6050_ACCEL_4G  = 0x08,
	MPU6050_ACCEL_8G  = 0x10,
	MPU6050_ACCEL_16G = 0x18,
} mpu6050_accel_scale_t;

/** Gyroscope full scale (GYRO_CONFIG FS_SEL). */
typedef enum {
	MPU6050_GYRO_250_DPS  = 0x00, /**< ±250 °/s */
	MPU6050_GYRO_500_DPS  = 0x08, /**< ±500 °/s */
	MPU6050_GYRO_1000_DPS = 0x10, /**< ±1000 °/s */
	MPU6050_GYRO_2000_DPS = 0x18, /**< ±2000 °/s */
} mpu6050_gyro_scale_t;

/** MPU6050 device context (I2C only). */
typedef struct {
	mpu6050_iface_t iface;
	struct {
		struct _I2C_HandleTypeDef *i2c;
		uint8_t                    i2c_addr;  /**< 7-bit (0x68 or 0x69) */
	} hal;
	float accel_scale;  /**< m/s per LSB */
	float gyro_scale;    /**< rad/s per LSB */
} mpu6050_t;

/**
 * Initialize the MPU6050 over I2C using HAL.
 * @param dev         Device context (zeroed before use).
 * @param hi2c        HAL I2C handle (e.g. &hi2c1).
 * @param i2c_addr    7-bit address (0x68 if AD0 low, 0x69 if AD0 high).
 * @param accel_fs    Accelerometer full scale (e.g. MPU6050_ACCEL_16G).
 * @param gyro_fs     Gyroscope full scale (e.g. MPU6050_GYRO_2000_DPS).
 * @return 0 on success, negative on error.
 */
int mpu6050_init_i2c(mpu6050_t *dev, void *hi2c, uint8_t i2c_addr,
                     mpu6050_accel_scale_t accel_fs, mpu6050_gyro_scale_t gyro_fs);

/**
 * Read one IMU sample into the common data format.
 * @param dev   Device context (after init_i2c).
 * @param data  Output: accel (m/s²), gyro (rad/s), temp (°C).
 * @return 0 on success, negative on error.
 */
int mpu6050_read(mpu6050_t *dev, imu_data_t *data);

#endif /* MPU6050_H */
