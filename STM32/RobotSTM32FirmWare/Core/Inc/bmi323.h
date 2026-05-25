/**
 * @file bmi323.h
 * @brief BMI323 6-axis IMU driver (Bosch Sensortec) — public API.
 *
 * Implements the common IMU interface. Use init_spi or init_i2c with HAL
 * handles; then call read as usual.
 */

#ifndef BMI323_H
#define BMI323_H

#include "imu_driver.h"

struct _SPI_HandleTypeDef;
struct _I2C_HandleTypeDef;
struct _GPIO_TypeDef;

typedef enum {
	BMI323_IFACE_NONE = 0,
	BMI323_IFACE_SPI,
	BMI323_IFACE_I2C,
} bmi323_iface_t;

/** Accelerometer full-scale selection (ACC_CONF range field). */
typedef enum {
	BMI323_ACCEL_2G   = 0x00,
	BMI323_ACCEL_4G   = 0x01,
	BMI323_ACCEL_8G   = 0x02,
	BMI323_ACCEL_16G  = 0x03,
} bmi323_accel_scale_t;

/** Gyroscope full-scale selection (GYR_CONF range field). */
typedef enum {
	BMI323_GYRO_125_DPS   = 0x00,
	BMI323_GYRO_250_DPS   = 0x01,
	BMI323_GYRO_500_DPS   = 0x02,
	BMI323_GYRO_1000_DPS  = 0x03,
	BMI323_GYRO_2000_DPS  = 0x04,
} bmi323_gyro_scale_t;

/** BMI323 device context (HAL SPI or I2C). */
typedef struct {
	bmi323_iface_t iface;
	union {
		struct {
			struct _SPI_HandleTypeDef *spi;
			struct _GPIO_TypeDef       *cs_port;
			uint16_t                   cs_pin;
		} spi;
		struct {
			struct _I2C_HandleTypeDef *i2c;
			uint8_t                    i2c_addr;  /**< 7-bit I2C address */
		} i2c;
	} hal;
	float accel_scale;
	float gyro_scale;
} bmi323_t;

/**
 * Initialize the BMI323 over SPI using HAL.
 * SPI: one dummy byte before payload (driver handles it).
 * @param dev      Device context (zeroed before use).
 * @param hspi     HAL SPI handle (e.g. &hspi1).
 * @param cs_port  CS GPIO port (e.g. GPIOA).
 * @param cs_pin   CS GPIO pin (e.g. GPIO_PIN_4).
 * @param accel_fs Accelerometer full scale (e.g. BMI323_ACCEL_4G).
 * @param gyro_fs  Gyroscope full scale (e.g. BMI323_GYRO_2000_DPS).
 * @return 0 on success, negative on error.
 */
int bmi323_init_spi(bmi323_t *dev, void *hspi, void *cs_port, uint16_t cs_pin,
                    bmi323_accel_scale_t accel_fs, bmi323_gyro_scale_t gyro_fs);

/**
 * Initialize the BMI323 over I2C using HAL.
 * @param dev      Device context (zeroed before use).
 * @param hi2c     HAL I2C handle (e.g. &hi2c1).
 * @param i2c_addr 7-bit I2C address (e.g. 0x68).
 * @param accel_fs Accelerometer full scale (e.g. BMI323_ACCEL_4G).
 * @param gyro_fs  Gyroscope full scale (e.g. BMI323_GYRO_2000_DPS).
 * @return 0 on success, negative on error.
 */
int bmi323_init_i2c(bmi323_t *dev, void *hi2c, uint8_t i2c_addr,
                    bmi323_accel_scale_t accel_fs, bmi323_gyro_scale_t gyro_fs);

/**
 * Read one IMU sample into the common data format.
 * @param dev   Device context (after init_spi or init_i2c).
 * @param data  Output: accel (m/s²), gyro (rad/s), temp (°C).
 * @return 0 on success, negative on error.
 */
int bmi323_read(bmi323_t *dev, imu_data_t *data);

#endif /* BMI323_H */
