/**
 * @file icm45686.h
 * @brief ICM45686 6-axis IMU driver (TDK/InvenSense) — public API.
 *
 * Implements the common IMU interface. Use init_spi or init_i2c with HAL
 * handles; then call init and read as usual.
 */

#ifndef ICM45686_H
#define ICM45686_H

#include "imu_driver.h"

/* Opaque HAL types for init_spi/init_i2c (include imu_hal_platform.h in .c). */
struct _SPI_HandleTypeDef;
struct _I2C_HandleTypeDef;
struct _GPIO_TypeDef;

typedef enum {
	ICM45686_IFACE_NONE = 0,
	ICM45686_IFACE_SPI,
	ICM45686_IFACE_I2C,
} icm45686_iface_t;

/** Accelerometer full-scale selection (ACCEL_CONFIG0 accel_ui_fs_sel). */
typedef enum {
	ICM45686_ACCEL_2G   = 0x4,
	ICM45686_ACCEL_4G   = 0x3,
	ICM45686_ACCEL_8G   = 0x2,
	ICM45686_ACCEL_16G  = 0x1,
	ICM45686_ACCEL_32G  = 0x0,
} icm45686_accel_scale_t;

/** Gyroscope full-scale selection (GYRO_CONFIG0 gyro_ui_fs_sel). */
typedef enum {
	ICM45686_GYRO_15_625_DPS  = 8,
	ICM45686_GYRO_31_25_DPS   = 7,
	ICM45686_GYRO_62_5_DPS    = 6,
	ICM45686_GYRO_125_DPS     = 5,
	ICM45686_GYRO_250_DPS     = 4,
	ICM45686_GYRO_500_DPS     = 3,
	ICM45686_GYRO_1000_DPS    = 2,
	ICM45686_GYRO_2000_DPS    = 1,
	ICM45686_GYRO_4000_DPS    = 0,
} icm45686_gyro_scale_t;

/** ICM45686 device context (HAL SPI or I2C). */
typedef struct {
	icm45686_iface_t iface;
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
	float temp_scale;
} icm45686_t;

/**
 * Initialize the ICM45686 over SPI using HAL.
 * @param dev        Device context (zeroed before use).
 * @param hspi       HAL SPI handle (e.g. &hspi1).
 * @param cs_port    CS GPIO port (e.g. GPIOA).
 * @param cs_pin     CS GPIO pin (e.g. GPIO_PIN_4).
 * @param accel_fs   Accelerometer full scale (e.g. ICM45686_ACCEL_16G).
 * @param gyro_fs    Gyroscope full scale (e.g. ICM45686_GYRO_2000_DPS).
 * @return 0 on success, negative on error.
 */
int icm45686_init_spi(icm45686_t *dev, void *hspi, void *cs_port, uint16_t cs_pin,
                      icm45686_accel_scale_t accel_fs, icm45686_gyro_scale_t gyro_fs);

/**
 * Initialize the ICM45686 over I2C using HAL.
 * @param dev        Device context (zeroed before use).
 * @param hi2c       HAL I2C handle (e.g. &hi2c1).
 * @param i2c_addr   7-bit I2C address (e.g. 0x68).
 * @param accel_fs   Accelerometer full scale (e.g. ICM45686_ACCEL_16G).
 * @param gyro_fs    Gyroscope full scale (e.g. ICM45686_GYRO_2000_DPS).
 * @return 0 on success, negative on error.
 */
int icm45686_init_i2c(icm45686_t *dev, void *hi2c, uint8_t i2c_addr,
                      icm45686_accel_scale_t accel_fs, icm45686_gyro_scale_t gyro_fs);

/**
 * Read one IMU sample into the common data format.
 * @param dev   Device context (after init_spi or init_i2c).
 * @param data  Output: accel (m/s²), gyro (rad/s), temp (°C).
 * @return 0 on success, negative on error.
 */
int icm45686_read(icm45686_t *dev, imu_data_t *data);

#endif /* ICM45686_H */
