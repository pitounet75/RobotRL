/**
 * @file mpu6050.c
 * @brief MPU6050 6-axis IMU — HAL I2C (Invensense register map).
 *
 * Register layout: burst read 0x3B (14 bytes), big-endian 16-bit samples.
 */

#include "mpu6050.h"
#include "imu_hal_platform.h"
#include <string.h>

#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

#define MPU6050_WHO_AM_I_VAL     0x68

/* Wake: use PLL with X gyro ref (datasheet recommends after power-up) */
#define MPU6050_PWR_CLK_PLL_GX   0x01
#define MPU6050_SLEEP_MASK       0x40

static int reg_write8(mpu6050_t *dev, uint8_t reg, uint8_t val)
{
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)dev->hal.i2c;
	uint8_t addr = dev->hal.i2c.i2c_addr << 1;
	HAL_StatusTypeDef s = HAL_I2C_Mem_Write(hi2c, addr, reg, I2C_MEMADD_SIZE_8BIT,
		&val, 1, IMU_HAL_TIMEOUT_MS);
	return (s == HAL_OK) ? 0 : -1;
}

static int reg_read(mpu6050_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)dev->hal.i2c;
	uint8_t addr = dev->hal.i2c.i2c_addr << 1;
	HAL_StatusTypeDef s = HAL_I2C_Mem_Read(hi2c, addr, reg, I2C_MEMADD_SIZE_8BIT,
		buf, len, IMU_HAL_TIMEOUT_MS);
	return (s == HAL_OK) ? 0 : -1;
}

static float mpu6050_accel_lsb_per_g(mpu6050_accel_scale_t fs)
{
	switch (fs) {
	case MPU6050_ACCEL_2G:  return 16384.0f;
	case MPU6050_ACCEL_4G:  return 8192.0f;
	case MPU6050_ACCEL_8G:  return 4096.0f;
	case MPU6050_ACCEL_16G: return 2048.0f;
	default: return 2048.0f;
	}
}

static float mpu6050_gyro_lsb_per_dps(mpu6050_gyro_scale_t fs)
{
	switch (fs) {
	case MPU6050_GYRO_250_DPS:  return 131.0f;
	case MPU6050_GYRO_500_DPS:  return 65.5f;
	case MPU6050_GYRO_1000_DPS: return 32.8f;
	case MPU6050_GYRO_2000_DPS: return 16.4f;
	default: return 16.4f;
	}
}

static int16_t read16_be(const uint8_t *p)
{
	return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

int mpu6050_init_i2c(mpu6050_t *dev, void *hi2c, uint8_t i2c_addr,
                     mpu6050_accel_scale_t accel_fs, mpu6050_gyro_scale_t gyro_fs)
{
	if (!dev || !hi2c)
		return -1;
	memset(dev, 0, sizeof(*dev));
	dev->iface = MPU6050_IFACE_I2C;
	dev->hal.i2c      = hi2c;
	dev->hal.i2c_addr = i2c_addr;

	uint8_t who;
	int r = reg_read(dev, MPU6050_REG_WHO_AM_I, &who, 1);
	if (r)
		return r;
	if (who != MPU6050_WHO_AM_I_VAL)
		return -2;

	/* Exit sleep, select clock */
	uint8_t pwr;
	r = reg_read(dev, MPU6050_REG_PWR_MGMT_1, &pwr, 1);
	if (r)
		return r;
	pwr = (uint8_t)((pwr & ~MPU6050_SLEEP_MASK) | MPU6050_PWR_CLK_PLL_GX);
	r = reg_write8(dev, MPU6050_REG_PWR_MGMT_1, pwr);
	if (r)
		return r;
	HAL_Delay(10);

	/* 1 kHz internal sample basis; SMPLRT_DIV = 0 → 1 kHz for gyro (accel 1 kHz) */
	r = reg_write8(dev, MPU6050_REG_SMPLRT_DIV, 0);
	if (r)
		return r;

	/* DLPF: 0 = 260 Hz gyro / 256 Hz accel (minimal filtering) */
	r = reg_write8(dev, MPU6050_REG_CONFIG, 0);
	if (r)
		return r;

	r = reg_write8(dev, MPU6050_REG_GYRO_CONFIG, (uint8_t)gyro_fs);
	if (r)
		return r;
	r = reg_write8(dev, MPU6050_REG_ACCEL_CONFIG, (uint8_t)accel_fs);
	if (r)
		return r;

	dev->accel_scale = 9.80665f / mpu6050_accel_lsb_per_g(accel_fs);
	dev->gyro_scale = (3.14159265f / 180.0f) / mpu6050_gyro_lsb_per_dps(gyro_fs);
	return 0;
}

int mpu6050_read(mpu6050_t *dev, imu_data_t *data)
{
	uint8_t buf[14];
	if (!dev || !data || dev->iface != MPU6050_IFACE_I2C)
		return -1;
	int r = reg_read(dev, MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf));
	if (r)
		return r;

	data->accel_mps2[0] = (float)read16_be(buf + 0)  * dev->accel_scale;
	data->accel_mps2[1] = (float)read16_be(buf + 2)  * dev->accel_scale;
	data->accel_mps2[2] = (float)read16_be(buf + 4)  * dev->accel_scale;
	/* Datasheet: T = TEMP/340 + 36.53 °C */
	int16_t temp_raw = read16_be(buf + 6);
	data->temp_celsius = (float)temp_raw / 340.0f + 36.53f;
	data->gyro_rads[0]  = (float)read16_be(buf + 8)  * dev->gyro_scale;
	data->gyro_rads[1]  = (float)read16_be(buf + 10) * dev->gyro_scale;
	data->gyro_rads[2]  = (float)read16_be(buf + 12) * dev->gyro_scale;
	return 0;
}
