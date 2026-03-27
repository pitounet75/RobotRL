/**
 * @file icm45686.c
 * @brief ICM45686 6-axis IMU driver — HAL SPI and I2C.
 *
 * Register map from TDK InvenSense ICM45686 datasheet.
 * ICM45686 has a flat DREG map (0x00–0x7F) — no bank selection.
 * SPI: MSB of first byte = 1 for read; software CS. I2C: Mem_Read/Mem_Write.
 */

#include "icm45686.h"
#include "imu_hal_platform.h"
#include <string.h>

/* Register addresses (DREG, direct access — no BANK_SEL). TDK inv_imu_regmap_le.h */
#define ICM45686_REG_ACCEL_DATA_X   0x00   /* Accel X,Y,Z: 0x00-0x05 (6 bytes) */
#define ICM45686_REG_GYRO_DATA_X    0x06   /* Gyro X,Y,Z: 0x06-0x0B (6 bytes) */
#define ICM45686_REG_TEMP_DATA      0x0C   /* Temp: 0x0C-0x0D (2 bytes) */
#define ICM45686_REG_PWR_MGMT0      0x10   /* Sensor enable (no DEVICE_CONFIG in ICM45686) */
#define ICM45686_REG_ACCEL_CONFIG0  0x1B
#define ICM45686_REG_GYRO_CONFIG0   0x1C
#define ICM45686_REG_WHO_AM_I       0x72

#define ICM45686_WHO_AM_I_VAL       0xE9
#define ICM45686_ACCEL_ODR_800HZ    0x6
#define ICM45686_GYRO_ODR_800HZ     0x6

#define ICM45686_TEMP_LSB_PER_C     132.48f
#define ICM45686_TEMP_OFFSET_C      25.0f

#define ICM45686_SPI_READ_BIT     0x80

static void cs_assert(icm45686_t *dev)
{
	HAL_GPIO_WritePin((GPIO_TypeDef *)dev->hal.spi.cs_port, dev->hal.spi.cs_pin, GPIO_PIN_RESET);
}

static void cs_deassert(icm45686_t *dev)
{
	HAL_GPIO_WritePin((GPIO_TypeDef *)dev->hal.spi.cs_port, dev->hal.spi.cs_pin, GPIO_PIN_SET);
}

static int reg_read(icm45686_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)dev->hal.spi.spi;
	if (dev->iface == ICM45686_IFACE_SPI) {
		cs_assert(dev);
		uint8_t tx = reg | ICM45686_SPI_READ_BIT;
		HAL_StatusTypeDef s = HAL_SPI_Transmit(hspi, &tx, 1, IMU_HAL_TIMEOUT_MS);
		if (s != HAL_OK) {
			cs_deassert(dev);
			return -1;
		}
		s = HAL_SPI_Receive(hspi, buf, len, IMU_HAL_TIMEOUT_MS);
		cs_deassert(dev);
		return (s == HAL_OK) ? 0 : -1;
	}
	if (dev->iface == ICM45686_IFACE_I2C) {
		I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)dev->hal.i2c.i2c;
		uint8_t addr = dev->hal.i2c.i2c_addr << 1;
		HAL_StatusTypeDef s = HAL_I2C_Mem_Read(hi2c, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, IMU_HAL_TIMEOUT_MS);
		return (s == HAL_OK) ? 0 : -1;
	}
	return -1;
}

static int reg_write(icm45686_t *dev, uint8_t reg, const uint8_t *buf, uint16_t len)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)dev->hal.spi.spi;
	if (dev->iface == ICM45686_IFACE_SPI) {
		cs_assert(dev);
		uint8_t tx[32];
		if (len + 1 > sizeof(tx))
			return -1;
		tx[0] = reg & ~ICM45686_SPI_READ_BIT;
		memcpy(tx + 1, buf, len);
		HAL_StatusTypeDef s = HAL_SPI_Transmit(hspi, tx, 1 + len, IMU_HAL_TIMEOUT_MS);
		cs_deassert(dev);
		return (s == HAL_OK) ? 0 : -1;
	}
	if (dev->iface == ICM45686_IFACE_I2C) {
		I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)dev->hal.i2c.i2c;
		uint8_t addr = dev->hal.i2c.i2c_addr << 1;
		HAL_StatusTypeDef s = HAL_I2C_Mem_Write(hi2c, addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, IMU_HAL_TIMEOUT_MS);
		return (s == HAL_OK) ? 0 : -1;
	}
	return -1;
}

static int write_reg8(icm45686_t *dev, uint8_t reg, uint8_t val)
{
	return reg_write(dev, reg, &val, 1);
}

/* LSB per g for accel: 32768 / (2 * full_scale_g) */
static float accel_lsb_per_g(icm45686_accel_scale_t fs)
{
	static const float lsb[] = {
		[ICM45686_ACCEL_32G]  = 512.0f,
		[ICM45686_ACCEL_16G]  = 2048.0f,
		[ICM45686_ACCEL_8G]   = 4096.0f,
		[ICM45686_ACCEL_4G]   = 8192.0f,
		[ICM45686_ACCEL_2G]   = 16384.0f,
	};
	return lsb[fs];
}

/* LSB per dps for gyro (typical values from datasheet) */
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

int icm45686_init_spi(icm45686_t *dev, void *hspi, void *cs_port, uint16_t cs_pin,
                      icm45686_accel_scale_t accel_fs, icm45686_gyro_scale_t gyro_fs)
{
	if (!dev || !hspi || !cs_port)
		return -1;
	memset(dev, 0, sizeof(*dev));
	dev->iface = ICM45686_IFACE_SPI;
	dev->hal.spi.spi     = hspi;
	dev->hal.spi.cs_port = cs_port;
	dev->hal.spi.cs_pin  = cs_pin;

	uint8_t who;
	int r = reg_read(dev, ICM45686_REG_WHO_AM_I, &who, 1);
	if (r) return r;
	if (who != ICM45686_WHO_AM_I_VAL)
		return -2;
	/* PWR_MGMT0: accel_mode=LN(3), gyro_mode=LN(3) -> 0x0F */
	r = write_reg8(dev, ICM45686_REG_PWR_MGMT0, 0x0F);
	if (r) return r;
	/* ACCEL_CONFIG0: ODR 800Hz, accel_ui_fs_sel */
	r = write_reg8(dev, ICM45686_REG_ACCEL_CONFIG0,
	              (uint8_t)((ICM45686_ACCEL_ODR_800HZ & 0x0F) | ((accel_fs & 0x07) << 4)));
	if (r) return r;
	/* GYRO_CONFIG0: ODR 800Hz, gyro_ui_fs_sel */
	r = write_reg8(dev, ICM45686_REG_GYRO_CONFIG0,
	              (uint8_t)((ICM45686_GYRO_ODR_800HZ & 0x0F) | ((gyro_fs & 0x0F) << 4)));
	if (r) return r;

	dev->accel_scale = 9.80665f / accel_lsb_per_g(accel_fs);
	dev->gyro_scale  = (3.14159265f / 180.0f) / gyro_lsb_per_dps(gyro_fs);
	dev->temp_scale  = 1.0f / ICM45686_TEMP_LSB_PER_C;
	return 0;
}

int icm45686_init_i2c(icm45686_t *dev, void *hi2c, uint8_t i2c_addr,
                      icm45686_accel_scale_t accel_fs, icm45686_gyro_scale_t gyro_fs)
{
	if (!dev || !hi2c)
		return -1;
	memset(dev, 0, sizeof(*dev));
	dev->iface = ICM45686_IFACE_I2C;
	dev->hal.i2c.i2c      = hi2c;
	dev->hal.i2c.i2c_addr = i2c_addr;

	uint8_t who;
	int r = reg_read(dev, ICM45686_REG_WHO_AM_I, &who, 1);
	if (r) return r;
	if (who != ICM45686_WHO_AM_I_VAL)
		return -2;
	r = write_reg8(dev, ICM45686_REG_PWR_MGMT0, 0x0F);
	if (r) return r;
	r = write_reg8(dev, ICM45686_REG_ACCEL_CONFIG0,
	               (uint8_t)((ICM45686_ACCEL_ODR_800HZ & 0x0F) | ((accel_fs & 0x07) << 4)));
	if (r) return r;
	r = write_reg8(dev, ICM45686_REG_GYRO_CONFIG0,
	               (uint8_t)((ICM45686_GYRO_ODR_800HZ & 0x0F) | ((gyro_fs & 0x0F) << 4)));
	if (r) return r;

	dev->accel_scale = 9.80665f / accel_lsb_per_g(accel_fs);
	dev->gyro_scale  = (3.14159265f / 180.0f) / gyro_lsb_per_dps(gyro_fs);
	dev->temp_scale  = 1.0f / ICM45686_TEMP_LSB_PER_C;
	return 0;
}

static int16_t read16_le(const uint8_t *p)
{
	return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

int icm45686_read(icm45686_t *dev, imu_data_t *data)
{
	uint8_t buf[14];
	if (!dev || !data || dev->iface == ICM45686_IFACE_NONE)
		return -1;
	/* Block read: Accel(0x00-0x05) + Gyro(0x06-0x0B) + Temp(0x0C-0x0D) */
	int r = reg_read(dev, ICM45686_REG_ACCEL_DATA_X, buf, sizeof(buf));
	if (r) return r;

	data->accel_mps2[0] = (float)read16_le(buf + 0) * dev->accel_scale;
	data->accel_mps2[1] = (float)read16_le(buf + 2) * dev->accel_scale;
	data->accel_mps2[2] = (float)read16_le(buf + 4) * dev->accel_scale;
	data->gyro_rads[0]  = (float)read16_le(buf + 6) * dev->gyro_scale;
	data->gyro_rads[1]  = (float)read16_le(buf + 8) * dev->gyro_scale;
	data->gyro_rads[2]  = (float)read16_le(buf + 10) * dev->gyro_scale;
	data->temp_celsius  = ICM45686_TEMP_OFFSET_C + (float)read16_le(buf + 12) * dev->temp_scale;
	return 0;
}
