/**
 * @file icm45686.c
 * @brief ICM45686 6-axis IMU driver — HAL SPI and I2C.
 *
 * Register map from TDK InvenSense ICM45686 datasheet.
 * ICM45686 has a flat DREG map (0x00–0x7F) — no bank selection.
 * SPI: MSB of first byte = 1 for read; software CS. I2C: Mem_Read/Mem_Write.
 */

#include "icm45686.h"
#include "icm45686_parse.h"
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

#define ICM45686_SPI_READ_BIT     0x80

/** Last WHO_AM_I byte read (for debugger when init returns -2). */
volatile uint8_t icm45686_last_who_am_i;
volatile uint8_t icm45686_who_am_i_mode0;
volatile uint8_t icm45686_who_am_i_mode3;

static void cs_gpio_enable_clock(GPIO_TypeDef *port)
{
	if (port == GPIOA) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
	} else if (port == GPIOB) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
	} else if (port == GPIOC) {
		__HAL_RCC_GPIOC_CLK_ENABLE();
	} else if (port == GPIOD) {
		__HAL_RCC_GPIOD_CLK_ENABLE();
	}
}

static void cs_gpio_init(icm45686_t *dev)
{
	GPIO_InitTypeDef gpio = {0};
	GPIO_TypeDef *port = (GPIO_TypeDef *)dev->hal.spi.cs_port;

	cs_gpio_enable_clock(port);
	gpio.Pin = dev->hal.spi.cs_pin;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(port, &gpio);
	cs_deassert(dev);
}

static int spi_prepare(SPI_HandleTypeDef *hspi)
{
	if (hspi->State != HAL_SPI_STATE_READY) {
		(void)HAL_SPI_Abort(hspi);
	}
	return 0;
}

static int spi_apply_mode(SPI_HandleTypeDef *hspi, uint32_t cpol, uint32_t cpha, uint32_t prescaler)
{
	if (spi_prepare(hspi) != 0) {
		return -1;
	}
	(void)HAL_SPI_DeInit(hspi);
	hspi->Init.CLKPolarity = cpol;
	hspi->Init.CLKPhase = cpha;
	hspi->Init.BaudRatePrescaler = prescaler;
	if (HAL_SPI_Init(hspi) != HAL_OK) {
		return -1;
	}
	return 0;
}

static void cs_assert(icm45686_t *dev)
{
	HAL_GPIO_WritePin((GPIO_TypeDef *)dev->hal.spi.cs_port, dev->hal.spi.cs_pin, GPIO_PIN_RESET);
}

static void cs_deassert(icm45686_t *dev)
{
	HAL_GPIO_WritePin((GPIO_TypeDef *)dev->hal.spi.cs_port, dev->hal.spi.cs_pin, GPIO_PIN_SET);
}

static int reg_read_spi(icm45686_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)dev->hal.spi.spi;
	uint8_t tx[32];
	uint8_t rx[32];
	const uint16_t total = (uint16_t)(1u + len);

	if (total > sizeof(tx)) {
		return -1;
	}

	tx[0] = (uint8_t)(reg | ICM45686_SPI_READ_BIT);
	memset(tx + 1, 0, len);

	(void)spi_prepare(hspi);
	cs_assert(dev);
	HAL_StatusTypeDef s = HAL_SPI_TransmitReceive(hspi, tx, rx, total, IMU_HAL_TIMEOUT_MS);
	cs_deassert(dev);
	if (s != HAL_OK) {
		return -1;
	}

	memcpy(buf, rx + 1, len);
	return 0;
}

static int reg_read(icm45686_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
	if (dev->iface == ICM45686_IFACE_SPI) {
		return reg_read_spi(dev, reg, buf, len);
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

	cs_gpio_init(dev);
	HAL_Delay(50);

	SPI_HandleTypeDef *bus = (SPI_HandleTypeDef *)hspi;
	uint8_t who = 0;
	int r = -1;

	icm45686_who_am_i_mode0 = 0xFFu;
	icm45686_who_am_i_mode3 = 0xFFu;

	if (spi_apply_mode(bus, SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_32) == 0) {
		r = reg_read(dev, ICM45686_REG_WHO_AM_I, &who, 1);
		icm45686_who_am_i_mode3 = who;
	}
	if (r != 0 || who != ICM45686_WHO_AM_I_VAL) {
		if (spi_apply_mode(bus, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, SPI_BAUDRATEPRESCALER_32) == 0) {
			r = reg_read(dev, ICM45686_REG_WHO_AM_I, &who, 1);
			icm45686_who_am_i_mode0 = who;
		}
	}
	icm45686_last_who_am_i = who;
	if (r != 0) {
		return r;
	}
	if (who != ICM45686_WHO_AM_I_VAL) {
		return -2;
	}

	(void)spi_apply_mode(bus, bus->Init.CLKPolarity, bus->Init.CLKPhase, SPI_BAUDRATEPRESCALER_8);

	uint8_t who_check;
	r = reg_read(dev, ICM45686_REG_WHO_AM_I, &who_check, 1);
	icm45686_last_who_am_i = who_check;
	if (r) return r;
	if (who_check != ICM45686_WHO_AM_I_VAL)
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

	icm45686_scales_t scales;
	icm45686_scales_for_config(accel_fs, gyro_fs, &scales);
	dev->accel_scale = scales.accel_scale;
	dev->gyro_scale = scales.gyro_scale;
	dev->temp_scale = scales.temp_scale;
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
	icm45686_last_who_am_i = who;
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

	icm45686_scales_t scales;
	icm45686_scales_for_config(accel_fs, gyro_fs, &scales);
	dev->accel_scale = scales.accel_scale;
	dev->gyro_scale = scales.gyro_scale;
	dev->temp_scale = scales.temp_scale;
	return 0;
}

int icm45686_read(icm45686_t *dev, imu_data_t *data)
{
	uint8_t buf[14];
	if (!dev || !data || dev->iface == ICM45686_IFACE_NONE)
		return -1;
	/* Block read: Accel(0x00-0x05) + Gyro(0x06-0x0B) + Temp(0x0C-0x0D) */
	int r = reg_read(dev, ICM45686_REG_ACCEL_DATA_X, buf, sizeof(buf));
	if (r) return r;

	icm45686_scales_t scales = {
		.accel_scale = dev->accel_scale,
		.gyro_scale = dev->gyro_scale,
		.temp_scale = dev->temp_scale,
	};
	return icm45686_parse_raw14(buf, &scales, data) ? 0 : -1;
}
