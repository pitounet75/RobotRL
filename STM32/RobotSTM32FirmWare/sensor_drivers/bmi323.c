/**
 * @file bmi323.c
 * @brief BMI323 6-axis IMU driver — HAL SPI and I2C (Bosch Sensortec).
 *
 * Register map and init sequence aligned with Linux drivers/iio/imu/bmi323 and
 * Bosch BMI323 datasheet (soft reset 0xDEAF, feature engine, I2C 2 dummy bytes).
 *
 * SPI: one dummy byte before each read payload (16-bit registers, LE).
 * I2C: two dummy bytes before each read payload (per datasheet §4).
 */

#include "bmi323.h"
#include "imu_hal_platform.h"
#include <string.h>

/* --- Register map (Linux bmi323.h / datasheet) --- */
#define BMI323_REG_CHIP_ID       0x00
#define BMI323_REG_ERR_REG       0x01
#define BMI323_REG_STATUS        0x02
#define BMI323_REG_ACC_X         0x03
#define BMI323_REG_FEAT_IO1      0x11
#define BMI323_REG_FEAT_IO2      0x12
#define BMI323_REG_FEAT_IO_STAT  0x14
#define BMI323_REG_ACC_CONF      0x20
#define BMI323_REG_GYRO_CONF     0x21
#define BMI323_REG_FEAT_CTRL     0x40
#define BMI323_REG_CMD           0x7E

#define BMI323_CHIP_ID_EXPECT    0x0043u
#define BMI323_CHIP_ID_MASK      0x00FFu

/** Soft reset: write 16-bit LE 0xDEAF to CMD (not single byte 0xDE). */
#define BMI323_CMD_RST_LO        0xAFu
#define BMI323_CMD_RST_HI        0xDEu

#define BMI323_STATUS_POR_MSK    0x0001u

#define BMI323_SPI_READ_BIT      0x80u
#define BMI323_SPI_DUMMY_BYTES   1u
#define BMI323_I2C_DUMMY_BYTES   2u

/* ACC_CONF / GYRO_CONF bit fields (Linux GENMASK) */
#define BMI323_CONF_MODE_MSK     0x7000u  /* bits 14:12 */
#define BMI323_CONF_BW_MSK       0x0080u  /* bit 7 */
#define BMI323_CONF_SCL_MSK      0x0070u  /* bits 6:4 range */
#define BMI323_CONF_ODR_MSK      0x000Fu  /* bits 3:0 */

#define BMI323_MODE_DISABLE      0u
#define BMI323_MODE_DUTYCYCLE    3u
#define BMI323_MODE_CONTINUOUS   4u

/** ODR field value for 100 Hz (Linux bmi323_acc_gyro_odr index 7 → written 8). */
#define BMI323_ODR_CODE_100HZ    8u

#define BMI323_BW_ODR_DIV_2      0u  /* clear BW bit 7 */

#define BMI323_FEAT_IO2_LOAD     0x012Cu
#define BMI323_FEAT_IO_STAT_SET  0x0001u
#define BMI323_FEAT_CTRL_EN      0x0001u
#define BMI323_FEAT_IO1_RDY_MSK  0x000Fu

/* Datasheet / Linux: T_C = (temp_raw / 512) + 23 */
#define BMI323_TEMP_ZERO_C       23.0f
#define BMI323_TEMP_LSB_PER_C    512.0f

static float bmi323_accel_lsb_per_g(bmi323_accel_scale_t fs)
{
	static const float lsb[] = {
		[BMI323_ACCEL_2G]  = 16384.0f,
		[BMI323_ACCEL_4G]  = 8192.0f,
		[BMI323_ACCEL_8G]  = 4096.0f,
		[BMI323_ACCEL_16G] = 2048.0f,
	};
	return lsb[fs];
}

static float bmi323_gyro_lsb_per_dps(bmi323_gyro_scale_t fs)
{
	static const float lsb[] = {
		[BMI323_GYRO_125_DPS]   = 262.1f,
		[BMI323_GYRO_250_DPS]   = 131.1f,
		[BMI323_GYRO_500_DPS]   = 65.5f,
		[BMI323_GYRO_1000_DPS]  = 32.8f,
		[BMI323_GYRO_2000_DPS]  = 16.4f,
	};
	return lsb[fs];
}

static void cs_assert(bmi323_t *dev)
{
	HAL_GPIO_WritePin((GPIO_TypeDef *)dev->hal.spi.cs_port, dev->hal.spi.cs_pin, GPIO_PIN_RESET);
}

static void cs_deassert(bmi323_t *dev)
{
	HAL_GPIO_WritePin((GPIO_TypeDef *)dev->hal.spi.cs_port, dev->hal.spi.cs_pin, GPIO_PIN_SET);
}

static int reg_read_spi(bmi323_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)dev->hal.spi.spi;
	uint16_t total = BMI323_SPI_DUMMY_BYTES + len;
	uint8_t tx[40];
	uint8_t rx[40];
	if (total > sizeof(tx))
		return -1;
	tx[0] = reg | BMI323_SPI_READ_BIT;
	memset(tx + 1, 0, total - 1);
	cs_assert(dev);
	HAL_StatusTypeDef s = HAL_SPI_TransmitReceive(hspi, tx, rx, total, IMU_HAL_TIMEOUT_MS);
	cs_deassert(dev);
	if (s != HAL_OK)
		return -1;
	memcpy(buf, rx + BMI323_SPI_DUMMY_BYTES, len);
	return 0;
}

static int reg_read_i2c(bmi323_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)dev->hal.i2c.i2c;
	uint8_t addr = dev->hal.i2c.i2c_addr << 1;
	uint8_t tmp[40];
	if ((uint32_t)len + BMI323_I2C_DUMMY_BYTES > sizeof(tmp))
		return -1;
	HAL_StatusTypeDef s = HAL_I2C_Mem_Read(hi2c, addr, reg, I2C_MEMADD_SIZE_8BIT,
		tmp, (uint16_t)(len + BMI323_I2C_DUMMY_BYTES), IMU_HAL_TIMEOUT_MS);
	if (s != HAL_OK)
		return -1;
	memcpy(buf, tmp + BMI323_I2C_DUMMY_BYTES, len);
	return 0;
}

static int reg_read(bmi323_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
	if (dev->iface == BMI323_IFACE_SPI)
		return reg_read_spi(dev, reg, buf, len);
	if (dev->iface == BMI323_IFACE_I2C)
		return reg_read_i2c(dev, reg, buf, len);
	return -1;
}

static int reg_write_spi(bmi323_t *dev, uint8_t reg, const uint8_t *buf, uint16_t len)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)dev->hal.spi.spi;
	uint8_t tx[24];
	if (len + 1 > sizeof(tx))
		return -1;
	tx[0] = reg & ~BMI323_SPI_READ_BIT;
	memcpy(tx + 1, buf, len);
	cs_assert(dev);
	HAL_StatusTypeDef s = HAL_SPI_Transmit(hspi, tx, 1 + len, IMU_HAL_TIMEOUT_MS);
	cs_deassert(dev);
	return (s == HAL_OK) ? 0 : -1;
}

static int reg_write_i2c(bmi323_t *dev, uint8_t reg, const uint8_t *buf, uint16_t len)
{
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)dev->hal.i2c.i2c;
	uint8_t addr = dev->hal.i2c.i2c_addr << 1;
	HAL_StatusTypeDef s = HAL_I2C_Mem_Write(hi2c, addr, reg, I2C_MEMADD_SIZE_8BIT,
		(uint8_t *)buf, len, IMU_HAL_TIMEOUT_MS);
	return (s == HAL_OK) ? 0 : -1;
}

static int reg_write(bmi323_t *dev, uint8_t reg, const uint8_t *buf, uint16_t len)
{
	if (dev->iface == BMI323_IFACE_SPI)
		return reg_write_spi(dev, reg, buf, len);
	if (dev->iface == BMI323_IFACE_I2C)
		return reg_write_i2c(dev, reg, buf, len);
	return -1;
}

static int reg_write_u16_le(bmi323_t *dev, uint8_t reg, uint16_t v)
{
	uint8_t b[2] = { (uint8_t)(v & 0xFFu), (uint8_t)(v >> 8) };
	return reg_write(dev, reg, b, 2);
}

/** Soft reset + SPI dummy read + POR + chip ID + feature engine + err check. */
static int bmi323_chip_init(bmi323_t *dev)
{
	uint8_t id_raw[2];
	uint8_t st_raw[2];
	uint8_t err_raw[2];
	int r;

	/* §5.17: write 0xDEAF to CMD; wait ≥1.5 ms */
	uint8_t rst[2] = { BMI323_CMD_RST_LO, BMI323_CMD_RST_HI };
	r = reg_write(dev, BMI323_REG_CMD, rst, 2);
	if (r)
		return r;
	HAL_Delay(2);

	/* §7.2.1 SPI: dummy read after reset enables SPI */
	r = reg_read(dev, BMI323_REG_CHIP_ID, id_raw, 2);
	if (r)
		return r;

	r = reg_read(dev, BMI323_REG_STATUS, st_raw, 2);
	if (r)
		return r;
	uint16_t status = (uint16_t)st_raw[0] | ((uint16_t)st_raw[1] << 8);
	if ((status & BMI323_STATUS_POR_MSK) == 0)
		return -3;

	r = reg_read(dev, BMI323_REG_CHIP_ID, id_raw, 2);
	if (r)
		return r;
	uint16_t chip = (uint16_t)id_raw[0] | ((uint16_t)id_raw[1] << 8);
	if ((chip & BMI323_CHIP_ID_MASK) != (BMI323_CHIP_ID_EXPECT & BMI323_CHIP_ID_MASK))
		return -2;

	/* Feature engine enable (Linux bmi323_feature_engine_enable) */
	r = reg_write_u16_le(dev, BMI323_REG_FEAT_IO2, BMI323_FEAT_IO2_LOAD);
	if (r)
		return r;
	r = reg_write_u16_le(dev, BMI323_REG_FEAT_IO_STAT, BMI323_FEAT_IO_STAT_SET);
	if (r)
		return r;
	r = reg_write_u16_le(dev, BMI323_REG_FEAT_CTRL, BMI323_FEAT_CTRL_EN);
	if (r)
		return r;

	for (int i = 0; i < 200; i++) {
		HAL_Delay(2);
		r = reg_read(dev, BMI323_REG_FEAT_IO1, id_raw, 2);
		if (r)
			return r;
		uint16_t feat = (uint16_t)id_raw[0] | ((uint16_t)id_raw[1] << 8);
		if ((feat & BMI323_FEAT_IO1_RDY_MSK) == 1u)
			break;
		if (i == 199)
			return -4;
	}

	r = reg_read(dev, BMI323_REG_ERR_REG, err_raw, 2);
	if (r)
		return r;
	uint16_t err = (uint16_t)err_raw[0] | ((uint16_t)err_raw[1] << 8);
	if (err != 0)
		return -5;

	return 0;
}

static int configure_acc_gyro(bmi323_t *dev, bmi323_accel_scale_t accel_fs, bmi323_gyro_scale_t gyro_fs)
{
	uint8_t buf[2];
	uint16_t val;
	int r;

	/* 100 Hz, continuous mode, BW ODR/2, user range in SCL field */
	r = reg_read(dev, BMI323_REG_ACC_CONF, buf, 2);
	if (r)
		return r;
	val = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
	val = (uint16_t)(val & ~(BMI323_CONF_MODE_MSK | BMI323_CONF_ODR_MSK |
		BMI323_CONF_SCL_MSK | BMI323_CONF_BW_MSK));
	val |= (uint16_t)(BMI323_MODE_CONTINUOUS << 12);
	val |= (uint16_t)(BMI323_ODR_CODE_100HZ << 0);
	val |= (uint16_t)((accel_fs & 7u) << 4);
	val |= (uint16_t)(BMI323_BW_ODR_DIV_2 << 7);
	buf[0] = (uint8_t)(val & 0xFFu);
	buf[1] = (uint8_t)(val >> 8);
	r = reg_write(dev, BMI323_REG_ACC_CONF, buf, 2);
	if (r)
		return r;

	r = reg_read(dev, BMI323_REG_GYRO_CONF, buf, 2);
	if (r)
		return r;
	val = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
	val = (uint16_t)(val & ~(BMI323_CONF_MODE_MSK | BMI323_CONF_ODR_MSK |
		BMI323_CONF_SCL_MSK | BMI323_CONF_BW_MSK));
	val |= (uint16_t)(BMI323_MODE_CONTINUOUS << 12);
	val |= (uint16_t)(BMI323_ODR_CODE_100HZ << 0);
	val |= (uint16_t)((gyro_fs & 7u) << 4);
	val |= (uint16_t)(BMI323_BW_ODR_DIV_2 << 7);
	buf[0] = (uint8_t)(val & 0xFFu);
	buf[1] = (uint8_t)(val >> 8);
	return reg_write(dev, BMI323_REG_GYRO_CONF, buf, 2);
}

int bmi323_init_spi(bmi323_t *dev, void *hspi, void *cs_port, uint16_t cs_pin,
                    bmi323_accel_scale_t accel_fs, bmi323_gyro_scale_t gyro_fs)
{
	if (!dev || !hspi || !cs_port)
		return -1;
	memset(dev, 0, sizeof(*dev));
	dev->iface = BMI323_IFACE_SPI;
	dev->hal.spi.spi     = hspi;
	dev->hal.spi.cs_port = cs_port;
	dev->hal.spi.cs_pin  = cs_pin;

	int r = bmi323_chip_init(dev);
	if (r)
		return r;
	r = configure_acc_gyro(dev, accel_fs, gyro_fs);
	if (r)
		return r;

	dev->accel_scale = 9.80665f / bmi323_accel_lsb_per_g(accel_fs);
	dev->gyro_scale  = (3.14159265f / 180.0f) / bmi323_gyro_lsb_per_dps(gyro_fs);
	return 0;
}

int bmi323_init_i2c(bmi323_t *dev, void *hi2c, uint8_t i2c_addr,
                    bmi323_accel_scale_t accel_fs, bmi323_gyro_scale_t gyro_fs)
{
	if (!dev || !hi2c)
		return -1;
	memset(dev, 0, sizeof(*dev));
	dev->iface = BMI323_IFACE_I2C;
	dev->hal.i2c.i2c      = hi2c;
	dev->hal.i2c.i2c_addr = i2c_addr;

	int r = bmi323_chip_init(dev);
	if (r)
		return r;
	r = configure_acc_gyro(dev, accel_fs, gyro_fs);
	if (r)
		return r;

	dev->accel_scale = 9.80665f / bmi323_accel_lsb_per_g(accel_fs);
	dev->gyro_scale  = (3.14159265f / 180.0f) / bmi323_gyro_lsb_per_dps(gyro_fs);
	return 0;
}

static int16_t read16_le(const uint8_t *p)
{
	return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

int bmi323_read(bmi323_t *dev, imu_data_t *data)
{
	uint8_t buf[14];
	if (!dev || !data || dev->iface == BMI323_IFACE_NONE)
		return -1;
	int r = reg_read(dev, BMI323_REG_ACC_X, buf, sizeof(buf));
	if (r)
		return r;

	data->accel_mps2[0] = (float)read16_le(buf + 0) * dev->accel_scale;
	data->accel_mps2[1] = (float)read16_le(buf + 2) * dev->accel_scale;
	data->accel_mps2[2] = (float)read16_le(buf + 4) * dev->accel_scale;
	data->gyro_rads[0]  = (float)read16_le(buf + 6) * dev->gyro_scale;
	data->gyro_rads[1]  = (float)read16_le(buf + 8) * dev->gyro_scale;
	data->gyro_rads[2]  = (float)read16_le(buf + 10) * dev->gyro_scale;
	data->temp_celsius  = BMI323_TEMP_ZERO_C + (float)read16_le(buf + 12) / BMI323_TEMP_LSB_PER_C;
	return 0;
}
