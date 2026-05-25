/**
 * @file imu_async.c
 */

#include "imu_async.h"
#include "app_time_us.h"
#include "imu_spi_async.h"
#include "spi.h"

#include "stm32h7xx_hal.h"
#include "icm45686.h"
#include "icm45686_parse.h"
#include "bmi323.h"
#include "mpu6050.h"

#include <string.h>

#if APP_IMU_SELECTED == APP_IMU_ICM45686
#define IMU_SPI_READ_REG   0x00u
#define IMU_SPI_READ_LEN   14u
#elif APP_IMU_SELECTED == APP_IMU_BMI323
#define IMU_SPI_READ_REG   0x03u
#define IMU_SPI_READ_LEN   14u
#else
#define IMU_SPI_READ_LEN   0u
#endif

static icm45686_t s_icm45686;
static bmi323_t s_bmi323;
static mpu6050_t s_mpu6050;

static uint8_t s_spi_tx[1u + IMU_SPI_READ_LEN];
static uint8_t s_spi_rx[1u + IMU_SPI_READ_LEN];

static imu_async_done_cb s_user_cb;
static void *s_user_ctx;
static imu_sample_t s_sample;

volatile int32_t g_imu_init_err;
volatile uint8_t g_imu_who_am_i;
volatile uint8_t g_imu_miso_idle;
volatile uint32_t g_imu_read_ok_count;
volatile uint32_t g_imu_read_fail_count;

static void publish(bool ok)
{
    imu_async_done_cb cb = s_user_cb;
    void *ctx = s_user_ctx;
    s_user_cb = NULL;
    s_user_ctx = NULL;
    if (cb != NULL) {
        cb(ctx, &s_sample, ok);
    }
}

#if APP_IMU_SELECTED == APP_IMU_ICM45686
static bool parse_icm45686_raw(const uint8_t *raw14, imu_data_t *data)
{
    icm45686_scales_t scales = {
        .accel_scale = s_icm45686.accel_scale,
        .gyro_scale = s_icm45686.gyro_scale,
        .temp_scale = s_icm45686.temp_scale,
    };
    return icm45686_parse_raw14(raw14, &scales, data);
}
#elif APP_IMU_SELECTED == APP_IMU_BMI323
static int16_t read16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool parse_bmi323_raw(const uint8_t *raw14, imu_data_t *data)
{
    data->accel_mps2[0] = (float)read16_le(raw14 + 0) * s_bmi323.accel_scale;
    data->accel_mps2[1] = (float)read16_le(raw14 + 2) * s_bmi323.accel_scale;
    data->accel_mps2[2] = (float)read16_le(raw14 + 4) * s_bmi323.accel_scale;
    data->gyro_rads[0] = (float)read16_le(raw14 + 6) * s_bmi323.gyro_scale;
    data->gyro_rads[1] = (float)read16_le(raw14 + 8) * s_bmi323.gyro_scale;
    data->gyro_rads[2] = (float)read16_le(raw14 + 10) * s_bmi323.gyro_scale;
    data->temp_celsius = 23.0f + (float)read16_le(raw14 + 12) / 512.0f;
    return true;
}
#endif

#if APP_IMU_SELECTED == APP_IMU_ICM45686 || APP_IMU_SELECTED == APP_IMU_BMI323

static void spi_done(void *user_ctx, bool ok)
{
    (void)user_ctx;
    s_sample.t_us = app_time_us_now();
    s_sample.valid = false;

    if (!ok) {
        publish(false);
        return;
    }

    const uint8_t *raw = &s_spi_rx[1];
#if APP_IMU_SELECTED == APP_IMU_ICM45686
    s_sample.valid = parse_icm45686_raw(raw, &s_sample.data);
#elif APP_IMU_SELECTED == APP_IMU_BMI323
    s_sample.valid = parse_bmi323_raw(raw, &s_sample.data);
#endif
    publish(s_sample.valid);
}

#endif /* SPI IMUs */

bool imu_async_init(void)
{
    memset(&s_sample, 0, sizeof(s_sample));
    g_imu_init_err = 0;
    g_imu_who_am_i = 0u;
    g_imu_read_ok_count = 0u;
    g_imu_read_fail_count = 0u;

#if APP_IMU_SELECTED == APP_IMU_ICM45686
    imu_spi_async_init();
    g_imu_miso_idle = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
    {
        const int rc = icm45686_init_spi(&s_icm45686, &hspi1, APP_IMU_SPI_CS_PORT, APP_IMU_SPI_CS_PIN,
                                         ICM45686_ACCEL_16G, ICM45686_GYRO_2000_DPS);
        if (rc != 0) {
            g_imu_init_err = rc;
            g_imu_who_am_i = icm45686_last_who_am_i;
            return false;
        }
    }
    return true;
#elif APP_IMU_SELECTED == APP_IMU_BMI323
    imu_spi_async_init();
    {
        const int rc = bmi323_init_spi(&s_bmi323, &hspi1, APP_IMU_SPI_CS_PORT, APP_IMU_SPI_CS_PIN,
                                       BMI323_ACCEL_16G, BMI323_GYRO_2000_DPS);
        if (rc != 0) {
            g_imu_init_err = rc;
            return false;
        }
    }
    return true;
#elif APP_IMU_SELECTED == APP_IMU_MPU6050
    /* MPU6050 is I2C-only; wire hi2c + addr when an I2C bus is added in CubeMX. */
    (void)s_mpu6050;
    return false;
#else
    return false;
#endif
}

bool imu_async_busy(void)
{
#if APP_IMU_SELECTED == APP_IMU_ICM45686 || APP_IMU_SELECTED == APP_IMU_BMI323
    return imu_spi_async_busy();
#else
    return false;
#endif
}

bool imu_async_start_read(imu_async_done_cb cb, void *user_ctx)
{
    if (cb == NULL || imu_async_busy()) {
        return false;
    }

#if APP_IMU_SELECTED == APP_IMU_ICM45686 || APP_IMU_SELECTED == APP_IMU_BMI323
    s_user_cb = cb;
    s_user_ctx = user_ctx;

    memset(s_spi_tx, 0, sizeof(s_spi_tx));
    s_spi_tx[0] = (uint8_t)(IMU_SPI_READ_REG | 0x80u);

    return imu_spi_async_xfer(s_spi_tx, s_spi_rx, (uint16_t)(1u + IMU_SPI_READ_LEN), spi_done, NULL);
#else
    (void)user_ctx;
    return false;
#endif
}
