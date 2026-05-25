/**
 * @file imu_spi_async.c
 */

#include "imu_spi_async.h"
#include "app_config.h"
#include "spi.h"

#include "stm32h7xx_hal.h"

static imu_spi_async_done_cb s_done_cb;
static void *s_done_ctx;
static volatile bool s_busy;

static void cs_low(void)
{
    HAL_GPIO_WritePin(APP_IMU_SPI_CS_PORT, APP_IMU_SPI_CS_PIN, GPIO_PIN_RESET);
}

static void cs_high(void)
{
    HAL_GPIO_WritePin(APP_IMU_SPI_CS_PORT, APP_IMU_SPI_CS_PIN, GPIO_PIN_SET);
}

void app_imu_cs_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = APP_IMU_SPI_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(APP_IMU_SPI_CS_PORT, &gpio);
    cs_high();
}

void imu_spi_async_init(void)
{
    app_imu_cs_gpio_init();
    s_busy = false;

    /* Required for HAL_SPI_TransmitReceive_IT; not enabled in CubeMX SPI1 MSP. */
    HAL_NVIC_SetPriority(SPI1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

bool imu_spi_async_busy(void)
{
    return s_busy;
}

bool imu_spi_async_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len, imu_spi_async_done_cb cb, void *user_ctx)
{
    if (tx == NULL || rx == NULL || len == 0u || cb == NULL || s_busy) {
        return false;
    }

    s_done_cb = cb;
    s_done_ctx = user_ctx;
    s_busy = true;
    cs_low();

#if APP_IMU_SPI_USE_DMA
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)tx, rx, len) != HAL_OK) {
        cs_high();
        s_busy = false;
        return false;
    }
#else
    if (HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)tx, rx, len) != HAL_OK) {
        cs_high();
        s_busy = false;
        return false;
    }
#endif
    return true;
}

static void finish(bool ok)
{
    if (!s_busy) {
        return;
    }
    cs_high();
    s_busy = false;
    imu_spi_async_done_cb cb = s_done_cb;
    void *ctx = s_done_ctx;
    s_done_cb = NULL;
    s_done_ctx = NULL;
    if (cb != NULL) {
        cb(ctx, ok);
    }
}

void imu_spi_async_on_txrx_complete(void)
{
    finish(true);
}

void imu_spi_async_on_error(void)
{
    finish(false);
}

void imu_spi_async_abort(void)
{
    if (!s_busy) {
        return;
    }
    (void)HAL_SPI_Abort(&hspi1);
    finish(false);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        imu_spi_async_on_txrx_complete();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        imu_spi_async_on_error();
    }
}
