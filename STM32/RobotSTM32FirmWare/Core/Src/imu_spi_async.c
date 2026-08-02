/**
 * @file imu_spi_async.c
 */

#include "imu_spi_async.h"
#include "app_config.h"
#include "app_dma_buffers.h"
#include "dma.h"
#include "spi.h"

#include "stm32h7xx_hal.h"

#ifndef APP_IMU_SPI_TIMEOUT_MS
#define APP_IMU_SPI_TIMEOUT_MS  10u
#endif

#define IMU_SPI_HANDLE hspi3

static imu_spi_async_done_cb s_done_cb;
static void *s_done_ctx;
static volatile bool s_busy;
static uint8_t *s_active_rx;
static uint16_t s_active_len;

static void app_imu_cs_enable_clock(void)
{
    if (APP_IMU_SPI_CS_PORT == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (APP_IMU_SPI_CS_PORT == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (APP_IMU_SPI_CS_PORT == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    } else if (APP_IMU_SPI_CS_PORT == GPIOD) {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    } else if (APP_IMU_SPI_CS_PORT == GPIOE) {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
}

static void cs_low(void)
{
    HAL_GPIO_WritePin(APP_IMU_SPI_CS_PORT, APP_IMU_SPI_CS_PIN, GPIO_PIN_RESET);
}

static void cs_high(void)
{
    HAL_GPIO_WritePin(APP_IMU_SPI_CS_PORT, APP_IMU_SPI_CS_PIN, GPIO_PIN_SET);
}

static void imu_spi_prepare_bus(void)
{
    if (IMU_SPI_HANDLE.State != HAL_SPI_STATE_READY) {
        (void)HAL_SPI_Abort(&IMU_SPI_HANDLE);
    }
}

static void imu_spi_dma_prepare_tx(const uint8_t *tx, uint16_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_CleanDCache_by_Addr((uint32_t *)(uintptr_t)tx, (int32_t)len);
    }
#else
    (void)tx;
    (void)len;
#endif
}

static void imu_spi_dma_invalidate_rx(const uint8_t *rx, uint16_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)(uintptr_t)rx, (int32_t)len);
    }
#else
    (void)rx;
    (void)len;
#endif
}

void app_imu_cs_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    app_imu_cs_enable_clock();
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

#if APP_IMU_SPI_USE_DMA
    app_imu_dma_nvic_apply();
#endif
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
    s_active_rx = rx;
    s_active_len = len;
    cs_low();
    imu_spi_prepare_bus();

#if APP_IMU_SPI_USE_DMA
    imu_spi_dma_prepare_tx(tx, len);
    if (HAL_SPI_TransmitReceive_DMA(&IMU_SPI_HANDLE, (uint8_t *)tx, rx, len) != HAL_OK) {
        cs_high();
        s_busy = false;
        s_active_rx = NULL;
        s_active_len = 0u;
        return false;
    }
    return true;
#else
    /*
     * Blocking full-duplex xfer (same as icm45686 reg_read_spi in main probe).
     * SPI IT was leaving s_spi_rx zero when the completion IRQ did not run reliably.
     */
    const HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive(&IMU_SPI_HANDLE, (uint8_t *)tx, rx, len, APP_IMU_SPI_TIMEOUT_MS);
    cs_high();
    s_busy = false;
    imu_spi_async_done_cb done = s_done_cb;
    void *ctx = s_done_ctx;
    s_done_cb = NULL;
    s_done_ctx = NULL;
    if (done != NULL) {
        done(ctx, st == HAL_OK);
    }
    return st == HAL_OK;
#endif
}

static void finish(bool ok)
{
    if (!s_busy) {
        return;
    }
#if APP_IMU_SPI_USE_DMA
    if (ok && s_active_rx != NULL && s_active_len > 0u) {
        imu_spi_dma_invalidate_rx(s_active_rx, s_active_len);
    }
#endif
    cs_high();
    s_busy = false;
    s_active_rx = NULL;
    s_active_len = 0u;
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
    (void)HAL_SPI_Abort(&IMU_SPI_HANDLE);
    finish(false);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &IMU_SPI_HANDLE) {
        imu_spi_async_on_txrx_complete();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &IMU_SPI_HANDLE) {
        imu_spi_async_on_error();
    }
}
