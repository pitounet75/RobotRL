/**
 * @file imu_spi_async.h
 * @brief Non-blocking SPI1 transfers for IMU burst reads (DMA or IT).
 */
#ifndef IMU_SPI_ASYNC_H
#define IMU_SPI_ASYNC_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*imu_spi_async_done_cb)(void *user_ctx, bool ok);

/** Drive CS high; call before MX_SPI1_Init so the bus idles safely. */
void app_imu_cs_gpio_init(void);

void imu_spi_async_init(void);
bool imu_spi_async_busy(void);

/**
 * Full-duplex SPI transaction with software CS.
 * @param tx        TX buffer (must stay valid until callback)
 * @param rx        RX buffer (same length as tx)
 * @param len       Byte count
 */
bool imu_spi_async_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len, imu_spi_async_done_cb cb, void *user_ctx);

void imu_spi_async_on_txrx_complete(void);
void imu_spi_async_on_error(void);
void imu_spi_async_abort(void);

#endif /* IMU_SPI_ASYNC_H */
