/**
 * @file imu_async.h
 * @brief FreeRTOS-friendly async IMU reads (ICM45686 / BMI323 / MPU6050).
 */
#ifndef IMU_ASYNC_H
#define IMU_ASYNC_H

#include "app_config.h"
#include "imu_driver.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t t_us;
    imu_data_t data;
    bool valid;
} imu_sample_t;

typedef void (*imu_async_done_cb)(void *user_ctx, const imu_sample_t *sample, bool ok);

bool imu_async_init(void);
bool imu_async_busy(void);
bool imu_async_start_read(imu_async_done_cb cb, void *user_ctx);

/** Call after main() probe when icm45686_init_spi already succeeded (skips HAL_Delay re-init). */
void imu_async_note_hw_probed_ok(uint8_t who_am_i);

/** 0 = OK; -1 SPI/WHO read fail; -2 WHO_AM_I mismatch (expect 0xE9). */
extern volatile int32_t g_imu_init_err;
/** Raw WHO_AM_I from last init attempt (valid when g_imu_init_err != 0). */
extern volatile uint8_t g_imu_who_am_i;
/** MISO (PA6) level before first SPI xfer; 1 is normal with pull-up. */
extern volatile uint8_t g_imu_miso_idle;
/** Incremented on each successful fused publish path. */
extern volatile uint32_t g_imu_read_ok_count;
extern volatile uint32_t g_imu_read_fail_count;

#endif /* IMU_ASYNC_H */
