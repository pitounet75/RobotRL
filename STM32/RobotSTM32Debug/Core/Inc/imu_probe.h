#pragma once

#include <stdint.h>

/** 0 = WHO+config OK. -1 = SPI, -2 = WHO_AM_I != 0xE9. */
extern volatile int32_t g_mpu_init;
/** Last WHO_AM_I (ICM45686 = 0xE9). */
extern volatile uint32_t g_mpu_who;
/** 1 if last sample parsed. */
extern volatile uint32_t g_mpu_ok;
/** Accel in milli-g (still, one axis ≈ ±1000). */
extern volatile int32_t g_mpu_ax_mg;
extern volatile int32_t g_mpu_ay_mg;
extern volatile int32_t g_mpu_az_mg;
/** Gyro in milli-rad/s. */
extern volatile int32_t g_mpu_gx_mrads;
extern volatile int32_t g_mpu_gy_mrads;
extern volatile int32_t g_mpu_gz_mrads;
extern volatile int32_t g_mpu_temp_c;
extern volatile uint32_t g_mpu_read_ok;
extern volatile uint32_t g_mpu_read_fail;

void imu_probe_init(void);
/** Blocking SPI sample ~100 Hz. Safe to call from the encoder while(1). */
void imu_probe_poll(void);
