/**
 * @file app_imu_offset.h
 * @brief Rest-bias for gyro/accel: RAM apply + last-sector flash store.
 */
#ifndef APP_IMU_OFFSET_H
#define APP_IMU_OFFSET_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_IMU_OFFSET_ST_WAIT_IMU = 0,
    APP_IMU_OFFSET_ST_DETECT,
    APP_IMU_OFFSET_ST_SKIPPED,
    APP_IMU_OFFSET_ST_PULSE_IN,
    APP_IMU_OFFSET_ST_WAIT_UPRIGHT, /* operator stands the robot vertical */
    APP_IMU_OFFSET_ST_SETTLE,
    APP_IMU_OFFSET_ST_SAMPLE,
    APP_IMU_OFFSET_ST_PULSE_OUT,
    APP_IMU_OFFSET_ST_SAVE,
    APP_IMU_OFFSET_ST_DONE,
    APP_IMU_OFFSET_ST_SAVE_FAIL,
} app_imu_offset_state_t;

void app_imu_offset_init(void);
void app_imu_offset_apply(float accel_mps2[3], float gyro_rads[3]);
void app_imu_offset_get(float accel_bias_mps2[3], float gyro_bias_rads[3]);
void app_imu_offset_set_runtime(const float accel_bias_mps2[3],
                                const float gyro_bias_rads[3]);
bool app_imu_offset_save_flash(void);

bool app_imu_offset_owns_motors(void);
void app_imu_offset_set_owns_motors(bool own);
void app_imu_offset_set_motor_torque(float left_nm, float right_nm);
void app_imu_offset_motor_torque(float *left_nm, float *right_nm);

void app_imu_offset_request_fusion_reset(void);
bool app_imu_offset_take_fusion_reset(void);

extern volatile uint32_t g_imu_offset_state;
extern volatile uint32_t g_imu_offset_flash_ok; /* 0=none 1=ok 2=rejected */
extern volatile uint32_t g_imu_offset_owns;
extern volatile uint32_t g_imu_offset_sample_n;
extern volatile float g_imu_offset_a_up;
extern volatile float g_imu_offset_accel_mps2[3];
extern volatile float g_imu_offset_gyro_rads[3];

#endif /* APP_IMU_OFFSET_H */
