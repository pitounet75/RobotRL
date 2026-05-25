/**
 * @file app_samples.h
 * @brief Timestamped sensor snapshots (seqlock publish / read).
 */
#ifndef APP_SAMPLES_H
#define APP_SAMPLES_H

#include "app_config.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t t_us;
    uint32_t seq;
    bool valid;
    float pitch_rad;
    float pitch_rate_rads;
    float accel_mps2[3];
    float gyro_rads[3];
} app_imu_sample_t;

void app_samples_imu_publish(const app_imu_sample_t *sample);
bool app_samples_imu_read(app_imu_sample_t *out);

typedef struct {
    uint32_t t_us;
    uint32_t seq;
    bool valid;
    int32_t count;
    float pos_turns;
    float vel_turns_s;
} app_encoder_sample_t;

void app_samples_encoder_publish(const app_encoder_sample_t *sample);
bool app_samples_encoder_read(app_encoder_sample_t *out);

/** One XDrive / ODrive board (axis0 encoder estimates on its CAN node_id). */
typedef struct {
    uint32_t node_id;
    bool valid;
    float pos_turns;
    float vel_turns_s;
    int32_t pos_counts;
    uint32_t last_update_ms;
} app_odrive_drive_sample_t;

typedef struct {
    uint32_t t_us;
    uint32_t seq;
    /** drive[0] = node APP_ODRIVE_DRIVE0_NODE_ID, drive[1] = node APP_ODRIVE_DRIVE1_NODE_ID */
    app_odrive_drive_sample_t drive[APP_ODRIVE_DRIVE_COUNT];
} app_odrive_sample_t;

void app_samples_odrive_publish(const app_odrive_sample_t *sample);
bool app_samples_odrive_read(app_odrive_sample_t *out);

#endif /* APP_SAMPLES_H */
