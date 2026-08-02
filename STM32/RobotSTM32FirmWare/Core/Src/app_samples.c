/**
 * @file app_samples.c
 */

#include "app_samples.h"

#include <string.h>

#if defined(__ARM_ARCH)
#define APP_SAMPLES_DMB() __asm volatile("dmb" ::: "memory")
#elif defined(__GNUC__)
#define APP_SAMPLES_DMB() __asm volatile("" ::: "memory")
#else
#define APP_SAMPLES_DMB() ((void)0)
#endif

static volatile uint32_t s_imu_seq;
static app_imu_sample_t s_imu;

static volatile uint32_t s_encoder_seq;
static app_encoder_sample_t s_encoder;

static volatile uint32_t s_encoder_bank_seq;
static app_encoder_bank_sample_t s_encoder_bank;

static volatile uint32_t s_odrive_seq;
static app_odrive_sample_t s_odrive;

void app_samples_imu_publish(const app_imu_sample_t *sample)
{
    if (sample == NULL) {
        return;
    }

    uint32_t seq = s_imu_seq + 1u;
    s_imu_seq = seq;
    s_imu = *sample;
    s_imu_seq = seq + 1u;
}

bool app_samples_imu_read(app_imu_sample_t *out)
{
    if (out == NULL) {
        return false;
    }

    for (uint32_t attempt = 0u; attempt < 4u; attempt++) {
        uint32_t seq1 = s_imu_seq;
        if ((seq1 & 1u) != 0u) {
            continue;
        }
        *out = s_imu;
        APP_SAMPLES_DMB();
        if (s_imu_seq == seq1) {
            return out->valid;
        }
    }

    return false;
}

void app_samples_encoder_publish(const app_encoder_sample_t *sample)
{
    if (sample == NULL) {
        return;
    }

    uint32_t seq = s_encoder_seq + 1u;
    s_encoder_seq = seq;
    s_encoder = *sample;
    s_encoder_seq = seq + 1u;
}

bool app_samples_encoder_read(app_encoder_sample_t *out)
{
    if (out == NULL) {
        return false;
    }

    for (uint32_t attempt = 0u; attempt < 4u; attempt++) {
        uint32_t seq1 = s_encoder_seq;
        if ((seq1 & 1u) != 0u) {
            continue;
        }
        *out = s_encoder;
        APP_SAMPLES_DMB();
        if (s_encoder_seq == seq1) {
            return out->valid;
        }
    }

    return false;
}

void app_samples_encoder_bank_publish(const app_encoder_bank_sample_t *sample)
{
    if (sample == NULL) {
        return;
    }

    uint32_t seq = s_encoder_bank_seq + 1u;
    s_encoder_bank_seq = seq;
    s_encoder_bank = *sample;
    s_encoder_bank_seq = seq + 1u;
}

bool app_samples_encoder_bank_read(app_encoder_bank_sample_t *out)
{
    if (out == NULL) {
        return false;
    }

    for (uint32_t attempt = 0u; attempt < 4u; attempt++) {
        uint32_t seq1 = s_encoder_bank_seq;
        if ((seq1 & 1u) != 0u) {
            continue;
        }
        *out = s_encoder_bank;
        APP_SAMPLES_DMB();
        if (s_encoder_bank_seq == seq1) {
            for (uint32_t i = 0u; i < APP_LOCAL_ENCODER_COUNT; i++) {
                if (out->encoder[i].valid) {
                    return true;
                }
            }
            return false;
        }
    }

    return false;
}

void app_samples_odrive_publish(const app_odrive_sample_t *sample)
{
    if (sample == NULL) {
        return;
    }

    uint32_t seq = s_odrive_seq + 1u;
    s_odrive_seq = seq;
    s_odrive = *sample;
    s_odrive_seq = seq + 1u;
}

bool app_samples_odrive_read(app_odrive_sample_t *out)
{
    if (out == NULL) {
        return false;
    }

    for (uint32_t attempt = 0u; attempt < 4u; attempt++) {
        uint32_t seq1 = s_odrive_seq;
        if ((seq1 & 1u) != 0u) {
            continue;
        }
        *out = s_odrive;
        APP_SAMPLES_DMB();
        if (s_odrive_seq == seq1) {
            return (out->drive[0].valid || out->drive[1].valid);
        }
    }

    return false;
}
