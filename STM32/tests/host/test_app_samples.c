/**
 * @file test_app_samples.c
 */

#include "test_harness.h"

#include "app_samples.h"

void test_app_samples_publish_read(void)
{
    app_imu_sample_t in = {
        .t_us = 42u,
        .seq = 1u,
        .valid = true,
        .pitch_rad = 0.1f,
        .pitch_rate_rads = 0.2f,
    };
    app_imu_sample_t out = {0};

    app_samples_imu_publish(&in);
    TEST_ASSERT(app_samples_imu_read(&out));
    TEST_ASSERT(out.valid);
    TEST_ASSERT(out.t_us == 42u);
    TEST_ASSERT(out.seq == 1u);
    TEST_ASSERT_NEAR(0.1f, out.pitch_rad, 1e-6f);
    TEST_ASSERT_NEAR(0.2f, out.pitch_rate_rads, 1e-6f);
}

void test_app_samples_invalid_before_publish(void)
{
    app_imu_sample_t out = {.valid = true};
    TEST_ASSERT(!app_samples_imu_read(&out));
}
