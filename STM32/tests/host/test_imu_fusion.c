/**
 * @file test_imu_fusion.c
 */

#include "test_harness.h"

#include "imu_fusion.h"

#include <math.h>

void test_imu_fusion_level_init(void)
{
    imu_fusion_state_t state;
    imu_fusion_out_t out;
    const float accel[3] = {0.0f, 0.0f, 9.80665f};
    const float gyro[3] = {0.0f, 0.0f, 0.0f};

    imu_fusion_reset(&state);
    TEST_ASSERT(imu_fusion_update(&state, accel, gyro, 1000u, 0.001f, 0.98f,
                                  0, 2, 1, 1.0f, 3.0f, &out));
    TEST_ASSERT_NEAR(0.0f, out.pitch_rad, 0.01f);
    TEST_ASSERT_NEAR(0.0f, out.pitch_rate_rads, 1e-6f);
}

void test_imu_fusion_gyro_integrates(void)
{
    imu_fusion_state_t state;
    imu_fusion_out_t out;
    const float level[3] = {0.0f, 0.0f, 9.80665f};
    const float gyro_rate[3] = {0.0f, 1.0f, 0.0f}; /* 1 rad/s pitch rate */

    imu_fusion_reset(&state);
    TEST_ASSERT(imu_fusion_update(&state, level, gyro_rate, 1000u, 0.001f, 0.98f,
                                  0, 2, 1, 1.0f, 3.0f, &out));
    TEST_ASSERT(imu_fusion_update(&state, level, gyro_rate, 2000u, 0.001f, 0.98f,
                                  0, 2, 1, 1.0f, 3.0f, &out));
    /* alpha=0.98: pitch ≈ 0.98 * 0.001 rad after second sample */
    TEST_ASSERT(out.pitch_rad > 0.0005f);
    TEST_ASSERT(out.pitch_rad < 0.002f);
    TEST_ASSERT_NEAR(1.0f, out.pitch_rate_rads, 1e-6f);
}

void test_imu_fusion_rejects_linear_accel(void)
{
    imu_fusion_state_t state;
    imu_fusion_out_t out;
    const float level[3] = {0.0f, 0.0f, 9.80665f};
    /* ‖a‖ ≈ 19.6 — looks like ~90° tilt if not gated, but is thrust + g. */
    const float thrust[3] = {9.80665f, 0.0f, 9.80665f};
    const float gyro_zero[3] = {0.0f, 0.0f, 0.0f};

    imu_fusion_reset(&state);
    TEST_ASSERT(imu_fusion_update(&state, level, gyro_zero, 1000u, 0.001f, 0.92f,
                                  0, 2, 1, 1.0f, 3.0f, &out));
    TEST_ASSERT_NEAR(0.0f, out.pitch_rad, 0.01f);

    TEST_ASSERT(imu_fusion_update(&state, thrust, gyro_zero, 2000u, 0.001f, 0.92f,
                                  0, 2, 1, 1.0f, 3.0f, &out));
    /* Gyro-only: pitch must stay near 0, not jump toward atan2(-g,g)=π/4. */
    TEST_ASSERT(fabsf(out.pitch_rad) < 0.02f);
}

void test_imu_fusion_pitch_from_accel(void)
{
    const float tilt[3] = {-9.80665f, 0.0f, 0.0f};
    TEST_ASSERT_NEAR(1.5707963f, imu_fusion_pitch_from_accel(tilt), 0.01f);
}
