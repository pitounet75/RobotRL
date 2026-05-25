/**
 * @file test_icm45686_parse.c
 */

#include "test_harness.h"

#include "icm45686_parse.h"

void test_icm45686_parse_level(void)
{
    icm45686_scales_t scales;
    icm45686_scales_for_config(ICM45686_ACCEL_16G, ICM45686_GYRO_2000_DPS, &scales);

    /* 1 g on Z: raw 2048 LSB @ 16G; temp raw 0 -> 25 C */
    uint8_t raw[14] = {0};
    raw[4] = 0x00;
    raw[5] = 0x08;

    imu_data_t data;
    TEST_ASSERT(icm45686_parse_raw14(raw, &scales, &data));
    TEST_ASSERT_NEAR(0.0f, data.accel_mps2[0], 1e-3f);
    TEST_ASSERT_NEAR(0.0f, data.accel_mps2[1], 1e-3f);
    TEST_ASSERT_NEAR(9.80665f, data.accel_mps2[2], 0.05f);
    TEST_ASSERT_NEAR(25.0f, data.temp_celsius, 0.01f);
}

void test_icm45686_parse_gyro(void)
{
    icm45686_scales_t scales;
    icm45686_scales_for_config(ICM45686_ACCEL_16G, ICM45686_GYRO_2000_DPS, &scales);

    /* 16 LSB = 1 dps @ 2000 dps FS -> pi/180 rad/s on gyro Y */
    uint8_t raw[14] = {0};
    raw[8] = 0x10;
    raw[9] = 0x00;

    imu_data_t data;
    TEST_ASSERT(icm45686_parse_raw14(raw, &scales, &data));
    TEST_ASSERT_NEAR(0.0f, data.gyro_rads[0], 1e-5f);
    TEST_ASSERT_NEAR(3.14159265f / 180.0f, data.gyro_rads[1], 1e-4f);
    TEST_ASSERT_NEAR(0.0f, data.gyro_rads[2], 1e-5f);
}

void test_icm45686_parse_null(void)
{
    imu_data_t data;
    icm45686_scales_t scales = {1.0f, 1.0f, 1.0f};
    uint8_t raw[14] = {0};
    TEST_ASSERT(!icm45686_parse_raw14(NULL, &scales, &data));
    TEST_ASSERT(!icm45686_parse_raw14(raw, NULL, &data));
    TEST_ASSERT(!icm45686_parse_raw14(raw, &scales, NULL));
}
