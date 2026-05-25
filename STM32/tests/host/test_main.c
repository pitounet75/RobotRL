/**
 * @file test_main.c
 */

#include "test_harness.h"

void test_icm45686_parse_level(void);
void test_icm45686_parse_gyro(void);
void test_icm45686_parse_null(void);
void test_imu_fusion_level_init(void);
void test_imu_fusion_gyro_integrates(void);
void test_imu_fusion_pitch_from_accel(void);
void test_app_samples_publish_read(void);
void test_app_samples_invalid_before_publish(void);

int main(void)
{
    g_tests_run = 0;
    g_tests_failed = 0;

    TEST_RUN(test_icm45686_parse_level);
    TEST_RUN(test_icm45686_parse_gyro);
    TEST_RUN(test_icm45686_parse_null);
    TEST_RUN(test_imu_fusion_level_init);
    TEST_RUN(test_imu_fusion_gyro_integrates);
    TEST_RUN(test_imu_fusion_pitch_from_accel);
    TEST_RUN(test_app_samples_publish_read);
    TEST_RUN(test_app_samples_invalid_before_publish);

    return test_harness_summary();
}
