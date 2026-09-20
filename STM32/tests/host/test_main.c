/**
 * @file test_main.c
 */

#include "test_harness.h"

void test_icm45686_parse_level(void);
void test_icm45686_parse_gyro(void);
void test_icm45686_parse_null(void);
void test_imu_fusion_level_init(void);
void test_imu_fusion_gyro_integrates(void);
void test_imu_fusion_rejects_linear_accel(void);
void test_imu_fusion_pitch_from_accel(void);
void test_app_samples_publish_read(void);
void test_app_samples_invalid_before_publish(void);
void test_vel_fit_invalid_until_window_full(void);
void test_vel_fit_constant_velocity(void);
void test_vel_fit_constant_acceleration(void);
void test_vel_fit_reports_centre_not_newest(void);
void test_vel_fit_quantisation_noise_bounded(void);

int g_tests_run;
int g_tests_failed;

int main(void)
{
    g_tests_run = 0;
    g_tests_failed = 0;

    TEST_RUN(test_icm45686_parse_level);
    TEST_RUN(test_icm45686_parse_gyro);
    TEST_RUN(test_icm45686_parse_null);
    TEST_RUN(test_imu_fusion_level_init);
    TEST_RUN(test_imu_fusion_gyro_integrates);
    TEST_RUN(test_imu_fusion_rejects_linear_accel);
    TEST_RUN(test_imu_fusion_pitch_from_accel);
    /* Must precede any publish: it asserts the pre-publish state. */
    TEST_RUN(test_app_samples_invalid_before_publish);
    TEST_RUN(test_app_samples_publish_read);
    TEST_RUN(test_vel_fit_invalid_until_window_full);
    TEST_RUN(test_vel_fit_constant_velocity);
    TEST_RUN(test_vel_fit_constant_acceleration);
    TEST_RUN(test_vel_fit_reports_centre_not_newest);
    TEST_RUN(test_vel_fit_quantisation_noise_bounded);

    return test_harness_summary();
}
