/**
 * @file task_bias.c
 * @brief Startup IMU rest-bias: lie-down trigger, stand vertical, 20 s average.
 */

#include "tasks/tasks.h"

#include "app_config.h"
#include "app_imu_offset.h"
#include "app_samples.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PULSE_HIGH = 0,
    PULSE_GAP,
} pulse_phase_t;

typedef struct {
    uint32_t remaining;
    uint32_t elapsed_ms;
    pulse_phase_t phase;
} pulse_seq_t;

static pulse_seq_t s_pulse;
static uint32_t s_lie_ms;
static uint32_t s_upright_ms;
static uint32_t s_cal_ms;
static uint32_t s_settle_ms;
static uint32_t s_sample_wait_ms;
static uint32_t s_sample_n;
static uint32_t s_last_imu_seq;
static double s_acc_sum[3];
static double s_gyro_sum[3];

static void pulse_start(uint32_t count)
{
    s_pulse.remaining = count;
    s_pulse.elapsed_ms = 0u;
    s_pulse.phase = PULSE_HIGH;
    app_imu_offset_set_motor_torque(APP_IMU_OFFSET_PULSE_NM, APP_IMU_OFFSET_PULSE_NM);
}

/** @return true when the requested pulse train has finished. */
static bool pulse_tick(uint32_t dt_ms)
{
    if (s_pulse.remaining == 0u) {
        app_imu_offset_set_motor_torque(0.0f, 0.0f);
        return true;
    }

    s_pulse.elapsed_ms += dt_ms;

    if (s_pulse.phase == PULSE_HIGH) {
        if (s_pulse.elapsed_ms >= APP_IMU_OFFSET_PULSE_MS) {
            s_pulse.remaining--;
            s_pulse.elapsed_ms = 0u;
            app_imu_offset_set_motor_torque(0.0f, 0.0f);
            if (s_pulse.remaining == 0u) {
                return true;
            }
            s_pulse.phase = PULSE_GAP;
        }
        return false;
    }

    if (s_pulse.elapsed_ms >= APP_IMU_OFFSET_PULSE_GAP_MS) {
        s_pulse.elapsed_ms = 0u;
        s_pulse.phase = PULSE_HIGH;
        app_imu_offset_set_motor_torque(APP_IMU_OFFSET_PULSE_NM, APP_IMU_OFFSET_PULSE_NM);
    }
    return false;
}

static void robot_raw_accel(const app_imu_sample_t *imu, float a[3])
{
    float accel_bias[3];
    float gyro_bias[3];
    uint32_t i;

    app_imu_offset_get(accel_bias, gyro_bias);
    (void)gyro_bias;
    for (i = 0u; i < 3u; i++) {
        a[i] = imu->accel_mps2[i] + accel_bias[i];
    }
}

static float robot_raw_a_up(const app_imu_sample_t *imu)
{
    float a[3];
    robot_raw_accel(imu, a);
    return a[APP_IMU_PITCH_ACCEL_UP_AXIS];
}

static bool robot_accel_norm_ok(const app_imu_sample_t *imu)
{
    float a[3];
    float n;

    robot_raw_accel(imu, a);
    n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    return (n > APP_IMU_OFFSET_NORM_MIN_MPS2) && (n < APP_IMU_OFFSET_NORM_MAX_MPS2);
}

static bool robot_is_lying(const app_imu_sample_t *imu)
{
    if (!robot_accel_norm_ok(imu)) {
        return false;
    }
    return fabsf(robot_raw_a_up(imu)) < (APP_IMU_OFFSET_LIE_SIN * APP_IMU_G_MPS2);
}

/** Within π/12 of vertical — operator is holding the robot still and upright. */
static bool robot_is_upright(const app_imu_sample_t *imu)
{
    if (!robot_accel_norm_ok(imu)) {
        return false;
    }
    return fabsf(robot_raw_a_up(imu)) > (APP_IMU_OFFSET_UPRIGHT_COS * APP_IMU_G_MPS2);
}

static void accel_bias_remove_gravity(const float mean[3], float bias[3])
{
    const float n = sqrtf(mean[0] * mean[0] + mean[1] * mean[1] + mean[2] * mean[2]);
    uint32_t i;

    if (n < (0.5f * APP_IMU_G_MPS2)) {
        for (i = 0u; i < 3u; i++) {
            bias[i] = mean[i];
        }
        return;
    }
    {
        const float scale = APP_IMU_G_MPS2 / n;
        for (i = 0u; i < 3u; i++) {
            bias[i] = mean[i] - mean[i] * scale;
        }
    }
}

static void sample_reset(void)
{
    uint32_t i;
    s_sample_n = 0u;
    s_sample_wait_ms = 0u;
    s_last_imu_seq = 0u;
    g_imu_offset_sample_n = 0u;
    for (i = 0u; i < 3u; i++) {
        s_acc_sum[i] = 0.0;
        s_gyro_sum[i] = 0.0;
    }
}

static void sample_add(const app_imu_sample_t *imu)
{
    float accel_bias[3];
    float gyro_bias[3];
    uint32_t i;

    if (!imu->valid || imu->seq == s_last_imu_seq) {
        return;
    }
    s_last_imu_seq = imu->seq;
    app_imu_offset_get(accel_bias, gyro_bias);
    for (i = 0u; i < 3u; i++) {
        s_acc_sum[i] += (double)(imu->accel_mps2[i] + accel_bias[i]);
        s_gyro_sum[i] += (double)(imu->gyro_rads[i] + gyro_bias[i]);
    }
    s_sample_n++;
    g_imu_offset_sample_n = s_sample_n;
}

static void commit_means(void)
{
    float accel_mean[3];
    float gyro_mean[3];
    float accel_bias[3];
    const double n = (s_sample_n > 0u) ? (double)s_sample_n : 1.0;
    uint32_t i;

    for (i = 0u; i < 3u; i++) {
        accel_mean[i] = (float)(s_acc_sum[i] / n);
        gyro_mean[i] = (float)(s_gyro_sum[i] / n);
    }
    accel_bias_remove_gravity(accel_mean, accel_bias);
    app_imu_offset_set_runtime(accel_bias, gyro_mean);
    app_imu_offset_request_fusion_reset();
}

void task_bias(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(APP_BIAS_PERIOD_MS);
    const uint32_t dt_ms = APP_BIAS_PERIOD_MS;
    app_imu_offset_state_t st = APP_IMU_OFFSET_ST_WAIT_IMU;

    s_lie_ms = 0u;
    s_upright_ms = 0u;
    s_settle_ms = 0u;
    g_imu_offset_state = (uint32_t)st;

    for (;;) {
        vTaskDelayUntil(&wake, period);

        app_imu_sample_t imu;
        const bool imu_ok = app_samples_imu_read(&imu) && imu.valid;
        if (imu_ok) {
            g_imu_offset_a_up = robot_raw_a_up(&imu);
        }

        switch (st) {
        case APP_IMU_OFFSET_ST_WAIT_IMU:
            if (imu_ok) {
                st = APP_IMU_OFFSET_ST_DETECT;
                s_lie_ms = 0u;
                s_upright_ms = 0u;
            }
            break;

        case APP_IMU_OFFSET_ST_DETECT:
            if (!imu_ok) {
                break;
            }
            if (robot_is_lying(&imu)) {
                s_lie_ms += dt_ms;
                s_upright_ms = 0u;
                if (s_lie_ms >= APP_IMU_OFFSET_LIE_HOLD_MS) {
                    app_imu_offset_set_owns_motors(true);
                    pulse_start(2u);
                    st = APP_IMU_OFFSET_ST_PULSE_IN;
                }
            } else {
                s_lie_ms = 0u;
                s_upright_ms += dt_ms;
                if (s_upright_ms >= APP_IMU_OFFSET_UPRIGHT_SKIP_MS) {
                    st = APP_IMU_OFFSET_ST_SKIPPED;
                }
            }
            break;

        case APP_IMU_OFFSET_ST_PULSE_IN:
            if (pulse_tick(dt_ms)) {
                s_settle_ms = 0u;
                st = APP_IMU_OFFSET_ST_WAIT_UPRIGHT;
            }
            break;

        case APP_IMU_OFFSET_ST_WAIT_UPRIGHT:
            /* Hold still after the stand-up; do not average while picking the robot up. */
            if (imu_ok && robot_is_upright(&imu)) {
                s_settle_ms = 0u;
                st = APP_IMU_OFFSET_ST_SETTLE;
            }
            break;

        case APP_IMU_OFFSET_ST_SETTLE:
            if (!imu_ok || !robot_is_upright(&imu)) {
                s_settle_ms = 0u;
                st = APP_IMU_OFFSET_ST_WAIT_UPRIGHT;
                break;
            }
            s_settle_ms += dt_ms;
            if (s_settle_ms >= APP_IMU_OFFSET_SETTLE_MS) {
                sample_reset();
                st = APP_IMU_OFFSET_ST_SAMPLE;
            }
            break;

        case APP_IMU_OFFSET_ST_SAMPLE:
            if (!imu_ok || !robot_is_upright(&imu)) {
                break;
            }
            s_sample_wait_ms += dt_ms;
            if (s_sample_wait_ms >= APP_IMU_OFFSET_SAMPLE_PERIOD_MS) {
                s_sample_wait_ms = 0u;
                sample_add(&imu);
            }
            if (s_sample_n >= APP_IMU_OFFSET_SAMPLE_COUNT) {
                commit_means();
                pulse_start(3u);
                st = APP_IMU_OFFSET_ST_PULSE_OUT;
            }
            break;

        case APP_IMU_OFFSET_ST_PULSE_OUT:
            if (pulse_tick(dt_ms)) {
                st = APP_IMU_OFFSET_ST_SAVE;
            }
            break;

        case APP_IMU_OFFSET_ST_SAVE:
            if (app_imu_offset_save_flash()) {
                st = APP_IMU_OFFSET_ST_DONE;
            } else {
                st = APP_IMU_OFFSET_ST_SAVE_FAIL;
            }
            app_imu_offset_set_owns_motors(false);
            break;

        case APP_IMU_OFFSET_ST_SKIPPED:
        case APP_IMU_OFFSET_ST_DONE:
        case APP_IMU_OFFSET_ST_SAVE_FAIL:
            break;

        default:
            break;
        }

        if (app_imu_offset_owns_motors()) {
            s_cal_ms += dt_ms;
            if (s_cal_ms >= APP_IMU_OFFSET_CAL_TIMEOUT_MS) {
                app_imu_offset_set_owns_motors(false);
                st = APP_IMU_OFFSET_ST_SAVE_FAIL;
            }
        } else {
            s_cal_ms = 0u;
        }

        g_imu_offset_state = (uint32_t)st;
    }
}
