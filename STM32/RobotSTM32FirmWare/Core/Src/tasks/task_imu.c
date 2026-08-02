/**

 * @file task_imu.c

 * @brief 1 kHz IMU read, complementary pitch fusion, publish snapshot.

 */



#include "tasks/tasks.h"



#include "app_config.h"

#include "app_drivers.h"

#include "app_samples.h"

#include "imu_async.h"

#include "imu_fusion.h"

#include "imu_spi_async.h"



#include "FreeRTOS.h"

#include "task.h"



#include <string.h>



static TaskHandle_t s_task;

static imu_sample_t s_raw_sample;

static volatile bool s_raw_valid;

static imu_fusion_state_t s_fusion;

static uint32_t s_pub_seq;



static void imu_read_done(void *user_ctx, const imu_sample_t *sample, bool ok)

{

    (void)user_ctx;

    BaseType_t wake = pdFALSE;



    s_raw_valid = false;

    if (ok && sample != NULL && sample->valid) {

        s_raw_sample = *sample;

        s_raw_valid = true;

    }



    if (s_task != NULL) {

        if (xPortIsInsideInterrupt() != pdFALSE) {

            vTaskNotifyGiveFromISR(s_task, &wake);

            portYIELD_FROM_ISR(wake);

        } else {

            xTaskNotifyGive(s_task);

        }

    }

}



static void publish_fused_sample(const imu_sample_t *raw)

{

    imu_fusion_out_t fused;

    const float dt_default_s = (float)APP_IMU_PERIOD_MS * 1e-3f;



    if (!imu_fusion_update(&s_fusion,

                           raw->data.accel_mps2,

                           raw->data.gyro_rads,

                           raw->t_us,

                           dt_default_s,

                           APP_IMU_COMPLEMENTARY_ALPHA,

                           APP_IMU_PITCH_ACCEL_FORWARD_AXIS,
                           APP_IMU_PITCH_ACCEL_UP_AXIS,
                           APP_IMU_PITCH_GYRO_AXIS,

                           APP_IMU_PITCH_GYRO_SIGN,

                           &fused)) {

        return;

    }



    app_imu_sample_t out = {0};

    out.t_us = raw->t_us;

    out.seq = ++s_pub_seq;

    out.valid = true;

    out.pitch_rad = fused.pitch_rad;

    out.pitch_rate_rads = fused.pitch_rate_rads;

    memcpy(out.accel_mps2, raw->data.accel_mps2, sizeof(out.accel_mps2));

    memcpy(out.gyro_rads, raw->data.gyro_rads, sizeof(out.gyro_rads));

    app_samples_imu_publish(&out);

}



void task_imu(void *argument)

{

    (void)argument;

    s_task = xTaskGetCurrentTaskHandle();

    imu_fusion_reset(&s_fusion);



    TickType_t wake = xTaskGetTickCount();

    const TickType_t period = pdMS_TO_TICKS(APP_IMU_PERIOD_MS);
    TickType_t next_init_retry = wake;



    for (;;) {
        vTaskDelayUntil(&wake, period);



        if (!app_imu_is_ready()) {
            const TickType_t now = xTaskGetTickCount();
            if ((TickType_t)(now - next_init_retry) < (TickType_t)0x80000000UL) {
                (void)app_imu_init();
                next_init_retry = now + pdMS_TO_TICKS(APP_IMU_INIT_RETRY_MS);
            }

            continue;

        }



        if (imu_async_busy()) {

            continue;

        }



        s_raw_valid = false;

        if (!imu_async_start_read(imu_read_done, NULL)) {
            continue;
        }

        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5u));
        if (imu_spi_async_busy()) {
            imu_spi_async_abort();
            g_imu_read_fail_count++;
            continue;
        }

        if (s_raw_valid) {
            g_imu_read_ok_count++;
            publish_fused_sample(&s_raw_sample);
        } else {
            g_imu_read_fail_count++;
        }

    }

}

