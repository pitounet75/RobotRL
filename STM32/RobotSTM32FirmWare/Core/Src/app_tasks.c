/**
 * @file app_tasks.c
 * @brief Create application FreeRTOS threads.
 */

#include "app_tasks.h"
#include "tasks/tasks.h"

#include "cmsis_os.h"

static const osThreadAttr_t attr_control = {
    .name = "control",
    .stack_size = 512u * 4u,
    .priority = osPriorityRealtime,
};

static const osThreadAttr_t attr_motor_tx = {
    .name = "motor_tx",
    .stack_size = 256u * 4u,
    .priority = osPriorityHigh,
};

static const osThreadAttr_t attr_imu = {
    .name = "imu",
    .stack_size = 384u * 4u,
    .priority = osPriorityHigh,
};

static const osThreadAttr_t attr_encoder = {
    .name = "encoder",
    .stack_size = 256u * 4u,
    .priority = osPriorityHigh,
};

static const osThreadAttr_t attr_odrive = {
    .name = "odrive",
    .stack_size = 256u * 4u,
    .priority = osPriorityAboveNormal,
};

static const osThreadAttr_t attr_bias = {
    .name = "bias",
    .stack_size = 256u * 4u,
    .priority = osPriorityAboveNormal,
};

static const osThreadAttr_t attr_watchdog = {
    .name = "watchdog",
    .stack_size = 192u * 4u,
    .priority = osPriorityAboveNormal,
};

static const osThreadAttr_t attr_jetson = {
    .name = "jetson",
    .stack_size = 256u * 4u,
    .priority = osPriorityNormal,
};

static const osThreadAttr_t attr_telemetry = {
    .name = "telemetry",
    .stack_size = 256u * 4u,
    .priority = osPriorityLow,
};

void app_tasks_create(void)
{
    (void)osThreadNew(task_control, NULL, &attr_control);
    (void)osThreadNew(task_motor_tx, NULL, &attr_motor_tx);
    (void)osThreadNew(task_imu, NULL, &attr_imu);
    (void)osThreadNew(task_encoder, NULL, &attr_encoder);
    (void)osThreadNew(task_odrive, NULL, &attr_odrive);
    (void)osThreadNew(task_bias, NULL, &attr_bias);
    (void)osThreadNew(task_watchdog, NULL, &attr_watchdog);
    (void)osThreadNew(task_jetson, NULL, &attr_jetson);
    (void)osThreadNew(task_telemetry, NULL, &attr_telemetry);
}
