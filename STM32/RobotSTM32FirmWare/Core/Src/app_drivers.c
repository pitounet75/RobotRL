/**
 * @file app_drivers.c
 */

#include "app_drivers.h"
#include "app_config.h"
#include "app_halt.h"
#include "app_time_us.h"
#include "dma.h"
#include "fdcan.h"
#include "imu_async.h"
#include "imu_spi_async.h"
#include "odrive_can_async.h"
#include "odrive_can_hal.h"
#include "odrive_can_dma.h"
#include "odrive_torque_mode_startup.h"
#include "wheel_encoder_abz.h"

static bool s_imu_ready;

volatile uint32_t g_app_odrive_rtos_boot_tried;
volatile uint32_t g_app_odrive_rtos_fdcan_ok;

/** If main skipped ODrive boot (old elf) or FDCAN never started, bring up CAN + ODrive once. */
static void app_odrive_boot_if_needed(void)
{
#if defined(DEBUG_DESK_NO_ODRIVE) || (APP_TELEMETRY_BENCH_MODE != 0)
    return;
#else
    g_app_odrive_rtos_boot_tried++;

    const bool left_ok = odrive_can_fdcan_ensure_started(&ODRIVE_CAN_LEFT_HANDLE);
    const bool right_ok = odrive_can_fdcan_ensure_started(&ODRIVE_CAN_RIGHT_HANDLE);
    g_app_odrive_rtos_fdcan_ok = (left_ok && right_ok) ? 1u : 0u;
    if (g_app_odrive_rtos_fdcan_ok == 0u) {
        return;
    }

    if (g_odrive_startup_last_error == (uint32_t)ODRIVE_TORQUE_STARTUP_IN_PROGRESS) {
        (void)odrive_torque_mode_startup(&ODRIVE_CAN_LEFT_HANDLE);
        (void)odrive_torque_mode_startup(&ODRIVE_CAN_RIGHT_HANDLE);
    }
#endif
}

bool app_imu_init(void)
{
    s_imu_ready = imu_async_init();
    return s_imu_ready;
}

bool app_imu_is_ready(void)
{
    return s_imu_ready;
}

bool app_drivers_rtos_init(void)
{
    app_dma_nvic_apply();
    app_time_us_init();
    wheel_encoder_abz_init();

    app_odrive_boot_if_needed();

    if (!odrive_can_dma_init(&ODRIVE_CAN_LEFT_HANDLE)) {
        return false;
    }
    if (!odrive_can_dma_init(&ODRIVE_CAN_RIGHT_HANDLE)) {
        return false;
    }

    if (!odrive_can_async_init(&ODRIVE_CAN_LEFT_HANDLE)) {
        return false;
    }
    if (!odrive_can_async_start()) {
        return false;
    }

    /* IMU hw init runs in task_imu (main probe may already have configured the chip). */
    imu_spi_async_init();
    return true;
}

void app_drivers_irq_enable(void)
{
    odrive_can_hal_rx_irq_enable(&ODRIVE_CAN_LEFT_HANDLE);
    odrive_can_hal_rx_irq_enable(&ODRIVE_CAN_RIGHT_HANDLE);
}
