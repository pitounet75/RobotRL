/**

 * @file app_drivers.h

 * @brief RTOS-era driver bring-up (after osKernelInitialize).

 */

#ifndef APP_DRIVERS_H

#define APP_DRIVERS_H



#include <stdbool.h>
#include <stdint.h>



bool app_drivers_rtos_init(void);

bool app_imu_init(void);

bool app_imu_is_ready(void);

/** 1 after RTOS tried odrive_can_fdcan_ensure_started; 0 if FDCAN start failed. */
extern volatile uint32_t g_app_odrive_rtos_boot_tried;
extern volatile uint32_t g_app_odrive_rtos_fdcan_ok;

#endif /* APP_DRIVERS_H */

