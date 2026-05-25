/**
 * @file imu_hal_platform.h
 * @brief Platform include for STM32 HAL used by IMU drivers.
 *
 * Include your MCU HAL before using init_spi/init_i2c. For STM32H7:
 *   #include "stm32h7xx_hal.h"
 * Adjust for your family (stm32f4xx_hal.h, etc.) in your project or here.
 */
#ifndef IMU_HAL_PLATFORM_H
#define IMU_HAL_PLATFORM_H

#include <stdint.h>

#ifdef STM32H7xx
#include "stm32h7xx_hal.h"
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
#elif defined(STM32F7xx)
#include "stm32f7xx_hal.h"
#else
/* Forward-declare when HAL not detected; provide your HAL include in project. */
typedef struct __SPI_HandleTypeDef SPI_HandleTypeDef;
typedef struct __I2C_HandleTypeDef I2C_HandleTypeDef;
typedef struct __GPIO_TypeDef     GPIO_TypeDef;
#endif

#ifndef IMU_HAL_TIMEOUT_MS
#define IMU_HAL_TIMEOUT_MS  10
#endif

#endif /* IMU_HAL_PLATFORM_H */
