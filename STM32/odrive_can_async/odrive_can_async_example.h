/**
 * @file odrive_can_async_example.h
 * @brief Declarations for `odrive_can_async_example.c` (optional reference build).
 */
#ifndef ODRIVE_CAN_ASYNC_EXAMPLE_H
#define ODRIVE_CAN_ASYNC_EXAMPLE_H

#if defined(STM32F4xx)
#include "stm32f4xx_hal_can.h"
#elif defined(STM32F7xx)
#include "stm32f7xx_hal_can.h"
#elif defined(STM32L4xx)
#include "stm32l4xx_hal_can.h"
#else
#include "stm32f4xx_hal_can.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

void odrive_can_async_example_on_rx_fifo0(CAN_HandleTypeDef *hcan);
void odrive_can_async_example_init(void);
void odrive_can_async_example_run_periodic(void);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_ASYNC_EXAMPLE_H */
