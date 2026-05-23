/**
 * @file odrive_can_dma_example.h
 * @brief Declarations for `odrive_can_dma_example.c` (optional build).
 */
#ifndef ODRIVE_CAN_DMA_EXAMPLE_H
#define ODRIVE_CAN_DMA_EXAMPLE_H

#include "odrive_can_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

void odrive_can_dma_example_on_rx_fifo0(CAN_HandleTypeDef *hcan);
void odrive_can_dma_example_init(void);
void odrive_can_dma_example_run_periodic(void);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_DMA_EXAMPLE_H */
