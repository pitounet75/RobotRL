/**
 * @file odrive_can_dma_example.h
 */
#ifndef ODRIVE_CAN_DMA_EXAMPLE_H
#define ODRIVE_CAN_DMA_EXAMPLE_H

#include "odrive_can_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

void odrive_can_dma_example_on_rx_fifo0(ODriveCanHalHandle *hcan);
void odrive_can_dma_example_init(void);
void odrive_can_dma_example_run_periodic(void);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_DMA_EXAMPLE_H */
