/**
 * @file app_dma_buffers.h
 * @brief DMA-visible buffers for STM32H7 (32-byte aligned, RAM_D2 section).
 */
#ifndef APP_DMA_BUFFERS_H
#define APP_DMA_BUFFERS_H

#include <stdint.h>

#define APP_DMA_BUFFER_ALIGN  32u

#if defined(__GNUC__)
#define APP_DMA_BUFFER_SECTION \
    __attribute__((section(".dma_buffer"), aligned(APP_DMA_BUFFER_ALIGN)))
#else
#define APP_DMA_BUFFER_SECTION
#endif

#endif /* APP_DMA_BUFFERS_H */
