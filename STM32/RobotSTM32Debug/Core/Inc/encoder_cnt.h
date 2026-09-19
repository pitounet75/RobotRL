#pragma once

#include <stdint.h>

/** TIM2 (PA0/PA1, 32-bit HW) — left-style wheel on the H743 board. */
extern volatile int64_t g_enc_tim2_cnt;
/** TIM4 (PD12/PD13, 16-bit HW) — right-style wheel. */
extern volatile int64_t g_enc_tim4_cnt;

/** Raw TIM CNT (watch these in CubeIDE if int64 Live Expressions stay at 0). */
extern volatile uint32_t g_enc_tim2_raw;
extern volatile uint32_t g_enc_tim4_raw;
/** GPIOA IDR[1:0] — must toggle when you spin the TIM2 encoder. */
extern volatile uint32_t g_enc_pa01;
/** TIM2 CR1 / SMCR / CCER snapshot (CEN, SMS encoder, CC1E/CC2E). */
extern volatile uint32_t g_enc_tim2_cr1;
extern volatile uint32_t g_enc_tim2_smcr;
extern volatile uint32_t g_enc_tim2_ccer;
/** HAL_TIM_Encoder_Start results (0 = HAL_OK). */
extern volatile uint32_t g_enc_tim2_start;
extern volatile uint32_t g_enc_tim4_start;

void encoder_cnt_init(void);
/** Fold HW CNT into the two int64s. Call often enough that TIM4 cannot
 * wrap more than half its range between calls (~2 turns @ x2 / 32768 CPR). */
void encoder_cnt_poll(void);
