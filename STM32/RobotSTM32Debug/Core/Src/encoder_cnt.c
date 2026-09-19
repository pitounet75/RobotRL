#include "encoder_cnt.h"

#include "tim.h"

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t mask;
  uint8_t bits;
  uint32_t last_raw;
} enc_ch_t;

volatile int64_t g_enc_tim2_cnt;
volatile int64_t g_enc_tim4_cnt;
volatile uint32_t g_enc_tim2_raw;
volatile uint32_t g_enc_tim4_raw;
volatile uint32_t g_enc_pa01;
volatile uint32_t g_enc_tim2_cr1;
volatile uint32_t g_enc_tim2_smcr;
volatile uint32_t g_enc_tim2_ccer;
volatile uint32_t g_enc_tim2_start;
volatile uint32_t g_enc_tim4_start;

static enc_ch_t s_tim2;
static enc_ch_t s_tim4;
static uint8_t s_ready;

static int32_t signed_delta(uint32_t raw, uint32_t last, uint8_t bits) {
  const uint32_t diff = raw - last;
  if (bits == 16u) {
    return (int32_t)(int16_t)(uint16_t)diff;
  }
  return (int32_t)diff;
}

static int64_t fold(enc_ch_t *ch, int64_t accum) {
  if (ch->htim == NULL || ch->htim->Instance == NULL) {
    return accum;
  }
  const uint32_t raw = ((uint32_t)__HAL_TIM_GET_COUNTER(ch->htim)) & ch->mask;
  accum += (int64_t)signed_delta(raw, ch->last_raw, ch->bits);
  ch->last_raw = raw;
  return accum;
}

static void force_encoder_run(TIM_HandleTypeDef *htim) {
  TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
  TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
  __HAL_TIM_ENABLE(htim);
}

static void tim2_gpio_af(void) {
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &gpio);
}

void encoder_cnt_init(void) {
  /* H743: PA0/PA1 analog switches default CLOSED (ADC VINP). That path is
   * TIM2-only — TIM4 on PD12/PD13 has no such switch. Open them so CH1/CH2
   * see the pad as digital encoder inputs. */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_OPEN);
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA1, SYSCFG_SWITCH_PA1_OPEN);
  tim2_gpio_af();

  g_enc_tim2_start = (uint32_t)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  g_enc_tim4_start = (uint32_t)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  if (g_enc_tim2_start != (uint32_t)HAL_OK) {
    force_encoder_run(&htim2);
  }
  if (g_enc_tim4_start != (uint32_t)HAL_OK) {
    force_encoder_run(&htim4);
  }

  s_tim2.htim = &htim2;
  s_tim2.mask = 0xFFFFFFFFu;
  s_tim2.bits = 32u;
  s_tim4.htim = &htim4;
  s_tim4.mask = 0x0000FFFFu;
  s_tim4.bits = 16u;

  g_enc_tim2_cnt = 0;
  g_enc_tim4_cnt = 0;
  s_tim2.last_raw = ((uint32_t)__HAL_TIM_GET_COUNTER(&htim2)) & s_tim2.mask;
  s_tim4.last_raw = ((uint32_t)__HAL_TIM_GET_COUNTER(&htim4)) & s_tim4.mask;
  s_ready = 1u;
}

void encoder_cnt_poll(void) {
  if (s_ready == 0u) {
    encoder_cnt_init();
    if (s_ready == 0u) {
      return;
    }
  }
  g_enc_tim2_cnt = fold(&s_tim2, g_enc_tim2_cnt);
  g_enc_tim4_cnt = fold(&s_tim4, g_enc_tim4_cnt);
  g_enc_tim2_raw = (uint32_t)TIM2->CNT;
  g_enc_tim4_raw = (uint32_t)TIM4->CNT & 0xFFFFu;
  g_enc_pa01 = (uint32_t)(GPIOA->IDR & 0x3u);
  g_enc_tim2_cr1 = TIM2->CR1;
  g_enc_tim2_smcr = TIM2->SMCR;
  g_enc_tim2_ccer = TIM2->CCER;
}
