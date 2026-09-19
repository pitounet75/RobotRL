#include "mcpwm_driver.h"

#include <driver/mcpwm.h>
#include <soc/mcpwm_struct.h>

#include "dma_adc.h"

namespace {

bool adc_isr_on_ = false;

void mcpwmIsr(void *) {
  const uint32_t st = MCPWM0.int_st.val;
  MCPWM0.int_clr.val = st;
  /* timer0_tep = bit 6 — peak of up-down count = PWM center. */
  if (((st >> 6) & 1u) == 0) {
    return;
  }
  static uint8_t div;
  div = (uint8_t)(div + 1u);
  if ((div & 1u) != 0) {
    return;
  }
  DmaAdc::captureFromIsr();
}

}  // namespace

void mcpwmAttachAdcIsr() {
  if (!adc_isr_on_) {
    mcpwm_isr_register(MCPWM_UNIT_0, mcpwmIsr, nullptr, 0, nullptr);
    adc_isr_on_ = true;
  }
  MCPWM0.int_ena.timer0_tep_int_ena = 1;
}

void mcpwmPauseAdcIsr() {
  if (adc_isr_on_) {
    MCPWM0.int_ena.timer0_tep_int_ena = 0;
  }
}

void mcpwmResumeAdcIsr() {
  if (adc_isr_on_) {
    MCPWM0.int_ena.timer0_tep_int_ena = 1;
  }
}
