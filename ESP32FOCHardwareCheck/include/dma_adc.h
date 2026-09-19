#pragma once

#include <stdint.h>

/** ADC1 oneshot, captured on MCPWM timer0 TEZ (every 2nd PWM). */
namespace DmaAdc {

bool begin(int pin_a, int pin_b);
bool ok();
void captureFromIsr();
void voltages(float *va, float *vb);
float voltage(int pin);
uint32_t samples();
uint32_t stalls();

}  // namespace DmaAdc
