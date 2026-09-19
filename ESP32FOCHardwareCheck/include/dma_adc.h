#pragma once

#include <stdint.h>

/** ESP32 ADC1 digital controller + I2S DMA. Never analogRead. */
namespace DmaAdc {

bool begin(int pin_a, int pin_b);
bool ok();
/** Latest voltages on the two sense pins (volts), begin() order. */
void voltages(float *va, float *vb);
/** Voltage on a GPIO that was passed to begin(), else 0. */
float voltage(int pin);
uint32_t samples();

}  // namespace DmaAdc
