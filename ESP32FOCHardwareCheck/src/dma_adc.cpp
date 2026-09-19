#include "dma_adc.h"

#include <driver/adc.h>
#include <esp_adc_cal.h>

namespace DmaAdc {
namespace {

constexpr uint32_t kVrefMv = 1100;

int pin_a_ = -1;
int pin_b_ = -1;
adc1_channel_t ch_a_ = ADC1_CHANNEL_0;
adc1_channel_t ch_b_ = ADC1_CHANNEL_0;
volatile uint16_t raw_a_ = 0;
volatile uint16_t raw_b_ = 0;
volatile uint32_t samples_ = 0;
bool ok_ = false;
esp_adc_cal_characteristics_t cal_{};

adc1_channel_t gpioToAdc1(int gpio) {
  switch (gpio) {
    case 36:
      return ADC1_CHANNEL_0;
    case 37:
      return ADC1_CHANNEL_1;
    case 38:
      return ADC1_CHANNEL_2;
    case 39:
      return ADC1_CHANNEL_3;
    case 32:
      return ADC1_CHANNEL_4;
    case 33:
      return ADC1_CHANNEL_5;
    case 34:
      return ADC1_CHANNEL_6;
    case 35:
      return ADC1_CHANNEL_7;
    default:
      return static_cast<adc1_channel_t>(-1);
  }
}

float rawToVolt(uint16_t raw) {
  const uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &cal_);
  return (float)mv * 0.001f;
}

}  // namespace

void captureFromIsr() {
  const int a = adc1_get_raw(ch_a_);
  const int b = adc1_get_raw(ch_b_);
  if (a < 0 || b < 0) {
    return;
  }
  raw_a_ = (uint16_t)a;
  raw_b_ = (uint16_t)b;
  samples_ += 2;
}

bool begin(int pin_a, int pin_b) {
  if (ok_) {
    return true;
  }
  const adc1_channel_t cha = gpioToAdc1(pin_a);
  const adc1_channel_t chb = gpioToAdc1(pin_b);
  if ((int)cha < 0 || (int)chb < 0) {
    return false;
  }
  pin_a_ = pin_a;
  pin_b_ = pin_b;
  ch_a_ = cha;
  ch_b_ = chb;

  adc_power_acquire();
  if (adc1_config_width(ADC_WIDTH_BIT_12) != ESP_OK) {
    return false;
  }
  if (adc1_config_channel_atten(ch_a_, ADC_ATTEN_DB_12) != ESP_OK) {
    return false;
  }
  if (adc1_config_channel_atten(ch_b_, ADC_ATTEN_DB_12) != ESP_OK) {
    return false;
  }
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, kVrefMv, &cal_);

  for (int i = 0; i < 16; ++i) {
    captureFromIsr();
  }
  ok_ = samples_ > 0;
  return ok_;
}

bool ok() { return ok_; }

void voltages(float *va, float *vb) {
  *va = rawToVolt(raw_a_);
  *vb = rawToVolt(raw_b_);
}

float voltage(int pin) {
  if (pin == pin_a_) {
    return rawToVolt(raw_a_);
  }
  if (pin == pin_b_) {
    return rawToVolt(raw_b_);
  }
  return 0.0f;
}

uint32_t samples() { return samples_; }

uint32_t stalls() { return 0; }

}  // namespace DmaAdc
