#include "dma_adc.h"

#include <Arduino.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/soc_caps.h>
#include <string.h>

#include "hal/adc_types.h"

namespace DmaAdc {
namespace {

constexpr uint32_t kSampleHz = 40000;
constexpr uint32_t kBufBytes = 256;
constexpr uint32_t kVrefMv = 1100;

int pin_a_ = -1;
int pin_b_ = -1;
uint8_t ch_a_ = 0;
uint8_t ch_b_ = 0;
volatile uint16_t raw_a_ = 0;
volatile uint16_t raw_b_ = 0;
volatile uint32_t samples_ = 0;
bool ok_ = false;
esp_adc_cal_characteristics_t cal_;

int gpioToAdc1(int gpio) {
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
      return -1;
  }
}

float rawToVolt(uint16_t raw) {
  const uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &cal_);
  return (float)mv * 0.001f;
}

void drainTask(void *) {
  uint8_t buf[kBufBytes];
  for (;;) {
    uint32_t n = 0;
    const esp_err_t err = adc_digi_read_bytes(buf, sizeof(buf), &n, 20);
    if (err != ESP_OK || n < sizeof(adc_digi_output_data_t)) {
      continue;
    }
    const uint32_t count = n / sizeof(adc_digi_output_data_t);
    const auto *out = reinterpret_cast<const adc_digi_output_data_t *>(buf);
    uint16_t last_a = raw_a_;
    uint16_t last_b = raw_b_;
    uint32_t got = 0;
    for (uint32_t i = 0; i < count; ++i) {
      const uint16_t ch = out[i].type1.channel;
      const uint16_t data = out[i].type1.data;
      if (ch == ch_a_) {
        last_a = data;
        ++got;
      } else if (ch == ch_b_) {
        last_b = data;
        ++got;
      }
    }
    if (got > 0) {
      raw_a_ = last_a;
      raw_b_ = last_b;
      samples_ += got;
    }
  }
}

}  // namespace

bool begin(int pin_a, int pin_b) {
  if (ok_) {
    return true;
  }
  const int cha = gpioToAdc1(pin_a);
  const int chb = gpioToAdc1(pin_b);
  if (cha < 0 || chb < 0) {
    ok_ = false;
    return false;
  }
  pin_a_ = pin_a;
  pin_b_ = pin_b;
  ch_a_ = (uint8_t)cha;
  ch_b_ = (uint8_t)chb;

  /* GPIO36/39 glitch when SAR power-gates. Keep ADC powered. */
  adc_power_acquire();

  adc_digi_init_config_t init_cfg{};
  init_cfg.max_store_buf_size = 1024;
  init_cfg.conv_num_each_intr = kBufBytes;
  init_cfg.adc1_chan_mask = (1u << ch_a_) | (1u << ch_b_);
  init_cfg.adc2_chan_mask = 0;
  if (adc_digi_initialize(&init_cfg) != ESP_OK) {
    ok_ = false;
    return false;
  }

  adc_digi_pattern_config_t pattern[2]{};
  /* unit must be ADC_NUM_1 (0). ADC_UNIT_1 is 1 and writes the ADC2 table. */
  pattern[0].atten = ADC_ATTEN_DB_12;
  pattern[0].channel = ch_a_;
  pattern[0].unit = 0;
  pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
  pattern[1].atten = ADC_ATTEN_DB_12;
  pattern[1].channel = ch_b_;
  pattern[1].unit = 0;
  pattern[1].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

  adc_digi_configuration_t dig{};
  dig.conv_limit_en = true;
  dig.conv_limit_num = 250;
  dig.pattern_num = 2;
  dig.adc_pattern = pattern;
  dig.sample_freq_hz = kSampleHz;
  dig.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  dig.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;
  if (adc_digi_controller_configure(&dig) != ESP_OK) {
    adc_digi_deinitialize();
    ok_ = false;
    return false;
  }
  if (adc_digi_start() != ESP_OK) {
    adc_digi_deinitialize();
    ok_ = false;
    return false;
  }

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, kVrefMv, &cal_);

  xTaskCreatePinnedToCore(drainTask, "adc", 3072, nullptr, 10, nullptr, 0);
  delay(20);
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

}  // namespace DmaAdc
