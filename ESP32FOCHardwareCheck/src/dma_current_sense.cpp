#include "dma_current_sense.h"

#include <Arduino.h>

#include "dma_adc.h"

namespace {

PhaseCurrent_s averageCurrents(DmaInlineCurrentSense *cs, int n) {
  PhaseCurrent_s acc{};
  acc.a = 0.0f;
  acc.b = 0.0f;
  acc.c = 0.0f;
  for (int i = 0; i < n; ++i) {
    const PhaseCurrent_s c = cs->getPhaseCurrents();
    acc.a += c.a;
    acc.b += c.b;
    delay(2);
  }
  acc.a /= (float)n;
  acc.b /= (float)n;
  return acc;
}

}  // namespace

DmaInlineCurrentSense::DmaInlineCurrentSense(float shunt_ohm, float amp_gain, int pin_a,
                                             int pin_b)
    : pin_a_(pin_a),
      pin_b_(pin_b),
      gain_a_(1.0f / (shunt_ohm * amp_gain)),
      gain_b_(1.0f / (shunt_ohm * amp_gain)),
      offset_a_(0.0f),
      offset_b_(0.0f) {}

void DmaInlineCurrentSense::calibrateOffsets() {
  offset_a_ = 0.0f;
  offset_b_ = 0.0f;
  constexpr int kRounds = 256;
  for (int i = 0; i < kRounds; ++i) {
    offset_a_ += DmaAdc::voltage(pin_a_);
    offset_b_ += DmaAdc::voltage(pin_b_);
    delay(1);
  }
  offset_a_ /= (float)kRounds;
  offset_b_ /= (float)kRounds;
}

int DmaInlineCurrentSense::init() {
  if (!DmaAdc::begin(pin_a_, pin_b_)) {
    initialized = false;
    return 0;
  }
  calibrateOffsets();
  skip_align = false;
  initialized = true;
  return 1;
}

PhaseCurrent_s DmaInlineCurrentSense::getPhaseCurrents() {
  PhaseCurrent_s cur{};
  cur.a = (DmaAdc::voltage(pin_a_) - offset_a_) * gain_a_;
  cur.b = (DmaAdc::voltage(pin_b_) - offset_b_) * gain_b_;
  cur.c = 0.0f;
  return cur;
}

int DmaInlineCurrentSense::driverAlign(float voltage) {
  if (skip_align) {
    return 1;
  }
  if (driver == nullptr || !initialized) {
    return 0;
  }

  int exit_flag = 1;
  constexpr float kMinA = 0.05f;

  driver->setPwm(voltage, 0.0f, 0.0f);
  delay(250);
  PhaseCurrent_s c = averageCurrents(this, 40);
  driver->setPwm(0.0f, 0.0f, 0.0f);
  delay(50);

  const float ab = (c.b != 0.0f) ? fabsf(c.a / c.b) : 100.0f;
  if (fabsf(c.a) < kMinA && fabsf(c.b) < kMinA) {
    Serial.println("cs: align A too weak — skip (csflip if Iq fights)");
    skip_align = true;
    return 1;
  }
  if (ab > 1.5f) {
    gain_a_ *= (c.a >= 0.0f) ? 1.0f : -1.0f;
  } else if (ab < 0.7f) {
    const int tmp_pin = pin_a_;
    pin_a_ = pin_b_;
    pin_b_ = tmp_pin;
    const float tmp_off = offset_a_;
    offset_a_ = offset_b_;
    offset_b_ = tmp_off;
    gain_a_ *= (c.b >= 0.0f) ? 1.0f : -1.0f;
    exit_flag = 2;
  } else {
    gain_a_ *= (c.a >= 0.0f) ? 1.0f : -1.0f;
  }

  driver->setPwm(0.0f, voltage, 0.0f);
  delay(250);
  c = averageCurrents(this, 40);
  driver->setPwm(0.0f, 0.0f, 0.0f);
  delay(50);

  const float ba = (c.a != 0.0f) ? fabsf(c.b / c.a) : 100.0f;
  if (fabsf(c.a) < kMinA && fabsf(c.b) < kMinA) {
    Serial.println("cs: align B too weak — skip (csflip if Iq fights)");
    skip_align = true;
    return exit_flag;
  }
  if (ba > 1.5f) {
    gain_b_ *= (c.b >= 0.0f) ? 1.0f : -1.0f;
  } else if (ba < 0.7f) {
    const int tmp_pin = pin_b_;
    pin_b_ = pin_a_;
    pin_a_ = tmp_pin;
    const float tmp_off = offset_b_;
    offset_b_ = offset_a_;
    offset_a_ = tmp_off;
    gain_b_ *= (c.a >= 0.0f) ? 1.0f : -1.0f;
    exit_flag = 2;
  } else {
    gain_b_ *= (c.b >= 0.0f) ? 1.0f : -1.0f;
  }

  if (gain_a_ < 0.0f || gain_b_ < 0.0f) {
    exit_flag += 2;
  }
  Serial.printf("cs: align=%d  gainA=%.2f gainB=%.2f  pinA=%d pinB=%d\n", exit_flag,
                (double)gain_a_, (double)gain_b_, pin_a_, pin_b_);
  return exit_flag;
}

void DmaInlineCurrentSense::flipGains() {
  gain_a_ = -gain_a_;
  gain_b_ = -gain_b_;
}
