#pragma once

#include <SimpleFOC.h>

/** Two-shunt inline sense fed by DmaAdc, not analogRead. */
class DmaInlineCurrentSense : public CurrentSense {
 public:
  DmaInlineCurrentSense(float shunt_ohm, float amp_gain, int pin_a, int pin_b);

  int init() override;
  PhaseCurrent_s getPhaseCurrents() override;
  int driverAlign(float align_voltage) override;

  void flipGains();
  void calibrateOffsets();
  float offsetA() const { return offset_a_; }
  float offsetB() const { return offset_b_; }
  float gainA() const { return gain_a_; }
  float gainB() const { return gain_b_; }

 private:

  int pin_a_;
  int pin_b_;
  float gain_a_;
  float gain_b_;
  float offset_a_;
  float offset_b_;
};
