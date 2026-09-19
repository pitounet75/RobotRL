#pragma once

#include <SimpleFOC.h>

#include "mt6835.h"

/** SimpleFOC Sensor over the RobotRL 24-clock MT6835 driver. */
class Mt6835Sensor : public Sensor {
 public:
  explicit Mt6835Sensor(int8_t cs_pin);

  void init();
  float getSensorAngle() override;

  Mt6835 &chip() { return chip_; }

 private:
  Mt6835 chip_;
};
