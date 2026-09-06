#include "mt6835_sensor.h"

#include "config.h"

Mt6835Sensor::Mt6835Sensor(int8_t cs_pin) : chip_(cs_pin) {}

void Mt6835Sensor::init() {
  chip_.begin();
  Sensor::init();
}

float Mt6835Sensor::getSensorAngle() {
  const uint32_t raw = chip_.readAngleRaw() & (MT6835_CPR - 1u);
  return raw * (_2PI / (float)MT6835_CPR);
}
