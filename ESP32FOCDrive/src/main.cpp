/** ESP32FOCDrive — step 1: bare SimpleFOC open-loop, one axis, loop() driven. */

#include <Arduino.h>
#include <SimpleFOC.h>

#include "board.h"
#include "config.h"

namespace {
constexpr int kAxis = (FOC_AXIS_MASK & 0b01) ? 0 : 1;

BLDCMotor motor(FOC_POLE_PAIRS);
BLDCDriver3PWM driver(kAxisPwm[kAxis][0], kAxisPwm[kAxis][1], kAxisPwm[kAxis][2]);
}  // namespace

void setup() {
  boardInit();
  Serial.begin(115200);
  delay(200);

  driver.voltage_power_supply = FOC_VBUS;
  driver.voltage_limit = FOC_VOLTAGE_LIMIT;
  driver.pwm_frequency = FOC_PWM_HZ;
  driver.init();

  motor.linkDriver(&driver);
  motor.voltage_limit = FOC_VOLTAGE_LIMIT;
  motor.foc_modulation = FOC_MODULATION;
  motor.controller = MotionControlType::velocity_openloop;
  motor.target = 3.0f;
  motor.init();
  motor.enable();
  boardMotorPowerRef(+1);

  Serial.printf("ESP32FOCDrive step1 axis=%c Vbus=%.1f Ulim=%.1f MEN=%d\n",
                kAxisName[kAxis], (double)FOC_VBUS, (double)FOC_VOLTAGE_LIMIT,
                (int)boardMotorPowered());
}

void loop() { motor.move(); }
