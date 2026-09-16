/** ESP32FOCDrive — step 2: command parser, minimal CLI, no auto-start. */

#include <Arduino.h>
#include <SimpleFOC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "board.h"
#include "cli.h"
#include "config.h"
#include "net.h"
#include "pcnt_encoder.h"

namespace {
constexpr int kAxis = (FOC_AXIS_MASK & 0b01) ? 0 : 1;

BLDCMotor motor(FOC_POLE_PAIRS);
BLDCDriver3PWM driver(kAxisPwm[kAxis][0], kAxisPwm[kAxis][1], kAxisPwm[kAxis][2]);
PcntEncoder encoder(kAxisEnc[kAxis][0], kAxisEnc[kAxis][1], kAxisEnc[kAxis][2], ENC_PPR,
                     pcnt_unit_t(kAxis), 6.0f * FOC_VEL_LIMIT, ENC_VEL_MIN_DT);
}  // namespace

/* Temporary seam, replaced by drive_api.h in task 7. */
void mainSetOpenloop(uint8_t axis_mask, float rad_s) {
  if (!(axis_mask & (1u << kAxis))) {
    return;
  }
  motor.target = rad_s;
  /* enable() always does setPwm(0,0,0) and resets the PIDs: re-arming an
   * already-armed motor would stall the shaft on every command. */
  if (!motor.enabled) {
    motor.enable();
    boardMotorPowerRef(+1);
  }
}

void mainIdle(uint8_t axis_mask) {
  if (!(axis_mask & (1u << kAxis))) {
    return;
  }
  motor.target = 0;
  const bool was_enabled = motor.enabled;
  motor.disable();
  if (was_enabled) {
    boardMotorPowerRef(-1);
  }
}

float mainVoltageLimit() { return motor.voltage_limit; }

void mainSetVoltageLimit(float v) { motor.voltage_limit = v; }

void mainPrintEncLine() {
  Serial.printf("enc %c cnt=%lld idx=%d zn=%lu A=%d B=%d Z=%d ang=%.4f\n", kAxisName[kAxis],
                (long long)encoder.count(), (int)encoder.indexFound(),
                (unsigned long)encoder.zEdges(), (int)digitalRead(kAxisEnc[kAxis][0]),
                (int)digitalRead(kAxisEnc[kAxis][1]), (int)digitalRead(kAxisEnc[kAxis][2]),
                (double)encoder.getAngle());
}

void cliPrintStatus() {
  Serial.printf("axis=%c tgt=%.2f rad/s Uq=%.2f V limit=%.2f V MEN=%d\n",
                kAxisName[kAxis], (double)motor.target, (double)motor.voltage.q,
                (double)motor.voltage_limit, (int)boardMotorPowered());
  netPrintInfo();
}

void setup() {
  boardInit();
  Serial.begin(115200);
  delay(200);

  /* This binary drives only kAxis, but env:dual marks both axes present and
   * boardInit() only parks absent axes; M_EN is shared between both gate
   * drivers, so the other axis' floating PWM inputs must be parked here
   * before M_EN goes high. */
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if (i == kAxis) {
      continue;
    }
    for (int p = 0; p < 3; ++p) {
      pinMode(kAxisPwm[i][p], OUTPUT);
      digitalWrite(kAxisPwm[i][p], LOW);
    }
  }

  encoder.init();

  driver.voltage_power_supply = FOC_VBUS;
  driver.voltage_limit = FOC_VOLTAGE_LIMIT;
  driver.pwm_frequency = FOC_PWM_HZ;
  driver.init();

  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);
  motor.voltage_limit = FOC_VOLTAGE_LIMIT;
  motor.foc_modulation = FOC_MODULATION;
  motor.controller = MotionControlType::velocity_openloop;
  motor.init();
  /* No auto-start: the motor stays idle and M_EN stays low until a CLI
   * command (ol) arms it. */

  cliInit();
  netSetup();

  Serial.printf("ESP32FOCDrive axis=%c Vbus=%.1f Ulim=%.1f MEN=%d\n",
                kAxisName[kAxis], (double)FOC_VBUS, (double)FOC_VOLTAGE_LIMIT,
                (int)boardMotorPowered());
}

void loop() {
  netLoop();
  if (netOtaActive()) {
    /* A tight handle() loop starves IDLE1 and the WDT aborts mid-flash. */
    vTaskDelay(1);
    return;
  }
  encoder.update();
  motor.move();
  cliPoll();
}
