/** ESP32FOCDrive — step 4: isochronous FOC task on core 0, CLI on core 1. */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "axis.h"
#include "board.h"
#include "cli.h"
#include "config.h"
#include "foc_task.h"
#include "net.h"

/* Temporary seam, replaced by drive_api.h in task 7. */
void mainSetOpenloop(uint8_t axis_mask, float rad_s) {
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if (!(axis_mask & (1u << i)) || !axes[i].present) {
      continue;
    }
    Axis &ax = axes[i];
    ax.motor.controller = MotionControlType::velocity_openloop;
    ax.motor.target = rad_s;
    /* Settle the mode (and let the task observe it) before arming: mirrors
     * the disarm ordering below, keeping mode and power state changes from
     * racing the task on the other core. */
    axisSetMode(ax, Mode::Openloop);
    axisArm(ax);
  }
}

void mainIdle(uint8_t axis_mask) {
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if (!(axis_mask & (1u << i)) || !axes[i].present) {
      continue;
    }
    Axis &ax = axes[i];
    ax.motor.target = 0.0f;
    /* Mode -> Off and synced BEFORE disarm: otherwise the task could still
     * be mid-loopFOC()/move() when M_EN drops, and with M_EN shared between
     * both gate drivers that is not harmless. */
    axisSetMode(ax, Mode::Off);
    axisDisarm(ax);
  }
}

float mainVoltageLimit(uint8_t axis_mask) {
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if ((axis_mask & (1u << i)) && axes[i].present) {
      return axes[i].voltage_limit;
    }
  }
  return FOC_VOLTAGE_LIMIT;
}

void mainSetVoltageLimit(uint8_t axis_mask, float v) {
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if (!(axis_mask & (1u << i)) || !axes[i].present) {
      continue;
    }
    axes[i].voltage_limit = v;
    axisApplyLimits(axes[i]);
  }
}

void mainPrintEncLine(uint8_t axis_mask) {
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if (!(axis_mask & (1u << i)) || !axes[i].present) {
      continue;
    }
    Axis &ax = axes[i];
    Serial.printf("enc %c cnt=%lld idx=%d zn=%lu A=%d B=%d Z=%d ang=%.4f\n", ax.name,
                  (long long)ax.encoder.count(), (int)ax.encoder.indexFound(),
                  (unsigned long)ax.encoder.zEdges(), (int)digitalRead(kAxisEnc[ax.idx][0]),
                  (int)digitalRead(kAxisEnc[ax.idx][1]), (int)digitalRead(kAxisEnc[ax.idx][2]),
                  (double)ax.encoder.getAngle());
  }
}

void cliPrintStatus() {
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if (!axes[i].present) {
      continue;
    }
    Axis &ax = axes[i];
    Serial.printf("axis=%c tgt=%.2f rad/s Uq=%.2f V limit=%.2f V armed=%d cal=%d\n", ax.name,
                  (double)ax.motor.target, (double)ax.motor.voltage.q, (double)ax.voltage_limit,
                  (int)ax.armed, (int)ax.calibrated);
  }
  const FocMetrics m = focGetMetrics();
  Serial.printf("MEN=%d hz=%lu dt=%lu us dtmax=%lu us late=%lu loops=%llu\n",
                (int)boardMotorPowered(), (unsigned long)m.hz, (unsigned long)m.dt_us,
                (unsigned long)m.dt_max_us, (unsigned long)m.late, (unsigned long long)m.loops);
  netPrintInfo();
}

void setup() {
  boardInit();
  Serial.begin(115200);
  delay(200);

  axisInitAll();
  /* No auto-start: motors stay idle and M_EN stays low until a CLI command
   * (ol) arms an axis. */

  cliInit();
  focTaskStart();
  netSetup();

  Serial.printf("ESP32FOCDrive mask=0x%x Vbus=%.1f Ulim=%.1f hz=%lu MEN=%d\n",
                (unsigned)FOC_AXIS_MASK, (double)FOC_VBUS, (double)FOC_VOLTAGE_LIMIT,
                (unsigned long)focHz(), (int)boardMotorPowered());
}

void loop() {
  netLoop();
  if (netOtaActive()) {
    /* A tight handle() loop starves IDLE1 and the WDT aborts mid-flash. */
    vTaskDelay(1);
    return;
  }
  /* No encoder.update()/motor.move() here: the core-0 FOC task does both,
   * in every mode including Off, so the PCNT count is re-read often enough
   * to unwrap before it hits the +-8192 hardware limit. */
  cliPoll();
  vTaskDelay(1);
}
