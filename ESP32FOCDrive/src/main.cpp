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

void mainPrintEncLine(uint8_t axis_mask) {
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if (!(axis_mask & (1u << i))) {
      continue;
    }
    if (!axes[i].present) {
      /* An explicit L/R prefix named an axis this binary was not built for
       * (the default, no-prefix axis_mask only ever covers axes that ARE
       * present -- see cmd_parse.h): say so instead of silently printing
       * nothing. */
      Serial.printf("enc %c: FAIL (axis not present in this build)\n", axes[i].name);
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
