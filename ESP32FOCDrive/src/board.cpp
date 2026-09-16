#include "board.h"

#include <Arduino.h>
#include <driver/rtc_io.h>
#include <esp_system.h>

#include "config.h"

namespace {
int power_refs = 0;

void holdPinLow(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}
}  // namespace

void boardInit() {
  holdPinLow(FOC_PIN_MEN);
  power_refs = 0;
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if (AXIS_PRESENT(i)) {
      continue;
    }
    for (int p = 0; p < 3; ++p) {
      holdPinLow(kAxisPwm[i][p]);
    }
  }
}

void boardMotorPowerRef(int delta) {
  power_refs += delta;
  if (power_refs < 0) {
    power_refs = 0;
  }
  digitalWrite(FOC_PIN_MEN, power_refs > 0 ? HIGH : LOW);
}

bool boardMotorPowered() { return power_refs > 0; }

void boardEnterDownload() {
  Serial.println("download: IO0 held low + restart -> ROM");
  Serial.flush();
  delay(50);
  /* ESP32 (not S2/S3) has no FORCE_DOWNLOAD_BOOT register: keep GPIO0 low
   * across esp_restart() with the RTC pad hold so the ROM samples download. */
  rtc_gpio_init(GPIO_NUM_0);
  rtc_gpio_set_direction(GPIO_NUM_0, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(GPIO_NUM_0, 0);
  rtc_gpio_hold_en(GPIO_NUM_0);
  esp_restart();
}
