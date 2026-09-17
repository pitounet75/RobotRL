#include "board.h"

#include <Arduino.h>
#include <driver/rtc_io.h>
#include <esp_system.h>

#include "config.h"

namespace {
int power_refs = 0;
/* See boardMotorPowerLock() in board.h for why this exists: axisArm()/
 * axisDisarm() (axis.cpp) are reachable from both cores since the task-7
 * failsafe, and this spinlock is what makes their read-modify-write of
 * power_refs indivisible across them. */
portMUX_TYPE s_power_mux = portMUX_INITIALIZER_UNLOCKED;

void holdPinLow(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}
}  // namespace

void boardInit() {
  /* boardEnterDownload() leaves GPIO0 held low via the RTC pad hold across
   * esp_restart() so the ROM samples download mode. Nothing ever released
   * that hold on a normal boot, so the board stayed stuck in download mode
   * after every flash until the user power-cycled it. These are no-ops when
   * no hold is active. */
  rtc_gpio_hold_dis(GPIO_NUM_0);
  rtc_gpio_deinit(GPIO_NUM_0);

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

void boardMotorPowerLock() { portENTER_CRITICAL(&s_power_mux); }
void boardMotorPowerUnlock() { portEXIT_CRITICAL(&s_power_mux); }

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
