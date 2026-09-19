#include "board.h"

#include <Arduino.h>
#include <driver/rtc_io.h>
#include <esp_system.h>

#include "config.h"

namespace {
int power_refs = 0;
/* See boardMotorPowerLock() in board.h for why this exists: axisArm() is
 * core-1-only, but axisDisarmLocked() (axis.cpp, the guarded body shared by
 * axisDisarm() and the FOC task's failsafe axisFailsafeDisarm()) is reachable
 * from both the CLI and the FOC task since the task-7 failsafe -- both run
 * on core 1 today, but the FOC task preempts the CLI at any instruction --
 * and this spinlock is what makes its read-modify-write of power_refs
 * indivisible against that. */
portMUX_TYPE s_power_mux = portMUX_INITIALIZER_UNLOCKED;

void holdPinLow(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}
}  // namespace

void boardInit() {
  /* M_EN feeds BOTH gate drivers (see board.h): it must be the very first
   * write this function makes, ahead of the GPIO0 hold release below, even
   * though that release has no real exposure window of its own today. */
  holdPinLow(FOC_PIN_MEN);
  power_refs = 0;

  /* boardEnterDownload() leaves GPIO0 held low via the RTC pad hold across
   * esp_restart() so the ROM samples download mode. Nothing ever released
   * that hold on a normal boot, so the board stayed stuck in download mode
   * after every flash until the user power-cycled it. These are no-ops when
   * no hold is active. */
  rtc_gpio_hold_dis(GPIO_NUM_0);
  rtc_gpio_deinit(GPIO_NUM_0);

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
