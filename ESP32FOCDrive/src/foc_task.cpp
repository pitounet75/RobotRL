/**
 * Core-0 FOC task. Body and timer setup ported from
 * ESP32FOCHardwareCheck/src/main.cpp:1146-1241, extended to loop over both
 * axes with an ownership check.
 */

#include "foc_task.h"

#include <Arduino.h>
#include <driver/timer.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "axis.h"
#include "config.h"

namespace {
TaskHandle_t s_task = nullptr;
uint32_t s_hz = FOC_LOOP_HZ;
volatile uint32_t s_dt_us = 0;
volatile uint32_t s_dt_max_us = 0;
volatile uint32_t s_late = 0;
volatile uint64_t s_loops = 0;
volatile uint32_t s_seq = 0;
volatile bool s_timer_paused = false;

bool IRAM_ATTR focTimerCb(void *) {
  BaseType_t hpw = pdFALSE;
  if (s_task != nullptr) {
    vTaskNotifyGiveFromISR(s_task, &hpw);
  }
  return hpw == pdTRUE;
}

void startFocTimer() {
  timer_config_t cfg{};
  cfg.alarm_en = TIMER_ALARM_EN;
  cfg.counter_en = TIMER_PAUSE;
  cfg.intr_type = TIMER_INTR_LEVEL;
  cfg.counter_dir = TIMER_COUNT_UP;
  cfg.auto_reload = TIMER_AUTORELOAD_EN;
  cfg.divider = 80;
  timer_init(TIMER_GROUP_1, TIMER_0, &cfg);
  timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
  timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 1000000ull / (uint64_t)s_hz);
  timer_enable_intr(TIMER_GROUP_1, TIMER_0);
  timer_isr_callback_add(TIMER_GROUP_1, TIMER_0, focTimerCb, nullptr, ESP_INTR_FLAG_IRAM);
  timer_start(TIMER_GROUP_1, TIMER_0);
}

void focTask(void *) {
  disableCore0WDT();
  startFocTimer();
  for (;;) {
    const uint32_t n = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (n > 1u) {
      s_late += n - 1u;
    }
    const int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < AXIS_COUNT; ++i) {
      Axis &ax = axes[i];
      if (!ax.present || ax.owner != Owner::Task) {
        continue;
      }
      if (ax.mode == Mode::Off) {
        ax.encoder.update();
        continue;
      }
      ax.motor.loopFOC();
      ax.motor.move();
    }
    const uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
    s_dt_us = dt;
    if (dt > s_dt_max_us) {
      s_dt_max_us = dt;
    }
    s_loops += 1;
    s_seq += 1;
    /* Overrun: notifications pile up, Take returns immediately, IDLE0 never
     * runs and the task WDT aborts. Drop the backlog, wait for a fresh tick. */
    (void)ulTaskNotifyTake(pdTRUE, 0);
  }
}
}  // namespace

void focTaskStart() {
  /* Created from core 1 (setup() runs there) but pinned to core 0: the
   * timer is started from inside focTask() itself so its ISR gets
   * allocated on core 0, not wherever focTaskStart() happened to run. */
  xTaskCreatePinnedToCore(focTask, "foc", 4096, nullptr, 20, &s_task, 0);
}

void focSetHz(uint32_t hz) {
  if (hz < FOC_LOOP_HZ_MIN) {
    hz = FOC_LOOP_HZ_MIN;
  }
  if (hz > FOC_LOOP_HZ_MAX) {
    hz = FOC_LOOP_HZ_MAX;
  }
  s_hz = hz;
  timer_pause(TIMER_GROUP_1, TIMER_0);
  timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 1000000ull / (uint64_t)s_hz);
  timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
  s_dt_max_us = 0;
  timer_start(TIMER_GROUP_1, TIMER_0);
}

uint32_t focHz() { return s_hz; }

bool focSyncWithTask() {
  if (s_task == nullptr || s_timer_paused) {
    /* Nothing to wait for: with no task started yet, or the timer
     * deliberately paused, the task cannot be touching any axis right now
     * either way, so there is no observation to fail. */
    return true;
  }
  const uint32_t start = s_seq;
  const uint32_t t0 = millis();
  while ((s_seq - start) < 2u) {
    if ((millis() - t0) > 50u) {
      return false; /* timer stopped or starved: do not block the CLI */
    }
    vTaskDelay(1);
  }
  return true;
}

void focPauseTimer() {
  s_timer_paused = true;
  timer_pause(TIMER_GROUP_1, TIMER_0);
}

void focResumeTimer() {
  timer_start(TIMER_GROUP_1, TIMER_0);
  s_timer_paused = false;
}

FocMetrics focGetMetrics() {
  FocMetrics m;
  m.dt_us = s_dt_us;
  m.dt_max_us = s_dt_max_us;
  m.late = s_late;
  m.hz = s_hz;
  m.loops = s_loops;
  return m;
}

void focResetMetrics() {
  s_dt_max_us = 0;
  s_late = 0;
}
