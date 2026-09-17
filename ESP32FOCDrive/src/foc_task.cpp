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
#include "drive_api.h"
#include "failsafe.h"

namespace {
TaskHandle_t s_task = nullptr;
uint32_t s_hz = FOC_LOOP_HZ;
volatile uint32_t s_dt_us = 0;
volatile uint32_t s_dt_max_us = 0;
volatile uint32_t s_late = 0;
volatile uint64_t s_loops = 0;
volatile uint32_t s_seq = 0;
volatile bool s_timer_paused = false;

/* vcap/vdump: see foc_task.h for the rationale. kVCapMax*sizeof(VCapSample)
 * (2500 * 12 bytes = ~29 KiB) is static RAM, not stack -- worth remembering
 * before raising kVCapMax. */
constexpr uint32_t kVCapMax = 2500;
struct VCapSample {
  int32_t dcount; /* counts since the previous kept sample: exact, no filter */
  float vel;
  float uq;
};
VCapSample s_vcap_buf[kVCapMax];
/* Cross-core: armed from the CLI (core 1), consumed every tick by the FOC
 * task (core 0) -- volatile for the same reason axis.h's mode/owner/etc are
 * (see the comment there): nothing here is protected by a lock, so the
 * qualifier is the only thing stopping a stale cached read/hoist. */
volatile uint8_t s_vcap_axis = 0;
volatile uint32_t s_vcap_idx = 0;
volatile uint32_t s_vcap_target = 0;
volatile uint32_t s_vcap_decim = 1;
volatile uint32_t s_vcap_loop_ctr = 0;
volatile int64_t s_vcap_last_count = 0;

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
    const uint32_t now_ms = millis();
    for (int i = 0; i < AXIS_COUNT; ++i) {
      Axis &ax = axes[i];
      if (!ax.present || ax.owner != Owner::Task) {
        continue;
      }
      if (ax.armed && failsafeExpired(now_ms, ax.last_cmd_ms, ax.cmd_timeout_ms)) {
        /* Stale command: park the axis. The task disarms itself here, on its
         * own core, so there is nothing to sync -- unlike axisSetMode(),
         * which exists precisely because a CLI/API write to ax.mode from the
         * other core needs the task to observe it.
         *
         * The test above is unlocked -- cheap, but by design stale the
         * instant it passes: a fresh command can land on core 1 in the gap
         * between this line and axisFailsafeDisarm() actually acquiring
         * boardMotorPowerLock() below. axisFailsafeDisarm() re-reads
         * last_cmd_ms/cmd_timeout_ms once it holds that same lock -- the one
         * axisSetVelocity()/axisSetTorque()'s fast path also holds around
         * their test+write+stamp -- and only parks the axis if the command
         * is STILL expired under that lock, so it can never clobber a
         * command that arrived in the window. Taking the lock here just to
         * run the cheap test, every axis, every 250 us tick, would cost more
         * than the race it closes -- only the disarm path pays for it, and
         * only when the unlocked pre-check already says it is due. */
        if (axisFailsafeDisarm(ax, now_ms)) {
          ax.mode = Mode::Off;
        }
      }
      if (ax.mode == Mode::Off) {
        ax.encoder.update();
      } else {
        ax.motor.loopFOC();
        ax.motor.move();
      }
      drivePublishState(ax, i);
      if (ax.mode != Mode::Off && i == (int)s_vcap_axis) {
        /* Per-iteration capture: a few array writes, nothing else -- see
         * foc_task.h for why. Only while this axis is actually driving
         * (same condition as before this moved), matching the "vel 3 then
         * vcap" bench flow. Moved after drivePublishState(): the capture
         * does not feed anything the publish needs, so running it first
         * only delayed the published snapshot for no benefit -- strictly
         * better placed here instead. */
        const uint32_t idx = s_vcap_idx;
        if (idx < s_vcap_target) {
          const uint32_t ctr = s_vcap_loop_ctr;
          s_vcap_loop_ctr = ctr + 1;
          if ((ctr % s_vcap_decim) == 0) {
            const int64_t c = ax.encoder.count();
            s_vcap_buf[idx].dcount = (int32_t)(c - s_vcap_last_count);
            s_vcap_buf[idx].vel = ax.motor.shaft_velocity;
            s_vcap_buf[idx].uq = ax.motor.voltage.q;
            s_vcap_last_count = c;
            s_vcap_idx = idx + 1;
          }
        }
      }
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

void vcapArm(uint8_t axis, uint32_t ms) {
  if (axis >= (uint8_t)AXIS_COUNT) {
    axis = 0;
  }
  if (ms == 0u) {
    ms = 1u;
  } else if (ms > 3600000u) { /* same 1 h ceiling cli.cpp's `fs` uses */
    ms = 3600000u;
  }
  /* 64-bit: at the max hz (16 kHz) and ms (1 h), ms*hz overflows a 32-bit
   * total well before decim/n bring it back down to kVCapMax. */
  const uint64_t total_loops = (uint64_t)ms * (uint64_t)s_hz / 1000ull;
  uint32_t decim = 1;
  if (total_loops > (uint64_t)kVCapMax) {
    decim = (uint32_t)((total_loops + kVCapMax - 1) / kVCapMax);
  }
  uint32_t n = (uint32_t)(total_loops / decim);
  if (n > kVCapMax) {
    n = kVCapMax;
  }
  /* Order matters for a capture already in flight when this re-arms it:
   * axis/decim/last_count first, target LAST (it gates whether the task
   * looks at any of the others), idx reset just ahead of target so the task
   * never sees a stale idx>=new-target and calls a fresh session "done"
   * before it starts. */
  s_vcap_axis = axis;
  s_vcap_loop_ctr = 0;
  s_vcap_decim = decim;
  s_vcap_last_count = axes[axis].encoder.count();
  s_vcap_idx = 0;
  s_vcap_target = n;
  Serial.printf(
      "vcap %c: arming %u samples, decim=%u (each dcount spans %.2f ms) covering ~%u ms -- vdump "
      "when idx=target\n",
      axes[axis].name, (unsigned)n, (unsigned)decim, (double)(decim * 1000.0 / (double)s_hz),
      (unsigned)ms);
}

bool vcapDone() { return s_vcap_target > 0u && s_vcap_idx >= s_vcap_target; }

void vcapDump() {
  const uint8_t axis = s_vcap_axis;
  const uint32_t idx = s_vcap_idx;
  const uint32_t target = s_vcap_target;
  Serial.printf("vdump %c: idx=%u target=%u decim=%u %s\n", axes[axis].name, (unsigned)idx,
                (unsigned)target, (unsigned)s_vcap_decim,
                (target > 0u && idx >= target) ? "(done)" : "(still capturing or empty)");
  Serial.println("idx,dcount,vel,uq");
  for (uint32_t i = 0; i < idx; ++i) {
    Serial.printf("%u,%ld,%.4f,%.4f\n", (unsigned)i, (long)s_vcap_buf[i].dcount,
                  (double)s_vcap_buf[i].vel, (double)s_vcap_buf[i].uq);
  }
}
