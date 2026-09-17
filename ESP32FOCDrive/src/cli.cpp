#include "cli.h"

#include <Arduino.h>
#include <WiFi.h>

#include "axis.h"
#include "board.h"
#include "cmd_parse.h"
#include "config.h"
#include "drive_api.h"
#include "foc_task.h"
#include "net.h"
#include "self_test.h"

/* Still provided by main.cpp: not part of the core/core drive API, just the
 * `enc` command's raw pin/count dump. */
void mainPrintEncLine(uint8_t axis_mask);

namespace {
char line[96];
uint8_t line_len = 0;
bool enc_stream = false;
uint8_t enc_stream_mask = (uint8_t)FOC_AXIS_MASK;
uint32_t enc_last_print_ms = 0;
bool mon_stream = false;
uint32_t mon_last_print_ms = 0;
/* Fixed demonstration setpoint for `fs <ms>` (see handleLine()): the same
 * magnitude the bench validation in task-7-brief.md uses for `vel 3`. */
constexpr float kFsTestVelRadS = 3.0f;

void selfTestReport(const char *name, bool ok, void *) {
  Serial.printf("selftest: %s %s\n", name, ok ? "PASS" : "FAIL");
}

const char *modeName(Mode m) {
  switch (m) {
    case Mode::Off:
      return "OFF";
    case Mode::Openloop:
      return "OL";
    case Mode::Velocity:
      return "VEL";
    case Mode::Torque:
      return "TQ";
  }
  return "?";
}

/* Duplicated from axis.cpp's own dirName() (kept file-local there): trivial
 * enough, and axis.cpp's copy lives in an anonymous namespace on purpose. */
const char *dirName(Direction d) {
  if (d == Direction::CW) {
    return "CW";
  }
  if (d == Direction::CCW) {
    return "CCW";
  }
  return "UNKNOWN";
}

/**
 * `status` and the `mon` stream both print this: one line per present axis
 * (mode/armed/cal/encoder/estimated current/calibration), then one global
 * line (FOC loop timing, shared M_EN, WiFi). Replaces the older, sparser
 * cliPrintStatus() seam in main.cpp, which nothing calls anymore now that
 * this exists (see task-6-report.md).
 *
 * Every per-axis field read here is either a plain struct member or a pure,
 * lock-protected read (encoder.count()/countAngle()) — never
 * encoder.getVelocity(), which is stateful (it advances vel_count_prev_) and
 * whose only safe caller is the core-0 FOC task already calling it every
 * pass via motor.move(); a second, unsynchronized caller here would corrupt
 * the running velocity estimate the closed loops depend on. motor.shaft_velocity,
 * the value move() already computed and stored, is what's printed instead.
 *
 * tgt is motor.target, in the MOTOR frame the FOC loop actually drives.
 * Since task 7, drive_api.h's setpoint calls apply cmd_sign in the ROBOT
 * frame before writing motor.target (motor.target = cmd_sign * cmd), and
 * cmd_sign is +-1, so cmd_sign is its own inverse: cmd = tgt * cmd_sign
 * recovers the robot-frame command with no extra state to keep in sync.
 */
void printStatusLine() {
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if (!axes[i].present) {
      continue;
    }
    const Axis &ax = axes[i];
    Serial.printf(
        "%c mode=%s armed=%d cal=%d idx=%d cnt=%lld ang=%.4f vel=%.3f tgt=%.3f cmd=%.3f "
        "Uq=%.3f Iest=%.3f zero=%.4f dir=%s lim=%.2f\n",
        ax.name, modeName(ax.mode), (int)ax.armed, (int)ax.calibrated,
        (int)ax.encoder.indexFound(), (long long)ax.encoder.count(),
        (double)ax.encoder.countAngle(), (double)ax.motor.shaft_velocity,
        (double)ax.motor.target, (double)(ax.motor.target * (float)ax.cmd_sign),
        (double)ax.motor.voltage.q, (double)axisCurrentEstimate(ax),
        (double)ax.motor.zero_electric_angle, dirName(ax.motor.sensor_direction),
        (double)ax.voltage_limit);
  }

  char wifi_buf[48];
  const wifi_mode_t wm = WiFi.getMode();
  if (wm == WIFI_MODE_NULL) {
    snprintf(wifi_buf, sizeof(wifi_buf), "OFF");
  } else if (wm == WIFI_MODE_STA && WiFi.status() == WL_CONNECTED) {
    snprintf(wifi_buf, sizeof(wifi_buf), "STA %s", WiFi.localIP().toString().c_str());
  } else {
    snprintf(wifi_buf, sizeof(wifi_buf), "AP %s", WiFi.softAPIP().toString().c_str());
  }
  const FocMetrics m = focGetMetrics();
  Serial.printf("hz=%lu dt=%lu dtmax=%lu late=%lu loops=%llu MEN=%d wifi=%s\n",
                (unsigned long)m.hz, (unsigned long)m.dt_us, (unsigned long)m.dt_max_us,
                (unsigned long)m.late, (unsigned long long)m.loops, (int)boardMotorPowered(),
                wifi_buf);
}

/**
 * `axes`: static per-build info (pin map, PCNT unit, present/calibrated) for
 * every axis, and the compile-time FOC_AXIS_MASK -- so a human at the
 * console can tell which env (left/right/dual) is actually running without
 * cross-checking against config.h. Unlike printStatusLine()'s per-axis line,
 * this does NOT skip absent axes: the whole point is to show the mask
 * itself, and an absent axis' pins/PCNT unit are still worth showing (they
 * are wired on the board even when this build does not drive them).
 */
void printAxesLine() {
  Serial.printf("axes: mask=0b%d%d", (int)AXIS_PRESENT(1), (int)AXIS_PRESENT(0));
  for (int i = 0; i < AXIS_COUNT; ++i) {
    const Axis &ax = axes[i];
    Serial.print(i == 0 ? "  " : "\n                 ");
    Serial.printf("%c pwm=%d/%d/%d abz=%d/%d/%d pcnt=%d present=%d cal=%d", ax.name,
                  kAxisPwm[i][0], kAxisPwm[i][1], kAxisPwm[i][2], kAxisEnc[i][0], kAxisEnc[i][1],
                  kAxisEnc[i][2], i, (int)ax.present, (int)ax.calibrated);
  }
  Serial.println();
}

void handleLine(const char *raw) {
  ParsedCmd p{};
  if (!parseCmd(raw, &p)) {
    return;
  }
  if (strcmp(p.cmd, "help") == 0 || strcmp(p.cmd, "?") == 0) {
    cliPrintHelp();
  } else if (strcmp(p.cmd, "status") == 0) {
    printStatusLine();
  } else if (strcmp(p.cmd, "axes") == 0) {
    printAxesLine();
  } else if (strcmp(p.cmd, "vel") == 0) {
    if (!p.has_value) {
      Serial.println("usage: vel [L|R] <rad/s>");
      return;
    }
    /* Robot frame: driveSetVelocity() applies cmd_sign once, per axis.
     * timeout_ms=0 so a bench session is never cut off. Printed only on
     * success (bool return, task-7 fixup): on an uncalibrated axis
     * axisSetVelocity() already prints "need cal L" through axisRequireCal(),
     * and a bare unconditional confirmation right after that would read like
     * the command went through anyway. */
    for (int i = 0; i < AXIS_COUNT; ++i) {
      if ((p.axis_mask & (1u << i)) && axes[i].present &&
          driveSetVelocity((uint8_t)i, p.value, 0)) {
        Serial.printf("vel %c: cmd=%.3f rad/s (robot frame)\n", axes[i].name, (double)p.value);
      }
    }
  } else if (strcmp(p.cmd, "tq") == 0) {
    if (!p.has_value) {
      Serial.println("usage: tq [L|R] <V>");
      return;
    }
    for (int i = 0; i < AXIS_COUNT; ++i) {
      if ((p.axis_mask & (1u << i)) && axes[i].present &&
          driveSetTorque((uint8_t)i, p.value, 0)) {
        Serial.printf("tq %c: cmd=%.3f V (robot frame)\n", axes[i].name, (double)p.value);
      }
    }
  } else if (strcmp(p.cmd, "fs") == 0) {
    /* Failsafe demo/test: sends a fixed velocity setpoint with an expiring
     * timeout, so `status` can be watched to confirm the axis parks itself
     * (armed=0, MEN=0) once timeout_ms elapses. Documents the mechanism the
     * balance loop will rely on; kept permanently, not just for this task's
     * bench validation. No axis prefix -> every axis present in this build,
     * same default as every other command's axis_mask. */
    if (!p.has_value) {
      Serial.println("usage: fs [L|R] <ms>");
      return;
    }
    /* Bound before the float->uint32_t cast below: a value outside the
     * target range is undefined behavior, not just a clamp. 3600000 ms (1 h)
     * is far past any sane failsafe test and well inside both ranges. */
    float ms = p.value;
    if (ms < 0.0f) {
      ms = 0.0f;
    } else if (ms > 3600000.0f) {
      ms = 3600000.0f;
    }
    const uint32_t timeout_ms = (uint32_t)ms;
    for (int i = 0; i < AXIS_COUNT; ++i) {
      if ((p.axis_mask & (1u << i)) && axes[i].present &&
          driveSetVelocity((uint8_t)i, kFsTestVelRadS, timeout_ms)) {
        Serial.printf("fs %c: cmd=%.1f rad/s timeout=%lu ms\n", axes[i].name,
                      (double)kFsTestVelRadS, (unsigned long)timeout_ms);
      }
    }
  } else if (strcmp(p.cmd, "mon") == 0) {
    if (!p.has_value) {
      Serial.printf("mon: %d\n", (int)mon_stream);
    } else if (p.value >= 0.5f) {
      mon_stream = true;
      mon_last_print_ms = millis();
      printStatusLine();
    } else {
      mon_stream = false;
    }
  } else if (strcmp(p.cmd, "ol") == 0) {
    if (!p.has_value) {
      Serial.println("usage: ol [L|R] <rad/s>");
      return;
    }
    float v = p.value;
    if (v > FOC_VEL_LIMIT) {
      v = FOC_VEL_LIMIT;
    } else if (v < -FOC_VEL_LIMIT) {
      v = -FOC_VEL_LIMIT;
    }
    for (int i = 0; i < AXIS_COUNT; ++i) {
      if ((p.axis_mask & (1u << i)) && axes[i].present) {
        /* Unlike vel/tq, open-loop has no calibration gate to refuse on --
         * the guarded axis index above is the only way this could fail, and
         * that is already excluded -- so an unconditional confirmation below
         * stays accurate; (void) just silences [[nodiscard]] deliberately. */
        (void)driveSetOpenloop((uint8_t)i, v, 0);
      }
    }
    Serial.printf("ol: mask=%u cmd=%.2f rad/s (robot frame)\n", (unsigned)p.axis_mask, (double)v);
  } else if (strcmp(p.cmd, "idle") == 0 || strcmp(p.cmd, "stop") == 0) {
    for (int i = 0; i < AXIS_COUNT; ++i) {
      if ((p.axis_mask & (1u << i)) && axes[i].present) {
        driveStop((uint8_t)i);
      }
    }
    Serial.println("idle");
  } else if (strcmp(p.cmd, "limit") == 0) {
    /* Voltage limit is per-axis configuration, not a drive-API setpoint: it
     * has no robot/motor frame distinction, so it stays inline here rather
     * than routing through drive_api.h (same pattern as `alignv` below). */
    if (!p.has_value) {
      float v = FOC_VOLTAGE_LIMIT;
      for (int i = 0; i < AXIS_COUNT; ++i) {
        if ((p.axis_mask & (1u << i)) && axes[i].present) {
          v = axes[i].voltage_limit;
          break;
        }
      }
      Serial.printf("limit: %.2f V\n", (double)v);
      return;
    }
    float v = p.value;
    if (v < 0.2f) {
      v = 0.2f;
    }
    if (v > FOC_VBUS) {
      v = FOC_VBUS;
    }
    for (int i = 0; i < AXIS_COUNT; ++i) {
      if (!(p.axis_mask & (1u << i)) || !axes[i].present) {
        continue;
      }
      axes[i].voltage_limit = v;
      axisApplyLimits(axes[i]);
    }
    Serial.printf("limit: %.2f V\n", (double)v);
  } else if (strcmp(p.cmd, "alignv") == 0) {
    if (!p.has_value) {
      for (int i = 0; i < AXIS_COUNT; ++i) {
        if ((p.axis_mask & (1u << i)) && axes[i].present) {
          Serial.printf("alignv %c: %.2f V\n", axes[i].name, (double)axes[i].align_voltage);
        }
      }
      return;
    }
    for (int i = 0; i < AXIS_COUNT; ++i) {
      if (!(p.axis_mask & (1u << i)) || !axes[i].present) {
        continue;
      }
      Axis &ax = axes[i];
      ax.align_voltage = p.value;
      if (ax.align_voltage < 0.2f) {
        ax.align_voltage = 0.2f;
      }
      /* Clamps to ax.voltage_limit and pushes into motor.voltage_sensor_align. */
      axisApplyLimits(ax);
      Serial.printf("alignv %c: %.2f V\n", ax.name, (double)ax.align_voltage);
    }
  } else if (strcmp(p.cmd, "cal") == 0) {
    /* No axis prefix -> both axes, one after the other: axisCal() spins the
     * shaft open-loop, so two axes at once would be two motors moving at
     * once for no reason and would make each ratio warning ambiguous. */
    for (int i = 0; i < AXIS_COUNT; ++i) {
      if (!(p.axis_mask & (1u << i)) || !axes[i].present) {
        continue;
      }
      axisCal(axes[i]);
    }
  } else if (strcmp(p.cmd, "zsearch") == 0) {
    for (int i = 0; i < AXIS_COUNT; ++i) {
      if (!(p.axis_mask & (1u << i)) || !axes[i].present) {
        continue;
      }
      axisZSearch(axes[i], true);
    }
  } else if (strcmp(p.cmd, "save") == 0) {
    for (int i = 0; i < AXIS_COUNT; ++i) {
      if (!(p.axis_mask & (1u << i)) || !axes[i].present) {
        continue;
      }
      axisSaveCal(axes[i]);
    }
  } else if (strcmp(p.cmd, "forget") == 0) {
    for (int i = 0; i < AXIS_COUNT; ++i) {
      if (!(p.axis_mask & (1u << i)) || !axes[i].present) {
        continue;
      }
      axisForgetCal(axes[i]);
    }
  } else if (strcmp(p.cmd, "enc") == 0) {
    if (!p.has_value) {
      mainPrintEncLine(p.axis_mask);
    } else if (p.value >= 0.5f) {
      enc_stream = true;
      enc_stream_mask = p.axis_mask;
      enc_last_print_ms = millis();
      mainPrintEncLine(p.axis_mask);
    } else {
      enc_stream = false;
    }
  } else if (strcmp(p.cmd, "download") == 0 || strcmp(p.cmd, "dl") == 0) {
    for (int i = 0; i < AXIS_COUNT; ++i) {
      driveStop((uint8_t)i);
    }
    boardEnterDownload();
  } else if (strcmp(p.cmd, "ota") == 0) {
    netPrintInfo();
  } else if (strcmp(p.cmd, "wifioff") == 0) {
    netWifiOff();
  } else if (strcmp(p.cmd, "hz") == 0) {
    if (!p.has_value) {
      const FocMetrics m = focGetMetrics();
      Serial.printf("hz=%lu dt=%lu us dtmax=%lu us late=%lu loops=%llu\n",
                    (unsigned long)m.hz, (unsigned long)m.dt_us, (unsigned long)m.dt_max_us,
                    (unsigned long)m.late, (unsigned long long)m.loops);
      return;
    }
    focSetHz((uint32_t)p.value);
    Serial.printf("hz: %lu\n", (unsigned long)focHz());
  } else if (strcmp(p.cmd, "dt") == 0) {
    focResetMetrics();
    Serial.println("dt: reset");
  } else if (strcmp(p.cmd, "selftest") == 0) {
    const SelfTestResult r = selfTestRun(selfTestReport, nullptr);
    Serial.printf("selftest: %d run, %d failed\n", r.run, r.failed);
  } else {
    Serial.println("unknown - help");
  }
}
}  // namespace

void cliInit() { line_len = 0; }

void cliPrintHelp() {
  Serial.println("ESP32FOCDrive  FS2804 x2  voltage FOC  MT6835 ABZ");
  Serial.println("  help status         axes  mask/pins/present/cal per axis");
  Serial.println("  ol [L|R] <rad/s>   idle [L|R]");
  Serial.println("  limit [L|R] <V>    alignv [L|R] <V>  download");
  Serial.println("  cal [L|R]          zsearch [L|R]  save [L|R]  forget [L|R]");
  Serial.println("  vel [L|R] <rad/s>  tq [L|R] <V>");
  Serial.println("  fs [L|R] <ms>      failsafe test: 3 rad/s expiring after <ms>");
  Serial.println("                     no L|R = every present axis (both wheels in dual)");
  Serial.println("  enc [0|1]          mon [0|1]      ota");
  Serial.println("  hz [Hz]            dt");
  Serial.println("  selftest           wifioff");
}

void cliPoll() {
  if (enc_stream && (millis() - enc_last_print_ms) >= 1000u) {
    enc_last_print_ms = millis();
    mainPrintEncLine(enc_stream_mask);
  }
  if (mon_stream && (millis() - mon_last_print_ms) >= 1000u) {
    mon_last_print_ms = millis();
    printStatusLine();
  }
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (line_len == 0) {
        continue;
      }
      line[line_len] = '\0';
      line_len = 0;
      Serial.write('\n');
      handleLine(line);
      continue;
    }
    if (c == 0x08 || c == 0x7f) {
      if (line_len > 0) {
        line_len -= 1;
        Serial.print("\b \b");
      }
      continue;
    }
    if (line_len + 1u < sizeof(line)) {
      line[line_len++] = c;
      Serial.write(c);
    }
  }
}
