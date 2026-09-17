#include "cli.h"

#include <Arduino.h>
#include <WiFi.h>

#include "axis.h"
#include "board.h"
#include "cmd_parse.h"
#include "config.h"
#include "foc_task.h"
#include "net.h"
#include "self_test.h"

/* Temporary seam, replaced by drive_api.h in task 7. */
void mainSetOpenloop(uint8_t axis_mask, float rad_s);
void mainIdle(uint8_t axis_mask);
float mainVoltageLimit(uint8_t axis_mask);
void mainSetVoltageLimit(uint8_t axis_mask, float v);
void mainPrintEncLine(uint8_t axis_mask);

namespace {
char line[96];
uint8_t line_len = 0;
bool enc_stream = false;
uint8_t enc_stream_mask = (uint8_t)FOC_AXIS_MASK;
uint32_t enc_last_print_ms = 0;
bool mon_stream = false;
uint32_t mon_last_print_ms = 0;

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
 */
void printStatusLine() {
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if (!axes[i].present) {
      continue;
    }
    const Axis &ax = axes[i];
    Serial.printf(
        "%c mode=%s armed=%d cal=%d idx=%d cnt=%lld ang=%.4f vel=%.3f tgt=%.3f Uq=%.3f "
        "Iest=%.3f zero=%.4f dir=%s lim=%.2f\n",
        ax.name, modeName(ax.mode), (int)ax.armed, (int)ax.calibrated,
        (int)ax.encoder.indexFound(), (long long)ax.encoder.count(),
        (double)ax.encoder.countAngle(), (double)ax.motor.shaft_velocity,
        (double)ax.motor.target, (double)ax.motor.voltage.q, (double)axisCurrentEstimate(ax),
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

void handleLine(const char *raw) {
  ParsedCmd p{};
  if (!parseCmd(raw, &p)) {
    return;
  }
  if (strcmp(p.cmd, "help") == 0 || strcmp(p.cmd, "?") == 0) {
    cliPrintHelp();
  } else if (strcmp(p.cmd, "status") == 0) {
    printStatusLine();
  } else if (strcmp(p.cmd, "vel") == 0) {
    if (!p.has_value) {
      Serial.println("usage: vel [L|R] <rad/s>");
      return;
    }
    float v = p.value;
    if (v > FOC_VEL_LIMIT) {
      v = FOC_VEL_LIMIT;
    } else if (v < -FOC_VEL_LIMIT) {
      v = -FOC_VEL_LIMIT;
    }
    for (int i = 0; i < AXIS_COUNT; ++i) {
      if (!(p.axis_mask & (1u << i)) || !axes[i].present) {
        continue;
      }
      if (axisSetVelocity(axes[i], v)) {
        Serial.printf("vel %c: tgt=%.3f rad/s\n", axes[i].name, (double)v);
      }
    }
  } else if (strcmp(p.cmd, "tq") == 0) {
    if (!p.has_value) {
      Serial.println("usage: tq [L|R] <V>");
      return;
    }
    for (int i = 0; i < AXIS_COUNT; ++i) {
      if (!(p.axis_mask & (1u << i)) || !axes[i].present) {
        continue;
      }
      Axis &ax = axes[i];
      float v = p.value;
      if (v > ax.voltage_limit) {
        v = ax.voltage_limit;
      } else if (v < -ax.voltage_limit) {
        v = -ax.voltage_limit;
      }
      if (axisSetTorque(ax, v)) {
        Serial.printf("tq %c: tgt=%.3f V\n", ax.name, (double)v);
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
    mainSetOpenloop(p.axis_mask, v);
    Serial.printf("ol: mask=%u tgt=%.2f rad/s\n", (unsigned)p.axis_mask, (double)v);
  } else if (strcmp(p.cmd, "idle") == 0 || strcmp(p.cmd, "stop") == 0) {
    mainIdle(p.axis_mask);
    Serial.println("idle");
  } else if (strcmp(p.cmd, "limit") == 0) {
    if (!p.has_value) {
      Serial.printf("limit: %.2f V\n", (double)mainVoltageLimit(p.axis_mask));
      return;
    }
    float v = p.value;
    if (v < 0.2f) {
      v = 0.2f;
    }
    if (v > FOC_VBUS) {
      v = FOC_VBUS;
    }
    mainSetVoltageLimit(p.axis_mask, v);
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
    mainIdle(0b11);
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
  Serial.println("  help status");
  Serial.println("  ol [L|R] <rad/s>   idle [L|R]");
  Serial.println("  limit [L|R] <V>    alignv [L|R] <V>  download");
  Serial.println("  cal [L|R]          zsearch [L|R]  save [L|R]  forget [L|R]");
  Serial.println("  vel [L|R] <rad/s>  tq [L|R] <V>");
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
