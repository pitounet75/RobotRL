#include "cli.h"

#include <Arduino.h>

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

void selfTestReport(const char *name, bool ok, void *) {
  Serial.printf("selftest: %s %s\n", name, ok ? "PASS" : "FAIL");
}

void handleLine(const char *raw) {
  ParsedCmd p{};
  if (!parseCmd(raw, &p)) {
    return;
  }
  if (strcmp(p.cmd, "help") == 0 || strcmp(p.cmd, "?") == 0) {
    cliPrintHelp();
  } else if (strcmp(p.cmd, "status") == 0) {
    cliPrintStatus();
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
  Serial.println("  enc [0|1]          ota");
  Serial.println("  hz [Hz]            dt");
  Serial.println("  selftest           wifioff");
}

void cliPoll() {
  if (enc_stream && (millis() - enc_last_print_ms) >= 1000u) {
    enc_last_print_ms = millis();
    mainPrintEncLine(enc_stream_mask);
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
