#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "cal_record.h"
#include "cmd_parse.h"
#include "enc_math.h"

/**
 * Pure-logic checks, shared by two runners: the Unity suite on target, and
 * the firmware's own `selftest` command. This board cannot be flashed over
 * USB without a manual BOOT+RESET, so validation has to ride the OTA path.
 */
struct SelfTestResult {
  int run;
  int failed;
};

typedef void (*SelfTestReport)(const char *name, bool ok, void *ctx);

#define SELF_TEST_CHECK(name, cond)        \
  do {                                     \
    const bool ok_ = (cond);               \
    res.run += 1;                          \
    if (!ok_) {                            \
      res.failed += 1;                     \
    }                                      \
    if (report != nullptr) {               \
      report(name, ok_, ctx);              \
    }                                      \
  } while (0)

inline bool selfTestNear(float a, float b, float tol) { return fabsf(a - b) <= tol; }

inline CalRecord selfTestMakeCalRecord() {
  CalRecord r{};
  r.magic = kCalMagic;
  r.zero_electric_angle = 1.234f;
  r.sensor_direction = 1;
  r.pole_pairs = 7;
  r.enc_ppr = 16384;
  r.axis = 'L';
  return r;
}

inline SelfTestResult selfTestRun(SelfTestReport report, void *ctx) {
  SelfTestResult res{0, 0};
  ParsedCmd p{};

  SELF_TEST_CHECK("parse_bare", parseCmd("status", &p) && strcmp(p.cmd, "status") == 0 &&
                                    p.axis_mask == (uint8_t)FOC_AXIS_MASK && !p.has_value);
  SELF_TEST_CHECK("parse_value", parseCmd("  vel  3.5 ", &p) && strcmp(p.cmd, "vel") == 0 &&
                                     p.axis_mask == (uint8_t)FOC_AXIS_MASK && p.has_value &&
                                     selfTestNear(p.value, 3.5f, 1e-4f));
  SELF_TEST_CHECK("parse_axis_r", parseCmd("vel R -2", &p) && strcmp(p.cmd, "vel") == 0 &&
                                      p.axis_mask == 0b10 && p.has_value &&
                                      selfTestNear(p.value, -2.0f, 1e-4f));
  SELF_TEST_CHECK("parse_axis_l", parseCmd("cal l", &p) && strcmp(p.cmd, "cal") == 0 &&
                                      p.axis_mask == 0b01 && !p.has_value);
  SELF_TEST_CHECK("parse_empty", !parseCmd("   ", &p));
  SELF_TEST_CHECK("parse_truncate",
                  parseCmd("abcdefghijklmnopqrstuvwxyz 1", &p) && strlen(p.cmd) == 11 && p.has_value);

  SELF_TEST_CHECK("fold_fwd", encFoldDelta(100, 105, 16384) == 5);
  SELF_TEST_CHECK("fold_back", encFoldDelta(105, 100, 16384) == -5);
  SELF_TEST_CHECK("fold_hlim", encFoldDelta(16380, -16374, 16384) == 10);
  SELF_TEST_CHECK("fold_llim", encFoldDelta(-16380, 16374, 16384) == -10);

  int32_t rot = 0;
  float shaft = 0.0f;
  encAngleFromCount(65536 + 16384, 65536, &rot, &shaft);
  SELF_TEST_CHECK("angle_pos", rot == 1 && selfTestNear(shaft, 1.5708f, 1e-3f));
  encAngleFromCount(-16384, 65536, &rot, &shaft);
  SELF_TEST_CHECK("angle_neg", rot == -1 && selfTestNear(shaft, 4.7124f, 1e-3f));

  SELF_TEST_CHECK("cal_match", calRecordValid(selfTestMakeCalRecord(), 'L', 16384));
  SELF_TEST_CHECK("cal_other_axis", !calRecordValid(selfTestMakeCalRecord(), 'R', 16384));
  /* Changing ENC_PPR silently invalidates every stored electrical zero. */
  SELF_TEST_CHECK("cal_ppr_change", !calRecordValid(selfTestMakeCalRecord(), 'L', 4096));
  {
    CalRecord bad = selfTestMakeCalRecord();
    bad.magic = 0xdeadbeefu;
    SELF_TEST_CHECK("cal_bad_magic", !calRecordValid(bad, 'L', 16384));
  }
  {
    CalRecord bad = selfTestMakeCalRecord();
    bad.sensor_direction = 0;
    SELF_TEST_CHECK("cal_no_dir", !calRecordValid(bad, 'L', 16384));
  }

  return res;
}
