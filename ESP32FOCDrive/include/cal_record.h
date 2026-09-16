#pragma once

#include <math.h>
#include <stdint.h>

/** 'DRV1'. Deliberately different from HardwareCheck's 'HCK2': that firmware
 * stored a zero taken with a current-sense alignment path that no longer
 * exists, so its records must not be reused. */
constexpr uint32_t kCalMagic = 0x44525631u;

struct CalRecord {
  uint32_t magic;
  float zero_electric_angle;
  int8_t sensor_direction;  /* +1 CW, -1 CCW, 0 unknown */
  uint8_t pole_pairs;
  uint16_t enc_ppr;
  char axis;
};

inline bool calRecordValid(const CalRecord &rec, char axis, uint16_t ppr) {
  if (rec.magic != kCalMagic || rec.axis != axis) {
    return false;
  }
  if (rec.pole_pairs == 0 || rec.sensor_direction == 0) {
    return false;
  }
  if (rec.enc_ppr != ppr) {
    return false;
  }
  return isfinite(rec.zero_electric_angle) != 0;
}
