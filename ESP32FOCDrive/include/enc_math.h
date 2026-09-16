#pragma once

#include <stdint.h>

#ifndef ENC_MATH_2PI
#define ENC_MATH_2PI 6.28318530718f
#endif

/**
 * ESP32 PCNT is 16-bit and RESETS TO 0 at h_lim/l_lim instead of wrapping in
 * two's complement. With symmetric +/-lim, every reset is a known step of
 * `lim` counts, so the raw difference can be folded in software on each read:
 * no limit ISR, no pcnt_counter_clear, no lost counts.
 *
 * Caller must read often enough that a true motion never exceeds lim/2
 * (8192 counts here, about 10 ms at 80 rad/s with 65536 CPR).
 */
inline int32_t encFoldDelta(int16_t prev, int16_t now, int32_t lim) {
  int32_t d = (int32_t)now - (int32_t)prev;
  if (d < -(lim / 2)) {
    d += lim;
  } else if (d > (lim / 2)) {
    d -= lim;
  }
  return d;
}

/** Split a 64-bit count into full rotations and a shaft angle in [0, 2PI). */
inline void encAngleFromCount(int64_t count, int64_t cpr, int32_t *rotations,
                              float *shaft) {
  int64_t rot = count / cpr;
  int64_t rem = count % cpr;
  if (rem < 0) {
    rem += cpr;
    rot -= 1;
  }
  *rotations = (int32_t)rot;
  *shaft = (float)rem * (ENC_MATH_2PI / (float)cpr);
}
