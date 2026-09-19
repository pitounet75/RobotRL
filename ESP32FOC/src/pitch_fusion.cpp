#include "pitch_fusion.h"

#include <math.h>

#include "config.h"

namespace {

constexpr float kG = 9.80665f;

bool axis_ok(int a) { return a >= 0 && a <= 2; }

float accel_norm(const float a[3]) {
  return sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

}  // namespace

void pitch_fusion_reset(PitchFusionState *s) {
  if (!s) {
    return;
  }
  s->pitch_rad = 0.0f;
  s->sample_count = 0;
  s->last_t_us = 0;
}

bool pitch_fusion_update(PitchFusionState *s,
                         const float accel_mps2[3],
                         const float gyro_rads[3],
                         uint32_t t_us,
                         PitchFusionOut *out) {
  if (!s || !accel_mps2 || !gyro_rads || !out) {
    return false;
  }
  if (!axis_ok(IMU_PITCH_ACCEL_FORWARD_AXIS) || !axis_ok(IMU_PITCH_ACCEL_UP_AXIS) ||
      !axis_ok(IMU_PITCH_GYRO_AXIS)) {
    return false;
  }

  float dt_s = (float)(t_us - s->last_t_us) * 1e-6f;
  if (s->last_t_us == 0u || dt_s <= 0.0f || dt_s > 0.05f) {
    dt_s = 0.001f;
  }
  s->last_t_us = t_us;

  const float pitch_accel =
      atan2f(-accel_mps2[IMU_PITCH_ACCEL_FORWARD_AXIS], accel_mps2[IMU_PITCH_ACCEL_UP_AXIS]);
  const float pitch_rate = IMU_PITCH_GYRO_SIGN * gyro_rads[IMU_PITCH_GYRO_AXIS];
  const bool accel_ok = fabsf(accel_norm(accel_mps2) - kG) <= 3.0f;
  const float a = IMU_COMPLEMENTARY_ALPHA;

  if (s->sample_count == 0u) {
    s->pitch_rad = accel_ok ? pitch_accel : 0.0f;
  } else if (accel_ok) {
    s->pitch_rad = a * (s->pitch_rad + pitch_rate * dt_s) + (1.0f - a) * pitch_accel;
  } else {
    s->pitch_rad = s->pitch_rad + pitch_rate * dt_s;
  }
  s->sample_count++;

  out->pitch_rad = s->pitch_rad;
  out->pitch_rate_rads = pitch_rate;
  return true;
}
