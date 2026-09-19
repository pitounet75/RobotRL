#pragma once

#include <stdint.h>

struct PitchFusionState {
  float pitch_rad;
  uint32_t sample_count;
  uint32_t last_t_us;
};

struct PitchFusionOut {
  float pitch_rad;
  float pitch_rate_rads;
};

void pitch_fusion_reset(PitchFusionState *s);

bool pitch_fusion_update(PitchFusionState *s,
                         const float accel_mps2[3],
                         const float gyro_rads[3],
                         uint32_t t_us,
                         PitchFusionOut *out);
