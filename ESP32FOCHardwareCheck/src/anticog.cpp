#include "anticog.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

#include <SimpleFOC.h>

#include "config.h"

static_assert(ACOG_BINS_MAX == 3600u, "Anticog::map_ is 3600");

namespace {

constexpr uint32_t kMagic = 0x41434F47u; /* ACOG */
constexpr const char *kNs = "acog";
constexpr uint32_t kChunk = 900;
constexpr uint32_t kFinalizeChunk = 32;

struct Header {
  uint32_t magic;
  uint16_t bins;
  char axis;
  uint8_t reserved;
};

float wrapPi(float x) {
  while (x > _PI) {
    x -= _2PI;
  }
  while (x < -_PI) {
    x += _2PI;
  }
  return x;
}

}  // namespace

bool Anticog::load() {
  Header hdr{};
  Preferences prefs;
  if (!prefs.begin(kNs, true)) {
    return false;
  }
  if (prefs.getBytes("hdr", &hdr, sizeof(hdr)) != sizeof(hdr)) {
    prefs.end();
    return false;
  }
  if (hdr.magic != kMagic || hdr.axis != HWCHK_AXIS_CHAR || hdr.bins == 0 ||
      hdr.bins > ACOG_BINS_MAX) {
    prefs.end();
    return false;
  }
  memset(map_, 0, sizeof(map_));
  bool ok = true;
  for (uint32_t i = 0; i < 4; ++i) {
    const uint32_t off = i * kChunk;
    if (off >= hdr.bins) {
      break;
    }
    uint32_t n = hdr.bins - off;
    if (n > kChunk) {
      n = kChunk;
    }
    char key[4] = {'m', (char)('0' + i), 0};
    if (prefs.getBytes(key, &map_[off], n * sizeof(float)) != n * sizeof(float)) {
      ok = false;
      break;
    }
  }
  prefs.end();
  if (!ok) {
    return false;
  }
  bins_ = hdr.bins;
  valid_ = true;
  enabled_ = false;
  calib_ = false;
  return true;
}

bool Anticog::save() {
  if (calib_ || !valid_ || bins_ == 0) {
    return false;
  }
  Header hdr{};
  hdr.magic = kMagic;
  hdr.bins = bins_;
  hdr.axis = HWCHK_AXIS_CHAR;
  hdr.reserved = 0;
  Preferences prefs;
  if (!prefs.begin(kNs, false)) {
    return false;
  }
  bool ok = prefs.putBytes("hdr", &hdr, sizeof(hdr)) == sizeof(hdr);
  for (uint32_t i = 0; ok && i < 4; ++i) {
    const uint32_t off = i * kChunk;
    if (off >= bins_) {
      break;
    }
    uint32_t n = bins_ - off;
    if (n > kChunk) {
      n = kChunk;
    }
    char key[4] = {'m', (char)('0' + i), 0};
    ok = prefs.putBytes(key, &map_[off], n * sizeof(float)) == n * sizeof(float);
  }
  prefs.end();
  return ok;
}

void Anticog::forget() {
  abort();
  valid_ = false;
  enabled_ = false;
  bins_ = 0;
  Preferences prefs;
  if (prefs.begin(kNs, false)) {
    prefs.remove("hdr");
    prefs.remove("m0");
    prefs.remove("m1");
    prefs.remove("m2");
    prefs.remove("m3");
    prefs.end();
  }
}

bool Anticog::start(uint16_t bins, float angle_rad) {
  if (bins < 16u) {
    bins = 16u;
  }
  if (bins > ACOG_BINS_MAX) {
    bins = (uint16_t)ACOG_BINS_MAX;
  }
  memset(map_, 0, sizeof(map_));
  memset(rev_, 0, sizeof(rev_));
  bins_ = bins;
  index_ = 0;
  phase_ = 0;
  settle_streak_ = 0;
  finalize_idx_ = 0;
  finalize_sum_ = 0.0f;
  finalize_mean_ = 0.0f;
  valid_ = false;
  enabled_ = false;
  calib_ = true;
  const float wrapped = wrapTurns(angle_rad);
  turn_base_ = angle_rad - wrapped;
  hold_ = turn_base_;
  return true;
}

void Anticog::abort() {
  calib_ = false;
  phase_ = 0;
  settle_streak_ = 0;
}

float Anticog::wrapTurns(float angle_rad) const {
  float w = fmodf(angle_rad, _2PI);
  if (w < 0.0f) {
    w += _2PI;
  }
  return w;
}

bool Anticog::tick(float angle_rad, float vel_rad_s, float iq_sp, uint32_t loop_hz) {
  if (!calib_) {
    return false;
  }

  if (phase_ >= 2) {
    const uint32_t end = finalize_idx_ + kFinalizeChunk;
    const uint32_t stop = end > bins_ ? bins_ : end;
    if (phase_ == 2) {
      for (uint32_t j = finalize_idx_; j < stop; ++j) {
        const float v = 0.5f * (map_[j] + rev_[j]);
        map_[j] = v;
        finalize_sum_ += v;
      }
    } else {
      for (uint32_t j = finalize_idx_; j < stop; ++j) {
        map_[j] -= finalize_mean_;
      }
    }
    finalize_idx_ = stop;
    if (finalize_idx_ >= bins_) {
      finalize_idx_ = 0;
      if (phase_ == 2) {
        finalize_mean_ = finalize_sum_ / (float)bins_;
        phase_ = 3;
      } else {
        index_ = 0;
        valid_ = true;
        calib_ = false;
        phase_ = 0;
        return true;
      }
    }
    return false;
  }

  hold_ = turn_base_ + ((float)index_ * _2PI) / (float)bins_;
  const float pos_err = wrapPi(hold_ - angle_rad);
  const bool settled = (fabsf(pos_err) <= ACOG_SETTLE_POS) && (fabsf(vel_rad_s) < ACOG_SETTLE_VEL);
  if (settled) {
    if (settle_streak_ < 60000u) {
      settle_streak_ += 1;
    }
  } else {
    settle_streak_ = 0;
  }
  uint16_t need = (uint16_t)((float)loop_hz * ACOG_SETTLE_S);
  if (need < 8u) {
    need = 8u;
  }
  settle_need_ = need;
  if (settle_streak_ < need) {
    return false;
  }
  settle_streak_ = 0;

  if (phase_ == 0) {
    map_[index_] = iq_sp;
    index_ += 1;
    if (index_ >= bins_) {
      phase_ = 1;
      index_ = (uint16_t)(bins_ - 1u);
    }
  } else {
    rev_[index_] = iq_sp;
    if (index_ == 0) {
      phase_ = 2;
      finalize_idx_ = 0;
      finalize_sum_ = 0.0f;
    } else {
      index_ -= 1;
    }
  }
  return false;
}

float Anticog::feedforward(float angle_rad) const {
  if (!enabled_ || !valid_ || calib_ || bins_ == 0) {
    return 0.0f;
  }
  const float wrapped = wrapTurns(angle_rad);
  const float binf = wrapped * ((float)bins_ / _2PI);
  int i0 = (int)binf;
  if (i0 < 0) {
    i0 = 0;
  }
  if (i0 >= (int)bins_) {
    i0 = (int)bins_ - 1;
  }
  int i1 = i0 + 1;
  if (i1 >= (int)bins_) {
    i1 = 0;
  }
  const float frac = binf - (float)i0;
  return map_[i0] + frac * (map_[i1] - map_[i0]);
}
