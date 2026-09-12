#pragma once

#include <stdint.h>

/**
 * ODrive-style anticogging: fwd then rev, average, zero-mean, FF on Iq.
 * Completion is calib==false, never index==0.
 */
class Anticog {
 public:
  bool load();
  bool save();
  void forget();

  bool start(uint16_t bins, float angle_rad);
  void abort();
  /** Call from foc_task only. Returns true when finalize finished. */
  bool tick(float angle_rad, float vel_rad_s, float iq_sp, uint32_t loop_hz);
  float feedforward(float angle_rad) const;
  float holdAngle() const { return hold_; }

  bool calibrating() const { return calib_; }
  bool valid() const { return valid_; }
  bool enabled() const { return enabled_; }
  void setEnabled(bool on) { enabled_ = on && valid_ && !calib_; }
  uint16_t bins() const { return bins_; }
  uint16_t index() const { return index_; }
  uint8_t phase() const { return phase_; }

 private:
  void zeroMeanChunk();
  float wrapTurns(float angle_rad) const;

  float map_[3600];
  float rev_[3600];
  float hold_;
  float turn_base_;
  float finalize_sum_;
  float finalize_mean_;
  uint32_t finalize_idx_;
  uint16_t bins_;
  uint16_t index_;
  uint16_t settle_need_;
  uint16_t settle_streak_;
  uint8_t phase_;
  bool calib_;
  bool valid_;
  bool enabled_;
};
