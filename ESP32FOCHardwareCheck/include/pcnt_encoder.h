#pragma once

#include <SimpleFOC.h>
#include <stdint.h>

#include "driver/pcnt.h"

/** MT6835 ABZ via ESP32 PCNT full-quad (CPR = 4 * PPR). Z is a GPIO index ISR. */
class PcntEncoder : public Sensor {
 public:
  PcntEncoder(int pin_a, int pin_b, int pin_z, uint32_t ppr, pcnt_unit_t unit);

  void init() override;
  void update() override;
  float getSensorAngle() override;
  float getMechanicalAngle() override;
  float getAngle() override;
  double getPreciseAngle() override;
  int32_t getFullRotations() override;
  /** Computed straight from count()/micros(), not the base class's
   * full_rotations wrap-detection heuristic — see getAngle()'s same
   * reasoning. That heuristic depends on update() being called at a
   * steady rate; it isn't (core0 in Velocity/Torque/Accal, core1 in
   * Enc/Idle), and a single misfire at a mode-switch boundary leaves
   * full_rotations permanently wrong, so getVelocity() silently reports
   * whatever that error implies forever after — while getAngle() stays
   * correct throughout, since it never uses full_rotations. */
  float getVelocity() override;
  int needsSearch() override;

  int64_t count() const;
  /** Multi-turn shaft angle from the 64-bit count. Not wrapped, not clamped. */
  float countAngle() const;
  float cpr() const { return cpr_; }
  bool ok() const { return ok_; }
  bool indexFound() const { return index_found_; }
  uint32_t zEdges() const { return z_edges_; }
  uint32_t jumps() const { return jumps_; }
  /** HW lim resets seen by the poll unwrap (not an ISR). */
  uint32_t overflowEvents() const { return wrap_events_; }
  void clearIndex();

  /** Diagnostic: ring buffer of the last kJumpLogCap large count jumps. */
  struct JumpEvent {
    uint32_t seq;
    int32_t delta;
    uint32_t elapsed_us;
    int32_t overflow_delta;
  };
  static constexpr uint8_t kJumpLogCap = 8;
  uint8_t jumpLogCap() const { return kJumpLogCap; }
  JumpEvent jumpLogAt(uint8_t i) const { return jump_log_[i % kJumpLogCap]; }

 private:
  static void zIsr();
  int64_t foldLocked() const;
  int64_t cprInt() const { return (int64_t)(cpr_ + 0.5f); }
  void angleFromCount(int64_t count, int32_t *rotations, float *shaft) const;

  int pin_a_;
  int pin_b_;
  int pin_z_;
  pcnt_unit_t unit_;
  float cpr_;
  mutable volatile int64_t accum_;
  mutable volatile int16_t hw_prev_;
  volatile int64_t count_at_z_;
  volatile bool index_found_;
  volatile uint32_t z_edges_;
  volatile uint32_t jumps_;
  mutable volatile uint32_t wrap_events_;
  int64_t jump_count_;
  uint32_t last_update_us_;
  int64_t vel_count_prev_;
  uint32_t vel_count_prev_us_;
  JumpEvent jump_log_[kJumpLogCap];
  uint8_t jump_log_idx_;
  bool accept_z_;
  bool ok_;
};
