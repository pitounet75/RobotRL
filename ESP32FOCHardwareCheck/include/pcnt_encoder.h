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
  int needsSearch() override;

  int64_t count() const;
  /** Multi-turn shaft angle from the 64-bit count. Not wrapped, not clamped. */
  float countAngle() const;
  float cpr() const { return cpr_; }
  bool ok() const { return ok_; }
  bool indexFound() const { return index_found_; }
  uint32_t zEdges() const { return z_edges_; }
  uint32_t jumps() const { return jumps_; }
  uint32_t overflowEvents() const { return overflow_events_; }
  void clearIndex();

  /** Diagnostic: ring buffer of the last kJumpLogCap large count jumps. */
  struct JumpEvent {
    uint32_t seq;
    int32_t delta;
    uint32_t elapsed_us;
    /** Portion of delta explained as N clean kPcntLim wraps. delta -
     * overflow_delta is the residual whose implied velocity triggered this
     * log entry (see PcntEncoder::update()). */
    int32_t overflow_delta;
  };
  static constexpr uint8_t kJumpLogCap = 8;
  uint8_t jumpLogCap() const { return kJumpLogCap; }
  JumpEvent jumpLogAt(uint8_t i) const { return jump_log_[i % kJumpLogCap]; }

 private:
  static void overflowIsr(void *arg);
  static void zIsr();
  int64_t rawCountLocked() const;
  int64_t cprInt() const { return (int64_t)(cpr_ + 0.5f); }
  void angleFromCount(int64_t count, int32_t *rotations, float *shaft) const;

  int pin_a_;
  int pin_b_;
  int pin_z_;
  pcnt_unit_t unit_;
  float cpr_;
  mutable volatile int64_t overflow_;
  volatile int64_t count_at_z_;
  volatile bool index_found_;
  volatile uint32_t z_edges_;
  volatile uint32_t jumps_;
  volatile uint32_t overflow_events_;
  int64_t jump_count_;
  uint32_t last_update_us_;
  JumpEvent jump_log_[kJumpLogCap];
  uint8_t jump_log_idx_;
  bool accept_z_;
  bool ok_;
};
