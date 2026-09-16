#include "pcnt_encoder.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "enc_math.h"

namespace {

/*
 * ESP32 PCNT is 16-bit and *resets to 0* at h_lim/l_lim (not two's-complement
 * wrap). Symmetric ±kPcntLim makes each reset a known step of L counts.
 * Unfold in software on every read — no lim ISR, no pcnt_counter_clear.
 */
constexpr int16_t kPcntLim = 16384;

}  // namespace

PcntEncoder::PcntEncoder(int pin_a, int pin_b, int pin_z, uint32_t ppr, pcnt_unit_t unit,
                          float max_plausible_vel_rad_s, float vel_min_dt_s)
    : pin_a_(pin_a),
      pin_b_(pin_b),
      pin_z_(pin_z),
      unit_(unit),
      cpr_(4.0f * (float)ppr),
      max_plausible_vel_rad_s_(max_plausible_vel_rad_s),
      vel_min_dt_s_(vel_min_dt_s),
      mux_(portMUX_INITIALIZER_UNLOCKED),
      accum_(0),
      hw_prev_(0),
      count_at_z_(0),
      index_found_(false),
      z_edges_(0),
      jumps_(0),
      wrap_events_(0),
      jump_count_(0),
      last_update_us_(0),
      vel_count_prev_(0),
      vel_count_prev_us_(0),
      jump_log_{},
      jump_log_idx_(0),
      accept_z_(false),
      ok_(false) {}

int64_t IRAM_ATTR PcntEncoder::foldLocked() const {
  int16_t hw = 0;
  pcnt_get_counter_value(unit_, &hw);
  const int32_t d = encFoldDelta(hw_prev_, hw, kPcntLim);
  if (d != (int32_t)hw - (int32_t)hw_prev_) {
    wrap_events_ += 1;
  }
  hw_prev_ = hw;
  accum_ += (int64_t)d;
  return accum_;
}

void IRAM_ATTR PcntEncoder::zIsr(void *arg) {
  PcntEncoder *self = static_cast<PcntEncoder *>(arg);
  if (!self->accept_z_) {
    return;
  }
  portENTER_CRITICAL_ISR(&self->mux_);
  const int64_t raw = self->foldLocked();
  self->z_edges_ += 1;
  /* First Z is the origin. Later pulses must not rebase count — that snaps
   * getAngle() by ~π or 2π and the velocity PID slams the shaft. */
  if (!self->index_found_) {
    self->count_at_z_ = raw;
    self->index_found_ = true;
  }
  portEXIT_CRITICAL_ISR(&self->mux_);
}

void PcntEncoder::init() {
  pinMode(pin_a_, INPUT);
  pinMode(pin_b_, INPUT);
  pinMode(pin_z_, INPUT_PULLDOWN);

  /* Full-quad: same channel map as ESP32Encoder attachFullQuad (legacy PCNT). */
  pcnt_config_t cfg{};
  cfg.pulse_gpio_num = pin_a_;
  cfg.ctrl_gpio_num = pin_b_;
  cfg.unit = unit_;
  cfg.channel = PCNT_CHANNEL_0;
  cfg.pos_mode = PCNT_COUNT_DEC;
  cfg.neg_mode = PCNT_COUNT_INC;
  cfg.lctrl_mode = PCNT_MODE_KEEP;
  cfg.hctrl_mode = PCNT_MODE_REVERSE;
  cfg.counter_h_lim = kPcntLim;
  cfg.counter_l_lim = static_cast<int16_t>(-kPcntLim);
  if (pcnt_unit_config(&cfg) != ESP_OK) {
    ok_ = false;
    return;
  }

  cfg.pulse_gpio_num = pin_b_;
  cfg.ctrl_gpio_num = pin_a_;
  cfg.channel = PCNT_CHANNEL_1;
  cfg.pos_mode = PCNT_COUNT_DEC;
  cfg.neg_mode = PCNT_COUNT_INC;
  cfg.lctrl_mode = PCNT_MODE_REVERSE;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  if (pcnt_unit_config(&cfg) != ESP_OK) {
    ok_ = false;
    return;
  }

  uint16_t filt = (uint16_t)ENC_PCNT_FILTER;
  if (filt > 1023u) {
    filt = 1023u;
  }
  if (filt == 0u) {
    pcnt_filter_disable(unit_);
  } else {
    pcnt_set_filter_value(unit_, filt);
    pcnt_filter_enable(unit_);
  }

  /* Lim still resets HW to 0 (ESP32). Do not IRQ or clear — foldLocked does. */
  pcnt_event_disable(unit_, PCNT_EVT_H_LIM);
  pcnt_event_disable(unit_, PCNT_EVT_L_LIM);
  pcnt_counter_pause(unit_);
  pcnt_intr_disable(unit_);

  count_at_z_ = 0;
  index_found_ = false;
  z_edges_ = 0;
  accept_z_ = false;
  wrap_events_ = 0;
  pcnt_counter_clear(unit_);
  int16_t hw0 = 0;
  pcnt_get_counter_value(unit_, &hw0);
  hw_prev_ = hw0;
  accum_ = 0;
  pcnt_counter_resume(unit_);

  attachInterruptArg(digitalPinToInterrupt(pin_z_), zIsr, this, ENC_Z_EDGE);
  delay(20);
  portENTER_CRITICAL(&mux_);
  z_edges_ = 0;
  index_found_ = false;
  count_at_z_ = 0;
  wrap_events_ = 0;
  pcnt_get_counter_value(unit_, &hw0);
  hw_prev_ = hw0;
  accum_ = 0;
  accept_z_ = true;
  portEXIT_CRITICAL(&mux_);

  ok_ = true;
  min_elapsed_time = vel_min_dt_s_;
  Sensor::init();
}

void PcntEncoder::clearIndex() {
  portENTER_CRITICAL(&mux_);
  index_found_ = false;
  count_at_z_ = 0;
  portEXIT_CRITICAL(&mux_);
}

int PcntEncoder::needsSearch() { return index_found_ ? 0 : 1; }

int64_t PcntEncoder::count() const {
  int64_t raw;
  int64_t z0 = 0;
  bool have_z = false;
  portENTER_CRITICAL(&mux_);
  raw = foldLocked();
  have_z = index_found_;
  z0 = count_at_z_;
  portEXIT_CRITICAL(&mux_);
  return have_z ? (raw - z0) : raw;
}

/* SimpleFOC 2.3.3: getSensorAngle must be [0, 2π). Negative is treated as
 * an error and update() returns without touching getAngle() — that froze ang
 * at the last positive value (~5×2π) once cnt went negative. */
void PcntEncoder::angleFromCount(int64_t count, int32_t *rotations, float *shaft) const {
  encAngleFromCount(count, cprInt(), rotations, shaft);
}

void PcntEncoder::update() {
  const uint32_t now_us = micros();
  const uint32_t elapsed_us = now_us - last_update_us_;
  const int64_t c = count();
  const int64_t dc = c - jump_count_;
  const float implied_vel_rad_s =
      ((float)dc / cpr_) * _2PI / ((float)elapsed_us * 1e-6f);
  if (elapsed_us > 0u && fabsf(implied_vel_rad_s) > max_plausible_vel_rad_s_) {
    jumps_ += 1;
    JumpEvent &ev = jump_log_[jump_log_idx_ % kJumpLogCap];
    ev.seq = jumps_;
    ev.delta = (int32_t)dc;
    ev.elapsed_us = elapsed_us;
    ev.overflow_delta = 0;
    jump_log_idx_ = (uint8_t)(jump_log_idx_ + 1);
  }
  last_update_us_ = now_us;
  jump_count_ = c;
  Sensor::update();
}

float PcntEncoder::getSensorAngle() {
  int32_t rot = 0;
  float shaft = 0.0f;
  angleFromCount(count(), &rot, &shaft);
  return shaft;
}

float PcntEncoder::getMechanicalAngle() {
  int32_t rot = 0;
  float shaft = 0.0f;
  angleFromCount(count(), &rot, &shaft);
  return shaft;
}

float PcntEncoder::countAngle() const {
  return (float)count() * (_2PI / cpr_);
}

float PcntEncoder::getAngle() { return countAngle(); }

double PcntEncoder::getPreciseAngle() {
  return (double)count() * (double)_2PI / (double)cpr_;
}

int32_t PcntEncoder::getFullRotations() {
  int32_t rot = 0;
  float shaft = 0.0f;
  angleFromCount(count(), &rot, &shaft);
  return rot;
}

float PcntEncoder::getVelocity() {
  const uint32_t now_us = micros();
  const float Ts = (float)(now_us - vel_count_prev_us_) * 1e-6f;
  if (Ts < min_elapsed_time) {
    return velocity;
  }
  const int64_t c = count();
  velocity = (float)(c - vel_count_prev_) * (_2PI / cpr_) / Ts;
  vel_count_prev_ = c;
  vel_count_prev_us_ = now_us;
  return velocity;
}
