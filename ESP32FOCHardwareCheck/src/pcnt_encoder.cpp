#include "pcnt_encoder.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "soc/pcnt_struct.h"
#include "soc/soc.h"

namespace {

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
bool s_isr_installed = false;
PcntEncoder *s_z_owner = nullptr;

/* Smaller than INT16_MAX so a missed fold is ~90°, not 180° at CPR 65536. */
constexpr int16_t kPcntLim = 16384;

}  // namespace

PcntEncoder::PcntEncoder(int pin_a, int pin_b, int pin_z, uint32_t ppr, pcnt_unit_t unit)
    : pin_a_(pin_a),
      pin_b_(pin_b),
      pin_z_(pin_z),
      unit_(unit),
      cpr_(4.0f * (float)ppr),
      overflow_(0),
      count_at_z_(0),
      index_found_(false),
      z_edges_(0),
      jumps_(0),
      overflow_events_(0),
      jump_count_(0),
      last_update_us_(0),
      jump_log_{},
      jump_log_idx_(0),
      accept_z_(false),
      ok_(false) {}

void IRAM_ATTR PcntEncoder::overflowIsr(void *arg) {
  auto *self = static_cast<PcntEncoder *>(arg);
  portENTER_CRITICAL_ISR(&s_mux);
  if (PCNT.status_unit[self->unit_].h_lim_lat) {
    self->overflow_ += (int64_t)kPcntLim;
    self->overflow_events_ += 1;
    pcnt_counter_clear(self->unit_);
  } else if (PCNT.status_unit[self->unit_].l_lim_lat) {
    self->overflow_ += (int64_t)(-kPcntLim);
    self->overflow_events_ += 1;
    pcnt_counter_clear(self->unit_);
  }
  PCNT.int_clr.val = BIT(self->unit_);
  portEXIT_CRITICAL_ISR(&s_mux);
}

void IRAM_ATTR PcntEncoder::zIsr() {
  if (s_z_owner == nullptr || !s_z_owner->accept_z_) {
    return;
  }
  portENTER_CRITICAL_ISR(&s_mux);
  const int64_t raw = s_z_owner->rawCountLocked();
  s_z_owner->z_edges_ += 1;
  /* First Z is the origin. Later pulses must not rebase count — that snaps
   * getAngle() by ~π or 2π and the velocity PID slams the shaft. */
  if (!s_z_owner->index_found_) {
    s_z_owner->count_at_z_ = raw;
    s_z_owner->index_found_ = true;
  }
  portEXIT_CRITICAL_ISR(&s_mux);
}

int64_t PcntEncoder::rawCountLocked() const {
  int16_t hw = 0;
  int64_t extra = 0;
  pcnt_get_counter_value(unit_, &hw);
  /* Same race as ESP32Encoder::getCountRaw: ISR may not have run yet. */
  if (PCNT.int_st.val & BIT(unit_)) {
    pcnt_get_counter_value(unit_, &hw);
    if (PCNT.status_unit[unit_].h_lim_lat) {
      extra = (int64_t)kPcntLim;
    } else if (PCNT.status_unit[unit_].l_lim_lat) {
      extra = (int64_t)(-kPcntLim);
    }
  }
  return overflow_ + extra + (int64_t)hw;
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

  pcnt_event_enable(unit_, PCNT_EVT_H_LIM);
  pcnt_event_enable(unit_, PCNT_EVT_L_LIM);
  pcnt_counter_pause(unit_);

  if (!s_isr_installed) {
    if (pcnt_isr_service_install(0) != ESP_OK) {
      ok_ = false;
      return;
    }
    s_isr_installed = true;
  }
  if (pcnt_isr_handler_add(unit_, overflowIsr, this) != ESP_OK) {
    ok_ = false;
    return;
  }

  overflow_ = 0;
  count_at_z_ = 0;
  index_found_ = false;
  z_edges_ = 0;
  accept_z_ = false;
  pcnt_counter_clear(unit_);
  pcnt_intr_enable(unit_);
  pcnt_counter_resume(unit_);

  s_z_owner = this;
  attachInterrupt(digitalPinToInterrupt(pin_z_), zIsr, ENC_Z_EDGE);
  delay(20);
  portENTER_CRITICAL(&s_mux);
  z_edges_ = 0;
  index_found_ = false;
  count_at_z_ = 0;
  overflow_ = 0;
  accept_z_ = true;
  portEXIT_CRITICAL(&s_mux);

  ok_ = true;
  min_elapsed_time = 0.002f;
  Sensor::init();
}

void PcntEncoder::clearIndex() {
  portENTER_CRITICAL(&s_mux);
  index_found_ = false;
  count_at_z_ = 0;
  portEXIT_CRITICAL(&s_mux);
}

int PcntEncoder::needsSearch() { return index_found_ ? 0 : 1; }

int64_t PcntEncoder::count() const {
  int64_t raw;
  int64_t z0 = 0;
  bool have_z = false;
  portENTER_CRITICAL(&s_mux);
  raw = rawCountLocked();
  have_z = index_found_;
  z0 = count_at_z_;
  portEXIT_CRITICAL(&s_mux);
  return have_z ? (raw - z0) : raw;
}

/* SimpleFOC 2.3.3: getSensorAngle must be [0, 2π). Negative is treated as
 * an error and update() returns without touching getAngle() — that froze ang
 * at the last positive value (~5×2π) once cnt went negative. */
void PcntEncoder::angleFromCount(int64_t count, int32_t *rotations, float *shaft) const {
  const int64_t icpr = cprInt();
  int64_t rot = count / icpr;
  int64_t rem = count % icpr;
  if (rem < 0) {
    rem += icpr;
    rot -= 1;
  }
  *rotations = (int32_t)rot;
  *shaft = (float)rem * (_2PI / (float)icpr);
}

void PcntEncoder::update() {
  const uint32_t now_us = micros();
  const uint32_t elapsed_us = now_us - last_update_us_;
  const int64_t c = count();
  const int64_t dc = c - jump_count_;
  /* Speed-independent glitch test: explain away up to a few clean kPcntLim
   * wraps (real h/l_lim crossings), then ask whether what's LEFT would
   * require an instantaneous velocity no real shaft can reach. A true
   * mechanical hiccup unfolds over many 125us loops (each with a small,
   * physically plausible per-loop delta) — it never shows up as one huge
   * delta in a single loop. So if the residual implies an impossible
   * velocity, it's a counting artifact, not real motion, regardless of
   * what speed we're testing at. */
  const int64_t n_wraps = (dc >= 0) ? (dc + kPcntLim / 2) / kPcntLim
                                     : -((-dc + kPcntLim / 2) / kPcntLim);
  const int64_t wrap_explained = n_wraps * (int64_t)kPcntLim;
  const int64_t residual = dc - wrap_explained;
  constexpr float kMaxPlausibleVelRadS = 6.0f * FOC_VEL_LIMIT;
  const float implied_vel_rad_s =
      ((float)residual / cpr_) * _2PI / ((float)elapsed_us * 1e-6f);
  if (fabsf(implied_vel_rad_s) > kMaxPlausibleVelRadS) {
    jumps_ += 1;
    JumpEvent &ev = jump_log_[jump_log_idx_ % kJumpLogCap];
    ev.seq = jumps_;
    ev.delta = (int32_t)dc;
    ev.elapsed_us = elapsed_us;
    ev.overflow_delta = (int32_t)wrap_explained;
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
