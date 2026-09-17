#include "drive_api.h"

#include <Arduino.h>

#include "axis.h"
#include "config.h"

namespace {
/* One seqlock slot per axis. seq is volatile: written by the core-0 FOC
 * task, read by core 1 (CLI today, the balance loop later) -- the same
 * cross-core contract axis.h documents for Axis::mode/owner. The payload
 * (st) does not need its own volatile: the __sync_synchronize() barriers
 * around every write/read of it are full compiler+hardware barriers, so
 * nothing can be hoisted across them regardless. */
struct PubSlot {
  volatile uint32_t seq;
  AxisState st;
};
PubSlot s_pub[AXIS_COUNT];
}  // namespace

void driveSetVelocity(uint8_t axis, float rad_s, uint32_t timeout_ms) {
  if (axis >= AXIS_COUNT || !axes[axis].present) {
    return;
  }
  Axis &ax = axes[axis];
  ax.cmd_timeout_ms = timeout_ms;
  ax.last_cmd_ms = millis();
  (void)axisSetVelocity(ax, (float)ax.cmd_sign * rad_s);
}

void driveSetTorque(uint8_t axis, float volts, uint32_t timeout_ms) {
  if (axis >= AXIS_COUNT || !axes[axis].present) {
    return;
  }
  Axis &ax = axes[axis];
  ax.cmd_timeout_ms = timeout_ms;
  ax.last_cmd_ms = millis();
  (void)axisSetTorque(ax, (float)ax.cmd_sign * volts);
}

void driveSetOpenloop(uint8_t axis, float rad_s, uint32_t timeout_ms) {
  if (axis >= AXIS_COUNT || !axes[axis].present) {
    return;
  }
  Axis &ax = axes[axis];
  float v = (float)ax.cmd_sign * rad_s;
  if (v > FOC_VEL_LIMIT) {
    v = FOC_VEL_LIMIT;
  } else if (v < -FOC_VEL_LIMIT) {
    v = -FOC_VEL_LIMIT;
  }
  ax.cmd_timeout_ms = timeout_ms;
  ax.last_cmd_ms = millis();
  ax.motor.controller = MotionControlType::velocity_openloop;
  ax.motor.target = v;
  /* Settle the mode (and let the task observe it) before arming: mirrors
   * driveStop()'s ordering below, keeping mode and power state changes from
   * racing the task on the other core. */
  axisSetMode(ax, Mode::Openloop);
  axisArm(ax);
}

void driveStop(uint8_t axis) {
  if (axis >= AXIS_COUNT || !axes[axis].present) {
    return;
  }
  Axis &ax = axes[axis];
  ax.motor.target = 0.0f;
  ax.cmd_timeout_ms = 0;
  /* Mode -> Off and synced BEFORE disarm: otherwise the task could still be
   * mid-loopFOC()/move() when M_EN drops, and with M_EN shared between both
   * gate drivers that is not harmless. */
  axisSetMode(ax, Mode::Off);
  axisDisarm(ax);
}

/* Publisher, FOC task only. */
void drivePublishState(const Axis &ax, uint8_t i) {
  s_pub[i].seq += 1; /* odd: write in progress */
  __sync_synchronize();
  s_pub[i].st.angle = ax.encoder.countAngle();
  s_pub[i].st.velocity = ax.motor.shaft_velocity;
  s_pub[i].st.uq = ax.motor.voltage.q;
  s_pub[i].st.armed = ax.armed;
  s_pub[i].st.calibrated = ax.calibrated;
  __sync_synchronize();
  s_pub[i].seq += 1; /* even: readable */
}

bool driveGetState(uint8_t axis, AxisState *out) {
  if (axis >= AXIS_COUNT || !axes[axis].present) {
    return false;
  }
  for (int tries = 0; tries < 4; ++tries) {
    const uint32_t s0 = s_pub[axis].seq;
    if ((s0 & 1u) != 0u) {
      continue;
    }
    __sync_synchronize();
    *out = s_pub[axis].st;
    __sync_synchronize();
    if (s_pub[axis].seq == s0) {
      return true;
    }
  }
  return false;
}
