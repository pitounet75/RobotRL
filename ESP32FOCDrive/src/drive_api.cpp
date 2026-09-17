#include "drive_api.h"

#include <Arduino.h>

#include "axis.h"
#include "board.h"
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

bool driveSetVelocity(uint8_t axis, float rad_s, uint32_t timeout_ms) {
  if (axis >= AXIS_COUNT || !axes[axis].present) {
    return false;
  }
  Axis &ax = axes[axis];
  /* Stamping is axisSetVelocity()'s job now, not this layer's: this layer
   * cannot tell which internal path (fast/locked vs. slow/mode-change) the
   * call took, and that distinction is exactly what decides where the
   * stamp has to happen to stay race-free (see axisStampCmd() in axis.h). */
  return axisSetVelocity(ax, (float)ax.cmd_sign * rad_s, timeout_ms);
}

bool driveSetTorque(uint8_t axis, float volts, uint32_t timeout_ms) {
  if (axis >= AXIS_COUNT || !axes[axis].present) {
    return false;
  }
  Axis &ax = axes[axis];
  return axisSetTorque(ax, (float)ax.cmd_sign * volts, timeout_ms);
}

bool driveSetOpenloop(uint8_t axis, float rad_s, uint32_t timeout_ms) {
  if (axis >= AXIS_COUNT || !axes[axis].present) {
    return false;
  }
  Axis &ax = axes[axis];
  float v = (float)ax.cmd_sign * rad_s;
  if (v > FOC_VEL_LIMIT) {
    v = FOC_VEL_LIMIT;
  } else if (v < -FOC_VEL_LIMIT) {
    v = -FOC_VEL_LIMIT;
  }
  /* Fast path: test, target write and stamp all happen under the same
   * power lock axisArm()/axisDisarm() use, so the three are indivisible --
   * same pattern as axisSetVelocity()/axisSetTorque() (axis.cpp), and for
   * the same reason: stamping after releasing the lock would leave a
   * window where a stale deadline from a PREVIOUS call could already be
   * due and the failsafe would park the axis under this freshly-accepted
   * setpoint. See axisStampCmd() in axis.h. */
  boardMotorPowerLock();
  const bool fast_path = ax.armed && ax.mode == Mode::Openloop;
  if (fast_path) {
    ax.motor.target = v; /* live setpoint: do NOT re-arm/resync, same fast
                           * path as axisSetVelocity()/axisSetTorque(). */
    axisStampCmd(ax, timeout_ms);
  }
  boardMotorPowerUnlock();
  if (fast_path) {
    return true;
  }
  /* Slow path (mode change / first arm): axisSetMode() below can block up
   * to 50 ms in focSyncWithTask(). Zero cmd_timeout_ms FIRST -- disabling
   * the failsafe for the whole transition, the same trick driveStop() uses
   * -- so a deadline left over from ax's PREVIOUS mode cannot expire
   * mid-transition and have the failsafe overwrite the mode this call is
   * about to set. Stamped for real, with the caller's timeout_ms, only once
   * the transition has fully completed below. */
  ax.cmd_timeout_ms = 0;
  ax.motor.controller = MotionControlType::velocity_openloop;
  ax.motor.target = v;
  /* Settle the mode (and let the task observe it) before arming: mirrors
   * driveStop()'s ordering below, keeping mode and power state changes from
   * racing the task on the other core. Only reached once per mode entry,
   * same as the other two setters. */
  axisSetMode(ax, Mode::Openloop);
  axisArm(ax);
  axisStampCmd(ax, timeout_ms);
  return true;
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
