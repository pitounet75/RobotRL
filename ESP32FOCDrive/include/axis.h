#pragma once

#include <SimpleFOC.h>
#include <stdint.h>

#include "config.h"
#include "pcnt_encoder.h"

/**
 * Per-axis state shared between core 1 (CLI) and the core-0 FOC task.
 * Ownership is the guard rail: the task only touches loopFOC()/move() for an
 * axis whose owner is Task. Nothing in this build ever sets owner to Cli yet
 * (that lands with zero-search / calibration in a later task) but the field
 * exists now so the struct layout does not change under those tasks.
 */
enum class Mode : uint8_t { Off, Openloop, Velocity, Torque };
enum class Owner : uint8_t { Task, Cli };

struct Axis {
  char name;
  int idx;
  int8_t cmd_sign;
  bool present;
  PcntEncoder encoder;
  BLDCDriver3PWM driver;
  BLDCMotor motor;
  /* Written by core 1, read every iteration by the core-0 FOC task: the
   * volatile qualifier is the contract, not a formality. It happens to work
   * without it today because the task's reads are separated by non-inlinable
   * calls into other translation units (no LTO), but nothing stops a future
   * change from letting the compiler hoist a stale copy — owner especially,
   * since it is exactly the field a future task will flip across cores. */
  volatile Mode mode;
  volatile Owner owner;
  /* Written by core 1 (drive_api.cpp), read every iteration by the core-0
   * FOC task via failsafeExpired() (foc_task.cpp): volatile for the same
   * reason as mode/owner above. failsafeExpired() is `inline` in a header,
   * so its two loads inline straight into the task's loop body -- without
   * volatile, nothing stops a sufficiently aggressive build from hoisting
   * cmd_timeout_ms out of the loop and silently disabling the failsafe. */
  volatile uint32_t last_cmd_ms;
  volatile uint32_t cmd_timeout_ms;
  bool calibrated;
  float voltage_limit;
  float align_voltage;
  /* Read every iteration by the core-0 FOC task (the failsafe check in
   * foc_task.cpp) and, since the failsafe made axisDisarm() reachable from
   * that task, also read and written from core 1's fast setpoint paths
   * (axisSetVelocity()/axisSetTorque(), drive_api.cpp's driveSetOpenloop()):
   * volatile for the same reason as mode/owner/last_cmd_ms/cmd_timeout_ms
   * above -- it is inter-core state now, not just a formality against
   * compiler hoisting. Being volatile does not by itself make a fast path's
   * `if (ax.armed && ...)` test-then-write atomic with the other core's
   * disarm; those call sites additionally take boardMotorPowerLock() around
   * the test and the write for that reason (see axisSetVelocity()). */
  volatile bool armed;

  explicit Axis(int i);
};

/* AXIS_COUNT is a fixed compile-time constant (2), independent of
 * FOC_AXIS_MASK: axes not in the mask are still constructed (cheap, no HW
 * side effects until init()) but stay present=false and untouched. */
extern Axis axes[AXIS_COUNT];

/** Builds the driver/encoder/motor for every axis in FOC_AXIS_MASK. */
void axisInitAll();
/** Pushes ax.voltage_limit into the driver, motor and velocity PID output
 * limit, and re-clamps ax.align_voltage to it (pushing the result into
 * motor.voltage_sensor_align) so a `limit` below the current align voltage
 * cannot leave calibration driving more volts than just asked for. Callable
 * any time, including while the axis is armed. */
void axisApplyLimits(Axis &ax);
/** Arms the motor (idempotent) and takes one M_EN power reference. Guarded
 * by ax.armed, not motor.enabled: SimpleFOC's initFOC() can call disable()
 * on the motor itself on failure, which would flip motor.enabled without
 * going through axisDisarm() and desync the shared M_EN refcount if that
 * were the guard instead. */
void axisArm(Axis &ax);
/** Drops the M_EN power reference (idempotent) and disables the motor.
 * See axisArm() for why ax.armed, not motor.enabled, is the guard. */
void axisDisarm(Axis &ax);
/** Writes ax.mode, then blocks until the FOC task has observed it. */
void axisSetMode(Axis &ax, Mode m);

/**
 * Ownership handover. The FOC task only calls loopFOC()/move() for an axis
 * whose owner is Owner::Task (see foc_task.cpp); while owner is Owner::Cli
 * the task skips that axis entirely, encoder included.
 *
 * That last part is the contract a caller taking ownership must honor:
 * while an axis is owned by core 1, ITS OWNER is responsible for reading the
 * encoder at least once every 8192 counts, or the software unwrap of the
 * 16-bit hardware count silently loses turns (see PcntEncoder). The
 * calibration loop that will be the first Owner::Cli caller (task 5)
 * satisfies this by calling loopFOC() in a tight loop, which reads the
 * sensor on every pass — no separate encoder.update() is needed on top of
 * that. A caller that does not poll the sensor at least that often must not
 * take ownership.
 *
 * Both functions write ax.owner and call focSyncWithTask() so no caller has
 * to remember to pair the two: axisTakeOwnership() only returns true once
 * the task has observed the axis is no longer its to touch; axisReleaseOwnership
 * blocks the same way but does not report failure (nothing is still writing
 * ax once released, so a missed observation there is not a safety issue,
 * only a delay). axisCal()/axisZSearch()/axisForgetCal() (task 5) are their
 * first users.
 *
 * axisTakeOwnership() can return false: focSyncWithTask() is bounded to
 * 50 ms so a starved or stopped FOC task never blocks the CLI forever, but
 * that also means the caller cannot assume the task has actually stopped
 * touching the axis. A caller that goes on to drive the motor directly
 * (loopFOC()/move()/setPhaseVoltage()) after a false return would then race
 * the FOC task on the same axis — every caller here checks the return value
 * and aborts, arming nothing, if it is false.
 */
[[nodiscard]] bool axisTakeOwnership(Axis &ax);
void axisReleaseOwnership(Axis &ax);

/**
 * Per-axis electrical calibration, ported from
 * ESP32FOCHardwareCheck/src/main.cpp with every current-sense step removed
 * (this firmware has no current sense: torque is voltage-only).
 *
 * axisCal() takes ownership of ax for its whole duration and drives it
 * directly (loopFOC()/move()/setPhaseVoltage()), never through the core-0
 * FOC task: index search, open-loop direction detection with a sanity
 * check against the commanded open-loop speed, a gently ramped electrical
 * zero capture, then initFOC(). Leaves ax idle (mode Off, disarmed) and
 * ownership released either way; sets ax.calibrated on success.
 */
bool axisCal(Axis &ax);
/**
 * Open-loop index (Z) search. With park=false, the caller already owns ax
 * (axisCal()'s use) and the axis is left armed/owned on return either way.
 * With park=true (the standalone CLI `zsearch`), axisZSearch() manages its
 * own ownership: on success it also tries to reload a saved NVS zero via
 * axisLoadCal() and re-run initFOC(), then idles and releases ax.
 */
bool axisZSearch(Axis &ax, bool park);
/** Persists the axis' current electrical zero/direction/pole pairs to NVS
 * (namespace "drive", key "cal0"/"cal1"), guarded by ax.calibrated. */
bool axisSaveCal(Axis &ax);
/** Clears the stored NVS record and the axis' in-RAM electrical zero. */
void axisForgetCal(Axis &ax);
/** Loads a stored NVS record and applies it to ax.motor if it validates
 * against calRecordValid() for this axis' name and the live ENC_PPR, AND
 * its pole_pairs matches the live FOC_POLE_PAIRS — calRecordValid()'s
 * signature is fixed by the record format itself and only knows about
 * axis/ENC_PPR, so the pole_pairs guard (symmetric with the ENC_PPR one:
 * a firmware rebuilt for a different motor must not silently keep applying
 * the old pole count) lives here instead. Does not run initFOC(). */
bool axisLoadCal(Axis &ax);
/** Prints "need cal" and returns false unless ax is calibrated and its
 * encoder currently holds an index. */
bool axisRequireCal(Axis &ax);

/**
 * Stamps ax.last_cmd_ms then ax.cmd_timeout_ms, in that order, never
 * reversed: the core-0 FOC task's failsafe check (foc_task.cpp) can run
 * between the two stores at any time, with no lock on this specific pair.
 * last_cmd_ms-first means that window can only ever push the deadline
 * (last_cmd_ms + cmd_timeout_ms) LATER than intended, never earlier --
 * reversed, the task could observe a fresh cmd_timeout_ms next to a stale
 * (or zero, at boot) last_cmd_ms and immediately fail a command that had
 * just been accepted.
 *
 * That ordering only rules out the race between these two stores
 * themselves. Where the caller calls this from matters just as much, which
 * is why every caller (axisSetVelocity()/axisSetTorque() below,
 * driveSetOpenloop() in drive_api.cpp) does one of:
 *  - call it from inside the same boardMotorPowerLock() critical section as
 *    the fast-path armed-test and target write, so test+write+stamp become
 *    one indivisible step and nothing can observe "armed, with the OLD
 *    deadline" in the gap; or
 *  - zero cmd_timeout_ms up front, before a slow path's mode change --
 *    disabling the failsafe for the whole transition, the same trick
 *    driveStop() uses -- then call this only once axisSetMode()+axisArm()
 *    (which can block up to 50 ms in focSyncWithTask()) have completed.
 * Calling this before either of those (stamping first, ahead of the
 * delegation) leaves a real window on both paths: on a fast path, between
 * releasing the lock and stamping, a stale deadline from a PREVIOUS call
 * can already be due and the failsafe parks the axis under a
 * freshly-accepted setpoint; on a slow path, the up-to-50 ms sync leaves
 * the axis armed under the OLD deadline while the NEW mode is still being
 * written, so the failsafe can fire mid-transition and overwrite the mode
 * the caller just set.
 */
void axisStampCmd(Axis &ax, uint32_t timeout_ms);

/**
 * Closed-loop velocity setpoint. The PID output is VOLTS, not amps: this
 * firmware has no current sense anywhere, so PID_velocity.limit is the same
 * ax.voltage_limit axisApplyLimits() already pushes everywhere else.
 *
 * Refuses (false, nothing changed, no stamp) until axisRequireCal() passes.
 * Rewrites motor.target in place when ax is already armed in Mode::Velocity
 * -- SimpleFOC's enable() always does setPwm(0,0,0) and resets the PIDs, so
 * re-arming on every command would stall the shaft before each new setpoint
 * took effect; this fast path calls axisStampCmd() from inside the same
 * lock as the armed test and the target write (see axisStampCmd()).
 * Otherwise (first use, or coming from another mode) it poses
 * motor.controller/torque_controller itself rather than trusting whatever
 * calibration or a previous mode left behind, deliberately WITHOUT a
 * priming encoder read (see the comment at the call site for why), zeroes
 * cmd_timeout_ms before touching mode/controller so the failsafe cannot
 * fire mid-transition, sets Mode::Velocity, arms, then stamps for real with
 * timeout_ms.
 */
[[nodiscard]] bool axisSetVelocity(Axis &ax, float rad_s, uint32_t timeout_ms);
/**
 * Same contract as axisSetVelocity(), for Mode::Torque /
 * MotionControlType::torque. The target is volts, clamped to
 * +-ax.voltage_limit: there is no current sense, so torque is voltage-only.
 */
[[nodiscard]] bool axisSetTorque(Axis &ax, float volts, uint32_t timeout_ms);
/**
 * Display-only estimated phase current, I_est = (Uq - BEMF) / R, computed
 * alongside SimpleFOC from FOC_PHASE_R/FOC_KV. Never fed back into the
 * motor: setting motor.phase_resistance would also flip the sign convention
 * SimpleFOC applies to the velocity PID gains, making the loop reason in
 * unmeasured amps instead of the volts it actually commands.
 */
float axisCurrentEstimate(const Axis &ax);
