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
  uint32_t last_cmd_ms;
  uint32_t cmd_timeout_ms;
  bool calibrated;
  float voltage_limit;
  float align_voltage;
  bool armed;

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
/** Arms the motor (idempotent) and takes one M_EN power reference. */
void axisArm(Axis &ax);
/** Drops the M_EN power reference (idempotent) and disables the motor. */
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
 * to remember to pair the two: axisTakeOwnership() only returns once the
 * task has observed the axis is no longer its to touch; axisReleaseOwnership
 * only returns once the task is guaranteed to pick the axis back up on its
 * next iteration. axisCal()/axisZSearch()/axisForgetCal() (task 5) are their
 * first users.
 */
void axisTakeOwnership(Axis &ax);
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
 * against calRecordValid() for this axis' name and the live ENC_PPR. Does
 * not run initFOC(). */
bool axisLoadCal(Axis &ax);
/** Prints "need cal" and returns false unless ax is calibrated and its
 * encoder currently holds an index. */
bool axisRequireCal(Axis &ax);
