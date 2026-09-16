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
 * limit. Callable any time, including while the axis is armed. */
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
 * next iteration. Nothing in this task calls either yet — task 5's
 * calibration is their first user.
 */
void axisTakeOwnership(Axis &ax);
void axisReleaseOwnership(Axis &ax);
