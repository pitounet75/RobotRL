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
  Mode mode;
  Owner owner;
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
