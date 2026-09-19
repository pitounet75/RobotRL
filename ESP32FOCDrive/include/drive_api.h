#pragma once

#include <stdint.h>

struct Axis; /* full definition in axis.h; only needed by reference here. */

/**
 * Boundary between the CLI (balance loop later) and the FOC task. All three
 * run on core 1 today -- the FOC task preempts at priority 20, above the
 * balance loop's planned 19 and the CLI's loop() at 1 -- so this is a task
 * boundary, not a core boundary, and the protections it relies on have to
 * hold under preemption alone, whether or not the FOC task ever moves back
 * to core 0. Commands are in the ROBOT frame: cmd_sign is applied here, once.
 *
 * ============================================================================
 * SINGLE WRITER PER AXIS, ENFORCED BY CONVENTION ONLY -- READ BEFORE ADDING
 * A SECOND CALLER ON CORE 1.
 *
 * The slow path of driveSetVelocity()/driveSetTorque()/driveSetOpenloop()
 * (a mode change or a first arm, as opposed to the fast in-lock target
 * rewrite) is NOT atomic end to end: it writes ax.motor.controller/
 * torque_controller/target outside any lock, then calls axisSetMode(), which
 * SLEEPS in focSyncWithTask() for up to 50 ms waiting for the FOC task to
 * observe the change. Nothing stops a second core-1 caller from entering the
 * same slow path on the same axis while the first is asleep in there. The
 * design intent is a single prioritized control task plus the CLI, both on
 * core 1 -- but until that control task exists, THE CLI MUST BE TREATED AS
 * THE ONLY WRITER for every axis it can reach. Two interleaved callers can
 * leave an axis in Mode::Torque with motor.controller left at `velocity` (or
 * the reverse) -- a velocity setpoint silently interpreted as volts, or a
 * torque setpoint silently interpreted as rad/s. Do not add a second core-1
 * writer without first making this path atomic (or serializing the two
 * callers some other way); documenting the requirement here, rather than
 * restructuring the slow path, is a deliberate choice -- do not read the
 * absence of a lock as evidence the race does not matter.
 * ============================================================================
 *
 * In steady state a setpoint call is a bound, a brief boardMotorPowerLock()
 * critical section (test, target store, timestamp stamp -- no printf, no
 * focSyncWithTask(), nothing slow) and nothing else: at 400 Hz on two axes
 * that is under 0.1% of a core. focSyncWithTask() only runs on arm and
 * disarm -- and, for these three setters, only on the FIRST call after a
 * mode change or an axis coming out of another mode: once armed in the
 * matching mode, repeat calls take the same short-locked fast path
 * axisSetVelocity()/axisSetTorque() (axis.cpp) and driveSetOpenloop() below
 * do -- see axisStampCmd() in axis.h for why the lock now also has to cover
 * the timestamp stamp, not just the armed test and the target write.
 *
 * Return true if the setpoint was actually applied, false if it was refused
 * (bad axis index, axis absent from this build, or -- driveSetVelocity()/
 * driveSetTorque() only -- not calibrated yet). A caller that does not check
 * this cannot tell "commanded" from "refused"; this is why these three
 * return bool instead of the void the task-7 brief originally specified.
 * driveStop() cannot be refused (parking an already-idle axis is a no-op),
 * so it stays void.
 */
[[nodiscard]] bool driveSetVelocity(uint8_t axis, float rad_s, uint32_t timeout_ms);
[[nodiscard]] bool driveSetTorque(uint8_t axis, float volts, uint32_t timeout_ms);
[[nodiscard]] bool driveSetOpenloop(uint8_t axis, float rad_s, uint32_t timeout_ms);
void driveStop(uint8_t axis);

struct AxisState {
  float angle;
  float velocity;
  float uq;
  bool armed;
  bool calibrated;
};
/**
 * Consistent snapshot published once per FOC iteration. Returns false (out
 * untouched) if a torn read is detected after a few retries, or if axis is
 * out of range or absent from this build.
 *
 * FROZEN, NOT STALE-MARKED, WHILE THE CLI OWNS THE AXIS: drivePublishState()
 * is only ever called by the core-1 FOC task's per-axis loop, which skips an
 * axis entirely for as long as ax.owner is Owner::Cli (axisTakeOwnership(),
 * axis.h) -- e.g. for the whole duration of `cal`, up to several seconds on
 * a dual-axis run, during which axisCal() arms the axis and spins it
 * directly, never through drivePublishState(). driveGetState() has no way
 * to signal any of this: it keeps returning true with the LAST snapshot
 * published before ownership changed hands -- angle, velocity AND armed all
 * silently frozen at their pre-handover values, even though the real axis
 * is, for example, actually armed and spinning open-loop under `cal` while
 * the published snapshot still claims armed=false. There is currently no
 * field in AxisState that reveals a CLI-owned axis; a future 400 Hz balance
 * loop reading this axis must not trust a steady reading here as proof the
 * wheel is actually idle, and has no way to detect the gap from this API
 * alone.
 */
bool driveGetState(uint8_t axis, AxisState *out);

/**
 * Publishes one axis' snapshot under the seqlock. Called by the core-1 FOC
 * task only, once per axis per iteration, for exactly the reason
 * driveGetState()'s callers must never touch the encoder themselves: this is
 * the one legitimate reader, because it *is* the FOC task.
 */
void drivePublishState(const Axis &ax, uint8_t i);
