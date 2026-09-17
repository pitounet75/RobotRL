#pragma once

#include <stdint.h>

struct Axis; /* full definition in axis.h; only needed by reference here. */

/**
 * Boundary between core 1 (CLI today, balance loop later) and the core 0 FOC
 * task. Commands are in the ROBOT frame: cmd_sign is applied here, once.
 *
 * In steady state a setpoint call is a bound plus two aligned 32-bit stores,
 * with no lock and no handshake: at 400 Hz on two axes that is under 0.1% of
 * a core. focSyncWithTask() only runs on arm and disarm -- and, for these
 * three setters, only on the FIRST call after a mode change or an axis
 * coming out of another mode: once armed in the matching mode, repeat calls
 * take the same lock-free fast path axisSetVelocity()/axisSetTorque() do.
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
/** Consistent snapshot published once per FOC iteration. */
bool driveGetState(uint8_t axis, AxisState *out);

/**
 * Publishes one axis' snapshot under the seqlock. Called by the core-0 FOC
 * task only, once per axis per iteration, for exactly the reason
 * driveGetState()'s callers must never touch the encoder themselves: this is
 * the one legitimate reader, because it *is* the FOC task.
 */
void drivePublishState(const Axis &ax, uint8_t i);
