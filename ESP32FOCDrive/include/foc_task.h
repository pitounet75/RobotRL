#pragma once

#include <stdint.h>

/**
 * Isochronous FOC loop on core 0, driven by a hardware timer (TIMER_GROUP_1,
 * TIMER_0). Started from focTaskStart(); the timer itself is armed from
 * inside the task so its ISR is allocated on core 0.
 */
void focTaskStart();
void focSetHz(uint32_t hz);
uint32_t focHz();
/**
 * Blocks the caller until the FOC task has observed a full loop iteration
 * that starts after this call. Waits for two notification increments, not
 * one, so a write made just before calling this is guaranteed seen by a
 * complete task pass. Bounded to 50 ms so a stopped/paused timer never
 * blocks the CLI.
 *
 * Returns true once that pass is confirmed observed (or when there is
 * nothing to observe: the task has not started yet, or the timer is
 * deliberately paused, in both of which cases the task cannot be mid-loop
 * on any axis either). Returns false on the 50 ms timeout — a starved or
 * stopped timer that never ticked the two passes this call waited for.
 * [[nodiscard]] because a caller relying on the handoff (axisTakeOwnership,
 * in particular) must not silently proceed as if it had been observed;
 * callers that don't care are expected to say so with an explicit
 * `(void)focSyncWithTask();`.
 */
[[nodiscard]] bool focSyncWithTask();
/** Stops the hardware timer (used across an OTA flash write). */
void focPauseTimer();
void focResumeTimer();

struct FocMetrics {
  uint32_t dt_us;
  uint32_t dt_max_us;
  uint32_t late;
  uint32_t hz;
  uint64_t loops;
};
FocMetrics focGetMetrics();
/** Clears dt_max_us and late; leaves dt_us/hz/loops untouched. */
void focResetMetrics();

/**
 * Per-iteration diagnostic capture ("vcap"), one axis at a time. A 1 Hz
 * status line samples an estimate that moves every FOC loop tick at a
 * essentially random phase -- it cannot tell a genuine mechanical
 * oscillation from an artifact of the velocity estimator. This instead
 * grabs one sample straight from inside the core-0 FOC task on every
 * iteration it runs (or every Nth, once decimated -- see vcapArm()), so a
 * captured run can be inspected for exactly that: if `vel` swings while
 * `dcount` (the exact encoder count delta since the previous kept sample)
 * stays smooth, the defect is in the estimate, not the shaft. There is no
 * current sense on this board, so `uq` (the commanded quadrature voltage)
 * stands in for the current channel the ported-from build captured.
 *
 * Arms a capture of up to ms milliseconds on the given axis (0/1 -- out of
 * range clamps to 0), default 300 ms from the CLI. A request that would
 * need more than kVCapMax samples is decimated so it still fits: only every
 * Nth loop iteration is kept, and dcount for a kept sample then spans N
 * iterations instead of one -- still an exact count, never a filtered one.
 * Safe to call again before a previous capture finishes; that just re-arms
 * it. Prints its own confirmation line (samples/decim/coverage).
 */
void vcapArm(uint8_t axis, uint32_t ms);
/** True once the armed capture has filled its target sample count (false
 * before anything has ever been armed, or while still capturing). */
bool vcapDone();
/** Prints a status line (axis/idx/target/decim/done-or-not), then the CSV
 * `idx,dcount,vel,uq` for every sample captured so far -- readable mid-
 * capture, not just once vcapDone() is true. */
void vcapDump();
