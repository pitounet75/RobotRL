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
