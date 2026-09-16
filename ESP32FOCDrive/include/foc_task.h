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
 */
void focSyncWithTask();
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
