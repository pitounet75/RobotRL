#pragma once

#include <stdint.h>

/**
 * A command is stale once timeout_ms has elapsed since last_ms.
 * timeout_ms == 0 disables the check: that is what the CLI passes, so a
 * bench session is never cut off. A control loop passes ~10 ms (4 periods at
 * 400 Hz), so a hung core 1 stops the wheels.
 * Unsigned arithmetic carries the millis() wrap; never compare timestamps.
 */
inline bool failsafeExpired(uint32_t now_ms, uint32_t last_ms, uint32_t timeout_ms) {
  if (timeout_ms == 0u) {
    return false;
  }
  return (uint32_t)(now_ms - last_ms) > timeout_ms;
}
