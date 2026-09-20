"""Analog-style keyboard stick: hold to ramp, release to coast back to 0."""

from __future__ import annotations

import math

# Seconds to reach max while held, and to fall back to 0 when released.
ACCEL_TIME_S = 0.70
DECEL_TIME_S = 0.22
DEFAULT_MAX_MPS = 0.50
DEFAULT_MAX_YAW_DEG_S = 45.0
SEND_PERIOD_S = 0.05
IDLE_EPS = 1.0e-4


def analog_step(
    value: float,
    held_dir: int,
    max_abs: float,
    dt_s: float,
    accel_time_s: float = ACCEL_TIME_S,
    decel_time_s: float = DECEL_TIME_S,
) -> float:
    """Linear ramp toward ±max while held, faster linear brake to 0 when released."""
    if max_abs <= 0.0 or dt_s <= 0.0:
        return 0.0
    if held_dir > 0:
        held_dir = 1
    elif held_dir < 0:
        held_dir = -1
    else:
        held_dir = 0
    if held_dir != 0:
        value += held_dir * (max_abs / accel_time_s) * dt_s
        return max(-max_abs, min(max_abs, value))
    rate = (max_abs / decel_time_s) * dt_s
    if abs(value) <= rate:
        return 0.0
    return value - math.copysign(rate, value)


def hold_dir(positive: bool, negative: bool) -> int:
    return (1 if positive else 0) + (-1 if negative else 0)
