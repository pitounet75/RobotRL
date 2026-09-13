"""Synthetic BalanceFrame payloads for exercising the web UI without hardware."""

from __future__ import annotations

import math

from telemetry.balance_frame import BALANCE_FRAME_STRUCT_V2


def make_payload(frame_number: int, t: float) -> bytes:
    """A slowly oscillating, physically-plausible BalanceFrame payload at time t (s)."""
    pitch = 0.05 * math.sin(t)
    return BALANCE_FRAME_STRUCT_V2.pack(
        frame_number & 0xFFFFFFFF,
        int(t * 1_000_000) & 0xFFFFFFFF,
        pitch,
        0.05 * math.cos(t),
        0.2,
        0.2,
        0.2,
        0.01 * math.sin(t),
        0.01 * math.sin(t),
        0.01 * math.sin(t),
        0.0,
        0.01,
        0.0,
        1,
        0,
        0,
        0,
    )
