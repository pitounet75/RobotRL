"""Vbus battery lamp: warn < 11.4 V, critical blink < 11 V."""

from __future__ import annotations

from typing import Literal

VBUS_PRESENT_V = 0.5
VBUS_WARN_V = 11.4
VBUS_CRIT_V = 11.0

VbusAlertLevel = Literal["off", "ok", "warn", "crit"]


def vbus_alert_level(left_v: float, right_v: float) -> VbusAlertLevel:
    """Level from filtered L/R voltages. Red wins over orange; missing rails ignored."""
    rails = [float(v) for v in (left_v, right_v) if float(v) > VBUS_PRESENT_V]
    if not rails:
        return "off"
    lowest = min(rails)
    if lowest < VBUS_CRIT_V:
        return "crit"
    if lowest < VBUS_WARN_V:
        return "warn"
    return "ok"
