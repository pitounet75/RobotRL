#!/usr/bin/env python3
"""One-shot: USB connect, go to closed loop on axis0, start anticogging cal, print progress."""
import sys
import time

import odrive
from fibre import Event
from odrive.enums import *

TIMEOUT_S = 25.0
POLL_S = 0.5
MAX_WAIT_S = 600.0  # cal can take many minutes


def main():
    cancel = Event()
    print("Connecting (USB, {:.0f}s timeout)...".format(TIMEOUT_S))
    odrv = odrive.find_any(
        path="usb",
        timeout=TIMEOUT_S,
        search_cancellation_token=cancel,
        channel_termination_token=cancel,
    )
    if odrv is None:
        print("No ODrive found. Check USB, power, and WinUSB/libusb drivers.")
        return 1

    ax = odrv.axis0
    print("Connected. axis0.error={} motor.error={} encoder.error={}".format(
        hex(ax.error), hex(ax.motor.error), hex(ax.encoder.error)))

    if ax.error != 0:
        print("Clear axis0 errors before calibration (dump_errors in odrivetool).")
        return 1

    if ax.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print("Requesting AXIS_STATE_CLOSED_LOOP_CONTROL...")
        ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        for _ in range(40):
            time.sleep(0.1)
            if ax.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                break
        if ax.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            print("Axis did not reach closed loop (state={}).".format(ax.current_state))
            return 1

    ctrl = ax.controller
    ac = ctrl.config.anticogging
    print("anticogging: enabled={} valid={} calib_running={}".format(
        ac.anticogging_enabled, ctrl.anticogging_valid, ac.calib_anticogging))

    print("Starting start_anticogging_calibration() on axis0...")
    ctrl.start_anticogging_calibration()

    t0 = time.monotonic()
    while time.monotonic() - t0 < MAX_WAIT_S:
        time.sleep(POLL_S)
        idx = ac.index
        cal = ac.calib_anticogging
        st = ax.current_state
        print("  t={:5.1f}s  anticogging.index={}  calib_anticogging={}  axis_state={}".format(
            time.monotonic() - t0, idx, cal, st))
        if not cal:
            print("Calibration finished (calib_anticogging == False).")
            print("anticogging_valid={} anticogging_enabled={}".format(
                ctrl.anticogging_valid, ac.anticogging_enabled))
            return 0

    print("Still running after {:.0f}s — stop with odrivetool or power cycle if needed.".format(MAX_WAIT_S))
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
