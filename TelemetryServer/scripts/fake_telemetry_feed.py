#!/usr/bin/env python3
"""CLI: send synthetic BalanceFrame UDP traffic to a local TelemetryServer."""

from __future__ import annotations

import argparse
import contextlib
import ctypes
import socket
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from telemetry.protocol import TELEM_MSG_BALANCE_FRAME, build_frame
from telemetry.synthetic_balance_frame import make_payload
from telemetry.udp_envelope import encode_udp_datagram

# How much of each period we spin on perf_counter instead of sleeping. Must stay
# above the OS sleep granularity we can actually get (1ms on Windows inside
# high_timer_resolution()), or time.sleep overshoots the deadline every cycle.
SPIN_MARGIN_S = 0.0015


@contextlib.contextmanager
def high_timer_resolution():
    """Ask Windows for a 1ms scheduler tick (no-op elsewhere).

    Without this, time.sleep() rounds up to the default ~15.6ms tick on
    Python < 3.11, capping this feed near 65Hz no matter what --hz says.
    """
    if sys.platform != "win32":
        yield
        return
    try:
        winmm = ctypes.WinDLL("winmm")
    except OSError:
        yield
        return
    raised = winmm.timeBeginPeriod(1) == 0
    try:
        yield
    finally:
        if raised:
            winmm.timeEndPeriod(1)


def main() -> int:
    p = argparse.ArgumentParser(description="Synthetic BalanceFrame UDP source")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--hz", type=float, default=500.0)
    p.add_argument("--duration-s", type=float, default=0.0, help="0 = run forever")
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / args.hz
    t_start = time.time()
    frame_number = 0
    # Deadline-based pacing rather than time.sleep(period): a fixed sleep per
    # iteration drifts by however much the OS overshoots, and at 2ms requested
    # the overshoot dominates.
    next_deadline = time.perf_counter()
    last_report_time = t_start
    last_report_count = 0
    try:
        with high_timer_resolution():
            while args.duration_s <= 0 or time.time() - t_start < args.duration_s:
                t = time.time() - t_start
                payload = make_payload(frame_number, t)
                tm_frame = build_frame(1, TELEM_MSG_BALANCE_FRAME, payload)
                datagram = encode_udp_datagram(
                    tm_frame, sequence=frame_number, frame_count=1
                )
                sock.sendto(datagram, (args.host, args.port))
                frame_number += 1

                now = time.time()
                if now - last_report_time >= 1.0:
                    achieved_hz = (frame_number - last_report_count) / (
                        now - last_report_time
                    )
                    print(f"achieved ~{achieved_hz:.0f} Hz (target {args.hz:.0f} Hz)")
                    last_report_time = now
                    last_report_count = frame_number

                next_deadline += period
                sleep_s = next_deadline - time.perf_counter()
                if sleep_s > 0:
                    # Sleep the bulk, then spin out the last ~1.5ms, which is
                    # finer than any sleep the OS will honour.
                    if sleep_s > SPIN_MARGIN_S:
                        time.sleep(sleep_s - SPIN_MARGIN_S)
                    while time.perf_counter() < next_deadline:
                        pass
                else:
                    # Fell behind: resync rather than fire a catch-up burst.
                    next_deadline = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
