#!/usr/bin/env python3
"""CLI: send synthetic BalanceFrame UDP traffic to a local TelemetryServer."""

from __future__ import annotations

import argparse
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
    try:
        while args.duration_s <= 0 or time.time() - t_start < args.duration_s:
            t = time.time() - t_start
            payload = make_payload(frame_number, t)
            tm_frame = build_frame(1, TELEM_MSG_BALANCE_FRAME, payload)
            datagram = encode_udp_datagram(tm_frame, sequence=frame_number, frame_count=1)
            sock.sendto(datagram, (args.host, args.port))
            frame_number += 1
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
