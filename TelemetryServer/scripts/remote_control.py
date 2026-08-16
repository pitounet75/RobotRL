#!/usr/bin/env python3
"""Mouse remote control for RobotRL (speed + heading over ESP32 telemetry RPC)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from telemetry.ctrl_client import ControlParamsClient
from telemetry.remote_control import run_remote_control


def main() -> int:
    p = argparse.ArgumentParser(
        description="RobotRL remote control — mouse stick → vel_ref + heading_ref"
    )
    p.add_argument("--esp32-host", required=True, help="ESP32 IP (e.g. 192.168.1.7)")
    p.add_argument("--esp32-port", type=int, default=5000)
    p.add_argument(
        "--bind-port",
        type=int,
        default=0,
        help="Local UDP bind port (0 = ephemeral; avoid 5000 if plotter is open)",
    )
    p.add_argument("--timeout", type=float, default=0.5, help="RPC reply timeout (seconds)")
    args = p.parse_args()

    client = ControlParamsClient(
        esp32_host=args.esp32_host,
        esp32_port=args.esp32_port,
        bind_port=args.bind_port,
        timeout_s=args.timeout,
        subscribe=False,  # never steal/flood with BalanceFrame stream
    )
    try:
        return run_remote_control(client)
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
