#!/usr/bin/env python3
"""Read or write STM32 control gains over telemetry (no recompile)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from telemetry.ctrl_client import ControlParamsClient
from telemetry.ctrl_params import PARAM_NAMES, format_snapshot


def main() -> int:
    p = argparse.ArgumentParser(description="Runtime control gain GET/SET via ESP32 UDP bridge")
    p.add_argument("--esp32-host", required=True, help="ESP32 IP (e.g. 10.0.0.53)")
    p.add_argument("--esp32-port", type=int, default=5000)
    p.add_argument("--bind-port", type=int, default=5000)
    p.add_argument("--timeout", type=float, default=1.5, help="Reply timeout (seconds)")
    sub = p.add_subparsers(dest="cmd", required=True)

    get_p = sub.add_parser("get", help="Read all control params from STM32")
    get_p.add_argument(
        "--only",
        nargs="+",
        choices=sorted(PARAM_NAMES.keys()),
        help="Print subset (default: all)",
    )

    set_p = sub.add_parser("set", help="Write one param")
    set_p.add_argument("param", help=f"Param name or id ({len(PARAM_NAMES)} tunables)")
    set_p.add_argument("value", type=float, help="New value")

    args = p.parse_args()

    client = ControlParamsClient(
        esp32_host=args.esp32_host,
        esp32_port=args.esp32_port,
        bind_port=args.bind_port,
        timeout_s=args.timeout,
    )
    try:
        if args.cmd == "get":
            snap = client.get_params()
            print(format_snapshot(snap, keys=args.only))
            return 0

        param = args.param
        if param.isdigit():
            param = int(param)
        ack_id, name, applied = client.set_param(param, args.value)
        print(f"OK {name} (id={ack_id}) = {applied:.6g}")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
