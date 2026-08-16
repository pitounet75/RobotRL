#!/usr/bin/env python3
"""Capture control params, command 2 m/s, record BalanceFrame for 1 s, then stop."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from telemetry.balance_frame import BalanceFrame
from telemetry.ctrl_client import ControlParamsClient
from telemetry.ctrl_params import format_snapshot
from telemetry.protocol import TELEM_MSG_BALANCE_FRAME, FrameParser
from telemetry.recorder import CsvRecorder
from telemetry.udp_envelope import UdpEnvelopeError, decode_udp_datagram


def mps_to_motor_turns_s(v_mps: float, wheel_radius_m: float, gear: float = 3.0 / 16.0) -> float:
    den = gear * 2.0 * math.pi * wheel_radius_m
    return 0.0 if den <= 1e-12 else v_mps / den


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--esp32-host", required=True)
    p.add_argument("--esp32-port", type=int, default=5000)
    p.add_argument("--bind-port", type=int, default=5000)
    p.add_argument("--speed-mps", type=float, default=2.0)
    p.add_argument("--duration-s", type=float, default=1.0)
    p.add_argument("--out-dir", type=Path, default=Path("logs"))
    args = p.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    params_path = args.out_dir / f"params_before_accel_{stamp}.txt"
    csv_path = args.out_dir / f"accel_{args.speed_mps:g}mps_{stamp}.csv"

    client = ControlParamsClient(
        esp32_host=args.esp32_host,
        esp32_port=args.esp32_port,
        bind_port=args.bind_port,
        timeout_s=2.0,
    )
    try:
        snap = client.get_params()
        params_path.write_text(format_snapshot(snap) + "\n", encoding="utf-8")
        print(f"PARAMS -> {params_path}")
        print(format_snapshot(snap, keys=[
            "cascade_vel_kp", "cascade_vel_ki", "cascade_vel_kd",
            "cascade_vel_err_ema_alpha", "cascade_vel_ema_kp",
            "cascade_pitch_ref_max_rad", "vel_ref_turns_s",
            "ff_grav_k", "ff_fb_k_pitch", "ff_fb_k_rate", "ff_output_alpha",
            "cmd_max_torque_nm", "heading_kp", "heading_kd",
        ]))

        r = float(snap.wheel_radius_m) if snap.wheel_radius_m > 1e-6 else 0.04
        turns = mps_to_motor_turns_s(args.speed_mps, r)
        print(f"SET vel_ref_turns_s={turns:.6g}  ({args.speed_mps:g} m/s, r={r})")
        client.set_param("vel_ref_turns_s", turns)

        recorder = CsvRecorder(csv_path)
        parser = FrameParser()
        t0 = time.time()
        n = 0
        while time.time() - t0 < args.duration_s:
            try:
                data, _addr = client._sock.recvfrom(65535)
            except Exception:
                continue
            try:
                datagram = decode_udp_datagram(data)
            except UdpEnvelopeError:
                continue
            for tf in parser.feed(datagram.payload):
                if not tf.ok or tf.message_type != TELEM_MSG_BALANCE_FRAME:
                    continue
                try:
                    bf = BalanceFrame.decode(tf.payload)
                except ValueError:
                    continue
                recorder.write(time.time(), bf)
                n += 1
        recorder.close()

        client.set_param("vel_ref_turns_s", 0.0)
        print(f"CSV -> {csv_path}  frames={n}")
        return 0 if n > 0 else 2
    finally:
        try:
            client.set_param("vel_ref_turns_s", 0.0)
        except Exception:
            pass
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
