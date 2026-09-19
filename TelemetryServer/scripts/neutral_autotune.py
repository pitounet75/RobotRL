#!/usr/bin/env python3
"""Capture neutral hold, score oscillation/drift, apply one gain step, repeat."""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from telemetry.balance_frame import BalanceFrame
from telemetry.ctrl_client import ControlParamsClient
from telemetry.ctrl_params import format_snapshot
from telemetry.protocol import TELEM_MSG_BALANCE_FRAME, FrameParser
from telemetry.udp_envelope import UdpEnvelopeError, decode_udp_datagram


@dataclass
class Metrics:
    n: int
    pitch_rms_rad: float
    pitch_rate_rms: float
    vel_mean_turns_s: float
    vel_rms_turns_s: float
    cmd_rms_nm: float
    cmd_jerk_rms: float
    pitch_dom_hz: float | None
    cmd_dom_hz: float | None

    @property
    def score(self) -> float:
        """Lower is better: oscillation + drift penalty."""
        drift_pen = abs(self.vel_mean_turns_s) * 8.0
        return (
            self.pitch_rms_rad * 40.0
            + self.pitch_rate_rms * 2.0
            + self.vel_rms_turns_s * 6.0
            + self.cmd_rms_nm * 8.0
            + drift_pen
        )


def dom_freq_hz(t: np.ndarray, y: np.ndarray, fs: float) -> float | None:
    y0 = y - np.mean(y)
    n = len(y0)
    if n < 64 or not math.isfinite(fs) or fs <= 0:
        return None
    spec = np.abs(np.fft.rfft(y0 * np.hanning(n)))
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    spec[0] = 0.0
    band = (freqs >= 0.4) & (freqs <= 12.0)
    if not np.any(band):
        return None
    i = int(np.argmax(spec[band]))
    return float(freqs[band][i])


def capture(client: ControlParamsClient, duration_s: float) -> list[BalanceFrame]:
    parser = FrameParser()
    rows: list[BalanceFrame] = []
    t0 = time.time()
    while time.time() - t0 < duration_s:
        try:
            data, _addr = client._sock.recvfrom(65535)
        except Exception:
            continue
        try:
            payload = decode_udp_datagram(data).payload
        except UdpEnvelopeError:
            payload = data
        for tf in parser.feed(payload):
            if not tf.ok or tf.message_type != TELEM_MSG_BALANCE_FRAME:
                continue
            try:
                rows.append(BalanceFrame.decode(tf.payload))
            except ValueError:
                continue
    return rows


def analyze(rows: list[BalanceFrame]) -> Metrics:
    t = np.array([r.time_us for r in rows], dtype=float)
    t = (t - t[0]) * 1e-6
    dt = np.diff(t)
    dt = dt[dt > 0]
    fs = float(1.0 / np.median(dt)) if len(dt) else float("nan")

    pitch = np.array([r.pitch_rad for r in rows], dtype=float)
    prate = np.array([r.pitch_rate_rads for r in rows], dtype=float)
    vel = np.array([r.vel_wheel_turns_s for r in rows], dtype=float)
    cmd = np.array([r.cmd_torque_nm for r in rows], dtype=float)

    pitch0 = pitch - np.mean(pitch)
    vel0 = vel - np.mean(vel)
    cmd0 = cmd - np.mean(cmd)
    if len(dt) >= 2:
        n = min(len(cmd) - 1, len(dt))
        dcmd = np.diff(cmd[: n + 1]) / dt[:n]
        jerk = float(np.sqrt(np.mean(dcmd**2)))
    else:
        jerk = 0.0

    return Metrics(
        n=len(rows),
        pitch_rms_rad=float(np.sqrt(np.mean(pitch0**2))),
        pitch_rate_rms=float(np.sqrt(np.mean((prate - np.mean(prate)) ** 2))),
        vel_mean_turns_s=float(np.mean(vel)),
        vel_rms_turns_s=float(np.sqrt(np.mean(vel0**2))),
        cmd_rms_nm=float(np.sqrt(np.mean(cmd0**2))),
        cmd_jerk_rms=jerk,
        pitch_dom_hz=dom_freq_hz(t, pitch, fs),
        cmd_dom_hz=dom_freq_hz(t, cmd, fs),
    )


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def propose(snap, m: Metrics) -> dict[str, float]:
    """One conservative step from metrics."""
    p = snap.as_dict()
    out: dict[str, float] = {}

    # Always ensure neutral teleop target during tune.
    out["vel_ref_turns_s"] = 0.0

    pitch_deg = math.degrees(m.pitch_rms_rad)
    drift = abs(m.vel_mean_turns_s)
    oscillating = pitch_deg > 0.35 or m.cmd_rms_nm > 0.0045 or (
        m.pitch_dom_hz is not None and m.pitch_dom_hz > 2.0
    )

    # Oscillation: add damping, trim snap; never raise kp/ema while oscillating hard.
    if oscillating:
        out["ff_fb_k_rate"] = clamp(float(p["ff_fb_k_rate"]) * 1.08, 0.013, 0.022)
        out["cascade_vel_kd"] = clamp(float(p["cascade_vel_kd"]) * 1.10, 0.006, 0.015)
        out["wheel_encoder_vel_lpf_alpha"] = clamp(
            float(p["wheel_encoder_vel_lpf_alpha"]) + 0.02, 0.75, 0.88
        )
        out["cascade_vel_err_ema_alpha"] = clamp(
            float(p["cascade_vel_err_ema_alpha"]) + 0.03, 0.85, 0.97
        )
        if pitch_deg > 0.45:
            out["cascade_vel_kp"] = clamp(float(p["cascade_vel_kp"]) * 0.94, 0.045, 0.08)
            out["ff_output_alpha"] = clamp(float(p["ff_output_alpha"]) + 0.04, 0.5, 0.72)

    # Slow drift only when oscillation is modest.
    if drift > 0.012 and pitch_deg < 0.45 and not oscillating:
        out["cascade_vel_ema_kp"] = clamp(float(p["cascade_vel_ema_kp"]) + 0.002, 0.02, 0.038)

    if drift > 0.025 and pitch_deg < 0.35 and m.vel_rms_turns_s < 0.08:
        out["cascade_vel_kp"] = clamp(float(p["cascade_vel_kp"]) + 0.003, 0.05, 0.085)

    if float(p["torque_deadband_pitch_max_rad"]) < 0.01:
        out["torque_deadband_pitch_max_rad"] = 0.05

    return out


def print_metrics(label: str, m: Metrics) -> None:
    print(
        f"{label}: n={m.n} score={m.score:.3f} "
        f"pitch_rms={math.degrees(m.pitch_rms_rad):.2f}deg "
        f"rate_rms={m.pitch_rate_rms:.3f}rad/s "
        f"vel_mean={m.vel_mean_turns_s:+.4f} vel_rms={m.vel_rms_turns_s:.4f} "
        f"cmd_rms={m.cmd_rms_nm:.4f}Nm jerk={m.cmd_jerk_rms:.3f}"
    )
    if m.pitch_dom_hz:
        print(f"  dom pitch {m.pitch_dom_hz:.2f} Hz  dom cmd {m.cmd_dom_hz or 0:.2f} Hz")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--esp32-host", default="192.168.1.7")
    ap.add_argument("--duration", type=float, default=4.0)
    ap.add_argument("--iterations", type=int, default=8)
    ap.add_argument("--target-score", type=float, default=0.55)
    ap.add_argument("--apply", action="store_true", help="SET proposed params each iter")
    args = ap.parse_args()

    client = ControlParamsClient(
        args.esp32_host, bind_port=0, subscribe=True, timeout_s=2.0
    )
    best_score = float("inf")
    best: dict[str, float] | None = None

    try:
        snap = client.get_params()
        client.set_param("vel_ref_turns_s", 0.0)
        time.sleep(0.3)

        for i in range(args.iterations):
            print(f"\n=== iteration {i + 1}/{args.iterations} ===")
            rows = capture(client, args.duration)
            if len(rows) < 80:
                print(f"too few frames: {len(rows)}")
                return 2
            m = analyze(rows)
            print_metrics("metrics", m)

            if m.score < best_score:
                best_score = m.score
                best = snap.as_dict()

            if m.score <= args.target_score:
                print(f"target score {args.target_score} reached")
                break

            changes = propose(snap, m)
            if not changes:
                print("no further proposals")
                break

            print("proposed changes:")
            for k, v in sorted(changes.items()):
                old = snap.as_dict()[k]
                print(f"  {k}: {old} -> {v}")

            if args.apply:
                for k, v in changes.items():
                    _pid, name, applied = client.set_param(k, v)
                    print(f"  SET {name}={applied:.6g}")
                time.sleep(0.5)
                snap = client.get_params()
            else:
                break

        print("\n=== final params ===")
        snap = client.get_params()
        print(format_snapshot(snap))
        print(f"\nbest score seen: {best_score:.3f}")
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
