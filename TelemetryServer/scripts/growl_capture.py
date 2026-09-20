"""Capture 2s BalanceFrame and print growl/oscillation diagnostics."""

from __future__ import annotations

import csv
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from telemetry.balance_frame import BalanceFrame
from telemetry.ctrl_client import ControlParamsClient
from telemetry.protocol import TELEM_MSG_BALANCE_FRAME, FrameParser
from telemetry.udp_envelope import UdpEnvelopeError, decode_udp_datagram


def main() -> int:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out = Path("logs") / f"growl_capture_{stamp}.csv"
    out.parent.mkdir(exist_ok=True)

    client = ControlParamsClient(
        "192.168.1.7", bind_port=0, subscribe=True, timeout_s=0.5
    )
    parser = FrameParser()
    rows: list[BalanceFrame] = []
    t0 = time.time()
    try:
        while time.time() - t0 < 2.0:
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
    finally:
        client.close()

    print(f"frames={len(rows)}  -> {out}")
    if len(rows) < 50:
        print("too few frames")
        return 2

    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "t_us",
                "frame",
                "pitch_rad",
                "pitch_rate",
                "vel",
                "vel_l",
                "vel_r",
                "cmd",
                "cmd_l",
                "cmd_r",
                "u_meca",
                "u_err",
                "pitch_ref",
            ]
        )
        for bf in rows:
            w.writerow(
                [
                    bf.time_us,
                    bf.frame_number,
                    bf.pitch_rad,
                    bf.pitch_rate_rads,
                    bf.vel_wheel_turns_s,
                    bf.vel_wheel_l_turns_s,
                    bf.vel_wheel_r_turns_s,
                    bf.cmd_torque_nm,
                    bf.cmd_torque_left_nm,
                    bf.cmd_torque_right_nm,
                    bf.u_meca_nm,
                    bf.u_err_nm,
                    bf.pitch_ref_rad,
                ]
            )

    def col(attr: str) -> np.ndarray:
        return np.array([getattr(r, attr) for r in rows], dtype=float)

    t = col("time_us")
    t = (t - t[0]) * 1e-6
    dt = np.diff(t)
    dt_pos = dt[dt > 0]
    fs = float(1.0 / np.median(dt_pos)) if len(dt_pos) else float("nan")

    pitch = col("pitch_rad")
    prate = col("pitch_rate_rads")
    vel = col("vel_wheel_turns_s")
    cmd = col("cmd_torque_nm")
    pref = col("pitch_ref_rad")
    u_err = col("u_err_nm")

    def stats(name: str, x: np.ndarray, unit: str = "") -> None:
        x0 = x - np.mean(x)
        print(
            f"{name:16s} mean={np.mean(x):+.5f}  rms={np.sqrt(np.mean(x0**2)):.5f}  "
            f"pkpk={float(np.max(x) - np.min(x)):.5f} {unit}"
        )

    print(
        f"dt_med={np.median(dt_pos)*1e3:.3f} ms  fs~{fs:.1f} Hz  "
        f"duration={t[-1]:.3f}s  strategy={rows[0].strategy_id}"
    )
    stats("pitch", pitch, "rad")
    stats("pitch_rate", prate, "rad/s")
    stats("pitch_ref", pref, "rad")
    stats("vel_wheel", vel, "turn/s")
    stats("cmd_torque", cmd, "Nm")
    stats("u_err", u_err, "Nm")

    def dom_freq(x: np.ndarray) -> tuple[float, float] | tuple[None, None]:
        x0 = x - np.mean(x)
        n = len(x0)
        if n < 64 or not np.isfinite(fs):
            return None, None
        spec = np.abs(np.fft.rfft(x0 * np.hanning(n)))
        freqs = np.fft.rfftfreq(n, d=1.0 / fs)
        spec[0] = 0.0
        i = int(np.argmax(spec))
        return float(freqs[i]), float(spec[i])

    for name, x in (
        ("pitch", pitch),
        ("pitch_rate", prate),
        ("vel", vel),
        ("cmd", cmd),
        ("pitch_ref", pref),
    ):
        f, _a = dom_freq(x)
        if f is not None:
            print(f"dom_freq {name:12s} {f:6.2f} Hz")

    dv = np.diff(vel)
    print(
        f"vel |dV| median={np.median(np.abs(dv)):.5f}  "
        f"p95={np.percentile(np.abs(dv), 95):.5f}  max={np.max(np.abs(dv)):.5f} turn/s"
    )
    vdot = dv / dt_pos[: len(dv)] if len(dt_pos) >= len(dv) else dv / np.median(dt_pos)
    # align lengths
    n = min(len(dv), len(dt))
    vdot = dv[:n] / dt[:n]
    vdot = vdot[np.isfinite(vdot)]
    kd = 0.0025
    p95 = float(np.percentile(np.abs(vdot), 95))
    print(
        f"v_dot est rms={float(np.sqrt(np.mean((vdot - np.mean(vdot)) ** 2))):.2f}  "
        f"p95={p95:.2f} turn/s2"
    )
    print(f"kd*v_dot p95 -> pitch_cmd ~ {kd * p95 * 180.0 / math.pi:.2f} deg")
    if np.std(vel) > 1e-9 and np.std(pref) > 1e-9:
        print(f"corr(vel, pitch_ref)={np.corrcoef(vel, pref)[0, 1]:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
