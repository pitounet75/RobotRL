"""Capture a free ring-down and print its spectrum: mechanics, no control loop.

Why: sweeping every control gain moved the growl's frequency but never its
amplitude, and halving the commanded torque at 38 Hz left the 38 Hz wheel
vibration untouched. That points at a mechanical resonance the loop merely
excites. A ring-down measures it directly, with the loop out of the way.

Procedure, done twice:

  1. Cut the torque (the drives stay armed, so the wheels are free):
       python scripts/tune_params.py --esp32-host <ip> set pitch_failsafe_rad 0.001
     THE ROBOT WILL FALL. Lay it down or hold it first.

  2. Run this script, then excite and let go:
       --mode air     robot held off the ground, flick one wheel
       --mode ground  robot on the ground, push the chassis fore-aft, sharply

  3. Restore:
       python scripts/tune_params.py --esp32-host <ip> set pitch_failsafe_rad 0.785398

Reading the result:

  - rings in BOTH modes    -> drivetrain: belt, pulleys, motor mount
  - rings only on the GROUND -> tyre compliance against the robot's mass, which
    is what the in-phase wheels (correlation 0.98) already suggested
  - rings in NEITHER       -> no passive resonance; the loop makes the
    oscillation on its own, and the control gains are the way out after all
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import List

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from telemetry.balance_frame import BalanceFrame
from telemetry.ctrl_client import ControlParamsClient
from telemetry.protocol import TELEM_MSG_BALANCE_FRAME, FrameParser
from telemetry.udp_envelope import UdpEnvelopeError, decode_udp_datagram


def collect(client: ControlParamsClient, seconds: float) -> List[BalanceFrame]:
    parser = FrameParser()
    rows: List[BalanceFrame] = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            data, _addr = client._sock.recvfrom(65535)
        except Exception:
            continue
        try:
            payload = decode_udp_datagram(data).payload
        except UdpEnvelopeError:
            payload = data
        for tf in parser.feed(payload):
            if tf.ok and tf.message_type == TELEM_MSG_BALANCE_FRAME:
                try:
                    rows.append(BalanceFrame.decode(tf.payload))
                except ValueError:
                    continue
    return rows


def spectrum(x: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    x = np.asarray(x, dtype=float)
    x = x - x.mean()
    n = len(x)
    w = np.hanning(n)
    spec = np.abs(np.fft.rfft(x * w))
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    amp = spec * 2.0 / np.sum(w)
    return freqs, amp


def decay_time_ms(x: np.ndarray, fs: float) -> float:
    """Time for the envelope to fall to 1/e. NaN if it never does."""
    env = np.abs(np.asarray(x, dtype=float))
    # Envelope over ~20 ms, long enough to smooth a 30-40 Hz carrier.
    k = max(3, int(0.02 * fs))
    env = np.convolve(env, np.ones(k) / k, mode="same")
    peak = env.max()
    if peak <= 0:
        return float("nan")
    i_peak = int(np.argmax(env))
    below = np.where(env[i_peak:] < peak / np.e)[0]
    if len(below) == 0:
        return float("nan")
    return float(below[0]) * 1000.0 / fs


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--esp32-host", required=True)
    p.add_argument("--mode", choices=["air", "ground"], required=True,
                   help="Only used to label the capture")
    p.add_argument("--seconds", type=float, default=12.0)
    p.add_argument("--window-ms", type=float, default=600.0,
                   help="Analysed after the impact")
    p.add_argument("--csv", type=Path, default=None)
    args = p.parse_args()

    client = ControlParamsClient(args.esp32_host, bind_port=0, subscribe=True, timeout_s=0.5)
    try:
        snap = client.get_params()
        failsafe = float(snap.as_dict()["pitch_failsafe_rad"])
        if failsafe > 0.01:
            print(f"ATTENTION: pitch_failsafe_rad = {failsafe:g}, le couple n'est pas coupe.")
            print("  Les moteurs vont repondre a l'excitation et la mesure ne sera pas libre.")
            print(f"  python scripts/tune_params.py --esp32-host {args.esp32_host}"
                  " set pitch_failsafe_rad 0.001")
            return 2

        print(f"[{args.mode}] capture {args.seconds:g} s — excite maintenant, puis lache.")
        rows = collect(client, args.seconds)
    finally:
        client.close()

    if len(rows) < 1000:
        print(f"trop peu de trames ({len(rows)})")
        return 2

    t = np.array([r.time_us for r in rows], dtype=np.float64)
    t = (t - t[0]) / 1e6
    dt = np.diff(t)
    fs = 1.0 / np.median(dt[(dt > 0) & (dt < 1.0)]) if len(dt) else 500.0
    vel = np.array([r.vel_wheel_l_turns_s for r in rows], dtype=float)
    vel_r = np.array([r.vel_wheel_r_turns_s for r in rows], dtype=float)
    pitch_rate = np.array([r.pitch_rate_rads for r in rows], dtype=float)
    tau = np.array([r.cmd_torque_left_nm for r in rows], dtype=float)

    if np.abs(tau).max() > 1e-6:
        print(f"ATTENTION: couple non nul pendant la capture (max {np.abs(tau).max():.4f} Nm):"
              " la reponse n'est pas libre.")

    # The impact is the sharpest step in wheel speed; analyse what follows it.
    jerk = np.abs(np.diff(vel, prepend=vel[0]))
    i0 = int(np.argmax(jerk))
    n_win = int(args.window_ms * 1e-3 * fs)
    seg = slice(i0, min(i0 + n_win, len(vel)))
    print(f"trames {len(rows)}  fs {fs:.1f} Hz  choc a t={t[i0]:.2f}s  "
          f"fenetre {args.window_ms:g} ms")

    # 128 samples at 500 Hz is 256 ms: ~9 periods of a 35 Hz ring, enough to
    # place the peak and to see it decay.
    if seg.stop - seg.start < 128:
        print("fenetre trop courte apres le choc")
        return 2

    for label, sig in (("vel_l", vel), ("vel_r", vel_r), ("pitch_rate", pitch_rate)):
        f, a = spectrum(sig[seg], fs)
        band = (f >= 5) & (f <= 120)
        if not band.any():
            continue
        top = np.argsort(a[band])[-3:][::-1]
        peaks = ", ".join(f"{f[band][i]:.1f} Hz ({a[band][i]:.3f})" for i in top)
        print(f"  {label:<11} pics: {peaks}   decroissance 1/e: "
              f"{decay_time_ms(sig[seg], fs):.0f} ms")

    if args.csv:
        args.csv.parent.mkdir(exist_ok=True)
        np.savetxt(args.csv, np.column_stack([t, vel, vel_r, pitch_rate, tau]),
                   delimiter=",", header="t_s,vel_l,vel_r,pitch_rate,tau_l", comments="")
        print(f"-> {args.csv}")

    print("\nUn pic net entre 25 et 45 Hz qui decroit en quelques dizaines de ms"
          " = la resonance cherchee.")
    print("Refaire dans l'autre mode: air et sol se comparent, c'est la comparaison"
          " qui designe le coupable.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
