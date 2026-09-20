"""Sweep one control param and measure the 42 Hz growl at each value.

Answers one question: is the ~42 Hz oscillation a control limit cycle or a
mechanical resonance? Sweep a gain and watch two numbers.

  - amplitude collapses, or the peak frequency moves  -> the loop sustains it
  - amplitude changes little and the peak stays put   -> mechanical, the loop
    only excites what is already resonant

Usage (the robot must be balancing, on a safe surface, someone ready to catch):

  python scripts/growl_sweep.py --esp32-host 192.168.1.7 cascade_vel_kd 0.008 0.004 0.002 0
  python scripts/growl_sweep.py --esp32-host 192.168.1.7 friction_mode 1 0
  python scripts/growl_sweep.py --esp32-host 192.168.1.7 ff_fb_k_rate 0.02 0.01

The original value is read first and restored on exit, including on Ctrl-C and
on error. Only one parameter is touched per run: two changes at once and the
result is uninterpretable.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
from typing import List, Optional, Sequence

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from telemetry.balance_frame import BalanceFrame
from telemetry.ctrl_client import ControlParamsClient
from telemetry.protocol import TELEM_MSG_BALANCE_FRAME, FrameParser
from telemetry.udp_envelope import UdpEnvelopeError, decode_udp_datagram

# Band searched for the peak. Wide enough to see the peak move, which is the
# whole point of the sweep.
BAND_LO_HZ = 15.0
BAND_HI_HZ = 80.0


def capture(client: ControlParamsClient, seconds: float) -> List[BalanceFrame]:
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
            if not tf.ok or tf.message_type != TELEM_MSG_BALANCE_FRAME:
                continue
            try:
                rows.append(BalanceFrame.decode(tf.payload))
            except ValueError:
                continue
    return rows


def peak_in_band(x: np.ndarray, fs: float, half_width_hz: float = 3.0) -> tuple[float, float]:
    """Peak frequency (Hz) and the amplitude of the whole peak, in units of x.

    Hann-windowed DFT rather than Welch: numpy only, and one 3 s capture has
    plenty of resolution for a 42 Hz line.

    The amplitude sums the energy within half_width_hz of the peak instead of
    reading the peak bin alone. On the robot the growl is not a pure line: it
    wanders and its amplitude breathes, so its energy spreads over several
    bins. Reading one bin reported 0.17 turn/s on a real capture whose band
    energy was 0.66 -- a four-fold underestimate, and worse the broader the
    peak, which would make a sweep look like it was working.
    """
    x = np.asarray(x, dtype=float)
    x = x - x.mean()
    n = len(x)
    if n < 256:
        return float("nan"), float("nan")
    w = np.hanning(n)
    spec = np.abs(np.fft.rfft(x * w))
    freqs = np.fft.rfftfreq(n, 1.0 / fs)

    # Two different normalisations, because they answer two questions.
    # Coherent gain (sum of w) locates the peak with a true sine amplitude;
    # noise power gain (sum of w^2) is the one that makes summing bins
    # correct, and summing is what a smeared peak needs.
    amp = spec * 2.0 / np.sum(w)
    psd = (spec ** 2) * 2.0 / (fs * np.sum(w ** 2))

    band = (freqs >= BAND_LO_HZ) & (freqs <= BAND_HI_HZ)
    if not band.any():
        return float("nan"), float("nan")
    f_peak = float(freqs[band][np.argmax(amp[band])])

    near = (freqs >= f_peak - half_width_hz) & (freqs <= f_peak + half_width_hz)
    rms = float(np.sqrt(np.sum(psd[near]) * (fs / n)))
    return f_peak, rms * np.sqrt(2.0)


def sample_rate(rows: Sequence[BalanceFrame]) -> float:
    t = np.array([r.time_us for r in rows], dtype=np.int64)
    # time_us is a free-running 32-bit microsecond counter: unwrap the rollover
    # instead of letting one wrap ruin the estimate.
    dt = np.diff(t.astype(np.float64))
    dt = dt[(dt > 0) & (dt < 1e6)]
    if len(dt) == 0:
        return float("nan")
    return 1e6 / float(np.median(dt))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--esp32-host", required=True)
    p.add_argument("param", help="Control param name, e.g. cascade_vel_kd")
    p.add_argument("values", nargs="+", type=float, help="Values to try, in order")
    p.add_argument("--seconds", type=float, default=3.0, help="Capture per value")
    p.add_argument("--settle-s", type=float, default=1.0, help="Discarded after each SET")
    p.add_argument("--csv", type=Path, default=Path("logs") / "growl_sweep.csv")
    args = p.parse_args()

    client = ControlParamsClient(args.esp32_host, bind_port=0, subscribe=True, timeout_s=0.5)
    original: Optional[float] = None
    results = []

    try:
        snap = client.get_params()
        original = float(snap.as_dict()[args.param])
        print(f"{args.param}: valeur actuelle {original:g}")
        print(f"{'valeur':>10} {'trames':>7} {'fs':>7} {'pic':>8} "
              f"{'ampl vel_l':>11} {'ampl couple':>12} {'accel impliquee':>16}")

        for value in args.values:
            client.set_param(args.param, value)
            time.sleep(args.settle_s)
            rows = capture(client, args.seconds)
            if len(rows) < 500:
                print(f"{value:>10g} {len(rows):>7}  trop peu de trames, robot arrete ?")
                continue

            fs = sample_rate(rows)
            vel_l = np.array([r.vel_wheel_l_turns_s for r in rows])
            tau_l = np.array([r.cmd_torque_left_nm for r in rows])
            f_vel, a_vel = peak_in_band(vel_l, fs)
            _f_tau, a_tau = peak_in_band(tau_l, fs)
            # A sine of amplitude A at f has acceleration A*2*pi*f.
            accel = a_vel * 2.0 * np.pi * f_vel
            print(f"{value:>10g} {len(rows):>7} {fs:>7.1f} {f_vel:>7.1f}Hz "
                  f"{a_vel:>11.3f} {a_tau:>12.4f} {accel:>13.0f} t/s2")
            results.append((args.param, value, len(rows), fs, f_vel, a_vel, a_tau, accel))
    except KeyboardInterrupt:
        print("\ninterrompu")
    finally:
        if original is not None:
            try:
                client.set_param(args.param, original)
                print(f"{args.param} remis a {original:g}")
            except Exception as exc:  # noqa: BLE001 - must report, never mask
                print(f"ATTENTION: restauration de {args.param} echouee: {exc}")
        client.close()

    if results:
        args.csv.parent.mkdir(exist_ok=True)
        with args.csv.open("w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["param", "value", "frames", "fs_hz", "peak_hz",
                        "amp_vel_l_turns_s", "amp_tau_l_nm", "accel_turns_s2"])
            w.writerows(results)
        print(f"-> {args.csv}")

        peaks = [r[4] for r in results]
        amps = [r[5] for r in results]
        if len(peaks) > 1:
            print(f"\npic: {min(peaks):.1f}..{max(peaks):.1f} Hz | "
                  f"amplitude: {min(amps):.3f}..{max(amps):.3f} turn/s")
            print("Pic fixe et amplitude stable -> mecanique. "
                  "Pic qui bouge ou amplitude qui s'effondre -> boucle de commande.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
