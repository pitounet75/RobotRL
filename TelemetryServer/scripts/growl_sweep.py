"""Sweep one control param and measure the 42 Hz growl at each value.

Answers one question: is the ~42 Hz oscillation a control limit cycle or a
mechanical resonance? Sweep a gain and watch two numbers.

  - amplitude collapses, or the peak frequency moves  -> the loop sustains it
  - amplitude changes little and the peak stays put   -> mechanical, the loop
    only excites what is already resonant

Usage (the robot must be balancing, on a safe surface, someone ready to catch):

  python scripts/growl_sweep.py --esp32-host 192.168.1.7 cascade_vel_kd 0.008 0.004 0.002 0
  python scripts/growl_sweep.py --esp32-host 192.168.1.7 friction_mode 1 0
  python scripts/growl_sweep.py --esp32-host 192.168.1.7 meca_k_pitch_damp 0.02 0.01

The original value is read first and restored on exit, including on Ctrl-C and
on error. Only one parameter is touched per run: two changes at once and the
result is uninterpretable.
"""

from __future__ import annotations

import argparse
import csv
import difflib
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


def low_freq_rms(x: np.ndarray, fs: float, cutoff_hz: float = 5.0) -> float:
    """RMS of what is below cutoff_hz: the robot's real motion, not the growl."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n < 256:
        return float("nan")
    spec = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(n)))
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    psd = (spec ** 2) * 2.0 / (fs * np.sum(np.hanning(n) ** 2))
    band = (freqs > 0) & (freqs <= cutoff_hz)
    return float(np.sqrt(np.sum(psd[band]) * (fs / n)))


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
    p.add_argument("--repeat", type=int, default=1,
                   help="Passes over the value list, interleaved (A B A B), then median")
    p.add_argument("--csv", type=Path, default=Path("logs") / "growl_sweep.csv")
    args = p.parse_args()

    client = ControlParamsClient(args.esp32_host, bind_port=0, subscribe=True, timeout_s=0.5)
    original: Optional[float] = None
    results = []

    try:
        snap = client.get_params()
        params = snap.as_dict()
        if args.param not in params:
            close = difflib.get_close_matches(args.param, params, n=5, cutoff=0.4)
            print(f"parametre inconnu: {args.param}")
            if close:
                print("  proche(s):", ", ".join(close))
            print("  (les noms ont change avec la refonte ctrl_*: ff_fb_k_rate ->"
                  " meca_k_pitch_damp, ff_fb_k_pitch -> err_k_pitch)")
            return 2
        original = float(params[args.param])
        print(f"{args.param}: valeur actuelle {original:g}")
        print(f"{'passe':>6} {'valeur':>10} {'trames':>7} {'fs':>7} {'pic':>8} "
              f"{'ampl vel_l':>11} {'ampl couple':>12} {'accel':>10} "
              f"{'pitch rms':>10} {'vel <5Hz':>9}")

        # Interleaved: every pass walks the whole list, so a slow drift (the
        # battery sagging) hits every value alike instead of only the last.
        for rep in range(max(1, args.repeat)):
          for value in args.values:
            client.set_param(args.param, value)
            time.sleep(args.settle_s)
            rows = capture(client, args.seconds)
            if len(rows) < 500:
                print(f"{rep + 1:>6} {value:>10g} {len(rows):>7}"
                      f"  trop peu de trames, robot arrete ?")
                continue

            fs = sample_rate(rows)
            vel_l = np.array([r.vel_wheel_l_turns_s for r in rows])
            tau_l = np.array([r.cmd_torque_left_nm for r in rows])
            pitch = np.array([r.pitch_rad for r in rows])
            pitch_ref = np.array([r.pitch_ref_rad for r in rows])
            f_vel, a_vel = peak_in_band(vel_l, fs)
            _f_tau, a_tau = peak_in_band(tau_l, fs)
            # A sine of amplitude A at f has acceleration A*2*pi*f.
            accel = a_vel * 2.0 * np.pi * f_vel
            # Killing the growl is easy if you are allowed to ruin the loop:
            # drop every damping gain and the robot wallows instead of buzzing.
            # These two say what the quieter run cost. pitch rms is tracking
            # error; vel below 5 Hz is how much the robot actually wanders.
            pitch_rms = float(np.sqrt(np.mean((pitch - pitch_ref) ** 2)))
            vel_slow = low_freq_rms(vel_l, fs)
            print(f"{rep + 1:>6} {value:>10g} {len(rows):>7} {fs:>7.1f} {f_vel:>7.1f}Hz "
                  f"{a_vel:>11.3f} {a_tau:>12.4f} {accel:>7.0f}t/s2 "
                  f"{pitch_rms:>10.4f} {vel_slow:>9.3f}")
            results.append((args.param, rep + 1, value, len(rows), fs, f_vel, a_vel,
                            a_tau, accel, pitch_rms, vel_slow))
    except KeyboardInterrupt:
        print("\ninterrompu")
    finally:
        if original is not None:
            try:
                client.set_param(args.param, original)
                print(f"{args.param} remis a {original:g}")
            except Exception as exc:  # noqa: BLE001 - must report, never mask
                print(f"ATTENTION: restauration de {args.param} echouee: {exc}")
                print("  Le robot tourne peut-etre encore avec la derniere valeur"
                      " essayee. Les parametres ne sont PAS en memoire non volatile:"
                      f" coupe et rallume le robot pour revenir a {original:g}.")
        client.close()

    if results:
        args.csv.parent.mkdir(exist_ok=True)
        with args.csv.open("w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["param", "pass", "value", "frames", "fs_hz", "peak_hz",
                        "amp_vel_l_turns_s", "amp_tau_l_nm", "accel_turns_s2",
                        "pitch_rms_rad", "vel_below_5hz_rms"])
            w.writerows(results)
        print(f"-> {args.csv}")

        print()
        print(f"{'valeur':>10} {'n':>3} {'pic med':>9} {'ampl med':>9} "
              f"{'etendue':>9} {'vel<5Hz med':>12}")
        for value in args.values:
            got = [r for r in results if r[2] == value]
            if not got:
                continue
            amps = sorted(r[6] for r in got)
            peaks = [r[5] for r in got]
            slows = [r[10] for r in got]
            print(f"{value:>10g} {len(got):>3} {float(np.median(peaks)):>8.1f}Hz "
                  f"{float(np.median(amps)):>9.3f} {amps[-1] - amps[0]:>9.3f} "
                  f"{float(np.median(slows)):>12.3f}")

        per_value = [len([r for r in results if r[2] == v]) for v in args.values]
        if max(per_value) < 2:
            # One pass ranks values that a second pass may reorder: the scatter
            # between runs measured the same size as the effect, and the
            # battery sags during a sweep, which penalises whatever is tested
            # last. Interleaving cancels the drift; repeats expose the scatter.
            print("Une seule passe: la dispersion entre essais vaut autant que l'effet,"
                  " et la batterie faiblit pendant le balayage, ce qui penalise les"
                  " dernieres valeurs. Relancer avec --repeat 3.")
        else:
            print("Comparer l'ecart entre medianes a l'etendue de chaque valeur:"
                  " un ecart plus petit que l'etendue ne prouve rien.")
        print("Pic qui bouge ou amplitude qui s'effondre -> boucle de commande."
              " Pic fixe et amplitude stable -> mecanique.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
