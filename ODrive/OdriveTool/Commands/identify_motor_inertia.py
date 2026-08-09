#!/usr/bin/env python3
"""
Identify motor-side inertia J and friction (c, b) with a torque step + coast.

Wheels MUST be free (off the ground). Anticogging as in normal use.

Sequence per trial:
  1) input_torque = +T for --accel-s (default 1.0 s)
  2) input_torque = 0   for --coast-s (default 1.0 s)  ← coast
  3) optional reverse trial with -T

Records encoder.vel_estimate (turn/s) and derives alpha. Fits:
  tau = J * alpha_rad + b * omega_rad + c * sign(omega)

Usage:
  python identify_motor_inertia.py --serial 55517627953204
  python identify_motor_inertia.py --serial ... --torques 0.008,0.010,0.012
  python identify_motor_inertia.py --serial ... --plot
  python identify_motor_inertia.py --serial ... --out logs/inertia_left.csv

Esc aborts (torque forced to 0).
"""
from __future__ import print_function

import argparse
import csv
import math
import os
import sys
import time

import odrive
from fibre import Event
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_TORQUE_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)

try:
    import select
    import termios
    import tty

    _HAS_UNIX_TTY = True
except ImportError:
    _HAS_UNIX_TTY = False

_WIN_ESC_DOWN = False
TWO_PI = 2.0 * math.pi


class UserAbort(Exception):
    pass


def escape_pressed():
    global _WIN_ESC_DOWN
    if sys.platform == "win32":
        try:
            import ctypes

            down = bool(ctypes.windll.user32.GetAsyncKeyState(0x1B) & 0x8000)
        except Exception:
            return False
        pressed = down and not _WIN_ESC_DOWN
        _WIN_ESC_DOWN = down
        return pressed
    if _HAS_UNIX_TTY and select.select([sys.stdin], [], [], 0)[0]:
        if sys.stdin.read(1) == "\x1b":
            return True
    return False


def sleep_interruptible(duration_s, period_s=0.01):
    end = time.time() + duration_s
    while True:
        if escape_pressed():
            raise UserAbort()
        remaining = end - time.time()
        if remaining <= 0:
            break
        time.sleep(min(period_s, remaining))


def enable_escape_key():
    if not _HAS_UNIX_TTY or sys.platform == "win32":
        return None
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return old


def disable_escape_key(old_attrs):
    if old_attrs is None or not _HAS_UNIX_TTY or sys.platform == "win32":
        return
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_attrs)


def connect(serial, timeout):
    cancel = Event()
    print("Connecting (USB)...", flush=True)
    odrv = odrive.find_any(
        path="usb",
        serial_number=serial,
        timeout=timeout,
        search_cancellation_token=cancel,
        channel_termination_token=cancel,
    )
    if odrv is None:
        print("No ODrive found.", file=sys.stderr)
        sys.exit(1)
    return odrv


def axis(odrv, axis_num):
    return odrv.axis0 if axis_num == 0 else odrv.axis1


def ensure_torque_mode_closed_loop(ax):
    ax.clear_errors()
    ax.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    # Avoid torque derating during the ramp (bring-up setting).
    try:
        ax.controller.config.enable_current_mode_vel_limit = False
    except Exception:
        pass
    ax.controller.input_torque = 0.0
    ax.controller.input_vel = 0.0
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    deadline = time.time() + 10.0
    while ax.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        if escape_pressed():
            raise UserAbort()
        if time.time() > deadline:
            print(
                "Failed to enter closed loop (state={}).".format(ax.current_state),
                file=sys.stderr,
            )
            sys.exit(1)
        time.sleep(0.05)


def parse_torques(s):
    vals = []
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        vals.append(float(part))
    if not vals:
        raise ValueError("empty --torques")
    return vals


def ema_derivative(t, y, alpha):
    """Causal EMA of dy/dt. First sample returns 0."""
    out = [0.0] * len(y)
    if len(y) < 2:
        return out
    dy = 0.0
    for i in range(1, len(y)):
        dt = t[i] - t[i - 1]
        if dt <= 1e-6:
            out[i] = out[i - 1]
            continue
        inst = (y[i] - y[i - 1]) / dt
        dy = alpha * inst + (1.0 - alpha) * dy
        out[i] = dy
    return out


def sign_eps(x, eps=0.05):
    """Deadzone sign to avoid chatter near zero speed (turn/s)."""
    if x > eps:
        return 1.0
    if x < -eps:
        return -1.0
    return 0.0


def fit_jbc(rows, settle_s=0.05, vel_min_turns=0.5):
    """
    Least-squares: tau = J*alpha_rad + b*omega_rad + c*sign(omega)
    Uses samples after settle_s into each phase, |vel| >= vel_min.
    Returns dict with J, b, c, n, or None if underdetermined.
    """
    # Group by trial; skip early settle within each contiguous phase.
    xs = []  # [alpha_rad, omega_rad, sign]
    ys = []  # tau
    if not rows:
        return None

    t0_phase = rows[0]["t"]
    prev_phase = rows[0]["phase"]
    for r in rows:
        if r["phase"] != prev_phase:
            t0_phase = r["t"]
            prev_phase = r["phase"]
        if (r["t"] - t0_phase) < settle_s:
            continue
        if abs(r["vel_turns"]) < vel_min_turns:
            continue
        if r["sign"] == 0.0:
            continue
        xs.append([r["alpha_rad"], r["omega_rad"], r["sign"]])
        ys.append(r["tau"])

    n = len(ys)
    if n < 10:
        return None

    # Normal equations A^T A x = A^T y
    ata = [[0.0] * 3 for _ in range(3)]
    aty = [0.0] * 3
    for row, y in zip(xs, ys):
        for i in range(3):
            aty[i] += row[i] * y
            for j in range(3):
                ata[i][j] += row[i] * row[j]

    # Solve 3x3
    try:
        sol = _solve3(ata, aty)
    except Exception:
        return None
    J, b, c = sol
    # Residual RMS
    sse = 0.0
    for row, y in zip(xs, ys):
        pred = J * row[0] + b * row[1] + c * row[2]
        err = y - pred
        sse += err * err
    rms = math.sqrt(sse / n)
    return {"J": J, "b": b, "c": c, "n": n, "rms_nm": rms}


def _solve3(a, b):
    """Gaussian elimination with partial pivot for 3x3."""
    m = [a[i][:] + [b[i]] for i in range(3)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-18:
            raise ZeroDivisionError("singular")
        m[col], m[piv] = m[piv], m[col]
        div = m[col][col]
        for j in range(col, 4):
            m[col][j] /= div
        for r in range(3):
            if r == col:
                continue
            f = m[r][col]
            for j in range(col, 4):
                m[r][j] -= f * m[col][j]
    return [m[0][3], m[1][3], m[2][3]]


def run_phase(ax, tau, duration_s, hz, phase_name, trial_id, t0, rows):
    ax.controller.input_torque = float(tau)
    period = 1.0 / float(hz)
    end = time.time() + duration_s
    while time.time() < end:
        if escape_pressed():
            raise UserAbort()
        now = time.time()
        vel = float(ax.encoder.vel_estimate)  # turn/s
        iq = float(ax.motor.current_control.Iq_measured)
        rows.append(
            {
                "t": now - t0,
                "trial": trial_id,
                "phase": phase_name,
                "tau": float(tau),
                "vel_turns": vel,
                "iq": iq,
            }
        )
        # Pace to target rate (USB often slower than requested).
        target = now + period
        while True:
            if escape_pressed():
                raise UserAbort()
            rem = target - time.time()
            if rem <= 0:
                break
            time.sleep(min(0.002, rem))


def enrich_rows(rows, alpha_ema):
    t = [r["t"] for r in rows]
    v = [r["vel_turns"] for r in rows]
    a_turns = ema_derivative(t, v, alpha_ema)
    for r, a in zip(rows, a_turns):
        r["alpha_turns"] = a
        r["alpha_rad"] = a * TWO_PI
        r["omega_rad"] = r["vel_turns"] * TWO_PI
        r["sign"] = sign_eps(r["vel_turns"])


def write_csv(path, rows):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fields = [
        "t",
        "trial",
        "phase",
        "tau",
        "vel_turns",
        "omega_rad",
        "alpha_turns",
        "alpha_rad",
        "sign",
        "iq",
    ]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print("Wrote {}".format(path), flush=True)


def maybe_plot(rows, fit, out_png):
    """Save PNG with Agg backend (avoids broken Tk/tcl on Windows Anaconda)."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skip --plot", file=sys.stderr)
        return
    t = [r["t"] for r in rows]
    v = [r["vel_turns"] for r in rows]
    a = [r["alpha_turns"] for r in rows]
    tau = [r["tau"] for r in rows]
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
    axes[0].plot(t, tau, label="tau_cmd")
    axes[0].set_ylabel("Nm")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, v, label="vel")
    axes[1].set_ylabel("turn/s")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(t, a, label="alpha (EMA)")
    axes[2].set_ylabel("turn/s^2")
    axes[2].set_xlabel("t [s]")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.3)
    title = "motor inertia step+coast"
    if fit:
        title += "  J={:.3e}  c={:.4f}  b={:.3e}".format(fit["J"], fit["c"], fit["b"])
    fig.suptitle(title)
    fig.tight_layout()
    if not out_png:
        out_png = "inertia_plot.png"
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    print("Wrote {}".format(out_png), flush=True)


def main():
    p = argparse.ArgumentParser(description="Identify motor J / friction via torque step + coast.")
    p.add_argument("--serial", required=True, help="ODrive serial number (USB)")
    p.add_argument("--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("--timeout", type=float, default=30.0)
    p.add_argument(
        "--torques",
        default="0.008,0.010,0.012",
        help="Comma-separated motor-shaft Nm amplitudes (each done + and -)",
    )
    p.add_argument("--accel-s", type=float, default=1.0, help="Duration of constant torque")
    p.add_argument("--coast-s", type=float, default=1.0, help="Duration with torque=0")
    p.add_argument("--pause-s", type=float, default=0.4, help="Pause between trials")
    p.add_argument("--hz", type=float, default=200.0, help="Requested sample rate")
    p.add_argument(
        "--alpha-ema",
        type=float,
        default=0.25,
        help="EMA factor for d(vel)/dt (higher = less lag, more noise)",
    )
    p.add_argument("--settle-s", type=float, default=0.05, help="Skip after phase change")
    p.add_argument("--vel-min", type=float, default=0.5, help="Min |vel| turn/s used in fit")
    p.add_argument("--no-reverse", action="store_true", help="Only positive torque trials")
    p.add_argument("--out", default="", help="CSV path (default: inertia_<serial>_<ts>.csv)")
    p.add_argument("--plot", action="store_true")
    p.add_argument("--png", default="", help="Optional plot PNG path")
    args = p.parse_args()

    torques = parse_torques(args.torques)
    signs = [1.0] if args.no_reverse else [1.0, -1.0]

    print("*** Wheels must be FREE (off ground). Esc = abort. ***", flush=True)
    print(
        "Plan: {} amplitudes x {} dir, accel={:.2f}s coast={:.2f}s @ ~{:.0f} Hz".format(
            len(torques), len(signs), args.accel_s, args.coast_s, args.hz
        ),
        flush=True,
    )

    old_tty = enable_escape_key()
    odrv = None
    ax = None
    rows = []
    try:
        odrv = connect(args.serial, args.timeout)
        ax = axis(odrv, args.axis)
        ensure_torque_mode_closed_loop(ax)
        print(
            "Closed loop OK. torque_lim={} current_lim={}".format(
                ax.motor.config.torque_lim, ax.motor.config.current_lim
            ),
            flush=True,
        )

        t0 = time.time()
        trial = 0
        for tau_amp in torques:
            for s in signs:
                trial += 1
                tau = s * abs(tau_amp)
                print(
                    "Trial {}: tau={:+.4f} Nm  accel {:.2f}s then coast {:.2f}s".format(
                        trial, tau, args.accel_s, args.coast_s
                    ),
                    flush=True,
                )
                run_phase(ax, tau, args.accel_s, args.hz, "accel", trial, t0, rows)
                run_phase(ax, 0.0, args.coast_s, args.hz, "coast", trial, t0, rows)
                ax.controller.input_torque = 0.0
                sleep_interruptible(args.pause_s)

        ax.controller.input_torque = 0.0
    except UserAbort:
        print("\nAborted (Esc).", flush=True)
        if ax is not None:
            try:
                ax.controller.input_torque = 0.0
            except Exception:
                pass
    finally:
        if ax is not None:
            try:
                ax.controller.input_torque = 0.0
            except Exception:
                pass
        disable_escape_key(old_tty)

    if not rows:
        print("No samples recorded.", file=sys.stderr)
        sys.exit(1)

    enrich_rows(rows, args.alpha_ema)
    fit = fit_jbc(rows, settle_s=args.settle_s, vel_min_turns=args.vel_min)

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = args.out or "inertia_{}_{}.csv".format(args.serial, ts)
    write_csv(out, rows)

    if fit is None:
        print("Fit failed (not enough valid samples). Check CSV / freewheel / torque > friction.")
    else:
        print("--- fit: tau = J*alpha + b*omega + c*sign(omega) ---")
        print("J  = {:.6e} kg.m^2  (motor shaft)".format(fit["J"]))
        print("b  = {:.6e} Nm/(rad/s)".format(fit["b"]))
        print("c  = {:.6f} Nm  (Coulomb)".format(fit["c"]))
        print("n  = {} samples   residual RMS = {:.5f} Nm".format(fit["n"], fit["rms_nm"]))
        if fit["J"] <= 0:
            print("WARNING: J <= 0 — bad data or torque too close to friction.", file=sys.stderr)

    if args.plot or args.png:
        png = args.png or (os.path.splitext(out)[0] + ".png")
        maybe_plot(rows, fit, png)


if __name__ == "__main__":
    main()
