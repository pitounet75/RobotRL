#!/usr/bin/env python3
"""
Ramp motor-shaft torque commands and estimate torque_constant (Kt) from Iq.

Use with a known load on a string (pulley test): watch when the mass lifts,
or pass --interactive to mark the lift step manually.

Default ramp: 0.055 → 0.063 Nm by steps of 0.001 (motor shaft).
Press Esc anytime to abort (torque reset to 0).

Usage:
  python refine_torque_constant.py --serial 55517627953204
  python refine_torque_constant.py --serial ... --interactive --mass-kg 0.251
  python refine_torque_constant.py --serial ... --apply --save

See measure_torque_pulley.txt for odrivetool one-liner equivalent.
"""
from __future__ import print_function

import argparse
import statistics
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


class UserAbort(Exception):
    """Raised when the user presses Esc to stop the ramp."""


def escape_pressed():
    """Esc detection without consuming stdin (y/n input stays intact on Windows)."""
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
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            return True
    return False


def sleep_interruptible(duration_s, period_s=0.05):
    end = time.time() + duration_s
    while True:
        if escape_pressed():
            raise UserAbort()
        remaining = end - time.time()
        if remaining <= 0:
            break
        time.sleep(min(period_s, remaining))


def enable_escape_key():
    """Raw stdin on Unix so Esc works during ramp waits."""
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


DEFAULT_GEAR_MOTOR = 15
DEFAULT_GEAR_WHEEL = 80
DEFAULT_MASS_KG = 0.251
DEFAULT_WHEEL_RADIUS_M = 0.04


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


def gear_ratio(motor_teeth, wheel_teeth):
    return float(wheel_teeth) / float(motor_teeth)


def torque_steps(t_min, t_max, step):
    if step <= 0:
        raise ValueError("step must be > 0")
    values = []
    t = t_min
    # Include t_max with a small epsilon for float drift.
    while t <= t_max + step * 0.5:
        values.append(round(t, 6))
        t += step
    return values


def ensure_torque_mode_closed_loop(ax):
    ax.clear_errors()
    ax.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
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


def sample_iq(ax, duration_s, period_s):
    samples = []
    end = time.time() + duration_s
    while time.time() < end:
        if escape_pressed():
            raise UserAbort()
        samples.append(float(ax.motor.current_control.Iq_measured))
        sleep_interruptible(period_s)
    if not samples:
        return 0.0, 0.0
    return statistics.mean(samples), statistics.stdev(samples) if len(samples) > 1 else 0.0


def wheel_torque_from_motor(tau_motor, ratio):
    return tau_motor * ratio


def motor_torque_from_load(mass_kg, wheel_radius_m, ratio):
    tau_wheel = mass_kg * 9.81 * wheel_radius_m
    return tau_wheel / ratio


def prompt_lift():
    while True:
        try:
            ans = input("  Mass lifted? [y/n/q]: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            raise UserAbort()
        if ans in ("y", "yes", "o", "oui"):
            return True
        if ans in ("n", "no", "non"):
            return False
        if ans in ("q", "quit"):
            raise UserAbort()
        print("  Invalid answer — type y or n.", flush=True)


def print_header(kt_cfg, ratio):
    print("")
    print("motor-shaft torque ramp | gear ratio wheel/motor = {:.3f}".format(ratio))
    print("configured torque_constant = {:.6f} Nm/A".format(kt_cfg))
    print("-" * 72)
    print(
        "{:>8} {:>8} {:>8} {:>10} {:>10} {:>8}".format(
            "cmd_Nm", "Iq_A", "Iq_std", "Kt_eff", "tau_wheel", "lift"
        )
    )
    print("-" * 72)


def main():
    p = argparse.ArgumentParser(
        description="Refine ODrive motor torque_constant with a pulley/load test."
    )
    p.add_argument("-s", "--serial", required=True, help="12 hex digit serial")
    p.add_argument("--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("--timeout", type=float, default=30.0)
    p.add_argument("--min", dest="t_min", type=float, default=0.055, help="Min motor torque [Nm]")
    p.add_argument("--max", dest="t_max", type=float, default=0.063, help="Max motor torque [Nm]")
    p.add_argument(
        "--step",
        type=float,
        default=0.001,
        help="Torque step [Nm]",
    )
    p.add_argument("--hold-s", type=float, default=0.4, help="Hold each step [s]")
    p.add_argument("--pause-s", type=float, default=2.0, help="Pause at 0 Nm between steps [s]")
    p.add_argument("--sample-period-s", type=float, default=0.05)
    p.add_argument("--gear-motor", type=int, default=DEFAULT_GEAR_MOTOR)
    p.add_argument("--gear-wheel", type=int, default=DEFAULT_GEAR_WHEEL)
    p.add_argument(
        "--mass-kg",
        type=float,
        default=DEFAULT_MASS_KG,
        help="Known load mass for Kt from physics (0 to skip)",
    )
    p.add_argument(
        "--wheel-radius-m",
        type=float,
        default=DEFAULT_WHEEL_RADIUS_M,
        help="Pulley/wheel radius [m]",
    )
    p.add_argument(
        "--interactive",
        action="store_true",
        help="Ask after each step whether the mass lifted",
    )
    p.add_argument(
        "--apply",
        action="store_true",
        help="Write recommended torque_constant to motor.config",
    )
    p.add_argument(
        "--save",
        action="store_true",
        help="Call save_configuration() after --apply",
    )
    args = p.parse_args()

    ratio = gear_ratio(args.gear_motor, args.gear_wheel)
    steps = torque_steps(args.t_min, args.t_max, args.step)
    if not steps:
        print("No torque steps in range.", file=sys.stderr)
        return 1

    odrv = connect(args.serial, args.timeout)
    ax = axis(odrv, args.axis)
    ensure_torque_mode_closed_loop(ax)

    kt_cfg = float(ax.motor.config.torque_constant)
    tau_motor_load = None
    if args.mass_kg > 0:
        tau_motor_load = motor_torque_from_load(
            args.mass_kg, args.wheel_radius_m, ratio
        )
        tau_wheel_load = args.mass_kg * 9.81 * args.wheel_radius_m
        print(
            "Load {:.3f} kg, r={:.3f} m → tau_wheel={:.4f} Nm, tau_motor={:.5f} Nm".format(
                args.mass_kg, args.wheel_radius_m, tau_wheel_load, tau_motor_load
            )
        )

    print_header(kt_cfg, ratio)
    print("Press Esc during ramp/pause to stop. Interactive: y/n + Enter (q to quit).", flush=True)

    rows = []
    lift_row = None
    aborted = False
    tty_old = enable_escape_key()

    try:
        for cmd in steps:
            if escape_pressed():
                raise UserAbort()
            ax.controller.input_torque = cmd
            iq_mean, iq_std = sample_iq(ax, args.hold_s, args.sample_period_s)
            kt_eff = cmd / abs(iq_mean) if abs(iq_mean) > 0.005 else float("nan")
            tau_wheel = wheel_torque_from_motor(cmd, ratio)

            lifted = False
            if args.interactive:
                print(
                    "cmd={:.4f} Nm  Iq={:.3f} A  tau_wheel={:.4f} Nm".format(
                        cmd, iq_mean, tau_wheel
                    )
                )
                lifted = prompt_lift()
            else:
                lifted = False

            mark = "yes" if lifted else ""
            print(
                "{:8.4f} {:8.3f} {:8.4f} {:10.4f} {:10.4f} {:>8}".format(
                    cmd, iq_mean, iq_std, kt_eff, tau_wheel, mark
                )
            )
            row = {
                "cmd": cmd,
                "iq": iq_mean,
                "iq_std": iq_std,
                "kt_eff": kt_eff,
                "lifted": lifted,
            }
            rows.append(row)
            if lifted and lift_row is None:
                lift_row = row

            ax.controller.input_torque = 0.0
            sleep_interruptible(args.pause_s)

    except UserAbort:
        aborted = True
        print("\nStopped by user (Esc).", flush=True)

    finally:
        disable_escape_key(tty_old)
        ax.controller.input_torque = 0.0
        print("-" * 72)
        print("Torque command reset to 0 Nm.")

    if not rows:
        if aborted:
            return 0
        print("No measurements recorded.")
        return 0

    # Recommend Kt
    kt_candidates = []

    valid_kt = [r["kt_eff"] for r in rows if r["kt_eff"] == r["kt_eff"]]
    if valid_kt:
        kt_candidates.append(("Kt mean (cmd/Iq all steps)", statistics.mean(valid_kt)))

    if lift_row is not None and abs(lift_row["iq"]) > 0.005:
        kt_lift_cmd = lift_row["cmd"] / abs(lift_row["iq"])
        kt_candidates.append(
            ("Kt at lift (cmd/Iq @ step {:.4f} Nm)".format(lift_row["cmd"]), kt_lift_cmd)
        )

    if tau_motor_load is not None:
        # Best physics-based estimate: use lowest step where Iq suggests lift or user marked lift
        ref = lift_row
        if ref is None:
            # Heuristic: step with highest Iq (likely fighting load near threshold)
            ref = max(rows, key=lambda r: abs(r["iq"]))
        if abs(ref["iq"]) > 0.005:
            kt_load = tau_motor_load / abs(ref["iq"])
            label = "Kt from load (tau_motor/Iq"
            if lift_row is not None:
                label += " @ lift"
            else:
                label += " @ max-Iq step"
            label += ")"
            kt_candidates.append((label, kt_load))

    if not kt_candidates:
        print("No Kt estimate (Iq too small?).")
        return 0

    print("")
    print("Kt estimates (motor shaft):")
    for label, val in kt_candidates:
        print("  {:40s} {:.6f} Nm/A".format(label + ":", val))

    # Prefer load-based if available, else lift cmd/Iq, else mean
    if tau_motor_load is not None and any("from load" in l for l, _ in kt_candidates):
        kt_rec = [v for l, v in kt_candidates if "from load" in l][0]
        rec_label = "load-based"
    elif lift_row is not None:
        kt_rec = lift_row["cmd"] / abs(lift_row["iq"])
        rec_label = "lift step cmd/Iq"
    else:
        kt_rec = statistics.mean(valid_kt)
        rec_label = "mean cmd/Iq"

    kv_rec = 8.27 / kt_rec
    print("")
    print("Recommended: torque_constant = {:.6f} Nm/A  (KV ≈ {:.0f})  [{}]".format(
        kt_rec, kv_rec, rec_label))
    print("Current:     torque_constant = {:.6f} Nm/A  (KV ≈ {:.0f})".format(
        kt_cfg, 8.27 / kt_cfg if kt_cfg > 0 else float("inf")))

    if args.apply:
        ax.motor.config.torque_constant = kt_rec
        print("Applied motor.config.torque_constant = {:.6f}".format(kt_rec))
        if args.save:
            print("Saving configuration...", flush=True)
            odrv.save_configuration()
            print("Saved. Reboot ODrive if needed.")

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
