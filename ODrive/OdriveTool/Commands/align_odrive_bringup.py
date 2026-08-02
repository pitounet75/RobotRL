#!/usr/bin/env python3
"""
Apply a common ODrive axis0 config for balance robot bring-up.

Usage (one USB drive at a time):
  python align_odrive_bringup.py --serial LEFT_SERIAL --direction -1 --step align --save
  python align_odrive_bringup.py --serial RIGHT_SERIAL --direction 1 --step align --save

After velocity tuning + anticogging:
  python align_odrive_bringup.py --serial ... --direction ... --step limit-torque --save

See BALANCE_BRINGUP.md for the full sequence.
"""
from __future__ import print_function

import argparse
import sys

import odrive
from fibre import Event
from odrive.enums import CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH

# Match left dump / 283 config unless noted.
DEFAULT_KV = 750.0
DEFAULT_CURRENT_LIM = 10.0
# Wheel pulley 80 / motor pulley 15 → torque at wheel = motor_torque * (80/15).
DEFAULT_GEAR_MOTOR_TEETH = 15
DEFAULT_GEAR_WHEEL_TEETH = 80
DEFAULT_WHEEL_TORQUE_LIM_NM = 0.1
DEFAULT_POLE_PAIRS = 7


def gear_torque_ratio(motor_teeth, wheel_teeth):
    """Motor turns faster; wheel torque = motor_torque * wheel_teeth / motor_teeth."""
    return float(wheel_teeth) / float(motor_teeth)


def motor_torque_from_wheel(wheel_torque_nm, motor_teeth, wheel_teeth):
    return wheel_torque_nm / gear_torque_ratio(motor_teeth, wheel_teeth)
DEFAULT_CALIB_CURRENT = 5.0
DEFAULT_ENCODER_CPR = 2097152


def connect(serial):
    cancel = Event()
    print("Connecting...", flush=True)
    odrv = odrive.find_any(
        path="usb",
        serial_number=serial,
        timeout=30.0,
        search_cancellation_token=cancel,
        channel_termination_token=cancel,
    )
    if odrv is None:
        print("No ODrive found.", file=sys.stderr)
        sys.exit(1)
    return odrv


def apply_align(ax, kv, direction, current_lim):
    kt = 8.27 / kv
    m = ax.motor.config
    c = ax.controller.config
    e = ax.encoder.config
    ac = c.anticogging
    cfg = ax.config

    m.torque_constant = kt
    m.current_lim = current_lim
    m.torque_lim = float("inf")
    m.pole_pairs = DEFAULT_POLE_PAIRS
    m.calibration_current = DEFAULT_CALIB_CURRENT
    m.direction = direction

    e.cpr = DEFAULT_ENCODER_CPR
    e.bandwidth = 3000.0

    cfg.can_node_id = 0
    cfg.startup_closed_loop_control = True
    cfg.startup_motor_calibration = False
    cfg.startup_encoder_offset_calibration = False

    # Velocity-mode baseline (tune in step 2 before anticogging).
    c.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    c.input_mode = INPUT_MODE_PASSTHROUGH
    c.pos_gain = 20.0
    c.vel_gain = 0.1
    c.vel_integrator_gain = 0.32
    c.vel_limit = 1200.0
    c.enable_overspeed_error = True

    ac.anticogging_enabled = False
    ac.pre_calibrated = False

    ax.controller.input_vel = 0.0
    ax.controller.input_torque = 0.0

    print("align: torque_constant={:.6f} Nm/A (KV={})".format(kt, kv))
    print("align: current_lim={} A  direction={}".format(current_lim, direction))
    print("align: vel_gain={}  vel_integrator_gain={}".format(c.vel_gain, c.vel_integrator_gain))


def apply_limit_torque(ax, motor_torque_lim_nm, kv, current_lim, wheel_torque_lim_nm=None):
    kt = 8.27 / kv
    i_for_lim = motor_torque_lim_nm / kt
    ax.motor.config.torque_lim = motor_torque_lim_nm
    if current_lim > i_for_lim:
        ax.motor.config.current_lim = i_for_lim
    msg = "limit-torque: motor torque_lim={:.5f} Nm  current_lim={:.2f} A (Kt={:.6f})".format(
        motor_torque_lim_nm, ax.motor.config.current_lim, kt)
    if wheel_torque_lim_nm is not None:
        msg += "  (~{:.3f} Nm at wheel)".format(wheel_torque_lim_nm)
    print(msg)


def main():
    p = argparse.ArgumentParser(description="ODrive bring-up config helper (axis0).")
    p.add_argument("-s", "--serial", required=True, help="12 hex digit serial")
    p.add_argument(
        "-d",
        "--direction",
        type=int,
        required=True,
        choices=(-1, 1),
        help="motor.config.direction: -1=left wheel, +1=right wheel",
    )
    p.add_argument(
        "--step",
        required=True,
        choices=("align", "limit-torque"),
        help="align=common config; limit-torque=cap motor torque (after anticogging)",
    )
    p.add_argument("--kv", type=float, default=DEFAULT_KV, help="Motor KV (RPM/V)")
    p.add_argument(
        "--gear-motor",
        type=int,
        default=DEFAULT_GEAR_MOTOR_TEETH,
        help="Motor-side pulley teeth (or diameter mm); default 15",
    )
    p.add_argument(
        "--gear-wheel",
        type=int,
        default=DEFAULT_GEAR_WHEEL_TEETH,
        help="Wheel-side pulley teeth (or diameter mm); default 80",
    )
    p.add_argument(
        "--current-lim",
        type=float,
        default=DEFAULT_CURRENT_LIM,
        help="Motor current_lim [A] for align step",
    )
    p.add_argument(
        "--wheel-torque-lim",
        type=float,
        default=DEFAULT_WHEEL_TORQUE_LIM_NM,
        help="Max torque at wheel [Nm]; converted to motor Nm via gear ratio",
    )
    p.add_argument(
        "--motor-torque-lim",
        type=float,
        default=None,
        help="Override: motor.config.torque_lim [Nm] (skips --wheel-torque-lim)",
    )
    p.add_argument(
        "--save",
        action="store_true",
        help="Call save_configuration() after applying",
    )
    args = p.parse_args()

    odrv = connect(args.serial)
    ax = odrv.axis0

    if args.step == "align":
        apply_align(ax, args.kv, args.direction, args.current_lim)
    elif args.step == "limit-torque":
        if args.motor_torque_lim is not None:
            motor_lim = args.motor_torque_lim
            wheel_lim = motor_lim * gear_torque_ratio(args.gear_motor, args.gear_wheel)
        else:
            wheel_lim = args.wheel_torque_lim
            motor_lim = motor_torque_from_wheel(
                wheel_lim, args.gear_motor, args.gear_wheel)
        print(
            "gear {}/{} → ratio {:.3f} (wheel/motor torque)".format(
                args.gear_motor, args.gear_wheel,
                gear_torque_ratio(args.gear_motor, args.gear_wheel)))
        apply_limit_torque(
            ax, motor_lim, args.kv, args.current_lim, wheel_torque_lim_nm=wheel_lim)

    if args.save:
        print("Saving configuration...", flush=True)
        odrv.save_configuration()
        print("Done. Reboot ODrive if needed.")

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
