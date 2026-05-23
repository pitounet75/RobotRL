#!/usr/bin/env python3
"""
Print controller / motor / encoder / anticogging parameters in one shot (odrivetool + USB).
Usage:
  python dump_anticogging_tuning_params.py
  python dump_anticogging_tuning_params.py --serial 306A396A3235
  python dump_anticogging_tuning_params.py --axis 1
"""
from __future__ import print_function
import argparse
import sys

import odrive
from fibre import Event


def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        "-s",
        "--serial",
        default=None,
        help="12 hex digit serial (same as odrivetool --serial-number)",
    )
    p.add_argument(
        "-a",
        "--axis",
        type=int,
        default=0,
        choices=(0, 1),
        help="Axis index (default 0)",
    )
    args = p.parse_args()

    cancel = Event()
    print("Connecting...", flush=True)
    odrv = odrive.find_any(
        path="usb",
        serial_number=args.serial,
        timeout=30.0,
        search_cancellation_token=cancel,
        channel_termination_token=cancel,
    )
    if odrv is None:
        print("No ODrive found.", file=sys.stderr)
        return 1

    ax = odrv.axis0 if args.axis == 0 else odrv.axis1
    c = ax.controller
    ac = c.config.anticogging
    m = ax.motor
    e = ax.encoder

    lines = [
        ("odrv0.axis{}.controller.config.control_mode".format(args.axis), c.config.control_mode),
        ("odrv0.axis{}.controller.config.input_mode".format(args.axis), c.config.input_mode),
        ("odrv0.axis{}.controller.config.pos_gain".format(args.axis), c.config.pos_gain),
        ("odrv0.axis{}.controller.config.vel_gain".format(args.axis), c.config.vel_gain),
        ("odrv0.axis{}.controller.config.vel_integrator_gain".format(args.axis), c.config.vel_integrator_gain),
        ("odrv0.axis{}.controller.config.vel_limit".format(args.axis), c.config.vel_limit),
        ("odrv0.axis{}.controller.config.vel_limit_tolerance".format(args.axis), c.config.vel_limit_tolerance),
        ("odrv0.axis{}.controller.config.enable_overspeed_error".format(args.axis), c.config.enable_overspeed_error),
    ]
    lines += [
        ("odrv0.axis{}.controller.config.circular_setpoints".format(args.axis), c.config.circular_setpoints),
        ("odrv0.axis{}.controller.config.circular_setpoint_range".format(args.axis), c.config.circular_setpoint_range),
        ("odrv0.axis{}.controller.config.anticogging.calib_pos_threshold".format(args.axis), ac.calib_pos_threshold),
        ("odrv0.axis{}.controller.config.anticogging.calib_vel_threshold".format(args.axis), ac.calib_vel_threshold),
        ("odrv0.axis{}.controller.config.anticogging.anticogging_enabled".format(args.axis), ac.anticogging_enabled),
        ("odrv0.axis{}.controller.anticogging_valid".format(args.axis), c.anticogging_valid),
        ("odrv0.axis{}.controller.config.anticogging.calib_anticogging".format(args.axis), ac.calib_anticogging),
        ("odrv0.axis{}.controller.config.anticogging.index".format(args.axis), ac.index),
        ("odrv0.axis{}.motor.config.current_lim".format(args.axis), m.config.current_lim),
        ("odrv0.axis{}.encoder.config.cpr".format(args.axis), e.config.cpr),
        ("odrv0.axis{}.encoder.config.bandwidth".format(args.axis), e.config.bandwidth),
        ("odrv0.axis{}.encoder.error".format(args.axis), hex(e.error) if e.error else e.error),
        ("odrv0.axis{}.motor.error".format(args.axis), hex(m.error) if m.error else m.error),
        ("odrv0.axis{}.controller.error".format(args.axis), hex(c.error) if c.error else c.error),
        ("odrv0.axis{}.error".format(args.axis), hex(ax.error) if ax.error else ax.error),
    ]

    w = max(len(n) for n, _ in lines)
    for name, val in lines:
        print("{:{width}} = {}".format(name, val, width=w))

    print("\n# Paste the block above when asking for tuning help.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
