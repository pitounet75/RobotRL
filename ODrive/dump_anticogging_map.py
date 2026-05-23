#!/usr/bin/env python3
"""
Dump anticogging cogging_map (3600 bins) from an ODrive over USB.

Many firmware builds do not expose `config.anticogging.cogging_map` on the wire.
This script uses `controller.get_anticogging_value(i)` when available (add it to
odrive-interface.yaml + implement on Controller, then rebuild/flash).

Usage:
  python dump_anticogging_map.py
  python dump_anticogging_map.py --serial 306A396A3235 --axis 1
  python dump_anticogging_map.py -o run1.csv
  python dump_anticogging_map.py -o run1.npy
  python dump_anticogging_map.py -o out/base_name   # writes out/base_name.csv (+ .npy if numpy)
"""
from __future__ import print_function

import argparse
import csv
import math
import sys

import odrive
from fibre import Event

try:
    import numpy as np
except ImportError:
    np = None


def _read_map_property(ac):
    return [float(x) for x in ac.cogging_map]


def _read_map_rpc(ctrl, progress_every=600):
    if not hasattr(ctrl, "get_anticogging_value"):
        return None
    getter = ctrl.get_anticogging_value
    out = []
    for i in range(3600):
        if progress_every and i and i % progress_every == 0:
            print("  reading bin {}/3600...".format(i), flush=True)
        # fibre/odrive: pass index as keyword or positional depending on version
        try:
            v = getter(index=i)
        except TypeError:
            v = getter(i)
        if isinstance(v, (list, tuple)):
            v = v[0]
        out.append(float(v))
    return out


def read_map(ax):
    ac = ax.controller.config.anticogging
    ctrl = ax.controller

    try:
        if hasattr(ac, "cogging_map"):
            return _read_map_property(ac), "config.anticogging.cogging_map"
    except AttributeError:
        pass

    rpc = _read_map_rpc(ctrl)
    if rpc is not None:
        return rpc, "controller.get_anticogging_value"

    raise RuntimeError(
        "Cannot read cogging map: firmware exposes neither config.anticogging.cogging_map "
        "nor controller.get_anticogging_value. Rebuild/flash firmware with get_anticogging_value "
        "in odrive-interface.yaml (see ODrive_S-fw project)."
    )


def _stats(values):
    n = len(values)
    if n == 0:
        return n, float("nan"), float("nan"), float("nan"), float("nan")
    s = sum(values)
    mean = s / n
    var = sum((x - mean) ** 2 for x in values) / n
    return n, min(values), max(values), mean, math.sqrt(var)


def main():
    p = argparse.ArgumentParser(description="Dump anticogging cogging_map to CSV/NPY.")
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
    p.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output file (.csv or .npy) or base path without extension for csv (+ npy)",
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

    try:
        values, source = read_map(ax)
    except RuntimeError as e:
        print(str(e), file=sys.stderr)
        return 1

    print("Source: {}".format(source), flush=True)

    n, vmin, vmax, mean, rms = _stats(values)

    print(
        "axis{}: bins={} anticogging_valid={} min={:.6g} max={:.6g} mean={:.6g} std={:.6g}".format(
            args.axis,
            n,
            ax.controller.anticogging_valid,
            vmin,
            vmax,
            mean,
            rms,
        ),
        flush=True,
    )

    if n != 3600:
        print("Warning: expected 3600 samples, got {}".format(n), file=sys.stderr)

    out = args.output
    if out is None:
        out = "anticogging_map_axis{}".format(args.axis)

    def write_csv(path):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["bin", "torque"])
            for i, v in enumerate(values):
                w.writerow([i, v])
        print("Wrote {}".format(path), flush=True)

    def write_npy(path):
        if np is None:
            print("numpy not installed; skip {}".format(path), file=sys.stderr)
            return
        np.save(path, np.asarray(values, dtype=np.float64))
        print("Wrote {}".format(path), flush=True)

    if out.lower().endswith(".csv"):
        write_csv(out)
    elif out.lower().endswith(".npy"):
        if np is None:
            print("numpy required for .npy output.", file=sys.stderr)
            return 1
        write_npy(out)
    else:
        csv_path = out + ".csv"
        write_csv(csv_path)
        write_npy(out + ".npy")

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
