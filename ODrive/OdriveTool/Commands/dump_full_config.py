#!/usr/bin/env python3
"""
Dump the COMPLETE configuration of an ODrive to a single JSON file, including the
anticogging cogging_map (3600 bins/axis) that `odrivetool backup-config` drops.

What gets saved
  - Every `.config` sub-tree of the device (identical to odrivetool backup-config).
  - For each axis: controller.config.anticogging.{pre_calibrated, anticogging_enabled,
    calib_pos_threshold, calib_vel_threshold} + the full cogging_map + (for reference
    only) cogging_ratio, index, anticogging_valid.

Reading the map needs firmware that exposes either
  - controller.config.anticogging.cogging_map  (array property), or
  - controller.get_anticogging_value(index)     (this repo's ODrive_S fw).

Restore with:  restore_full_config.py -i <this file>

Usage
  python dump_full_config.py
  python dump_full_config.py -s 325834763034 -o backups/odrive_motorA.json
  python dump_full_config.py --axis 0
"""
from __future__ import print_function

import argparse
import datetime
import json
import math
import sys

import odrive

try:
    from odrive.configuration import get_dict
except Exception:  # pragma: no cover - fall back to a local reimplementation
    import fibre.remote_object as _ro

    def get_dict(obj, is_config_object):
        result = {}
        for (k, v) in obj._remote_attributes.items():
            if isinstance(v, _ro.RemoteProperty) and is_config_object:
                result[k] = v.get_value()
            elif isinstance(v, _ro.RemoteObject):
                sub = get_dict(v, k == "config")
                if sub != {}:
                    result[k] = sub
        return result


MAP_BINS = 3600

# anticogging leaf props worth saving. Value = writable-on-restore?
ANTICOGGING_PROPS = {
    "pre_calibrated": True,
    "anticogging_enabled": True,
    "calib_pos_threshold": True,
    "calib_vel_threshold": True,
    "cogging_ratio": False,   # readonly - reference only
    "index": False,           # readonly - reference only
}


def read_cogging_map(ax):
    """Return (list[float], source_str). Raises RuntimeError if fw exposes neither path."""
    ac = ax.controller.config.anticogging
    try:
        if hasattr(ac, "cogging_map"):
            return [float(x) for x in ac.cogging_map], "config.anticogging.cogging_map"
    except Exception:
        pass

    getter = getattr(ax.controller, "get_anticogging_value", None)
    if getter is None:
        raise RuntimeError(
            "Firmware exposes neither config.anticogging.cogging_map nor "
            "controller.get_anticogging_value - cannot read the map. Flash the "
            "ODrive_S firmware from this repo (adds get/set_anticogging_value)."
        )
    out = []
    for i in range(MAP_BINS):
        if i and i % 600 == 0:
            print("  reading bin {}/{}...".format(i, MAP_BINS), flush=True)
        try:
            v = getter(index=i)
        except TypeError:
            v = getter(i)
        if isinstance(v, (list, tuple)):
            v = v[0]
        out.append(float(v))
    return out, "controller.get_anticogging_value"


def map_stats(values):
    n = len(values)
    if n == 0:
        return "empty"
    mean = sum(values) / n
    var = sum((x - mean) ** 2 for x in values) / n
    return "bins={} min={:.6g} max={:.6g} mean={:.6g} std={:.6g}".format(
        n, min(values), max(values), mean, math.sqrt(var)
    )


def axis_indices(odrv, requested):
    if requested is not None:
        return [requested]
    idx = []
    for i in (0, 1):
        if hasattr(odrv, "axis{}".format(i)):
            idx.append(i)
    return idx or [0]


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-s", "--serial", default=None,
                   help="12 hex digit serial (like odrivetool --serial-number)")
    p.add_argument("-a", "--axis", type=int, default=None, choices=(0, 1),
                   help="Only dump this axis' anticogging (default: all present)")
    p.add_argument("-o", "--output", default=None,
                   help="Output .json path (default: odrive_full_<serial>_<stamp>.json)")
    p.add_argument("--no-map", action="store_true",
                   help="Skip the cogging_map (flags/thresholds still saved)")
    args = p.parse_args()

    print("Connecting...", flush=True)
    odrv = odrive.find_any(path="usb", serial_number=args.serial, timeout=30.0)
    if odrv is None:
        print("No ODrive found.", file=sys.stderr)
        return 1

    serial = "{:012X}".format(int(odrv.serial_number))
    print("Connected to ODrive {}".format(serial), flush=True)

    doc = {
        "format": "odrive-full-config",
        "version": 1,
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "odrive": {
            "serial_number": serial,
            "fw_version": [int(odrv.fw_version_major), int(odrv.fw_version_minor),
                           int(odrv.fw_version_revision)],
            "hw_version": [int(odrv.hw_version_major), int(odrv.hw_version_minor),
                           int(odrv.hw_version_variant)],
            "user_config_loaded": bool(odrv.user_config_loaded),
        },
        "config": get_dict(odrv, False),
        "anticogging": {},
    }

    for i in axis_indices(odrv, args.axis):
        ax = getattr(odrv, "axis{}".format(i))
        ac = ax.controller.config.anticogging
        entry = {}
        for name in ANTICOGGING_PROPS:
            try:
                entry[name] = ac._remote_attributes[name].get_value()
            except Exception as e:
                entry[name] = None
                print("  axis{}: could not read anticogging.{}: {}".format(i, name, e),
                      file=sys.stderr)
        try:
            entry["anticogging_valid"] = bool(ax.controller.anticogging_valid)
        except Exception:
            pass

        if args.no_map:
            entry["cogging_map"] = None
        else:
            print("axis{}: reading cogging map...".format(i), flush=True)
            values, source = read_cogging_map(ax)
            entry["cogging_map"] = values
            entry["cogging_map_source"] = source
            print("axis{}: {} ({})".format(i, map_stats(values), source), flush=True)

        doc["anticogging"][str(i)] = entry

    out = args.output or "odrive_full_{}_{}.json".format(
        serial, datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    with open(out, "w") as f:
        json.dump(doc, f, indent=1)
    print("Wrote {}".format(out), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
