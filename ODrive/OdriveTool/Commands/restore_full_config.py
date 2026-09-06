#!/usr/bin/env python3
"""
Restore a full ODrive configuration produced by dump_full_config.py, including the
anticogging cogging_map (which odrivetool restore-config cannot write back).

Sequence
  1. Push the `.config` tree back            (like odrivetool restore-config)
  2. Per axis: disable anticogging, stream the 3600 map bins via
     controller.set_anticogging_value(i, v), read-back verify
  3. Re-apply anticogging flags/thresholds from the file
  4. save_configuration()  (unless --no-save)
  5. reboot()              (unless --no-reboot) - on boot the ODrive_S fw sets
     anticogging_valid from anticogging.pre_calibrated

Streaming the map needs firmware with controller.set_anticogging_value (this repo's
ODrive_S fw). Without it, use --skip-map (config-only restore).

Usage
  python restore_full_config.py -i backups/odrive_motorA.json
  python restore_full_config.py -i backups/odrive_motorA.json --dry-run
  python restore_full_config.py -i backups/odrive_motorA.json --skip-map
  python restore_full_config.py -i backups/odrive_motorA.json --axis 0 --no-reboot
"""
from __future__ import print_function

import argparse
import json
import sys
import time

import odrive

try:
    from odrive.configuration import set_dict
except Exception:  # pragma: no cover - local reimplementation
    import fibre.remote_object as _ro

    def set_dict(obj, path, config_dict):
        errors = []
        for (k, v) in config_dict.items():
            name = path + ("." if path != "" else "") + k
            if k not in obj._remote_attributes:
                errors.append("Could not restore {}: not found on device".format(name))
                continue
            attr = obj._remote_attributes[k]
            if isinstance(attr, _ro.RemoteObject):
                errors += set_dict(attr, name, v)
            else:
                try:
                    attr.set_value(v)
                except Exception as ex:
                    errors.append("Could not restore {}: {}".format(name, ex))
        return errors


MAP_BINS = 3600
WRITABLE_ANTICOGGING = ("calib_pos_threshold", "calib_vel_threshold",
                        "anticogging_enabled", "pre_calibrated")


def close_enough(a, b):
    # float32 round-trip tolerance
    return abs(a - b) <= 1e-6 + 1e-4 * abs(b)


def set_map_value(ctrl, i, v):
    try:
        ctrl.set_anticogging_value(index=i, value=v)
    except TypeError:
        ctrl.set_anticogging_value(i, v)


def get_map_value(ctrl, i):
    try:
        v = ctrl.get_anticogging_value(index=i)
    except TypeError:
        v = ctrl.get_anticogging_value(i)
    if isinstance(v, (list, tuple)):
        v = v[0]
    return float(v)


def restore_map(ax, values, dry_run):
    ctrl = ax.controller
    if not hasattr(ctrl, "set_anticogging_value"):
        raise RuntimeError(
            "Firmware has no controller.set_anticogging_value - flash the ODrive_S "
            "firmware from this repo, or re-run with --skip-map."
        )
    if len(values) != MAP_BINS:
        raise RuntimeError("map has {} bins, expected {}".format(len(values), MAP_BINS))

    ac = ctrl.config.anticogging
    if dry_run:
        diff = sum(1 for i, v in enumerate(values)
                   if not close_enough(get_map_value(ctrl, i), v))
        print("  [dry-run] {}/{} bins differ from current map".format(diff, MAP_BINS))
        return

    ac.anticogging_enabled = False       # don't feed a half-written map to control
    ac.pre_calibrated = False            # a reboot mid-stream must not activate it
    time.sleep(0.05)

    for i, v in enumerate(values):
        set_map_value(ctrl, i, v)
        if i and i % 600 == 0:
            print("  wrote bin {}/{}...".format(i, MAP_BINS), flush=True)

    bad = [i for i, v in enumerate(values) if not close_enough(get_map_value(ctrl, i), v)]
    if bad:
        raise RuntimeError("verify failed on {} bins (first: {})".format(len(bad), bad[:8]))
    print("  map verified ({} bins)".format(MAP_BINS), flush=True)
    # final anticogging_enabled / pre_calibrated come from the file, applied by caller


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-i", "--input", required=True, help="JSON file from dump_full_config.py")
    p.add_argument("-s", "--serial", default=None, help="Target ODrive serial (12 hex)")
    p.add_argument("-a", "--axis", type=int, default=None, choices=(0, 1),
                   help="Only restore this axis (default: every axis in the file)")
    p.add_argument("--skip-config", action="store_true", help="Do not touch the .config tree")
    p.add_argument("--skip-map", action="store_true", help="Do not restore the cogging map")
    p.add_argument("--dry-run", action="store_true",
                   help="Report only; no writes, no save, no reboot")
    p.add_argument("--no-save", action="store_true", help="Apply to RAM but skip save_configuration()")
    p.add_argument("--no-reboot", action="store_true", help="Skip the final reboot")
    p.add_argument("--force-serial", action="store_true",
                   help="Allow restoring onto an ODrive whose serial differs from the file")
    args = p.parse_args()

    with open(args.input) as f:
        doc = json.load(f)
    if doc.get("format") != "odrive-full-config":
        print("Not an odrive-full-config file: {}".format(args.input), file=sys.stderr)
        return 1

    print("Connecting...", flush=True)
    odrv = odrive.find_any(path="usb", serial_number=args.serial, timeout=30.0)
    if odrv is None:
        print("No ODrive found.", file=sys.stderr)
        return 1
    serial = "{:012X}".format(int(odrv.serial_number))
    file_serial = doc.get("odrive", {}).get("serial_number")
    print("Connected to ODrive {} (file was dumped from {})".format(serial, file_serial))
    if file_serial and serial != file_serial and not args.force_serial:
        print("Serial mismatch - pass --force-serial to restore anyway.", file=sys.stderr)
        return 1

    dry = args.dry_run

    # 1. config tree
    if not args.skip_config and "config" in doc:
        print("Restoring .config tree{}...".format(" [dry-run]" if dry else ""))
        if not dry:
            errors = set_dict(odrv, "", doc["config"])
            for e in errors:
                print("  " + e)
            if errors:
                print("  {} property(ies) could not be restored (see above).".format(len(errors)))

    # 2/3. anticogging per axis
    ac_doc = doc.get("anticogging", {})
    axes = [str(args.axis)] if args.axis is not None else sorted(ac_doc.keys())
    for key in axes:
        if key not in ac_doc:
            print("axis{}: nothing in file, skipped".format(key))
            continue
        entry = ac_doc[key]
        ax = getattr(odrv, "axis{}".format(key), None)
        if ax is None:
            print("axis{}: not present on device, skipped".format(key), file=sys.stderr)
            continue
        ac = ax.controller.config.anticogging

        if not args.skip_map and entry.get("cogging_map"):
            print("axis{}: restoring cogging map{}...".format(key, " [dry-run]" if dry else ""))
            restore_map(ax, entry["cogging_map"], dry)
        elif not args.skip_map:
            print("axis{}: file has no cogging_map, only flags".format(key))

        if not dry:
            for name in WRITABLE_ANTICOGGING:
                if entry.get(name) is not None:
                    try:
                        ac._remote_attributes[name].set_value(entry[name])
                    except Exception as e:
                        print("  axis{}: could not set anticogging.{}: {}".format(key, name, e),
                              file=sys.stderr)
            print("axis{}: flags -> pre_calibrated={} anticogging_enabled={}".format(
                key, ac.pre_calibrated, ac.anticogging_enabled))

    if dry:
        print("Dry run complete - nothing written.")
        return 0

    # 4. persist
    if args.no_save:
        print("--no-save: configuration left in RAM only (lost on reboot).")
    else:
        if not bool(odrv.user_config_loaded):
            print("WARNING: user_config_loaded is False (NVM not loaded / factory "
                  "defaults in RAM). Refusing to save. Fix power/USB and retry.",
                  file=sys.stderr)
            return 2
        print("Saving configuration...")
        try:
            odrv.save_configuration()
        except Exception:
            # save_configuration drops the USB link on some fw; treat as expected
            pass
        time.sleep(0.5)

    # 5. reboot
    if not args.no_reboot and not args.no_save:
        print("Rebooting...")
        try:
            odrv.reboot()
        except Exception:
            pass

    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
