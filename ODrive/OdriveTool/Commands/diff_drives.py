"""Diff the live config of two ODrives side by side.

Run from an odrivetool session that already has both boards connected:

    %run -i diff_drives.py

The -i is required: without it IPython runs the file in an empty namespace and
odrv0/odrv1 are invisible. Fresh dumps are also written next to the Configs/
history so they can be re-diffed later.

Runtime state (currents, positions, timing logs, counters) is filtered out: it
differs on every read and drowns the configuration differences that matter.
"""

import math
import os

import fibre.remote_object as ro

# Substrings marking values that change on their own between two reads.
VOLATILE_MARKERS = (
    "ibus", "vbus_voltage", "loop_counter", "serial_number", "uptime",
    "current_state", "requested_state", "error", "armed_state", "is_ready",
    "timing_log", "system_stats", "n_evt", "temperature", "thermistor",
    "Id_measured", "Iq_measured", "Iq_setpoint", "Id_setpoint", "I_bus",
    "current_control", "DC_calib", "final_v_", "phase", "effective_current_lim",
    "max_allowed_current", "overcurrent", "pos_estimate", "vel_estimate",
    "pos_abs", "pos_circular", "pos_cpr", "count_in_cpr", "shadow_count",
    "interpolation", "spi_error_rate", "input_pos", "input_vel", "input_torque",
    "pos_setpoint", "vel_setpoint", "torque_setpoint", "vel_integrator",
    "trajectory_done", "mechanical_power", "electrical_power", "sensorless",
    "step_dir_active", "lockin_state", "is_homed", "saturated", "misconfigured",
    "anticogging.index", "brake_resistor_armed", "test_property", "disarm",
)


def format_value(v):
    if isinstance(v, float):
        if math.isinf(v):
            return "float('inf')" if v > 0 else "float('-inf')"
        if math.isnan(v):
            return "float('nan')"
    return repr(v)


def walk(obj, prefix="", out=None):
    if out is None:
        out = {}
    for name, attr in sorted(obj._remote_attributes.items()):
        path = f"{prefix}.{name}" if prefix else name
        if isinstance(attr, ro.RemoteProperty):
            if not attr._can_read:
                continue
            try:
                out[path] = format_value(attr.get_value())
            except Exception as exc:
                out[path] = f"<read error: {exc}>"
        elif isinstance(attr, ro.RemoteObject):
            walk(attr, path, out)
    return out


def is_volatile(key):
    return any(marker in key for marker in VOLATILE_MARKERS)


def write_dump(values, path):
    with open(path, "w", encoding="utf-8") as fh:
        for key in sorted(values):
            fh.write(f"odrv0.{key} = {values[key]}\n")


def main(drive_a, drive_b, label_a="L", label_b="R", out_dir=None):
    a = walk(drive_a)
    b = walk(drive_b)
    print(f"{label_a}: {len(a)} cles    {label_b}: {len(b)} cles")

    if out_dir:
        write_dump(a, os.path.join(out_dir, f"fresh_{label_a}.txt"))
        write_dump(b, os.path.join(out_dir, f"fresh_{label_b}.txt"))
        print(f"dumps ecrits dans {out_dir}")

    keys = sorted(set(a) | set(b))
    diff = [k for k in keys if a.get(k) != b.get(k) and not is_volatile(k)]
    skipped = sum(1 for k in keys if a.get(k) != b.get(k) and is_volatile(k))

    print(f"\n=== {len(diff)} differences de config "
          f"({skipped} champs volatils ignores) ===\n")
    if not diff:
        print("  Les deux drives sont identiques sur toute la configuration.")
        return diff

    width = max(len(k) for k in diff)
    for k in diff:
        print(f"  {k:<{width}}  {label_a}={a.get(k, '<absent>')!s:<24} "
              f"{label_b}={b.get(k, '<absent>')}")
    return diff


if __name__ == "__main__":
    g = globals()
    if "odrv0" not in g or "odrv1" not in g:
        raise SystemExit(
            "odrv0 et odrv1 introuvables. Depuis odrivetool, avec les deux "
            "cartes connectees, relancez avec le drapeau -i :\n"
            "    %run -i diff_drives.py"
        )
    main(g["odrv0"], g["odrv1"], out_dir=os.path.join(os.path.dirname(
        os.path.abspath(__file__)), "Configs"))
