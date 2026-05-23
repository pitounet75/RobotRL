#!/usr/bin/env python3
"""
Collect ODrive parameters + cogging map statistics for anticogging troubleshooting.

Run from your ODrive venv (same as odrivetool):
  python diagnose_anticogging.py
  python diagnose_anticogging.py --serial 306A396A3235 --axis 0
  python diagnose_anticogging.py --json anticogging_diag.json

Paste the printed block (or attach the JSON) when asking for help.
"""
from __future__ import print_function

import argparse
import json
import math
import sys

import odrive
from fibre import Event

# Optional: human-readable enum names (pip odrive package may expose enums)
try:
    import odrive.enums as od_enums
except Exception:
    od_enums = None


def _enum_name(prefix, value):
    if od_enums is None:
        return None
    for name in dir(od_enums):
        if name.startswith(prefix) and getattr(od_enums, name, None) == value:
            return name
    return None


def _read_map_via_rpc(ctrl, progress_every=0):
    if not hasattr(ctrl, "get_anticogging_value"):
        return None
    getter = ctrl.get_anticogging_value
    out = []
    for i in range(3600):
        if progress_every and i and i % progress_every == 0:
            print("  reading map bin {}/3600...".format(i), flush=True)
        try:
            v = getter(index=i)
        except TypeError:
            v = getter(i)
        if isinstance(v, (list, tuple)):
            v = v[0]
        out.append(float(v))
    return out


def read_cogging_map(axis):
    """Return (list of 3600 floats or None, source string)."""
    ac = axis.controller.config.anticogging
    try:
        if hasattr(ac, "cogging_map"):
            return [float(x) for x in ac.cogging_map], "config.anticogging.cogging_map"
    except Exception:
        pass
    rpc = _read_map_via_rpc(axis.controller, progress_every=900)
    if rpc is not None:
        return rpc, "controller.get_anticogging_value"
    return None, None


def map_diagnostics(values):
    """Statistics relevant to jerk / bin-to-bin cliffs."""
    if not values or len(values) != 3600:
        return None
    n = len(values)
    mean = sum(values) / n
    var = sum((x - mean) ** 2 for x in values) / n
    std = math.sqrt(var)
    vmin, vmax = min(values), max(values)

    diffs = []
    for i in range(n):
        j = (i + 1) % n
        diffs.append(abs(values[j] - values[i]))
    max_adj = max(diffs)
    # Robust "typical" step (median absolute adjacent delta)
    sd = sorted(diffs)
    med_adj = sd[len(sd) // 2]

    return {
        "bins": n,
        "min": vmin,
        "max": vmax,
        "mean": mean,
        "std": std,
        "peak_to_peak": vmax - vmin,
        "max_adjacent_abs_delta": max_adj,
        "median_adjacent_abs_delta": med_adj,
    }


def safe_hex(x):
    if x is None:
        return None
    try:
        return hex(int(x)) if x else int(x)
    except Exception:
        return str(x)


def gather(odrv, axis_idx):
    ax = odrv.axis0 if axis_idx == 0 else odrv.axis1
    c = ax.controller
    ac = c.config.anticogging
    m = ax.motor
    e = ax.encoder
    tt = ax.trap_traj

    prefix = "odrv0.axis{}".format(axis_idx)
    cpr = float(e.config.cpr)
    calib_pos = float(ac.calib_pos_threshold)
    calib_vel = float(ac.calib_vel_threshold)

    data = {
        "device": {},
        "axis": axis_idx,
        "axis_state": {},
        "controller": {},
        "anticogging": {},
        "trap_traj": {},
        "motor": {},
        "encoder": {},
        "computed": {},
        "cogging_map_stats": None,
        "cogging_map_read_error": None,
    }

    # Device-level (optional on some builds)
    dev = data["device"]
    for attr in (
        "serial_number",
        "hw_version_major",
        "hw_version_minor",
        "fw_version_major",
        "fw_version_minor",
        "fw_version_unreleased",
    ):
        if hasattr(odrv, attr):
            try:
                dev[attr] = getattr(odrv, attr)
            except Exception:
                pass

    data["axis_state"] = {
        "current_state": int(ax.current_state) if hasattr(ax, "current_state") else None,
        "requested_state": int(ax.requested_state) if hasattr(ax, "requested_state") else None,
        "axis_error": safe_hex(ax.error) if hasattr(ax, "error") else None,
        "motor_error": safe_hex(m.error) if hasattr(m, "error") else None,
        "encoder_error": safe_hex(e.error) if hasattr(e, "error") else None,
        "controller_error": safe_hex(c.error) if hasattr(c, "error") else None,
        "motor_is_calibrated": bool(m.is_calibrated) if hasattr(m, "is_calibrated") else None,
        "encoder_is_ready": bool(e.is_ready) if hasattr(e, "is_ready") else None,
    }

    cm = int(c.config.control_mode) if hasattr(c.config, "control_mode") else None
    im = int(c.config.input_mode) if hasattr(c.config, "input_mode") else None
    data["controller"] = {
        "control_mode": cm,
        "control_mode_name": _enum_name("CONTROL_MODE_", cm),
        "input_mode": im,
        "input_mode_name": _enum_name("INPUT_MODE_", im),
        "pos_gain": float(c.config.pos_gain),
        "vel_gain": float(c.config.vel_gain),
        "vel_integrator_gain": float(c.config.vel_integrator_gain),
        "vel_limit": float(c.config.vel_limit),
        "vel_limit_tolerance": float(c.config.vel_limit_tolerance),
        "enable_vel_limit": bool(c.config.enable_vel_limit),
        "enable_overspeed_error": bool(c.config.enable_overspeed_error),
        "circular_setpoints": bool(c.config.circular_setpoints),
        "circular_setpoint_range": float(c.config.circular_setpoint_range),
        "inertia": float(c.config.inertia),
        "input_filter_bandwidth": float(c.config.input_filter_bandwidth),
        "enable_gain_scheduling": bool(c.config.enable_gain_scheduling),
        "gain_scheduling_width": float(c.config.gain_scheduling_width),
        "load_encoder_axis": int(c.config.load_encoder_axis)
        if hasattr(c.config, "load_encoder_axis")
        else None,
        "anticogging_valid": bool(c.anticogging_valid),
        "vel_integrator_torque": float(c.vel_integrator_torque)
        if hasattr(c, "vel_integrator_torque")
        else None,
    }

    cogging_ratio = None
    if hasattr(ac, "cogging_ratio"):
        try:
            cogging_ratio = float(ac.cogging_ratio)
        except Exception:
            pass

    data["anticogging"] = {
        "pre_calibrated": bool(ac.pre_calibrated) if hasattr(ac, "pre_calibrated") else None,
        "calib_anticogging": bool(ac.calib_anticogging),
        "calib_pos_threshold": calib_pos,
        "calib_vel_threshold": calib_vel,
        "anticogging_enabled": bool(ac.anticogging_enabled),
        "index": int(ac.index) if hasattr(ac, "index") else None,
        "cogging_ratio": cogging_ratio,
    }

    data["trap_traj"] = {
        "vel_limit": float(tt.config.vel_limit),
        "accel_limit": float(tt.config.accel_limit),
        "decel_limit": float(tt.config.decel_limit),
    }

    enc_mode = None
    if hasattr(e.config, "mode"):
        try:
            enc_mode = int(e.config.mode)
        except Exception:
            enc_mode = e.config.mode

    data["motor"] = {
        "motor_type": int(m.config.motor_type) if hasattr(m.config, "motor_type") else None,
        "pole_pairs": int(m.config.pole_pairs) if hasattr(m.config, "pole_pairs") else None,
        "torque_constant": float(m.config.torque_constant),
        "direction": int(m.config.direction) if hasattr(m.config, "direction") else None,
        "current_lim": float(m.config.current_lim),
    }

    data["encoder"] = {
        "cpr": cpr,
        "bandwidth": float(e.config.bandwidth),
        "mode": enc_mode,
        "mode_name": _enum_name("ENCODER_MODE_", enc_mode) if enc_mode is not None else None,
        "use_index": bool(e.config.use_index) if hasattr(e.config, "use_index") else None,
    }

    # Same units as firmware: pos_err in turns vs calib_pos_threshold / cpr
    data["computed"] = {
        "settle_pos_band_turns": calib_pos / cpr if cpr else None,
        "settle_vel_band_turns_per_s": calib_vel / cpr if cpr else None,
        "note": (
            "calib_* thresholds are compared as pos_err <= calib_pos_threshold/cpr (turns) "
            "and |vel| < calib_vel_threshold/cpr (turn/s)."
        ),
    }

    vals, src = read_cogging_map(ax)
    if vals is not None:
        data["cogging_map_source"] = src
        data["cogging_map_stats"] = map_diagnostics(vals)
    else:
        data["cogging_map_read_error"] = (
            "Map not exposed: flash firmware with get_anticogging_value or cogging_map in API."
        )

    # Flat list for easy paste (name -> value)
    lines = []

    def add(section, kv):
        lines.append("")
        lines.append("### " + section)
        for k, v in kv.items():
            lines.append("{}.{}{} = {}".format(prefix, section + "." if section else "", k, v))

    lines.append("### paste_start")
    for k, v in sorted(dev.items()):
        lines.append("odrv0.{} = {}".format(k, v))

    st = data["axis_state"]
    for k, v in sorted(st.items()):
        lines.append("{}.{} = {}".format(prefix, k, v))

    cc = data["controller"]
    for k in sorted(cc.keys()):
        if k in ("anticogging_valid", "vel_integrator_torque"):
            lines.append("{}.controller.{} = {}".format(prefix, k, cc[k]))
        else:
            lines.append("{}.controller.config.{} = {}".format(prefix, k, cc[k]))

    for k, v in sorted(data["anticogging"].items()):
        lines.append("{}.controller.config.anticogging.{} = {}".format(prefix, k, v))

    for k, v in sorted(data["trap_traj"].items()):
        lines.append("{}.trap_traj.config.{} = {}".format(prefix, k, v))

    for k, v in sorted(data["motor"].items()):
        lines.append("{}.motor.config.{} = {}".format(prefix, k, v))

    for k, v in sorted(data["encoder"].items()):
        lines.append("{}.encoder.config.{} = {}".format(prefix, k, v))

    for k, v in sorted(data["computed"].items()):
        lines.append("computed.{} = {}".format(k, v))

    if data["cogging_map_stats"]:
        lines.append("")
        lines.append("### cogging_map_stats ({})".format(data.get("cogging_map_source", "?")))
        for k, v in sorted(data["cogging_map_stats"].items()):
            lines.append("map.{} = {}".format(k, v))
    else:
        lines.append("")
        lines.append("### cogging_map_stats")
        lines.append(data["cogging_map_read_error"] or "unknown error")

    lines.append("### paste_end")

    data["_paste_block"] = "\n".join(lines)
    return data


def main():
    p = argparse.ArgumentParser(description="ODrive anticogging diagnostic dump.")
    p.add_argument(
        "-s",
        "--serial",
        default=None,
        help="12 hex digit USB serial (optional)",
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
        "--json",
        metavar="FILE",
        default=None,
        help="Write full diagnostic dict to this JSON file",
    )
    p.add_argument(
        "--quiet",
        action="store_true",
        help="Only write JSON, minimal stdout",
    )
    args = p.parse_args()

    if not args.quiet:
        print("Connecting...", flush=True)
    cancel = Event()
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

    data = gather(odrv, args.axis)

    # Do not embed huge paste in JSON twice
    out_json = {k: v for k, v in data.items() if not k.startswith("_")}
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out_json, f, indent=2, sort_keys=True)
        if not args.quiet:
            print("Wrote {}".format(args.json), flush=True)

    if not args.quiet:
        print(data["_paste_block"])
        print(
            "\n# Copy the block between paste_start and paste_end (or attach {}).".format(
                args.json or "the JSON file"
            ),
            flush=True,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
