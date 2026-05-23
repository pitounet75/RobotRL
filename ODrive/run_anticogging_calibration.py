#!/usr/bin/env python3
"""
Full anticogging calibration workflow over USB: clear errors, motor+encoder cal,
closed-loop, then sets calib_pos_threshold=250 and calib_vel_threshold=0.1*encoder.cpr
(see calibrate_anticoggging.txt). Sets input_pos=0, waits 1s for stabilization, then
temporarily sets pos_gain=200 and vel_integrator_gain=8 for the sweep (original values
restored afterward). Starts anticogging, polls, then on success enables anticogging, save_configuration(),
and writes a timestamped CSV with a parameter header + cogging_map.

Usage:
  python run_anticogging_calibration.py
  python run_anticogging_calibration.py --serial 306A396A3235 --axis 1
  python run_anticogging_calibration.py --outdir H:\\Projects\\RobotRL\\ODrive\\maps
"""
from __future__ import print_function

import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

import odrive
from fibre import Event
from odrive.enums import *


def _fmt_err(val):
    if val is None:
        return "None"
    try:
        if val:
            return hex(int(val))
    except (TypeError, ValueError):
        pass
    return str(val)


def _connect(serial, timeout):
    cancel = Event()
    print("Connecting (USB, {:.0f}s timeout)...".format(timeout), flush=True)
    odrv = odrive.find_any(
        path="usb",
        serial_number=serial,
        timeout=timeout,
        search_cancellation_token=cancel,
        channel_termination_token=cancel,
    )
    return odrv


def _axis(odrv, axis_num):
    return odrv.axis0 if axis_num == 0 else odrv.axis1


def _device_serial_str(odrv):
    try:
        sn = odrv.serial_number
        if sn is not None:
            return str(sn)
    except Exception:
        pass
    return ""


def _read_map(ax):
    ac = ax.controller.config.anticogging
    ctrl = ax.controller
    try:
        if hasattr(ac, "cogging_map"):
            return [float(x) for x in ac.cogging_map], "config.anticogging.cogging_map"
    except AttributeError:
        pass
    if hasattr(ctrl, "get_anticogging_value"):
        getter = ctrl.get_anticogging_value
        out = []
        for i in range(3600):
            try:
                v = getter(index=i)
            except TypeError:
                v = getter(i)
            if isinstance(v, (list, tuple)):
                v = v[0]
            out.append(float(v))
        return out, "controller.get_anticogging_value"
    raise RuntimeError(
        "Cannot read cogging map (no cogging_map property or get_anticogging_value)."
    )


def _collect_param_header(ax, axis_num, odrv, map_source, values):
    c = ax.controller
    ac = c.config.anticogging
    m = ax.motor
    e = ax.encoder
    pfx = "axis{}.controller".format(axis_num)
    out = []
    out.append("# anticogging calibration snapshot — {}".format(
        datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    ))
    sn = _device_serial_str(odrv)
    if sn:
        out.append("# device serial_number = {}".format(sn))
    try:
        pyver = odrive.__version__
    except AttributeError:
        pyver = "unknown"
    out.append("# odrive python package version = {}".format(pyver))

    spec = [
        ("{}.config.control_mode".format(pfx), lambda: c.config.control_mode),
        ("{}.config.input_mode".format(pfx), lambda: c.config.input_mode),
        ("{}.config.pos_gain".format(pfx), lambda: c.config.pos_gain),
        ("{}.config.vel_gain".format(pfx), lambda: c.config.vel_gain),
        ("{}.config.vel_integrator_gain".format(pfx), lambda: c.config.vel_integrator_gain),
        ("{}.config.vel_limit".format(pfx), lambda: c.config.vel_limit),
        ("{}.config.vel_limit_tolerance".format(pfx), lambda: c.config.vel_limit_tolerance),
        ("{}.config.enable_overspeed_error".format(pfx), lambda: c.config.enable_overspeed_error),
        ("{}.config.circular_setpoints".format(pfx), lambda: c.config.circular_setpoints),
        ("{}.config.circular_setpoint_range".format(pfx), lambda: c.config.circular_setpoint_range),
        ("{}.config.anticogging.calib_pos_threshold".format(pfx), lambda: ac.calib_pos_threshold),
        ("{}.config.anticogging.calib_vel_threshold".format(pfx), lambda: ac.calib_vel_threshold),
        ("calib_vel_threshold / encoder.cpr (=> max |vel| turn/s for settle)",
         lambda: ac.calib_vel_threshold / float(e.config.cpr) if float(e.config.cpr) else float("nan")),
        ("{}.config.anticogging.cogging_ratio".format(pfx), lambda: getattr(ac, "cogging_ratio", "n/a")),
        ("{}.config.anticogging.anticogging_enabled".format(pfx), lambda: ac.anticogging_enabled),
        ("{}.config.anticogging.pre_calibrated".format(pfx), lambda: ac.pre_calibrated),
        ("{}.anticogging_valid".format(pfx), lambda: c.anticogging_valid),
        ("axis{}.motor.config.current_lim".format(axis_num), lambda: m.config.current_lim),
        ("axis{}.encoder.config.cpr".format(axis_num), lambda: e.config.cpr),
        ("axis{}.encoder.config.bandwidth".format(axis_num), lambda: e.config.bandwidth),
    ]

    for name, fn in spec:
        try:
            v = fn()
            out.append("# {} = {}".format(name, v))
        except Exception as ex:
            out.append("# {} = <error: {}>".format(name, ex))

    try:
        _cpr = float(e.config.cpr)
        tpos = ac.calib_pos_threshold
        out.append("# settle band: |pos_err| <= calib_pos_threshold/cpr = {}".format(tpos / _cpr))
    except Exception:
        pass

    n = len(values)
    if n:
        mean = sum(values) / n
        std = math.sqrt(sum((x - mean) ** 2 for x in values) / n)
        out.append("# cogging_map stats: n={} min={} max={} mean={} std={}".format(
            n, min(values), max(values), mean, std))
    out.append("# map_source = {}".format(map_source))
    out.append("# --- begin data (bin,torque)")
    return out


def _write_map_csv(path, header_lines, values):
    with open(path, "w", newline="", encoding="utf-8") as f:
        for line in header_lines:
            f.write(line + "\n")
        w = csv.writer(f)
        w.writerow(["bin", "torque"])
        for i, v in enumerate(values):
            w.writerow([i, v])


def main():
    p = argparse.ArgumentParser(description="Run full anticogging calibration and save map CSV.")
    p.add_argument("-s", "--serial", default=None, help="12 hex digit ODrive serial")
    p.add_argument("-a", "--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("--timeout", type=float, default=30.0, help="USB connect timeout (s)")
    p.add_argument(
        "--max-wait",
        type=float,
        default=7200.0,
        help="Max time waiting for anticogging cal to finish (s)",
    )
    p.add_argument(
        "--poll",
        type=float,
        default=5.0,
        help="Progress poll interval (s)",
    )
    p.add_argument(
        "--outdir",
        default=None,
        help="Directory for output CSV (default: current working directory)",
    )
    args = p.parse_args()

    odrv = _connect(args.serial, args.timeout)
    if odrv is None:
        print("No ODrive found.", file=sys.stderr)
        return 1

    ax = _axis(odrv, args.axis)
    axn = args.axis

    print("Clearing errors (idle)...", flush=True)
    try:
        ax.requested_state = AXIS_STATE_IDLE
        time.sleep(0.2)
    except Exception:
        pass
    try:
        ax.clear_errors()
    except Exception:
        pass
    try:
        odrv.clear_errors()
    except Exception:
        pass

    busy = (
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
        AXIS_STATE_MOTOR_CALIBRATION,
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
        AXIS_STATE_ENCODER_INDEX_SEARCH,
    )

    print("Starting FULL_CALIBRATION_SEQUENCE on axis{}...".format(axn), flush=True)
    ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    t_cal = time.monotonic()
    while ax.current_state in busy:
        time.sleep(0.1)
        if time.monotonic() - t_cal > 1200:
            print("Timeout waiting for motor/encoder calibration.", file=sys.stderr)
            return 1
    if ax.error != 0 or ax.motor.error != 0 or ax.encoder.error != 0:
        print(
            "Calibration sequence ended with errors: axis={} motor={} encoder={}".format(
                _fmt_err(ax.error), _fmt_err(ax.motor.error), _fmt_err(ax.encoder.error)
            ),
            file=sys.stderr,
        )
        return 1

    print("Requesting CLOSED_LOOP_CONTROL...", flush=True)
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    for _ in range(100):
        time.sleep(0.05)
        if ax.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            break
    if ax.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print("Axis did not reach closed loop (state={}).".format(ax.current_state))
        return 1

    ctrl = ax.controller
    ac = ctrl.config.anticogging

    ctrl.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    ctrl.config.input_mode = INPUT_MODE_PASSTHROUGH

    try:
        ctrl.anticogging_valid = False
    except Exception:
        pass
    ac.anticogging_enabled = False
    ac.pre_calibrated = False

    # Match OdriveTool/Commands/calibrate_anticoggging.txt
    ac.calib_pos_threshold = 250.0
    ac.calib_vel_threshold = 0.1 * float(ax.encoder.config.cpr)

    ctrl.input_pos = 0.0
    print("input_pos=0, waiting 1s for stabilization (before cal gains)...", flush=True)
    time.sleep(1.0)

    saved_pos_gain = float(ctrl.config.pos_gain)
    saved_vel_integrator_gain = float(ctrl.config.vel_integrator_gain)

    cal_poll_timed_out = False
    try:
        ctrl.config.pos_gain = 200.0
        ctrl.config.vel_integrator_gain = 8.0
        print(
            "Anticogging cal gains: pos_gain={} vel_integrator_gain={} (saved {}, {})".format(
                ctrl.config.pos_gain,
                ctrl.config.vel_integrator_gain,
                saved_pos_gain,
                saved_vel_integrator_gain,
            ),
            flush=True,
        )

        print("Starting anticogging calibration...", flush=True)
        ctrl.start_anticogging_calibration()

        t0 = time.monotonic()
        first = True
        while time.monotonic() - t0 < args.max_wait:
            if not first:
                time.sleep(args.poll)
            first = False
            idx = ac.index
            cal_run = ac.calib_anticogging
            valid = ctrl.anticogging_valid
            finished = not cal_run
            print(
                "[{:.0f}s] step=anticogging.index={}  calib_running={}  finished={}  anticogging_valid={}  "
                "axis.err={} motor.err={} enc.err={} ctrl.err={}".format(
                    time.monotonic() - t0,
                    idx,
                    cal_run,
                    finished,
                    valid,
                    _fmt_err(ax.error),
                    _fmt_err(ax.motor.error),
                    _fmt_err(ax.encoder.error),
                    _fmt_err(ctrl.error),
                ),
                flush=True,
            )
            if finished:
                break
        else:
            print("Timed out after {:.0f}s (cal still running?).".format(args.max_wait))
            cal_poll_timed_out = True
    finally:
        ctrl.config.pos_gain = saved_pos_gain
        ctrl.config.vel_integrator_gain = saved_vel_integrator_gain
        print(
            "Restored pos_gain={} vel_integrator_gain={}".format(
                ctrl.config.pos_gain,
                ctrl.config.vel_integrator_gain,
            ),
            flush=True,
        )

    if cal_poll_timed_out:
        return 1

    #if ax.error or ax.motor.error or ax.encoder.error or ctrl.error:
    #    print("Errors present after cal; not saving.", file=sys.stderr)
    #    return 1

    # calibrate_anticoggging.txt (after successful cal): enable, pre_calibrated, save — only if valid
    if ctrl.anticogging_valid:
        ac.anticogging_enabled = True
        ac.pre_calibrated = True
        try:
            odrv.save_configuration()
            print("save_configuration() OK", flush=True)
        except Exception as ex:
            print("save_configuration() failed: {}".format(ex), file=sys.stderr)
            return 1
    else:
        print(
            "anticogging_valid is False; skipping anticogging_enabled / pre_calibrated / save_configuration.",
            file=sys.stderr,
        )
        return 1

    try:
        values, map_source = _read_map(ax)
    except RuntimeError as e:
        print(str(e), file=sys.stderr)
        return 1

    n = len(values)
    mean = sum(values) / n if n else float("nan")
    std = math.sqrt(sum((x - mean) ** 2 for x in values) / n) if n else float("nan")
    print(
        "Map: n={} min={:.6g} max={:.6g} mean={:.6g} std={:.6g} source={}".format(
            n, min(values), max(values), mean, std, map_source
        ),
        flush=True,
    )

    ts = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    fname = "anticogging_map_{}.csv".format(ts)
    outdir = args.outdir or os.getcwd()
    try:
        os.makedirs(outdir, exist_ok=True)
    except TypeError:
        if not os.path.isdir(outdir):
            os.makedirs(outdir)
    outpath = os.path.join(outdir, fname)

    header = _collect_param_header(ax, axn, odrv, map_source, values)
    _write_map_csv(outpath, header, values)
    print("Wrote {}".format(outpath), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
