#!/usr/bin/env python3
"""
One-shot diagnostic to run the moment axis0 freezes / stalls / heats.

It (1) grabs a short high-rate capture of the LIVE armed state, (2) analyses
whether the encoder position is frozen vs the current, (3) dumps errors and
static config, (4) writes it all to a timestamped file, then (5) disarms the
axis so the winding stops cooking.

Run:
  standalone:            python freeze_snapshot.py
                         python freeze_snapshot.py -a 0 --secs 3 --no-disarm
  inside odrivetool:     exec(open(r'freeze_snapshot.py').read())
                         snapshot()            # or snapshot(disarm=False)
"""
from __future__ import print_function

import datetime
import os
import sys
import time


def _hx(v):
    try:
        return hex(int(v))
    except Exception:
        return str(v)


def _connect(serial):
    g = globals()
    if g.get("odrv0") is not None:
        return g["odrv0"]
    import odrive
    print("Connecting...", flush=True)
    return odrive.find_any(path="usb", serial_number=serial, timeout=30.0)


def snapshot(axis=0, secs=2.0, hz=50.0, disarm=True, serial=None, out=None):
    odrv = _connect(serial)
    if odrv is None:
        print("No ODrive found.", file=sys.stderr)
        return 1

    ax = getattr(odrv, "axis{}".format(axis))
    m = ax.motor
    enc = ax.encoder
    cc = m.current_control

    lines = []

    def emit(s=""):
        print(s, flush=True)
        lines.append(s)

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    emit("=" * 92)
    emit("FREEZE SNAPSHOT  axis{}  {}".format(axis, datetime.datetime.now().isoformat(timespec="seconds")))
    emit("ODrive {:012X}   vbus {:.3f} V   fw {}.{}.{}".format(
        int(odrv.serial_number), odrv.vbus_voltage,
        odrv.fw_version_major, odrv.fw_version_minor, odrv.fw_version_revision))
    emit("=" * 92)

    # ---- 1. fast capture of the live (still armed) state ------------------
    dt = 1.0 / max(hz, 1.0)
    n = max(int(secs / dt), 1)
    cap = []
    t0 = time.time()
    for _ in range(n):
        cap.append((
            time.time() - t0,
            float(enc.spi_error_rate),
            int(ax.current_state),
            float(cc.Id_measured), float(cc.Iq_measured), float(cc.Iq_setpoint),
            float(enc.vel_estimate), float(ax.controller.vel_setpoint),
            float(enc.pos_estimate), int(enc.pos_abs), int(enc.shadow_count),
            float(ax.fet_thermistor.temperature),
            int(m.gate_driver.drv_fault), int(m.error), int(ax.error),
        ))
        time.sleep(dt)

    emit("CAPTURE ({} rows @ ~{:.0f} Hz)".format(len(cap), hz))
    emit("{:>6} {:>7} {:>5} {:>7} {:>7} {:>7} {:>7} {:>7} {:>9} {:>10} {:>10} {:>5} {:>3}".format(
        "t", "spi", "st", "Id", "Iq", "Iq_set", "v_est", "v_set", "pos_est", "pos_abs", "shadow", "fetC", "drv"))
    step = max(len(cap) // 20, 1)
    for r in cap[::step]:
        emit("{:6.2f} {:7.3f} {:5d} {:7.3f} {:7.3f} {:7.3f} {:7.3f} {:7.3f} {:9.4f} {:10d} {:10d} {:5.1f} {:3d}".format(
            r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10], r[11], r[12]))

    # ---- 2. analysis ----------------------------------------------------
    def col(i):
        return [r[i] for r in cap]

    def stats(xs):
        mu = sum(xs) / len(xs)
        sd = (sum((x - mu) ** 2 for x in xs) / len(xs)) ** 0.5
        return min(xs), max(xs), mu, sd

    spi_lo, spi_hi, spi_mu, spi_sd = stats(col(1))
    id_lo, id_hi, id_mu, id_sd = stats(col(3))
    iq_lo, iq_hi, iq_mu, iq_sd = stats(col(4))
    pabs = col(9)
    shadow = col(10)
    pos_span = max(pabs) - min(pabs)
    shadow_span = max(shadow) - min(shadow)
    ve_lo, ve_hi, ve_mu, ve_sd = stats(col(6))
    fet_rise = col(11)[-1] - col(11)[0]

    emit("")
    emit("ANALYSIS")
    emit("  spi_error_rate : mean {:.3f}  max {:.3f}".format(spi_mu, spi_hi))
    emit("  Iq [A]         : mean {:+.3f}  std {:.3f}  range [{:+.3f}, {:+.3f}]".format(iq_mu, iq_sd, iq_lo, iq_hi))
    emit("  Id [A]         : mean {:+.3f}  std {:.3f}".format(id_mu, id_sd))
    emit("  vel_estimate   : mean {:+.3f}  std {:.3f}".format(ve_mu, ve_sd))
    emit("  pos_abs span   : {} counts   ({:.5f} turn)".format(pos_span, pos_span / max(int(enc.config.cpr), 1)))
    emit("  shadow span    : {} counts".format(shadow_span))
    emit("  FET temp       : {:.1f} degC  (delta over capture {:+.2f})".format(col(11)[-1], fet_rise))

    verdict = []
    if spi_mu > 0.05:
        verdict.append("ENCODER SPI FAILING (spi_error_rate mean {:.2f}) -> position is stale/frozen -> commutation lost.".format(spi_mu))
    frozen = pos_span <= 2 and shadow_span <= 2
    if abs(iq_mu) > 0.5 and frozen:
        if spi_mu <= 0.05:
            verdict.append("STALL: sustained Iq {:+.2f} A with NO encoder movement while SPI is clean -> wrong offset / miscommutation. Redo AXIS_STATE_ENCODER_OFFSET_CALIBRATION.".format(iq_mu))
        else:
            verdict.append("STALL driven by the dead encoder above.")
    if abs(iq_mu) > 0.5 and not frozen:
        verdict.append("Motor still moving (pos_abs span {} counts) but drawing Iq {:+.2f} A -> load / cogging / tuning, not a hard lock.".format(pos_span, iq_mu))
    if col(12)[-1]:
        verdict.append("DRV FAULT active (drv_fault={}).".format(col(12)[-1]))
    if not verdict:
        verdict.append("No hard stall pattern in this window.")
    emit("")
    emit("VERDICT")
    for v in verdict:
        emit("  - " + v)

    # ---- 3. errors ----------------------------------------------------
    emit("")
    emit("ERRORS")
    try:
        from odrive.utils import dump_errors as _de
        import io
        buf = io.StringIO()
        _old = sys.stdout
        sys.stdout = buf
        try:
            _de(odrv)
        finally:
            sys.stdout = _old
        for ln in buf.getvalue().splitlines():
            emit("  " + ln)
    except Exception:
        emit("  axis.error   = {}".format(_hx(ax.error)))
        emit("  motor.error  = {}".format(_hx(m.error)))
        emit("  encoder.error= {}".format(_hx(enc.error)))
        emit("  controller.error = {}".format(_hx(ax.controller.error)))
        emit("  drv_fault    = {}".format(int(m.gate_driver.drv_fault)))

    # ---- 4. static config ------------------------------------------
    emit("")
    emit("CONFIG")
    emit("  motor_type={}  pole_pairs={}  R={:.4g} ohm  L={:.4g} H  Kt={:.4g}".format(
        int(m.config.motor_type), int(m.config.pole_pairs), m.config.phase_resistance,
        m.config.phase_inductance, m.config.torque_constant))
    emit("  current_lim={:.3g}  requested_current_range={:.3g}  current_control_bandwidth={:.4g}".format(
        m.config.current_lim, m.config.requested_current_range, m.config.current_control_bandwidth))
    emit("  motor.pre_calibrated={}  is_calibrated={}".format(m.config.pre_calibrated, m.is_calibrated))
    emit("  enc.mode={}  cpr={}  offset={}  offset_float={:.5f}  pre_calibrated={}  is_ready={}  dir={}".format(
        int(enc.config.mode), int(enc.config.cpr), int(enc.config.offset), enc.config.offset_float,
        enc.config.pre_calibrated, enc.is_ready, int(m.config.direction)))
    emit("  enc.bandwidth={:.4g}  spi_cs_pin={}  |  axis1.enc.mode={}".format(
        enc.config.bandwidth, int(enc.config.abs_spi_cs_gpio_pin), int(odrv.axis1.encoder.config.mode)))
    emit("  ctrl: mode={} input_mode={} vel_gain={:.4g} vel_integrator_gain={:.4g} vel_limit={:.3g} pos_gain={:.3g}".format(
        int(ax.controller.config.control_mode), int(ax.controller.config.input_mode),
        ax.controller.config.vel_gain, ax.controller.config.vel_integrator_gain,
        ax.controller.config.vel_limit, ax.controller.config.pos_gain))
    emit("  anticogging: enabled={} valid={}".format(
        ax.controller.config.anticogging.anticogging_enabled, ax.controller.anticogging_valid))
    emit("  startup: motor_cal={} enc_offset_cal={} closed_loop={} sensorless={}".format(
        ax.config.startup_motor_calibration, ax.config.startup_encoder_offset_calibration,
        ax.config.startup_closed_loop_control, ax.config.startup_sensorless_control))
    emit("  user_config_loaded={}".format(odrv.user_config_loaded))

    # ---- 5. make it safe ------------------------------------------
    emit("")
    if disarm:
        try:
            ax.requested_state = 1  # AXIS_STATE_IDLE
            emit("axis{} -> IDLE (disarmed).".format(axis))
        except Exception as e:
            emit("could not disarm: {}".format(e))
    else:
        emit("--no-disarm: axis left in its current state (winding may still be powered!).")

    # ---- write file ------------------------------------------------
    if out is None:
        out = "freeze_{:012X}_{}.txt".format(int(odrv.serial_number), stamp)
    try:
        with open(out, "w") as f:
            f.write("\n".join(lines) + "\n")
        emit("")
        emit("written: {}".format(os.path.abspath(out)))
    except Exception as e:
        emit("could not write {}: {}".format(out, e))
    return 0


def _parse(argv):
    import argparse
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-a", "--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("-s", "--serial", default=None)
    p.add_argument("--secs", type=float, default=2.0, help="live-capture duration before disarm")
    p.add_argument("--hz", type=float, default=50.0)
    p.add_argument("--no-disarm", dest="disarm", action="store_false", help="do NOT force IDLE afterwards")
    p.add_argument("-o", "--out", default=None)
    return p.parse_args(argv)


if __name__ == "__main__":
    _base = os.path.basename(sys.argv[0] or "")
    if _base.startswith("freeze_snapshot"):
        a = _parse(sys.argv[1:])
        sys.exit(snapshot(axis=a.axis, secs=a.secs, hz=a.hz, disarm=a.disarm,
                          serial=a.serial, out=a.out) or 0)
    else:
        snapshot()
