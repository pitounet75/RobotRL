#!/usr/bin/env python3
"""
Live health monitor for one ODrive axis: encoder SPI, commutation currents,
velocity tracking, temperature, errors. For chasing the "motor heats at no load"
/ "spi_error_rate spikes" problems.

Two ways to run:
  * standalone:      python monitor_axis.py           (axis0, ~5 Hz)
                     python monitor_axis.py -a 0 --hz 10
                     python monitor_axis.py --once
  * inside odrivetool (odrv0 already connected):
        exec(open(r'monitor_axis.py').read())
        monitor(hz=10)                 # Ctrl-C to stop, re-call with new args

Columns:
  state   axis current_state (1=IDLE 4=MOTOR_CAL 6=ENC_OFFSET 8=CLOSED_LOOP)
  spi     encoder.spi_error_rate   (>0.01 flagged 'SPI!')
  Id/Iq   current_control measured d/q currents [A]  (Id should be ~0)
  Iq_set  Iq setpoint [A]
  v_est   encoder.vel_estimate [turn/s]     v_set  controller.vel_setpoint
  pos     encoder.pos_estimate [turn]
  fetC    FET thermistor temp [degC]
  mErr/aErr  motor.error / axis.error (hex)   drv = gate_driver.drv_fault
Every `window` rows it also prints min/max/mean/std of Id, Iq, spi, v_est.
"""
from __future__ import print_function

import collections
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


def monitor(axis=0, hz=5.0, once=False, window=20, serial=None):
    odrv = _connect(serial)
    if odrv is None:
        print("No ODrive found.", file=sys.stderr)
        return 1

    ax = getattr(odrv, "axis{}".format(axis))
    m = ax.motor
    enc = ax.encoder
    cc = m.current_control

    print("=" * 96)
    print("ODrive {:012X}   axis{}   vbus {:.2f} V".format(
        int(odrv.serial_number), axis, odrv.vbus_voltage))
    print("motor_type={}  pole_pairs={}  R={:.4g}ohm  L={:.4g}H  Kt={:.4g}  current_lim={:.3g}".format(
        int(m.config.motor_type), int(m.config.pole_pairs), m.config.phase_resistance,
        m.config.phase_inductance, m.config.torque_constant, m.config.current_lim))
    print("enc: mode={} cpr={} offset={} pre_cal={} dir={}  |  axis1 enc mode={}".format(
        int(enc.config.mode), int(enc.config.cpr), int(enc.config.offset),
        enc.config.pre_calibrated, int(m.config.direction),
        int(odrv.axis1.encoder.config.mode)))
    print("ctrl: control_mode={} input_mode={} vel_limit={:.3g}".format(
        int(ax.controller.config.control_mode), int(ax.controller.config.input_mode),
        ax.controller.config.vel_limit))
    print("=" * 96)
    hdr = "{:>7} {:>5} {:>7} {:>8} {:>8} {:>8} {:>8} {:>8} {:>9} {:>5} {:>4} {:>7} {:>7}".format(
        "t", "state", "spi", "Id", "Iq", "Iq_set", "v_est", "v_set", "pos", "fetC", "drv",
        "mErr", "aErr")
    print(hdr)

    win = collections.deque(maxlen=max(window, 2))
    t0 = time.time()
    dt = 1.0 / max(hz, 0.2)
    n = 0
    try:
        while True:
            r = {
                "t": time.time() - t0,
                "state": int(ax.current_state),
                "spi": float(enc.spi_error_rate),
                "Id": float(cc.Id_measured),
                "Iq": float(cc.Iq_measured),
                "Iqs": float(cc.Iq_setpoint),
                "ve": float(enc.vel_estimate),
                "vs": float(ax.controller.vel_setpoint),
                "pe": float(enc.pos_estimate),
                "fetC": float(ax.fet_thermistor.temperature),
                "drv": int(m.gate_driver.drv_fault),
                "merr": int(m.error),
                "aerr": int(ax.error),
            }
            win.append(r)

            flag = ""
            if r["spi"] > 0.01:
                flag += " SPI!"
            if r["drv"]:
                flag += " DRV!"
            if r["merr"] or r["aerr"]:
                flag += " ERR!"

            print("{t:7.1f} {state:5d} {spi:7.3f} {Id:8.3f} {Iq:8.3f} {Iqs:8.3f} "
                  "{ve:8.3f} {vs:8.3f} {pe:9.4f} {fetC:5.1f} {drv:4d} {merr:>7} {aerr:>7}{flag}".format(
                      flag=flag, merr=_hx(r["merr"]), aerr=_hx(r["aerr"]),
                      **{k: r[k] for k in ("t", "state", "spi", "Id", "Iq", "Iqs", "ve", "vs", "pe", "fetC", "drv")}),
                  flush=True)

            n += 1
            if once:
                break
            if len(win) == win.maxlen and n % win.maxlen == 0:
                for key, lbl in (("Id", "Id"), ("Iq", "Iq"), ("spi", "spi"), ("ve", "v_est")):
                    xs = [w[key] for w in win]
                    mu = sum(xs) / len(xs)
                    sd = (sum((x - mu) ** 2 for x in xs) / len(xs)) ** 0.5
                    print("   [{}x] {:<5} min {:8.3f}  max {:8.3f}  mean {:8.3f}  std {:8.4f}".format(
                        win.maxlen, lbl, min(xs), max(xs), mu, sd))
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


def _parse(argv):
    import argparse
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-a", "--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("-s", "--serial", default=None)
    p.add_argument("--hz", type=float, default=5.0)
    p.add_argument("--once", action="store_true")
    p.add_argument("-n", "--window", type=int, default=20, help="rolling-stats window (rows)")
    return p.parse_args(argv)


if __name__ == "__main__":
    _base = os.path.basename(sys.argv[0] or "")
    if _base.startswith("monitor_axis"):
        a = _parse(sys.argv[1:])
        sys.exit(monitor(axis=a.axis, hz=a.hz, once=a.once, window=a.window, serial=a.serial) or 0)
    else:
        # pasted into odrivetool: start with defaults; Ctrl-C then call monitor(...) again
        monitor()
