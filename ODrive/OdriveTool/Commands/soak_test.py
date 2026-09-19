#!/usr/bin/env python3
"""
Constant-speed soak test for one ODrive axis. Spins at a fixed velocity for a
few minutes and logs the important metrics every N seconds (mean/std over a
short burst each time), to console and to a CSV. Detects a stall or a fault
and disarms.

Use it to tell apart "low-speed cogging stall" from "something drifts over
time" (thermal, spi_error_rate creep, DRV).

Run:
  standalone:          python soak_test.py --vel 5 --minutes 3
                       python soak_test.py -a 0 --vel 8 --minutes 5 --interval 10
  inside odrivetool:   exec(open(r'soak_test.py').read())
                       soak(vel=5, minutes=3)
Ctrl-C stops and disarms cleanly.
"""
from __future__ import print_function

import datetime
import os
import sys
import time

CLOSED_LOOP = 8
IDLE = 1


def _hx(v):
    try:
        return hex(int(v))
    except Exception:
        return str(v)


def _connect(serial, odrv=None):
    if odrv is not None:
        return odrv
    g = globals()
    if g.get("odrv0") is not None:
        return g["odrv0"]
    import odrive
    print("Connecting...", flush=True)
    return odrive.find_any(path="usb", serial_number=serial, timeout=30.0)


def soak(odrv=None, axis=0, vel=5.0, minutes=3.0, interval=10.0, burst_s=1.0, burst_hz=100.0,
         stall_stop=True, serial=None, out=None):
    odrv = _connect(serial, odrv)
    if odrv is None:
        print("No ODrive found.", file=sys.stderr)
        return 1

    ax = getattr(odrv, "axis{}".format(axis))
    m = ax.motor
    enc = ax.encoder
    cc = m.current_control

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    if out is None:
        out = "soak_{:012X}_ax{}_{:g}tps_{}.csv".format(int(odrv.serial_number), axis, vel, stamp)
    log = []

    def emit(s=""):
        print(s, flush=True)
        log.append(s)

    emit("# ODrive {:012X} axis{}  vbus {:.2f}  {}".format(
        int(odrv.serial_number), axis, odrv.vbus_voltage,
        datetime.datetime.now().isoformat(timespec="seconds")))
    emit("# motor_type={} pole_pairs={} R={:.4g} L={:.4g} Kt={:.4g} current_lim={:.3g} cc_bw={:.4g}".format(
        int(m.config.motor_type), int(m.config.pole_pairs), m.config.phase_resistance,
        m.config.phase_inductance, m.config.torque_constant, m.config.current_lim,
        m.config.current_control_bandwidth))
    emit("# enc mode={} cpr={} offset={} offset_float={:.4f} pre_cal={} dir={} bw={:.4g} | axis1 enc mode={}".format(
        int(enc.config.mode), int(enc.config.cpr), int(enc.config.offset), enc.config.offset_float,
        enc.config.pre_calibrated, int(m.config.direction), enc.config.bandwidth,
        int(odrv.axis1.encoder.config.mode)))
    emit("# ctrl mode={} input_mode={} vel_gain={:.4g} vel_int={:.4g} vel_limit={:.3g}  ||  target vel={:g} turn/s  for {:g} min".format(
        int(ax.controller.config.control_mode), int(ax.controller.config.input_mode),
        ax.controller.config.vel_gain, ax.controller.config.vel_integrator_gain,
        ax.controller.config.vel_limit, vel, minutes))

    cols = ["t_s", "ve_mean", "ve_min", "iq_mean", "iq_std", "iq_min", "iq_max",
            "id_mean", "iq_set", "spi_mean", "spi_max", "fetC", "vbus", "state",
            "drv", "m_err", "a_err", "e_err", "pos"]
    emit(",".join(cols))
    fmt = ("{t_s:6.1f},{ve_mean:7.3f},{ve_min:7.3f},{iq_mean:+7.3f},{iq_std:6.3f},"
           "{iq_min:+7.3f},{iq_max:+7.3f},{id_mean:+7.3f},{iq_set:+7.3f},"
           "{spi_mean:7.4f},{spi_max:7.4f},{fetC:5.1f},{vbus:5.2f},{state:5d},"
           "{drv:3d},{m_err},{a_err},{e_err},{pos:.3f}")

    def flush_file():
        try:
            with open(out, "w") as f:
                f.write("\n".join(log) + "\n")
        except Exception as e:
            print("  (could not write {}: {})".format(out, e), file=sys.stderr)

    def disarm(why=""):
        try:
            ax.controller.input_vel = 0.0
            time.sleep(0.3)
            ax.requested_state = IDLE
        except Exception:
            pass
        emit("# disarmed {}".format(why))

    # ---- arm ----
    try:
        ax.clear_errors()
    except Exception:
        pass
    ax.controller.input_vel = 0.0
    ax.requested_state = CLOSED_LOOP
    t_arm = time.time()
    while ax.current_state != CLOSED_LOOP and time.time() - t_arm < 5:
        time.sleep(0.1)
    if ax.current_state != CLOSED_LOOP:
        emit("# FAILED to enter CLOSED_LOOP (state={})".format(ax.current_state))
        try:
            from odrive.utils import dump_errors
            dump_errors(odrv)
        except Exception:
            pass
        flush_file()
        return 2
    ax.controller.input_vel = vel

    t0 = time.time()
    t_end = t0 + minutes * 60.0
    n_burst = max(int(burst_s * burst_hz), 5)
    stall_hits = 0
    rc = 0
    try:
        # let it spin up before first sample
        time.sleep(min(interval, 5.0))
        while time.time() < t_end:
            b_ve = []
            b_iq = []
            b_id = []
            b_spi = []
            for _ in range(n_burst):
                try:
                    b_ve.append(float(enc.vel_estimate))
                    b_iq.append(float(cc.Iq_measured))
                    b_id.append(float(cc.Id_measured))
                    b_spi.append(float(enc.spi_error_rate))
                except Exception:
                    pass
                time.sleep(1.0 / burst_hz)
            if not b_iq:
                emit("# read error, aborting")
                rc = 3
                break

            def mean(xs):
                return sum(xs) / len(xs)

            iqm = mean(b_iq)
            iqsd = (sum((x - iqm) ** 2 for x in b_iq) / len(b_iq)) ** 0.5
            row = dict(
                t_s=time.time() - t0,
                ve_mean=mean(b_ve), ve_min=min(b_ve),
                iq_mean=iqm, iq_std=iqsd, iq_min=min(b_iq), iq_max=max(b_iq),
                id_mean=mean(b_id),
                iq_set=float(cc.Iq_setpoint),
                spi_mean=mean(b_spi), spi_max=max(b_spi),
                fetC=float(ax.fet_thermistor.temperature),
                vbus=float(odrv.vbus_voltage),
                state=int(ax.current_state),
                drv=int(m.gate_driver.drv_fault),
                m_err=_hx(m.error), a_err=_hx(ax.error), e_err=_hx(enc.error),
                pos=float(enc.pos_estimate),
            )
            line = fmt.format(**row)
            flags = []
            if row["state"] != CLOSED_LOOP:
                flags.append("FAULT")
            if abs(row["ve_mean"]) < 0.5 * abs(vel):
                flags.append("STALL")
            if row["spi_mean"] > 0.02:
                flags.append("SPI")
            if row["drv"]:
                flags.append("DRV")
            if flags:
                line += "   <<< " + " ".join(flags)
            emit(line)
            flush_file()

            if row["state"] != CLOSED_LOOP or row["drv"] or int(m.error) or int(ax.error):
                emit("# FAULT detected -> stop")
                try:
                    from odrive.utils import dump_errors
                    _b = []
                    import io
                    _o = sys.stdout
                    sys.stdout = io.StringIO()
                    try:
                        dump_errors(odrv)
                        _b = sys.stdout.getvalue().splitlines()
                    finally:
                        sys.stdout = _o
                    for ln in _b:
                        emit("# " + ln)
                except Exception:
                    pass
                rc = 4
                break

            if abs(row["ve_mean"]) < 0.5 * abs(vel):
                stall_hits += 1
                if stall_hits >= 2:
                    emit("# STALLED ({}x consecutive, ve~{:.2f} vs {:.2f})".format(stall_hits, row["ve_mean"], vel))
                    if stall_stop:
                        rc = 5
                        break
            else:
                stall_hits = 0

            # sleep to next interval (burst already consumed burst_s)
            time.sleep(max(interval - burst_s, 0.5))
    except KeyboardInterrupt:
        emit("# Ctrl-C")
    finally:
        disarm("(end)")
        flush_file()
        emit("# written: {}".format(os.path.abspath(out)))

    # quick trend summary
    try:
        import csv as _csv
        with open(out) as f:
            rows = [r for r in _csv.DictReader(l for l in f if not l.startswith("#"))]
        if len(rows) >= 2:
            f0, fl = rows[0], rows[-1]
            emit("# TREND  fetC {}->{}  spi_mean {}->{}  iq_mean {}->{}  ve_mean {}->{}".format(
                f0["fetC"], fl["fetC"], f0["spi_mean"], fl["spi_mean"],
                f0["iq_mean"], fl["iq_mean"], f0["ve_mean"], fl["ve_mean"]))
    except Exception:
        pass
    return rc


def _parse(argv):
    import argparse
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-a", "--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("-s", "--serial", default=None)
    p.add_argument("--vel", type=float, default=5.0, help="constant velocity [turn/s]")
    p.add_argument("--minutes", type=float, default=3.0)
    p.add_argument("--interval", type=float, default=10.0, help="seconds between log rows")
    p.add_argument("--no-stall-stop", dest="stall_stop", action="store_false",
                   help="keep going even if it stalls (motor will heat!)")
    p.add_argument("-o", "--out", default=None)
    return p.parse_args(argv)


# Run the CLI path only when invoked as `python soak_test.py ...` (works even if
# an embedded IPython shell leaves __name__ != "__main__"). When exec()'d or
# imported, this does nothing -- call soak(vel=..., minutes=...) yourself.
if os.path.basename(sys.argv[0] or "").startswith("soak_test"):
    _a = _parse(sys.argv[1:])
    sys.exit(soak(axis=_a.axis, vel=_a.vel, minutes=_a.minutes, interval=_a.interval,
                  stall_stop=_a.stall_stop, serial=_a.serial, out=_a.out) or 0)
