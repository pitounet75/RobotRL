#!/usr/bin/env python3
"""
Background watcher for an intermittent encoder spi_error_rate spike.

Runs unattended for a long time at a low rate; the instant spi_error_rate
crosses a threshold it dumps the pre-trigger context (ring buffer) and
switches to a high-rate burst capture so you see the ramp and exactly what
else changed (state, motor.error, drv_fault, fetC, vbus). Survives ODrive
power-cycles (reconnects and keeps going), so you can provoke the fault
while it logs. Everything is streamed to a CSV.

Run:
  standalone:   python spi_watch.py --minutes 30
  odrivetool:   import sys; sys.path.insert(0, r'<this dir>')
                import spi_watch
                spi_watch.watch(odrv0, minutes=30)
Ctrl-C to stop.

Provoke while it runs:
  * power-cycle the ODrive main supply repeatedly (or odrv0.reboot() in a loop)
  * freeze spray then hot air on: encoder connector, SPI3 pin area, DRV8301
  * gently flex/twist the PCB, press on connectors
  * try with motor phase leads disconnected vs connected
"""
from __future__ import print_function

import collections
import datetime
import os
import sys
import time

THRESH = 0.02          # spi_error_rate that counts as "spiking"
BURST_HZ = 60.0        # sample rate during an event
BURST_MIN_S = 4.0      # keep bursting at least this long after a trigger
PRE_TRIGGER = 25       # ring-buffer rows kept for pre-event context


def _hx(v):
    try:
        return hex(int(v))
    except Exception:
        return str(v)


def _read(odrv, ax):
    m = ax.motor
    enc = ax.encoder
    return dict(
        wall=time.time(),
        spi=float(enc.spi_error_rate),
        state=int(ax.current_state),
        m_err=int(m.error),
        a_err=int(ax.error),
        e_err=int(enc.error),
        drv=int(m.gate_driver.drv_fault),
        fetC=float(ax.fet_thermistor.temperature),
        vbus=float(odrv.vbus_voltage),
        pos_abs=int(enc.pos_abs),
    )


COLS = ["iso", "t_s", "phase", "spi", "state", "drv", "m_err", "a_err", "e_err", "fetC", "vbus", "pos_abs"]


def _row(t0, phase, r):
    return "{iso},{t:.3f},{ph},{spi:.4f},{st},{drv},{me},{ae},{ee},{fet:.1f},{vb:.2f},{pos}".format(
        iso=datetime.datetime.now().isoformat(timespec="milliseconds"),
        t=r["wall"] - t0, ph=phase, spi=r["spi"], st=r["state"], drv=r["drv"],
        me=_hx(r["m_err"]), ae=_hx(r["a_err"]), ee=_hx(r["e_err"]),
        fet=r["fetC"], vb=r["vbus"], pos=r["pos_abs"])


def watch(odrv=None, axis=0, minutes=30.0, hz=5.0, serial=None, out=None, reconnect=True):
    import_odrive = None
    if odrv is None:
        g = globals()
        odrv = g.get("odrv0")
    if odrv is None:
        import odrive as import_odrive
        print("Connecting...", flush=True)
        odrv = import_odrive.find_any(path="usb", serial_number=serial, timeout=30.0)
    if odrv is None:
        print("No ODrive.", file=sys.stderr)
        return 1
    if import_odrive is None:
        import odrive as import_odrive
    try:
        serial = "{:012X}".format(int(odrv.serial_number))
    except Exception:
        serial = serial or "unknown"

    if out is None:
        out = "spi_watch_{}_{}.csv".format(serial, datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    f = open(out, "w")
    f.write("# spi_watch  serial={}  axis={}  start={}  thresh={}\n".format(
        serial, axis, datetime.datetime.now().isoformat(timespec="seconds"), THRESH))
    f.write(",".join(COLS) + "\n")
    f.flush()
    print("logging to {}".format(os.path.abspath(out)), flush=True)

    ax = getattr(odrv, "axis{}".format(axis))
    t0 = time.time()
    t_end = t0 + minutes * 60.0
    ring = collections.deque(maxlen=PRE_TRIGGER)
    base_dt = 1.0 / max(hz, 0.5)
    burst_dt = 1.0 / BURST_HZ
    in_event = False
    event_no = 0
    event_deadline = 0.0
    last_status = t0
    spi_hi_since_status = 0.0
    n_since_status = 0

    def reconnect_loop():
        nonlocal odrv, ax
        f.write("# {} DISCONNECT\n".format(datetime.datetime.now().isoformat(timespec="seconds")))
        f.flush()
        print("  disconnect - waiting for ODrive...", flush=True)
        while time.time() < t_end:
            try:
                nd = import_odrive.find_any(path="usb", serial_number=serial, timeout=10.0)
            except Exception:
                nd = None
            if nd is not None:
                odrv = nd
                ax = getattr(odrv, "axis{}".format(axis))
                f.write("# {} RECONNECT\n".format(datetime.datetime.now().isoformat(timespec="seconds")))
                f.flush()
                print("  reconnected", flush=True)
                return True
            time.sleep(1.0)
        return False

    try:
        while time.time() < t_end:
            try:
                r = _read(odrv, ax)
            except Exception as e:
                if not reconnect or not reconnect_loop():
                    break
                continue

            spi_hi_since_status = max(spi_hi_since_status, r["spi"])
            n_since_status += 1

            if not in_event:
                ring.append(r)
                if r["spi"] > THRESH or r["drv"] or r["m_err"] or r["a_err"]:
                    event_no += 1
                    in_event = True
                    event_deadline = time.time() + BURST_MIN_S
                    f.write("# ==== EVENT {} at t={:.2f}s  ({}) ====\n".format(
                        event_no, r["wall"] - t0, datetime.datetime.now().isoformat(timespec="milliseconds")))
                    for pr in ring:
                        f.write(_row(t0, "pre", pr) + "\n")
                    f.write(_row(t0, "TRIG", r) + "\n")
                    f.flush()
                    print("\n*** EVENT {}: spi={:.3f} state={} m_err={} drv={} fetC={:.1f} vbus={:.2f} (t={:.1f}s)".format(
                        event_no, r["spi"], r["state"], _hx(r["m_err"]), r["drv"], r["fetC"], r["vbus"], r["wall"] - t0),
                        flush=True)
                    ring.clear()
                time.sleep(base_dt)
            else:
                f.write(_row(t0, "evt", r) + "\n")
                f.flush()
                if r["spi"] < THRESH and not r["drv"] and not r["m_err"] and not r["a_err"] and time.time() > event_deadline:
                    in_event = False
                    f.write("# ---- event {} cleared at t={:.2f}s ----\n".format(event_no, r["wall"] - t0))
                    f.flush()
                    print("    event {} cleared (spi back to {:.3f})".format(event_no, r["spi"]), flush=True)
                time.sleep(burst_dt)

            now = time.time()
            if now - last_status > 30.0:
                print("  [{:5.0f}s] alive  spi_max_30s={:.3f}  events={}  fetC={:.1f}  state={}".format(
                    now - t0, spi_hi_since_status, event_no, r["fetC"], r["state"]), flush=True)
                last_status = now
                spi_hi_since_status = 0.0
                n_since_status = 0
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
    finally:
        f.write("# stopped {}  events={}\n".format(datetime.datetime.now().isoformat(timespec="seconds"), event_no))
        f.close()
        print("done. {} event(s). file: {}".format(event_no, os.path.abspath(out)), flush=True)
    return 0


def _parse(argv):
    import argparse
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-a", "--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("-s", "--serial", default=None)
    p.add_argument("--minutes", type=float, default=30.0)
    p.add_argument("--hz", type=float, default=5.0, help="base sample rate (burst is faster)")
    p.add_argument("--no-reconnect", dest="reconnect", action="store_false")
    p.add_argument("-o", "--out", default=None)
    return p.parse_args(argv)


if os.path.basename(sys.argv[0] or "").startswith("spi_watch"):
    _a = _parse(sys.argv[1:])
    sys.exit(watch(axis=_a.axis, minutes=_a.minutes, hz=_a.hz, serial=_a.serial,
                   out=_a.out, reconnect=_a.reconnect) or 0)
