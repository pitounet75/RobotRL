"""
Connect to ODrive with visible progress. Uses one transport only (serial OR usb).

If you see "Could not init terminal features." it is harmless (pip install pywin32 colorama).

Hang fixes:
- Use --serial COMn (your CDC port), not default usb, if Native USB / PyUSB misbehaves.
- This script uses find_any(..., timeout=...) so it will not wait forever.
"""
from __future__ import annotations

import argparse
import sys
import time


def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="ODrive connection test with logging")
    parser.add_argument(
        "--serial",
        metavar="COMn",
        default="COM5",
        help='Serial path for fibre (default: COM5). Example: --serial COM5',
    )
    parser.add_argument(
        "--usb",
        action="store_true",
        help="Use default USB bulk (Native Interface) instead of serial",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for a device (default: 30)",
    )
    args = parser.parse_args()

    path = "usb" if args.usb else f"serial:{args.serial}"

    log("importing odrive / fibre …")
    import odrive
    from fibre import Logger

    log("imports done")

    # Optional: list COM ports (helps confirm COM5 exists)
    try:
        import serial.tools.list_ports

        ports = [p.device for p in serial.tools.list_ports.comports()]
        log(f"pyserial sees ports: {ports if ports else '(none)'}")
    except Exception as ex:
        log(f"could not list serial ports ({ex!r}) — is pyserial installed?")

    verbose = Logger(verbose=True)
    log(f"calling find_any(path={path!r}, timeout={args.timeout}) …")
    log("(verbose logger ON — watch for 'Downloading json data' / USB errors)")

    try:
        odrv0 = odrive.find_any(path=path, timeout=args.timeout, logger=verbose)
    except Exception as ex:
        log(f"find_any raised: {type(ex).__name__}: {ex}")
        raise

    if odrv0 is None:
        log(
            f"find_any returned None after {args.timeout}s — no device answered on {path!r}. "
            "Check cable, firmware running, COM number, or Native USB driver."
        )
        sys.exit(2)

    log("connected; reading vbus_voltage …")
    try:
        v = odrv0.vbus_voltage
        log(f"vbus_voltage: {v}")
    except Exception as ex:
        log(f"read vbus_voltage failed: {type(ex).__name__}: {ex}")
        raise


if __name__ == "__main__":
    main()
