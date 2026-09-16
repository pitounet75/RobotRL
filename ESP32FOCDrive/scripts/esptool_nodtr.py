"""PIO uploader wrapper: enter ROM download, then run esptool without a DTR pulse.

Windows CH340 sets DTR on Serial.open(); that pulses EN, drops the GPIO0 RTC
hold, and the chip boots 0x13. esptool --before no_reset is not enough.
"""

from __future__ import print_function

import os
import sys
import time


def patch_serial_no_pulse():
    import serial

    class SerialNoPulse(serial.Serial):
        def open(self):
            self._dtr_state = False
            self._rts_state = False
            super(SerialNoPulse, self).open()
            try:
                self.dtr = False
                self.rts = False
            except Exception:
                pass

    _for_url = serial.serial_for_url

    def serial_for_url(url, *args, **kwargs):
        kwargs.pop("do_not_open", None)
        kwargs.pop("exclusive", None)
        if isinstance(url, str) and (
            url.upper().startswith("COM") or url.startswith("/dev/")
        ):
            inst = SerialNoPulse(*args, **kwargs)
            inst.port = url
            return inst
        return _for_url(url, *args, **kwargs)

    serial.Serial = SerialNoPulse
    serial.serial_for_url = serial_for_url


def poke_download(port):
    import serial

    print("enter_download: %s -> send 'download'" % port)
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.3
    ser.dtr = False
    ser.rts = False
    try:
        ser.open()
        ser.reset_input_buffer()
        ser.write(b"download\n")
        ser.flush()
        time.sleep(0.7)
    except Exception as exc:
        print("enter_download: %s" % exc)
        print("enter_download: first flash / dead app → R13 + Reset")
    finally:
        try:
            ser.close()
        except Exception:
            pass


def main():
    args = sys.argv[1:]
    port = None
    if "--port" in args:
        i = args.index("--port")
        if i + 1 < len(args):
            port = args[i + 1].strip('"')
    if port:
        poke_download(port)
    patch_serial_no_pulse()

    real = os.environ.get("PIO_REAL_ESPTOOL")
    if real and os.path.isfile(real):
        real_dir = os.path.dirname(os.path.abspath(real))
        contrib = os.path.join(real_dir, "_contrib")
        sys.path.insert(0, real_dir)
        if os.path.isdir(contrib):
            sys.path.insert(0, contrib)
    if "--before" not in args:
        args = args + ["--before", "no_reset"]
    sys.argv = ["esptool"] + args
    import esptool

    esptool._main()


if __name__ == "__main__":
    main()
