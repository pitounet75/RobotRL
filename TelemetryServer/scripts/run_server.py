#!/usr/bin/env python3
"""Receive STM32 BalanceFrame telemetry via ESP32 UDP bridge."""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from telemetry.balance_frame import BalanceFrame
from telemetry.plot_live import LiveBalancePlotter
from telemetry.protocol import TELEM_MSG_BALANCE_FRAME, FrameParser
from telemetry.recorder import CsvRecorder
from telemetry.udp_receiver import UdpTelemetryReceiver


def main() -> int:
    p = argparse.ArgumentParser(description="RobotRL balance telemetry server")
    p.add_argument("--bind-host", default="0.0.0.0")
    p.add_argument("--bind-port", type=int, default=5000)
    p.add_argument("--esp32-host", help="ESP32 IP for subscribe ping (learn-remote)")
    p.add_argument("--esp32-port", type=int, default=5000)
    p.add_argument("--plot", action="store_true", help="Live matplotlib plot")
    p.add_argument("--record", type=Path, help="CSV output path")
    p.add_argument("--history-s", type=float, default=10.0)
    p.add_argument("--verbose", action="store_true", help="Print UDP / parse stats")
    args = p.parse_args()

    receiver = UdpTelemetryReceiver(
        bind_host=args.bind_host,
        bind_port=args.bind_port,
        esp32_host=args.esp32_host,
        esp32_port=args.esp32_port,
    )
    parser = FrameParser()
    recorder = CsvRecorder(args.record) if args.record else None
    plotter = None
    if args.plot:
        try:
            plotter = LiveBalancePlotter(history_s=args.history_s)
        except Exception as exc:
            print(f"Plot disabled: {exc}")
            print("Tip: pip install PyQt5  OR  use --record only and plot_file.py later")
            if not args.record:
                print("Continuing headless (use --record to capture data).")

    if args.esp32_host:
        receiver.subscribe()
        print(f"Subscribe ping sent to {args.esp32_host}:{args.esp32_port} (repeats every 0.5 s)")

    print(f"Listening UDP {args.bind_host}:{args.bind_port} ...")

    lock = threading.Lock()
    stop = threading.Event()

    def rx_loop() -> None:
        frame_count = 0
        byte_count = 0
        bad_type = 0
        bad_crc = 0
        rejected = 0
        last_subscribe = 0.0
        last_stats = 0.0
        last_byte_count = 0
        last_frame_count = 0
        t_start = time.time()
        while not stop.is_set():
            now = time.time()
            if args.esp32_host and now - last_subscribe >= 0.5:
                receiver.subscribe()
                last_subscribe = now

            chunk = receiver.recv_chunk()
            if chunk is None:
                if args.verbose and now - last_stats >= 5.0:
                    dt = now - last_stats
                    bps = (byte_count - last_byte_count) / max(dt, 0.001)
                    fps = (frame_count - last_frame_count) / max(dt, 0.001)
                    print(
                        f"stats: udp={bps:.0f} B/s  frames={fps:.1f}/s  "
                        f"total={frame_count}  rejected={rejected}  bad_crc={bad_crc}"
                    )
                    last_stats = now
                    last_byte_count = byte_count
                    last_frame_count = frame_count
                continue
            byte_count += len(chunk)
            for tf in parser.feed(chunk):
                if not tf.ok:
                    bad_crc += 1
                    continue
                if tf.message_type != TELEM_MSG_BALANCE_FRAME:
                    bad_type += 1
                    if args.verbose and bad_type <= 3:
                        print(f"ignored msg type 0x{tf.message_type:04x}")
                    continue
                try:
                    bf = BalanceFrame.decode(tf.payload)
                except ValueError:
                    continue
                if not bf.is_sane():
                    rejected += 1
                    if plotter is not None:
                        plotter.note_reject()
                    continue
                host_t = time.time()
                with lock:
                    if recorder is not None:
                        recorder.write(host_t, bf)
                    if plotter is not None:
                        plotter.add(host_t, bf)
                frame_count += 1
                if frame_count == 1:
                    print(f"first frame #{bf.frame_number} pitch={bf.pitch_rad:.4f}")
                if frame_count % 100 == 0:
                    print(f"frames={frame_count}  pitch={bf.pitch_rad:.4f}  cmd={bf.cmd_torque_nm:.5f}")

    rx_thread = threading.Thread(target=rx_loop, daemon=True)
    rx_thread.start()

    try:
        if plotter is not None:
            plotter.run()
        else:
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop.set()
        rx_thread.join(timeout=1.0)
        if recorder is not None:
            recorder.close()
        receiver.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
