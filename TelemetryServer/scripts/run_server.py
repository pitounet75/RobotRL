#!/usr/bin/env python3
"""Receive STM32 BalanceFrame telemetry via ESP32 UDP bridge."""

from __future__ import annotations

import argparse
import sys
import threading
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from telemetry.balance_frame import BalanceFrame, BalanceFrameSanityLimits
from telemetry.plot_live import LiveBalancePlotter
from telemetry.protocol import TELEM_MSG_BALANCE_FRAME, FrameParser
from telemetry.recorder import CsvRecorder
from telemetry.udp_envelope import (
    DatagramSequenceTracker,
    UdpEnvelopeError,
    decode_udp_datagram,
)
from telemetry.udp_receiver import UdpTelemetryReceiver


def main() -> int:
    p = argparse.ArgumentParser(description="RobotRL balance telemetry server")
    p.add_argument("--bind-host", default="0.0.0.0")
    p.add_argument("--bind-port", type=int, default=5000)
    p.add_argument("--esp32-host", help="ESP32 IP for subscribe ping (learn-remote)")
    p.add_argument("--esp32-port", type=int, default=5000)
    p.add_argument("--plot", action="store_true", help="Live plot (pyqtgraph/PyQt5)")
    p.add_argument("--record", type=Path, help="CSV output path")
    p.add_argument("--history-s", type=float, default=10.0)
    p.add_argument("--receive-buffer", type=int, default=1024 * 1024, help="Requested UDP SO_RCVBUF bytes")
    p.add_argument("--max-torque-nm", type=float, default=2.0)
    p.add_argument("--max-pitch-rad", type=float, default=3.5)
    p.add_argument("--max-wheel-turns-s", type=float, default=50.0)
    p.add_argument("--max-strategy-id", type=int, default=32)
    p.add_argument("--verbose", action="store_true", help="Print UDP / parse stats")
    p.add_argument(
        "--diag-stall",
        action="store_true",
        help="Print when |pitch| is high but motor cmd is ~0 (balance stall hunt)",
    )
    p.add_argument(
        "--stall-pitch-deg",
        type=float,
        default=8.0,
        help="Pitch threshold for --diag-stall (degrees)",
    )
    p.add_argument(
        "--stall-cmd-eps",
        type=float,
        default=0.002,
        help="|cmd| below this counts as stalled for --diag-stall (Nm)",
    )
    args = p.parse_args()

    receiver = UdpTelemetryReceiver(
        bind_host=args.bind_host,
        bind_port=args.bind_port,
        esp32_host=args.esp32_host,
        esp32_port=args.esp32_port,
        receive_buffer_size=args.receive_buffer,
    )
    parser = FrameParser()
    sanity_limits = BalanceFrameSanityLimits(
        max_torque_nm=args.max_torque_nm,
        max_pitch_rad=args.max_pitch_rad,
        max_wheel_turns_s=args.max_wheel_turns_s,
        max_strategy_id=args.max_strategy_id,
    )
    recorder = CsvRecorder(args.record) if args.record else None
    plotter = None
    if args.plot:
        try:
            plotter = LiveBalancePlotter(history_s=args.history_s)
        except Exception as exc:
            print(f"Plot disabled: {exc}")
            print("Tip: pip install PyQt5 pyqtgraph  OR  use --record only and plot_file.py later")
            plotter = None
            if not args.record:
                print("Continuing headless (use --record to capture data).")

    if args.esp32_host:
        receiver.subscribe()
        print(f"Subscribe ping sent to {args.esp32_host}:{args.esp32_port} (repeats every 5 s)")

    print(
        f"Listening UDP {args.bind_host}:{args.bind_port} "
        f"(SO_RCVBUF={receiver.receive_buffer_size} bytes) ..."
    )

    lock = threading.Lock()
    stop = threading.Event()

    def rx_loop() -> None:
        frame_count = 0
        byte_count = 0
        bad_type = 0
        stm32_errors = 0
        decode_errors = 0
        sanity_encoding = 0
        physical_rejected = 0
        rejection_reasons: Counter[str] = Counter()
        udp_envelope_errors = 0
        udp_frame_count_errors = 0
        udp_sequences = DatagramSequenceTracker()
        balance_frame_gaps = 0
        stm32_source_drops = 0
        last_balance_frame_number = None
        last_source_drop_mod256 = None
        last_subscribe = 0.0
        last_stats = 0.0
        last_byte_count = 0
        last_frame_count = 0
        last_stall_print = 0.0
        stall_pitch_rad = abs(args.stall_pitch_deg) * 3.141592653589793 / 180.0
        t_start = time.time()
        subscribe_interval_s = 5.0
        while not stop.is_set():
            now = time.time()
            if args.esp32_host and now - last_subscribe >= subscribe_interval_s:
                receiver.subscribe()
                last_subscribe = now

            for chunk in receiver.recv_chunks():
                byte_count += len(chunk)
                try:
                    datagram = decode_udp_datagram(chunk)
                except UdpEnvelopeError as exc:
                    udp_envelope_errors += 1
                    if args.verbose and udp_envelope_errors <= 3:
                        print(f"rejected UDP envelope: {exc}")
                    continue

                if datagram.enveloped:
                    assert datagram.sequence is not None
                    udp_sequences.observe(datagram.sequence)

                frames = list(parser.feed(datagram.payload))
                if (
                    datagram.frame_count is not None
                    and len(frames) != datagram.frame_count
                ):
                    udp_frame_count_errors += 1
                for tf in frames:
                    if not tf.ok:
                        stm32_errors += 1
                        if args.verbose and stm32_errors <= 3:
                            print(
                                f"STM32 error: type=0x{tf.message_type:04x} "
                                f"error_code={tf.error_code}"
                            )
                        continue
                    if tf.message_type != TELEM_MSG_BALANCE_FRAME:
                        bad_type += 1
                        if args.verbose and bad_type <= 3:
                            print(f"ignored msg type 0x{tf.message_type:04x}")
                        continue
                    try:
                        bf = BalanceFrame.decode(tf.payload)
                    except ValueError:
                        decode_errors += 1
                        continue
                    issues = bf.sanity_issues(sanity_limits)
                    if issues:
                        categories = {issue.category for issue in issues}
                        sanity_encoding += "encoding" in categories
                        physical_rejected += "physical_limit" in categories
                        rejection_reasons.update(issue.reason for issue in issues)
                        if args.verbose and sanity_encoding + physical_rejected <= 3:
                            print(
                                "rejected BalanceFrame: "
                                + "; ".join(f"{issue.category}: {issue.reason}" for issue in issues)
                            )
                        if plotter is not None:
                            plotter.note_reject()
                        continue
                    if last_balance_frame_number is not None:
                        frame_delta = (
                            bf.frame_number - last_balance_frame_number
                        ) & 0xFFFFFFFF
                        if 1 < frame_delta < 0x80000000:
                            balance_frame_gaps += frame_delta - 1
                        elif frame_delta >= 0x80000000:
                            last_source_drop_mod256 = None
                    if last_source_drop_mod256 is not None:
                        source_delta = (
                            bf.source_drop_count_mod256 - last_source_drop_mod256
                        ) & 0xFF
                        if source_delta < 0x80:
                            stm32_source_drops += source_delta
                    last_balance_frame_number = bf.frame_number
                    last_source_drop_mod256 = bf.source_drop_count_mod256
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
                        pitch_deg = bf.pitch_rad * 180.0 / 3.141592653589793
                        print(
                            f"frames={frame_count}  pitch={bf.pitch_rad:.4f}rad "
                            f"({pitch_deg:+.1f}deg)  rate={bf.pitch_rate_rads:+.3f}  "
                            f"cmd={bf.cmd_torque_nm:+.5f}  "
                            f"L/R={bf.cmd_torque_left_nm:+.5f}/"
                            f"{bf.cmd_torque_right_nm:+.5f}  "
                            f"u_fb={bf.u_fb_nm:+.5f}  estop={bf.estop}  "
                            f"imu={bf.imu_valid}"
                        )
                    if args.diag_stall:
                        pitch_abs = abs(bf.pitch_rad)
                        cmd_ctrl = abs(bf.cmd_torque_nm)
                        cmd_motors = max(
                            abs(bf.cmd_torque_left_nm), abs(bf.cmd_torque_right_nm)
                        )
                        if pitch_abs >= stall_pitch_rad and (
                            cmd_ctrl <= args.stall_cmd_eps
                            or cmd_motors <= args.stall_cmd_eps
                        ):
                            now_stall = time.time()
                            if now_stall - last_stall_print >= 0.2:
                                pitch_deg = bf.pitch_rad * 180.0 / 3.141592653589793
                                if bf.estop:
                                    where = "ESTOP (failsafe/imu→motor cmd forced 0)"
                                elif cmd_ctrl <= args.stall_cmd_eps:
                                    where = "CTRL (strategy cmd≈0 while pitched)"
                                else:
                                    where = (
                                        "TX_PATH (ctrl cmd≠0 but L/R≈0; "
                                        "check estop publish path)"
                                    )
                                print(
                                    f"STALL {where}: pitch={pitch_deg:+.1f}deg  "
                                    f"cmd={bf.cmd_torque_nm:+.5f}  "
                                    f"L/R={bf.cmd_torque_left_nm:+.5f}/"
                                    f"{bf.cmd_torque_right_nm:+.5f}  "
                                    f"u_fb={bf.u_fb_nm:+.5f} u_ff={bf.u_ff_nm:+.5f}  "
                                    f"vel={bf.vel_wheel_turns_s:+.3f}  "
                                    f"estop={bf.estop} imu={bf.imu_valid}"
                                )
                                last_stall_print = now_stall

            now = time.time()
            if args.verbose and now - last_stats >= 5.0:
                dt = now - last_stats
                bps = (byte_count - last_byte_count) / max(dt, 0.001)
                fps = (frame_count - last_frame_count) / max(dt, 0.001)
                pc = parser.counters
                print(
                    f"stats: udp={bps:.0f} B/s  frames={fps:.1f}/s  "
                    f"accepted={frame_count}  wire_valid={pc.valid_frames}  "
                    f"bad_crc={pc.crc_errors}  bad_length={pc.length_errors}  "
                    f"bad_version={pc.version_errors}  discarded={pc.bytes_discarded}  "
                    f"udp_datagrams={udp_sequences.counters.received}  "
                    f"udp_missing={udp_sequences.counters.missing}  "
                    f"udp_duplicate={udp_sequences.counters.duplicates}  "
                    f"udp_reordered={udp_sequences.counters.out_of_order}  "
                    f"udp_resets={udp_sequences.counters.resets}  "
                    f"udp_envelope_errors={udp_envelope_errors}  "
                    f"udp_frame_count_errors={udp_frame_count_errors}  "
                    f"frame_gaps={balance_frame_gaps}  "
                    f"stm32_source_drops={stm32_source_drops}  "
                    f"stm32_errors={stm32_errors}  bad_type={bad_type}  "
                    f"decode_errors={decode_errors}  encoding_rejected={sanity_encoding}  "
                    f"physical_rejected={physical_rejected}"
                )
                if rejection_reasons:
                    print(f"rejection reasons: {dict(rejection_reasons.most_common())}")
                last_stats = now
                last_byte_count = byte_count
                last_frame_count = frame_count

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
