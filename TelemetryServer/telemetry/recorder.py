"""CSV recorder for decoded balance frames."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Optional, TextIO

from telemetry.balance_frame import BalanceFrame

CSV_HEADER = [
    "host_time_s",
    "frame_number",
    "time_us",
    "pitch_rad",
    "pitch_rate_rads",
    "vel_wheel_turns_s",
    "vel_wheel_l_turns_s",
    "vel_wheel_r_turns_s",
    "cmd_torque_nm",
    "cmd_torque_left_nm",
    "cmd_torque_right_nm",
    "u_ff_nm",
    "u_fb_nm",
    "pitch_ref_rad",
    "imu_valid",
    "estop",
    "strategy_id",
    "source_drop_count_mod256",
]


class CsvRecorder:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._fp: Optional[TextIO] = None
        self._writer: Optional[csv.writer] = None

    def open(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._fp)
        self._writer.writerow(CSV_HEADER)

    def write(self, host_time_s: float, frame: BalanceFrame) -> None:
        if self._writer is None:
            self.open()
        assert self._writer is not None
        self._writer.writerow(
            [
                f"{host_time_s:.6f}",
                frame.frame_number,
                frame.time_us,
                frame.pitch_rad,
                frame.pitch_rate_rads,
                frame.vel_wheel_turns_s,
                frame.vel_wheel_l_turns_s,
                frame.vel_wheel_r_turns_s,
                frame.cmd_torque_nm,
                frame.cmd_torque_left_nm,
                frame.cmd_torque_right_nm,
                frame.u_ff_nm,
                frame.u_fb_nm,
                frame.pitch_ref_rad,
                frame.imu_valid,
                frame.estop,
                frame.strategy_id,
                frame.source_drop_count_mod256,
            ]
        )

    def close(self) -> None:
        if self._fp is not None:
            self._fp.close()
            self._fp = None
            self._writer = None
