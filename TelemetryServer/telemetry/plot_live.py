"""Live plotter for balance telemetry."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional
import threading

from telemetry.mpl_backend import configure_matplotlib

configure_matplotlib()

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from matplotlib.widgets import CheckButtons

from telemetry.balance_frame import BalanceFrame


@dataclass
class _Series:
    label: str
    axis_idx: int
    data: Deque[float] = field(default_factory=lambda: deque(maxlen=1000))
    line: Optional[Line2D] = None
    visible: bool = True


class LiveBalancePlotter:
    def __init__(self, history_s: float = 10.0, expected_hz: float = 500.0) -> None:
        self.max_points = max(100, int(history_s * expected_hz))
        self.host_t: Deque[float] = deque(maxlen=self.max_points)
        self._t0: Optional[float] = None

        self._series: List[_Series] = [
            _Series("pitch_rad", 0),
            _Series("pitch_rate", 0),
            _Series("cmd_torque", 1),
            _Series("cmd_torque_l", 1),
            _Series("cmd_torque_r", 1),
            _Series("u_ff", 1),
            _Series("u_fb", 1),
            _Series("vel_l", 2),
            _Series("vel_r", 2),
        ]
        for s in self._series:
            s.data = deque(maxlen=self.max_points)

        self.fig, self.axes = plt.subplots(4, 1, figsize=(11, 8), sharex=True)
        self.fig.subplots_adjust(right=0.78)
        self.fig.suptitle("Balance telemetry (live)")

        for s in self._series:
            ax = self.axes[s.axis_idx]
            (s.line,) = ax.plot([], [], label=s.label)
            s.line.set_visible(s.visible)

        self.axes[0].set_ylabel("rad")
        self.axes[1].set_ylabel("Nm motor")
        self.axes[2].set_ylabel("turn/s")

        for ax in self.axes[:3]:
            ax.legend(loc="upper left")

        self.stats_text = self.axes[3].text(0.01, 0.5, "", transform=self.axes[3].transAxes, va="center")
        self.axes[3].set_axis_off()

        self._label_to_series: Dict[str, _Series] = {s.label: s for s in self._series}
        check_ax = self.fig.add_axes([0.80, 0.12, 0.18, 0.76])
        check_ax.set_title("Show", fontsize=9)
        check_ax.set_xticks([])
        check_ax.set_yticks([])
        labels = [s.label for s in self._series]
        actives = [s.visible for s in self._series]
        self._check = CheckButtons(check_ax, labels, actives)
        self._check.on_clicked(self._on_checkbox)

        self._last_frame_num: Optional[int] = None
        self._drop_count = 0
        self._reject_count = 0
        self._frame_total = 0
        self._t_first: Optional[float] = None
        self._last_estop_flag = 0
        self._last_imu_flag = 0
        self._lock = threading.Lock()
        self._animation = None

    def _on_checkbox(self, label: str) -> None:
        series = self._label_to_series.get(label)
        if series is None or series.line is None:
            return
        series.visible = not series.visible
        series.line.set_visible(series.visible)
        self.fig.canvas.draw_idle()

    def _series_by_label(self, label: str) -> _Series:
        return self._label_to_series[label]

    def add(self, host_time_s: float, frame: BalanceFrame) -> None:
        with self._lock:
            self._frame_total += 1
            if self._t_first is None:
                self._t_first = host_time_s
            if self._t0 is None:
                self._t0 = host_time_s
            t = host_time_s - self._t0

            if self._last_frame_num is not None:
                delta = frame.frame_number - self._last_frame_num
                if delta > 1:
                    self._drop_count += delta - 1
                elif delta <= 0:
                    self._last_frame_num = None

            self._last_frame_num = frame.frame_number
            self._last_estop_flag = frame.estop
            self._last_imu_flag = frame.imu_valid

            self.host_t.append(t)
            self._series_by_label("pitch_rad").data.append(frame.pitch_rad)
            self._series_by_label("pitch_rate").data.append(frame.pitch_rate_rads)
            self._series_by_label("cmd_torque").data.append(frame.cmd_torque_nm)
            self._series_by_label("cmd_torque_l").data.append(frame.cmd_torque_left_nm)
            self._series_by_label("cmd_torque_r").data.append(frame.cmd_torque_right_nm)
            self._series_by_label("u_ff").data.append(frame.u_ff_nm)
            self._series_by_label("u_fb").data.append(frame.u_fb_nm)
            self._series_by_label("vel_l").data.append(frame.vel_wheel_l_turns_s)
            self._series_by_label("vel_r").data.append(frame.vel_wheel_r_turns_s)

    def note_reject(self) -> None:
        with self._lock:
            self._reject_count += 1

    def _refresh_lines(self) -> List:
        with self._lock:
            xs = list(self.host_t)
            series_data = {s.label: list(s.data) for s in self._series}
            drops = self._drop_count
            estop = self._last_estop_flag
            imu = self._last_imu_flag
            rejects = self._reject_count
            hz = 0.0
            if len(xs) > 1:
                elapsed = max(0.001, xs[-1] - xs[0])
                hz = (len(xs) - 1) / elapsed

        for s in self._series:
            if s.line is not None:
                s.line.set_data(xs, series_data[s.label])

        for ax in self.axes[:3]:
            ax.relim()
            ax.autoscale_view()
        self.axes[1].set_ylim(-0.5, 0.5)

        self.stats_text.set_text(
            f"frames={len(xs)}  ~{hz:.0f}Hz  drops={drops}  rejected={rejects}  estop={estop}  imu={imu}"
        )
        artists: List = [s.line for s in self._series if s.line is not None]
        artists.append(self.stats_text)
        return artists

    def run(self, interval_ms: int = 50) -> None:
        def _anim(_i: int):
            return self._refresh_lines()

        self._animation = FuncAnimation(self.fig, _anim, interval=interval_ms, blit=False)
        plt.tight_layout(rect=[0, 0, 0.78, 0.96])
        plt.show()
