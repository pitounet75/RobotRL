"""Live plotter for balance telemetry."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple
import threading

from telemetry.mpl_backend import configure_matplotlib

configure_matplotlib()

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from matplotlib.widgets import Button, CheckButtons

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
        self._paused = False
        # True: X always shows full buffer. False: keep user zoom/pan on time axis.
        self._x_follow = True
        self._xlim: Optional[Tuple[float, float]] = None
        self._suppress_xlim_cb = False

        self._series: List[_Series] = [
            _Series("pitch_rad", 0),
            _Series("pitch_deg", 0),
            _Series("pitch_rate", 0),
            _Series("cmd_torque", 1),
            _Series("cmd_torque_l", 1),
            _Series("cmd_torque_r", 1),
            _Series("u_ff", 1),
            _Series("u_fb", 1),
            _Series("vel_l", 2),
            _Series("vel_r", 2),
            _Series("estop", 3),
            _Series("imu_valid", 3),
        ]
        for s in self._series:
            s.data = deque(maxlen=self.max_points)

        self.fig = plt.figure(figsize=(11, 9))
        self.axes = self.fig.subplots(4, 1, sharex=True)
        self.fig.subplots_adjust(right=0.78, left=0.08, top=0.92, bottom=0.08)
        self.fig.suptitle("Balance telemetry (live)")

        # Hide pitch_deg by default (same info as pitch_rad); keep for stall reading.
        self._series[1].visible = False

        for s in self._series:
            ax = self.axes[s.axis_idx]
            (s.line,) = ax.plot([], [], label=s.label)
            s.line.set_visible(s.visible)

        self.axes[0].set_ylabel("rad / deg")
        self.axes[1].set_ylabel("Nm motor")
        self.axes[2].set_ylabel("turn/s")
        self.axes[3].set_ylabel("flags")
        self.axes[3].set_ylim(-0.1, 1.2)

        for ax in self.axes:
            ax.legend(loc="upper left")
            ax.grid(True, alpha=0.25)

        self.stats_text = self.fig.text(0.01, 0.01, "", va="bottom")

        self._label_to_series: Dict[str, _Series] = {s.label: s for s in self._series}

        pause_ax = self.fig.add_axes([0.80, 0.915, 0.18, 0.035])
        self._pause_btn = Button(pause_ax, "Pause")
        self._pause_btn.on_clicked(self._on_pause_clicked)

        follow_ax = self.fig.add_axes([0.80, 0.870, 0.18, 0.035])
        self._follow_btn = Button(follow_ax, "Follow X")
        self._follow_btn.on_clicked(self._on_follow_clicked)

        check_ax = self.fig.add_axes([0.80, 0.10, 0.18, 0.74])
        check_ax.set_title("Show", fontsize=9)
        check_ax.set_xticks([])
        check_ax.set_yticks([])
        labels = [s.label for s in self._series]
        actives = [s.visible for s in self._series]
        self._check = CheckButtons(check_ax, labels, actives)
        self._check.on_clicked(self._on_checkbox)

        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        # Toolbar box-zoom / pan changes xlim on the shared axis.
        self.axes[0].callbacks.connect("xlim_changed", self._on_xlim_changed)

        self._last_frame_num: Optional[int] = None
        self._drop_count = 0
        self._reject_count = 0
        self._frame_total = 0
        self._t_first: Optional[float] = None
        self._last_estop_flag = 0
        self._last_imu_flag = 0
        self._lock = threading.Lock()
        self._animation = None
    def _set_follow(self, follow: bool) -> None:
        self._x_follow = follow
        self._follow_btn.label.set_text("Follow X" if follow else "Follow X (off)")
        if follow:
            self._xlim = None

    def _on_pause_clicked(self, _event) -> None:
        with self._lock:
            self._paused = not self._paused
            paused = self._paused
        self._pause_btn.label.set_text("Resume" if paused else "Pause")
        self.fig.suptitle(
            "Balance telemetry (PAUSED)" if paused else "Balance telemetry (live)"
        )
        self.fig.canvas.draw_idle()

    def _on_follow_clicked(self, _event) -> None:
        self._set_follow(True)
        self.fig.canvas.draw_idle()

    def _on_xlim_changed(self, ax) -> None:
        if self._suppress_xlim_cb:
            return
        # User zoomed/panned via toolbar or scroll — stop auto X follow.
        self._x_follow = False
        self._xlim = ax.get_xlim()
        self._follow_btn.label.set_text("Follow X (off)")

    def _on_scroll(self, event) -> None:
        if event.inaxes not in self.axes:
            return
        if event.xdata is None:
            return
        ax = self.axes[0]
        x0, x1 = ax.get_xlim()
        if x1 <= x0:
            return
        # Zoom in on scroll up, out on scroll down; keep cursor time fixed.
        scale = 0.8 if event.button == "up" else 1.25
        left = event.xdata - (event.xdata - x0) * scale
        right = event.xdata + (x1 - event.xdata) * scale
        if right - left < 1e-3:
            return
        self._x_follow = False
        self._xlim = (left, right)
        self._follow_btn.label.set_text("Follow X (off)")
        self._suppress_xlim_cb = True
        try:
            for a in self.axes:
                a.set_xlim(left, right)
        finally:
            self._suppress_xlim_cb = False
        self.fig.canvas.draw_idle()

    def _on_key(self, event) -> None:
        if event.key == " ":
            self._on_pause_clicked(event)
        elif event.key in ("r", "R", "f", "F"):
            self._on_follow_clicked(event)

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
            if self._paused:
                return
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
            self._series_by_label("pitch_deg").data.append(
                frame.pitch_rad * 180.0 / 3.141592653589793
            )
            self._series_by_label("pitch_rate").data.append(frame.pitch_rate_rads)
            self._series_by_label("cmd_torque").data.append(frame.cmd_torque_nm)
            self._series_by_label("cmd_torque_l").data.append(frame.cmd_torque_left_nm)
            self._series_by_label("cmd_torque_r").data.append(frame.cmd_torque_right_nm)
            self._series_by_label("u_ff").data.append(frame.u_ff_nm)
            self._series_by_label("u_fb").data.append(frame.u_fb_nm)
            self._series_by_label("vel_l").data.append(frame.vel_wheel_l_turns_s)
            self._series_by_label("vel_r").data.append(frame.vel_wheel_r_turns_s)
            self._series_by_label("estop").data.append(float(frame.estop))
            self._series_by_label("imu_valid").data.append(float(frame.imu_valid))

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
            paused = self._paused
            x_follow = self._x_follow
            xlim = self._xlim
            hz = 0.0
            if len(xs) > 1:
                elapsed = max(0.001, xs[-1] - xs[0])
                hz = (len(xs) - 1) / elapsed

        n = len(xs)
        for s in self._series:
            if s.line is None:
                continue
            ys = series_data[s.label]
            if len(ys) != n:
                # Avoid killing FuncAnimation on length mismatch.
                continue
            s.line.set_data(xs, ys)

        self._suppress_xlim_cb = True
        try:
            for ax in self.axes[:3]:
                ax.relim()
                if x_follow:
                    ax.autoscale_view()
                else:
                    ax.autoscale_view(scalex=False, scaley=True)
            self.axes[3].set_ylim(-0.1, 1.2)
            if not x_follow and xlim is not None:
                for ax in self.axes:
                    ax.set_xlim(*xlim)
        finally:
            self._suppress_xlim_cb = False

        pitch_deg = series_data["pitch_deg"][-1] if series_data["pitch_deg"] else 0.0
        cmd = series_data["cmd_torque"][-1] if series_data["cmd_torque"] else 0.0
        tags = []
        if paused:
            tags.append("PAUSED")
        if not x_follow:
            tags.append("X-ZOOM")
        tag = ("  " + " ".join(tags)) if tags else ""
        self.stats_text.set_text(
            f"frames={len(xs)}  ~{hz:.0f}Hz  drops={drops}  rejected={rejects}  "
            f"estop={estop}  imu={imu}  pitch={pitch_deg:+.1f}deg  cmd={cmd:+.4f}Nm"
            f"{tag}"
        )
        artists: List = [s.line for s in self._series if s.line is not None]
        artists.append(self.stats_text)
        return artists

    def start_animation(self, interval_ms: int = 50) -> None:
        def _anim(_i: int):
            return self._refresh_lines()

        # Keep a strong ref; some backends GC the animation otherwise.
        self._animation = FuncAnimation(
            self.fig,
            _anim,
            interval=interval_ms,
            blit=False,
            cache_frame_data=False,
        )

    def run(self, interval_ms: int = 50) -> None:
        """Standalone window (no Qt tabs). Prefer TelemetryMainWindow when using --plot."""
        self.start_animation(interval_ms=interval_ms)
        plt.tight_layout(rect=[0, 0.03, 0.78, 0.96])
        plt.show()
