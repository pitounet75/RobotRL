"""Live plotter for balance telemetry (pyqtgraph / PyQt5).

matplotlib's FuncAnimation redraws the whole figure every tick and can't
keep up with 500 Hz BalanceFrame arrival across 12 series; pyqtgraph draws
incrementally (GPU-backed via Qt) and supports per-curve downsampling, so
it stays smooth at this data rate.
"""

from __future__ import annotations

import math
import sys
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from telemetry.balance_frame import BalanceFrame

_SERIES_COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf",
    "#000000", "#999999",
]


@dataclass
class _Series:
    label: str
    axis_idx: int
    color: str
    visible: bool = True
    data: Deque[float] = field(default_factory=lambda: deque(maxlen=1000))
    curve: Optional[pg.PlotDataItem] = None


class LiveBalancePlotter:
    def __init__(self, history_s: float = 10.0, expected_hz: float = 500.0) -> None:
        self.max_points = max(100, int(history_s * expected_hz))
        self.host_t: Deque[float] = deque(maxlen=self.max_points)
        self._t0: Optional[float] = None

        self._series: List[_Series] = [
            _Series("pitch_rad", 0, _SERIES_COLORS[0]),
            _Series("pitch_deg", 0, _SERIES_COLORS[1], visible=False),
            _Series("pitch_rate", 0, _SERIES_COLORS[2]),
            _Series("cmd_torque", 1, _SERIES_COLORS[3]),
            _Series("cmd_torque_l", 1, _SERIES_COLORS[4]),
            _Series("cmd_torque_r", 1, _SERIES_COLORS[5]),
            _Series("u_ff", 1, _SERIES_COLORS[6]),
            _Series("u_fb", 1, _SERIES_COLORS[7]),
            _Series("vel_l", 2, _SERIES_COLORS[8]),
            _Series("vel_r", 2, _SERIES_COLORS[9]),
            _Series("estop", 3, _SERIES_COLORS[10]),
            _Series("imu_valid", 3, _SERIES_COLORS[11]),
        ]
        for s in self._series:
            s.data = deque(maxlen=self.max_points)
        self._label_to_series: Dict[str, _Series] = {s.label: s for s in self._series}

        pg.setConfigOptions(antialias=False, background="w", foreground="k")
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

        self.win = QtWidgets.QMainWindow()
        self.win.setWindowTitle("Balance telemetry (live)")
        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)
        self.win.setCentralWidget(central)

        self.glw = pg.GraphicsLayoutWidget()
        layout.addWidget(self.glw, 1)

        ylabels = ["rad / deg", "Nm motor", "turn/s", "flags"]
        self.plots: List[pg.PlotItem] = []
        prev_plot: Optional[pg.PlotItem] = None
        for i, ylabel in enumerate(ylabels):
            plot_item = self.glw.addPlot(row=i, col=0)
            plot_item.setLabel("left", ylabel)
            plot_item.showGrid(x=True, y=True, alpha=0.25)
            plot_item.addLegend(offset=(10, 10))
            if prev_plot is not None:
                plot_item.setXLink(prev_plot)
            prev_plot = plot_item
            self.plots.append(plot_item)
        self.plots[3].setYRange(-0.1, 1.2)
        self.plots[-1].setLabel("bottom", "time [s]")

        for s in self._series:
            curve = self.plots[s.axis_idx].plot(pen=pg.mkPen(s.color, width=1.5), name=s.label)
            curve.setDownsampling(auto=True, method="peak")
            curve.setClipToView(True)
            curve.setVisible(s.visible)
            s.curve = curve

        sidebar = QtWidgets.QWidget()
        sidebar.setFixedWidth(180)
        side_layout = QtWidgets.QVBoxLayout(sidebar)
        side_layout.addWidget(QtWidgets.QLabel("<b>Show</b>"))
        for s in self._series:
            checkbox = QtWidgets.QCheckBox(s.label)
            checkbox.setChecked(s.visible)
            checkbox.stateChanged.connect(
                lambda state, series=s: self._on_checkbox(series, state)
            )
            side_layout.addWidget(checkbox)
        side_layout.addStretch(1)
        self.stats_label = QtWidgets.QLabel("")
        self.stats_label.setWordWrap(True)
        side_layout.addWidget(self.stats_label)
        layout.addWidget(sidebar)

        self.win.resize(1200, 900)

        self._last_frame_num: Optional[int] = None
        self._drop_count = 0
        self._reject_count = 0
        self._frame_total = 0
        self._t_first: Optional[float] = None
        self._last_estop_flag = 0
        self._last_imu_flag = 0
        self._lock = threading.Lock()
        self._timer: Optional[QtCore.QTimer] = None

    def _on_checkbox(self, series: _Series, state: int) -> None:
        series.visible = state == QtCore.Qt.Checked
        if series.curve is not None:
            series.curve.setVisible(series.visible)

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
            self._series_by_label("pitch_deg").data.append(frame.pitch_rad * 180.0 / math.pi)
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

    def _refresh(self) -> None:
        with self._lock:
            xs = np.fromiter(self.host_t, dtype=np.float64, count=len(self.host_t))
            series_data = {
                s.label: np.fromiter(s.data, dtype=np.float64, count=len(s.data))
                for s in self._series
            }
            drops = self._drop_count
            estop = self._last_estop_flag
            imu = self._last_imu_flag
            rejects = self._reject_count
            frame_total = self._frame_total
            hz = 0.0
            if len(xs) > 1:
                elapsed = max(0.001, xs[-1] - xs[0])
                hz = (len(xs) - 1) / elapsed

        for s in self._series:
            if s.curve is None:
                continue
            ys = series_data[s.label]
            if len(ys) != len(xs):
                # Avoid a torn frame while a new series catches up mid-append.
                continue
            s.curve.setData(xs, ys)

        pitch_deg = series_data["pitch_deg"][-1] if len(series_data["pitch_deg"]) else 0.0
        cmd = series_data["cmd_torque"][-1] if len(series_data["cmd_torque"]) else 0.0
        self.stats_label.setText(
            f"frames={frame_total}\n"
            f"~{hz:.0f} Hz\n"
            f"drops={drops}\n"
            f"rejected={rejects}\n"
            f"estop={estop}\n"
            f"imu={imu}\n"
            f"pitch={pitch_deg:+.1f} deg\n"
            f"cmd={cmd:+.4f} Nm"
        )

    def run(self, interval_ms: int = 50) -> None:
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(interval_ms)
        self.win.show()
        self.app.exec_()
