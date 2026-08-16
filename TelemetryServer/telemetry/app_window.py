"""Qt main window: live graphs + gains editor tabs."""

from __future__ import annotations

import math
from typing import Optional

from telemetry.gains_panel import GainsPanel
from telemetry.mpl_backend import configure_matplotlib
from telemetry.plot_live import LiveBalancePlotter
from telemetry.rpc_mux import SharedRpcClient

configure_matplotlib()

try:
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
except ImportError:  # pragma: no cover
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT

try:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (
        QApplication,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QSlider,
        QTabWidget,
        QVBoxLayout,
        QWidget,
    )
except ImportError:  # pragma: no cover
    from PySide2.QtCore import Qt, QTimer
    from PySide2.QtWidgets import (
        QApplication,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QSlider,
        QTabWidget,
        QVBoxLayout,
        QWidget,
    )

_DEFAULT_WHEEL_RADIUS_M = 0.04
# Motor ABZ → wheel: gear motor:wheel = 3:16 → ω_wheel = ω_motor * (3/16).
_GEAR_WHEEL_PER_MOTOR = 3.0 / 16.0
_SLIDER_MM_S_MIN = -4000  # ±4.0 m/s in mm/s
_SLIDER_MM_S_MAX = 4000
_SLIDER_HEADING_DEG_MIN = -180
_SLIDER_HEADING_DEG_MAX = 180


class TelemetryMainWindow(QMainWindow):
    def __init__(
        self,
        plotter: LiveBalancePlotter,
        rpc: Optional[SharedRpcClient] = None,
    ) -> None:
        super().__init__()
        self.setWindowTitle("RobotRL Telemetry")
        self.resize(1200, 850)
        self._plotter = plotter
        self._rpc = rpc
        self._wheel_radius_m = _DEFAULT_WHEEL_RADIUS_M
        self._gear_wheel_per_motor = _GEAR_WHEEL_PER_MOTOR
        self._speed_send_pending_mm_s: Optional[int] = None
        self._speed_suppress = False
        self._heading_send_pending_deg: Optional[int] = None
        self._heading_suppress = False

        tabs = QTabWidget()
        self.setCentralWidget(tabs)

        graph_page = QWidget()
        graph_layout = QVBoxLayout(graph_page)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Speed"))
        self._speed_slider = QSlider(Qt.Horizontal)
        self._speed_slider.setMinimum(_SLIDER_MM_S_MIN)
        self._speed_slider.setMaximum(_SLIDER_MM_S_MAX)
        self._speed_slider.setValue(0)
        self._speed_slider.setTickPosition(QSlider.TicksBelow)
        self._speed_slider.setTickInterval(1000)
        self._speed_slider.setSingleStep(10)
        self._speed_slider.setPageStep(100)
        speed_row.addWidget(self._speed_slider, stretch=1)
        self._speed_label = QLabel(self._format_speed_label(0.0))
        self._speed_label.setMinimumWidth(200)
        speed_row.addWidget(self._speed_label)
        self._speed_zero_btn = QPushButton("0")
        self._speed_zero_btn.setFixedWidth(36)
        speed_row.addWidget(self._speed_zero_btn)
        graph_layout.addLayout(speed_row)

        heading_row = QHBoxLayout()
        heading_row.addWidget(QLabel("Heading"))
        self._heading_slider = QSlider(Qt.Horizontal)
        self._heading_slider.setMinimum(_SLIDER_HEADING_DEG_MIN)
        self._heading_slider.setMaximum(_SLIDER_HEADING_DEG_MAX)
        self._heading_slider.setValue(0)
        self._heading_slider.setTickPosition(QSlider.TicksBelow)
        self._heading_slider.setTickInterval(45)
        self._heading_slider.setSingleStep(1)
        self._heading_slider.setPageStep(15)
        heading_row.addWidget(self._heading_slider, stretch=1)
        self._heading_label = QLabel(self._format_heading_label(0))
        self._heading_label.setMinimumWidth(200)
        heading_row.addWidget(self._heading_label)
        self._heading_zero_btn = QPushButton("0")
        self._heading_zero_btn.setFixedWidth(36)
        heading_row.addWidget(self._heading_zero_btn)
        graph_layout.addLayout(heading_row)

        if rpc is None:
            self._speed_slider.setEnabled(False)
            self._speed_zero_btn.setEnabled(False)
            self._speed_label.setText("Speed (needs --esp32-host)")
            self._heading_slider.setEnabled(False)
            self._heading_zero_btn.setEnabled(False)
            self._heading_label.setText("Heading (needs --esp32-host)")
        else:
            self._speed_slider.valueChanged.connect(self._on_speed_slider)
            self._speed_zero_btn.clicked.connect(lambda: self._speed_slider.setValue(0))
            self._speed_timer = QTimer(self)
            self._speed_timer.setSingleShot(True)
            self._speed_timer.setInterval(80)  # ~12.5 Hz max SET rate while dragging
            self._speed_timer.timeout.connect(self._flush_speed_setpoint)

            self._heading_slider.valueChanged.connect(self._on_heading_slider)
            self._heading_zero_btn.clicked.connect(lambda: self._heading_slider.setValue(0))
            self._heading_timer = QTimer(self)
            self._heading_timer.setSingleShot(True)
            self._heading_timer.setInterval(80)
            self._heading_timer.timeout.connect(self._flush_heading_setpoint)

            self._load_setpoints_from_robot()

        canvas = FigureCanvasQTAgg(plotter.fig)
        toolbar = NavigationToolbar2QT(canvas, graph_page)
        graph_layout.addWidget(toolbar)
        graph_layout.addWidget(canvas)
        tabs.addTab(graph_page, "Graphs")

        gains = GainsPanel(rpc)
        tabs.addTab(gains, "Gains")
        # Auto-load once when opening Gains (lazy on first show via tab change).
        tabs.currentChanged.connect(
            lambda idx: gains.refresh() if idx == 1 and not gains._loaded else None
        )

    def _mps_to_motor_turns_s(self, v_mps: float) -> float:
        """Ground speed → motor shaft turn/s (matches ABZ / vel_ref units)."""
        wheel_m_per_motor_turn = (
            self._gear_wheel_per_motor * 2.0 * math.pi * self._wheel_radius_m
        )
        if wheel_m_per_motor_turn <= 1e-12:
            return 0.0
        return v_mps / wheel_m_per_motor_turn

    def _motor_turns_s_to_mps(self, motor_turns_s: float) -> float:
        return (
            motor_turns_s
            * self._gear_wheel_per_motor
            * 2.0
            * math.pi
            * self._wheel_radius_m
        )

    def _format_speed_label(self, v_mps: float) -> str:
        turns = self._mps_to_motor_turns_s(v_mps)
        return f"{v_mps:+.2f} m/s  ({turns:+.3f} motor turn/s)"

    def _format_heading_label(self, deg: int | float) -> str:
        rad = math.radians(float(deg))
        return f"{float(deg):+.0f}°  ({rad:+.3f} rad)"

    def _on_speed_slider(self, mm_s: int) -> None:
        if self._speed_suppress:
            return
        v_mps = mm_s / 1000.0
        self._speed_label.setText(self._format_speed_label(v_mps))
        self._speed_send_pending_mm_s = mm_s
        self._speed_timer.start()

    def _flush_speed_setpoint(self) -> None:
        if self._rpc is None or self._speed_send_pending_mm_s is None:
            return
        mm_s = self._speed_send_pending_mm_s
        self._speed_send_pending_mm_s = None
        v_mps = mm_s / 1000.0
        turns_s = self._mps_to_motor_turns_s(v_mps)
        try:
            _id, _name, applied = self._rpc.set_param("vel_ref_turns_s", turns_s)
            applied_mps = self._motor_turns_s_to_mps(applied)
            self._speed_label.setText(self._format_speed_label(applied_mps))
        except Exception as exc:
            self._speed_label.setText(f"SET failed: {exc}")

    def _on_heading_slider(self, deg: int) -> None:
        if self._heading_suppress:
            return
        self._heading_label.setText(self._format_heading_label(deg))
        self._heading_send_pending_deg = deg
        self._heading_timer.start()

    def _flush_heading_setpoint(self) -> None:
        if self._rpc is None or self._heading_send_pending_deg is None:
            return
        deg = self._heading_send_pending_deg
        self._heading_send_pending_deg = None
        rad = math.radians(float(deg))
        try:
            _id, _name, applied = self._rpc.set_param("heading_ref_rad", rad)
            applied_deg = int(round(math.degrees(applied)))
            applied_deg = max(
                _SLIDER_HEADING_DEG_MIN, min(_SLIDER_HEADING_DEG_MAX, applied_deg)
            )
            self._heading_suppress = True
            try:
                self._heading_slider.setValue(applied_deg)
            finally:
                self._heading_suppress = False
            self._heading_label.setText(self._format_heading_label(applied_deg))
        except Exception as exc:
            self._heading_label.setText(f"SET failed: {exc}")

    def _load_setpoints_from_robot(self) -> None:
        if self._rpc is None:
            return
        try:
            snap = self._rpc.get_params()
            if snap.wheel_radius_m > 1e-6:
                self._wheel_radius_m = float(snap.wheel_radius_m)
            v_mps = self._motor_turns_s_to_mps(float(snap.vel_ref_turns_s))
            mm_s = int(round(v_mps * 1000.0))
            mm_s = max(_SLIDER_MM_S_MIN, min(_SLIDER_MM_S_MAX, mm_s))
            self._speed_suppress = True
            try:
                self._speed_slider.setValue(mm_s)
            finally:
                self._speed_suppress = False
            self._speed_label.setText(self._format_speed_label(mm_s / 1000.0))

            deg = int(round(math.degrees(float(snap.heading_ref_rad))))
            deg = max(_SLIDER_HEADING_DEG_MIN, min(_SLIDER_HEADING_DEG_MAX, deg))
            self._heading_suppress = True
            try:
                self._heading_slider.setValue(deg)
            finally:
                self._heading_suppress = False
            self._heading_label.setText(self._format_heading_label(deg))
        except Exception as exc:
            self._speed_label.setText(f"Speed sync failed: {exc}")
            self._heading_label.setText(f"Heading sync failed: {exc}")


def run_telemetry_window(
    plotter: LiveBalancePlotter,
    rpc: Optional[SharedRpcClient] = None,
) -> int:
    app = QApplication.instance()
    owns_app = app is None
    if owns_app:
        app = QApplication([])
    win = TelemetryMainWindow(plotter, rpc=rpc)
    win.show()
    plotter.start_animation(interval_ms=50)
    code = app.exec_() if hasattr(app, "exec_") else app.exec()
    return int(code)
