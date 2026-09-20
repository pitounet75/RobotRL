"""Qt main window: live graphs + gains editor tabs."""

from __future__ import annotations

import math
from typing import Optional

from telemetry.drive_stick import (
    DEFAULT_MAX_MPS,
    DEFAULT_MAX_YAW_DEG_S,
    IDLE_EPS,
    SEND_PERIOD_S,
    analog_step,
    hold_dir,
)
from telemetry.gains_panel import GainsPanel
from telemetry.mpl_backend import configure_matplotlib
from telemetry.plot_live import LiveBalancePlotter
from telemetry.rpc_mux import SharedRpcClient
from telemetry.vbus_alert import vbus_alert_level

configure_matplotlib()

try:
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
except ImportError:  # pragma: no cover
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT

try:
    from PyQt5.QtCore import QEvent, Qt, QTimer
    from PyQt5.QtWidgets import (
        QApplication,
        QDoubleSpinBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QPushButton,
        QTabWidget,
        QVBoxLayout,
        QWidget,
    )
except ImportError:  # pragma: no cover
    from PySide2.QtCore import QEvent, Qt, QTimer
    from PySide2.QtWidgets import (
        QApplication,
        QDoubleSpinBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QPushButton,
        QTabWidget,
        QVBoxLayout,
        QWidget,
    )

_DEFAULT_WHEEL_RADIUS_M = 0.04
# Motor ABZ → wheel: gear motor:wheel = 3:16 → ω_wheel = ω_motor * (3/16).
_GEAR_WHEEL_PER_MOTOR = 3.0 / 16.0


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
        self._held = {"up": False, "down": False, "left": False, "right": False}
        self._cmd_mps = 0.0
        self._cmd_yaw = 0.0
        self._last_sent_mps: Optional[float] = None
        self._last_sent_yaw: Optional[float] = None

        tabs = QTabWidget()
        self.setCentralWidget(tabs)

        graph_page = QWidget()
        graph_layout = QVBoxLayout(graph_page)

        drive_row = QHBoxLayout()
        drive_row.addWidget(QLabel("↑↓ speed  ←→ yaw"))
        drive_row.addWidget(QLabel("Max speed"))
        self._max_mps = QDoubleSpinBox()
        self._max_mps.setRange(0.01, 4.0)
        self._max_mps.setSingleStep(0.05)
        self._max_mps.setDecimals(2)
        self._max_mps.setValue(DEFAULT_MAX_MPS)
        self._max_mps.setSuffix(" m/s")
        drive_row.addWidget(self._max_mps)
        drive_row.addWidget(QLabel("Max yaw"))
        self._max_yaw = QDoubleSpinBox()
        self._max_yaw.setRange(1.0, 229.0)
        self._max_yaw.setSingleStep(5.0)
        self._max_yaw.setDecimals(0)
        self._max_yaw.setValue(DEFAULT_MAX_YAW_DEG_S)
        self._max_yaw.setSuffix(" °/s")
        drive_row.addWidget(self._max_yaw)
        self._speed_label = QLabel(self._format_speed_label(0.0))
        self._speed_label.setMinimumWidth(220)
        drive_row.addWidget(self._speed_label)
        self._heading_label = QLabel(self._format_yaw_label(0.0))
        self._heading_label.setMinimumWidth(220)
        drive_row.addWidget(self._heading_label)
        self._vbus_lamp = QLabel()
        self._vbus_lamp.setFixedSize(14, 14)
        self._vbus_lamp.setToolTip("Vbus: orange < 11.4 V, rouge clignotant < 11 V")
        self._vbus_blink_on = True
        self._set_vbus_lamp("off")
        drive_row.addWidget(self._vbus_lamp)
        self._vbus_label = QLabel("Vbus L — V · R — V")
        self._vbus_label.setMinimumWidth(220)
        drive_row.addWidget(self._vbus_label)
        self._sync_l_lamp = QLabel()
        self._sync_l_lamp.setFixedSize(14, 14)
        self._sync_l_lamp.setToolTip("Vert = cette roue en SYNC / BOTH_AIR, rouge = NORMAL / RECOVERY")
        drive_row.addWidget(self._sync_l_lamp)
        drive_row.addWidget(QLabel("sync_l"))
        self._sync_r_lamp = QLabel()
        self._sync_r_lamp.setFixedSize(14, 14)
        self._sync_r_lamp.setToolTip("Vert = cette roue en SYNC / BOTH_AIR, rouge = NORMAL / RECOVERY")
        drive_row.addWidget(self._sync_r_lamp)
        drive_row.addWidget(QLabel("sync_r"))
        self._set_sync_lamp(self._sync_l_lamp, False)
        self._set_sync_lamp(self._sync_r_lamp, False)
        self._drive_zero_btn = QPushButton("0")
        self._drive_zero_btn.setFixedWidth(36)
        drive_row.addWidget(self._drive_zero_btn)
        drive_row.addStretch(1)
        graph_layout.addLayout(drive_row)

        self._drive_timer = QTimer(self)
        self._drive_timer.setInterval(16)
        self._drive_timer.timeout.connect(self._on_drive_tick)
        self._send_timer = QTimer(self)
        self._send_timer.setInterval(int(SEND_PERIOD_S * 1000))
        self._send_timer.timeout.connect(self._flush_drive)
        self._vbus_timer = QTimer(self)
        self._vbus_timer.setInterval(200)
        self._vbus_timer.timeout.connect(self._refresh_vbus_label)
        self._vbus_timer.start()

        if rpc is None:
            self._drive_zero_btn.setEnabled(False)
            self._speed_label.setText("Speed (needs --esp32-host)")
            self._heading_label.setText("Yaw (needs --esp32-host)")
        else:
            self._drive_zero_btn.clicked.connect(self._zero_drive)
            self._load_wheel_radius()
            app = QApplication.instance()
            if app is not None:
                app.installEventFilter(self)

        canvas = FigureCanvasQTAgg(plotter.fig)
        toolbar = NavigationToolbar2QT(canvas, graph_page)
        graph_layout.addWidget(toolbar)
        graph_layout.addWidget(canvas)
        tabs.addTab(graph_page, "Graphs")

        gains = GainsPanel(rpc)
        tabs.addTab(gains, "Gains")
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

    def _format_yaw_label(self, rad_s: float) -> str:
        return f"yaw {rad_s:+.2f} rad/s  ({math.degrees(rad_s):+.0f} °/s)"

    def _max_mps_value(self) -> float:
        return max(0.01, float(self._max_mps.value()))

    def _max_yaw_value(self) -> float:
        return math.radians(max(1.0, float(self._max_yaw.value())))

    def _drive_blocked(self) -> bool:
        focus = QApplication.focusWidget()
        return isinstance(focus, (QLineEdit, QDoubleSpinBox))

    def _key_name(self, key: int) -> str:
        if key == Qt.Key_Up:
            return "up"
        if key == Qt.Key_Down:
            return "down"
        if key == Qt.Key_Left:
            return "left"
        if key == Qt.Key_Right:
            return "right"
        return ""

    def eventFilter(self, obj, event):
        et = event.type()
        if et in (QEvent.KeyPress, QEvent.KeyRelease) and not event.isAutoRepeat():
            name = self._key_name(event.key())
            if name:
                if et == QEvent.KeyPress and self._drive_blocked():
                    return False
                self._held[name] = et == QEvent.KeyPress
                self._ensure_drive()
                return True
        if et == QEvent.ApplicationDeactivate:
            self._held = {k: False for k in self._held}
            self._ensure_drive()
        return super().eventFilter(obj, event)

    def _ensure_drive(self) -> None:
        if not self._drive_timer.isActive():
            self._drive_timer.start()
        if self._rpc is not None and not self._send_timer.isActive():
            self._send_timer.start()

    def _idle(self) -> bool:
        return (
            not any(self._held.values())
            and abs(self._cmd_mps) <= IDLE_EPS
            and abs(self._cmd_yaw) <= IDLE_EPS
        )

    def _on_drive_tick(self) -> None:
        dt = self._drive_timer.interval() / 1000.0
        self._cmd_mps = analog_step(
            self._cmd_mps,
            hold_dir(self._held["up"], self._held["down"]),
            self._max_mps_value(),
            dt,
        )
        self._cmd_yaw = analog_step(
            self._cmd_yaw,
            hold_dir(self._held["right"], self._held["left"]),
            self._max_yaw_value(),
            dt,
        )
        if abs(self._cmd_mps) < IDLE_EPS:
            self._cmd_mps = 0.0
        if abs(self._cmd_yaw) < IDLE_EPS:
            self._cmd_yaw = 0.0
        self._speed_label.setText(self._format_speed_label(self._cmd_mps))
        self._heading_label.setText(self._format_yaw_label(self._cmd_yaw))
        if self._idle():
            self._flush_drive()
            self._drive_timer.stop()
            self._send_timer.stop()

    def _flush_drive(self) -> None:
        if self._rpc is None:
            return
        vel_changed = (
            self._last_sent_mps is None or abs(self._cmd_mps - self._last_sent_mps) > IDLE_EPS
        )
        yaw_changed = (
            self._last_sent_yaw is None or abs(self._cmd_yaw - self._last_sent_yaw) > IDLE_EPS
        )
        if not vel_changed and not yaw_changed:
            return
        try:
            if vel_changed:
                self._rpc.set_param("vel_ref_turns_s", self._mps_to_motor_turns_s(self._cmd_mps))
                self._last_sent_mps = self._cmd_mps
            if yaw_changed:
                self._rpc.set_param("heading_ref_rad", self._cmd_yaw)
                self._last_sent_yaw = self._cmd_yaw
        except Exception as exc:
            self._speed_label.setText(f"SET failed: {exc}")

    def _zero_drive(self) -> None:
        self._held = {k: False for k in self._held}
        self._cmd_mps = 0.0
        self._cmd_yaw = 0.0
        self._speed_label.setText(self._format_speed_label(0.0))
        self._heading_label.setText(self._format_yaw_label(0.0))
        self._flush_drive()

    def _set_vbus_lamp(self, level: str) -> None:
        if level == "crit":
            color = "#e10600" if self._vbus_blink_on else "#3a0000"
        elif level == "warn":
            color = "#f0a202"
        else:
            color = "#2a2e34"
        self._vbus_lamp.setStyleSheet(
            f"border-radius: 7px; background: {color}; border: 1px solid #111;"
        )

    def _set_sync_lamp(self, lamp: QLabel, on: bool) -> None:
        color = "#1ecf4a" if on else "#e10600"
        lamp.setStyleSheet(
            f"border-radius: 7px; background: {color}; border: 1px solid #111;"
        )

    def _refresh_vbus_label(self) -> None:
        left = float(getattr(self._plotter, "vbus_l_v", 0.0))
        right = float(getattr(self._plotter, "vbus_r_v", 0.0))
        if left <= 0.5 and right <= 0.5:
            self._vbus_label.setText("Vbus L — V · R — V")
            self._set_vbus_lamp("off")
        else:
            self._vbus_label.setText(f"Vbus L {left:.2f} V · R {right:.2f} V")
            level = vbus_alert_level(left, right)
            if level == "crit":
                self._vbus_blink_on = not self._vbus_blink_on
            else:
                self._vbus_blink_on = True
            self._set_vbus_lamp(level)
        self._set_sync_lamp(self._sync_l_lamp, bool(getattr(self._plotter, "sync_l", 0)))
        self._set_sync_lamp(self._sync_r_lamp, bool(getattr(self._plotter, "sync_r", 0)))

    def _load_wheel_radius(self) -> None:
        if self._rpc is None:
            return
        try:
            snap = self._rpc.get_params()
            if snap.wheel_radius_m > 1e-6:
                self._wheel_radius_m = float(snap.wheel_radius_m)
            self._speed_label.setText(self._format_speed_label(self._cmd_mps))
        except Exception as exc:
            self._speed_label.setText(f"Speed sync failed: {exc}")


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
