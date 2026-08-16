"""Mouse stick remote control for RobotRL (speed + heading via telemetry RPC)."""

from __future__ import annotations

import math
import threading
from typing import Optional

from telemetry.ctrl_client import ControlParamsClient

try:
    from PyQt5.QtCore import Qt, QTimer, QPointF
    from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont
    from PyQt5.QtWidgets import (
        QApplication,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QSlider,
        QVBoxLayout,
        QWidget,
    )
except ImportError:  # pragma: no cover
    from PySide2.QtCore import Qt, QTimer, QPointF
    from PySide2.QtGui import QPainter, QColor, QPen, QBrush, QFont
    from PySide2.QtWidgets import (
        QApplication,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QSlider,
        QVBoxLayout,
        QWidget,
    )

_DEFAULT_WHEEL_RADIUS_M = 0.04
_GEAR_WHEEL_PER_MOTOR = 3.0 / 16.0

_SPEED_SENS_MIN_MM_S = 200
_SPEED_SENS_MAX_MM_S = 10000
_YAW_RATE_MIN_DEG_S = 5
_YAW_RATE_MAX_DEG_S = 180
_TX_HZ = 10
_TX_INTERVAL_MS = 1000 // _TX_HZ
_STICK_DEADZONE = 0.05


class StickPad(QWidget):
    """Click-drag stick: Y = forward/back, X = left/right (heading rate)."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(360, 360)
        self.setMouseTracking(True)
        self._armed = False
        self._nx = 0.0
        self._ny = 0.0
        self._on_change = None

    def set_change_callback(self, cb) -> None:
        self._on_change = cb

    def _emit(self) -> None:
        if self._on_change is not None:
            self._on_change(self._nx, self._ny)

    def _set_from_pos(self, pos) -> None:
        cx = self.width() * 0.5
        cy = self.height() * 0.5
        r = min(cx, cy) - 12.0
        if r < 1.0:
            return
        dx = float(pos.x()) - cx
        dy = float(pos.y()) - cy
        nx = dx / r
        ny = -dy / r
        mag = math.hypot(nx, ny)
        if mag > 1.0:
            nx /= mag
            ny /= mag
        self._nx = nx
        self._ny = ny
        self.update()
        self._emit()

    def zero(self) -> None:
        self._armed = False
        self._nx = 0.0
        self._ny = 0.0
        self.update()
        self._emit()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self._armed = True
            self._set_from_pos(event.pos())

    def mouseMoveEvent(self, event) -> None:
        if self._armed and (event.buttons() & Qt.LeftButton):
            self._set_from_pos(event.pos())

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self.zero()

    def leaveEvent(self, event) -> None:
        if self._armed:
            self.zero()
        super().leaveEvent(event)

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w * 0.5, h * 0.5
        r = min(cx, cy) - 12.0

        painter.fillRect(self.rect(), QColor(28, 30, 34))
        painter.setPen(QPen(QColor(70, 74, 82), 2))
        painter.setBrush(QBrush(QColor(40, 44, 52)))
        painter.drawEllipse(QPointF(cx, cy), r, r)

        painter.setPen(QPen(QColor(90, 94, 102), 1, Qt.DashLine))
        painter.drawLine(QPointF(cx - r, cy), QPointF(cx + r, cy))
        painter.drawLine(QPointF(cx, cy - r), QPointF(cx, cy + r))

        painter.setPen(QPen(QColor(160, 170, 180)))
        font = QFont()
        font.setPointSize(9)
        painter.setFont(font)
        painter.drawText(int(cx - 18), int(cy - r + 18), "FWD")
        painter.drawText(int(cx - 18), int(cy + r - 6), "REV")
        painter.drawText(int(cx - r + 8), int(cy + 4), "L")
        painter.drawText(int(cx + r - 18), int(cy + 4), "R")

        knob_x = cx + self._nx * r
        knob_y = cy - self._ny * r
        painter.setPen(QPen(QColor(220, 120, 60), 2))
        painter.setBrush(QBrush(QColor(240, 140, 70)))
        painter.drawEllipse(QPointF(knob_x, knob_y), 14, 14)


class RemoteControlWindow(QMainWindow):
    def __init__(self, client: ControlParamsClient) -> None:
        super().__init__()
        self.setWindowTitle("RobotRL Remote Control")
        self.resize(520, 640)
        self._client = client
        self._rpc_lock = threading.Lock()
        self._wheel_radius_m = _DEFAULT_WHEEL_RADIUS_M
        self._gear = _GEAR_WHEEL_PER_MOTOR
        self._nx = 0.0
        self._ny = 0.0
        self._pending_send = True
        self._heading_ref_rad = 0.0
        self._last_vel: Optional[float] = None
        self._last_heading: Optional[float] = None

        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        layout.addWidget(
            QLabel(
                "Hold left button: up/down = speed, left/right = yaw rate "
                f"({_TX_HZ} Hz → heading_ref).\n"
                "Release = v=0 (heading_ref kept). Do not use bind-port 5000 with the plotter."
            )
        )

        sens_speed = QHBoxLayout()
        sens_speed.addWidget(QLabel("Sens. avant/arrière"))
        self._speed_sens = QSlider(Qt.Horizontal)
        self._speed_sens.setMinimum(_SPEED_SENS_MIN_MM_S)
        self._speed_sens.setMaximum(_SPEED_SENS_MAX_MM_S)
        self._speed_sens.setValue(4000)
        self._speed_sens.setTickPosition(QSlider.TicksBelow)
        self._speed_sens.setTickInterval(1000)
        sens_speed.addWidget(self._speed_sens, stretch=1)
        self._speed_sens_label = QLabel("")
        self._speed_sens_label.setMinimumWidth(90)
        sens_speed.addWidget(self._speed_sens_label)
        layout.addLayout(sens_speed)

        sens_yaw = QHBoxLayout()
        sens_yaw.addWidget(QLabel("Sens. yaw (°/s)"))
        self._yaw_sens = QSlider(Qt.Horizontal)
        self._yaw_sens.setMinimum(_YAW_RATE_MIN_DEG_S)
        self._yaw_sens.setMaximum(_YAW_RATE_MAX_DEG_S)
        self._yaw_sens.setValue(45)
        self._yaw_sens.setTickPosition(QSlider.TicksBelow)
        self._yaw_sens.setTickInterval(15)
        sens_yaw.addWidget(self._yaw_sens, stretch=1)
        self._yaw_sens_label = QLabel("")
        self._yaw_sens_label.setMinimumWidth(90)
        sens_yaw.addWidget(self._yaw_sens_label)
        layout.addLayout(sens_yaw)

        self._pad = StickPad()
        self._pad.set_change_callback(self._on_stick)
        layout.addWidget(self._pad, stretch=1)

        self._cmd_label = QLabel("")
        layout.addWidget(self._cmd_label)

        btns = QHBoxLayout()
        self._btn_stop = QPushButton("STOP")
        self._btn_heading_reset = QPushButton("heading_reset")
        btns.addWidget(self._btn_stop)
        btns.addWidget(self._btn_heading_reset)
        btns.addStretch(1)
        layout.addLayout(btns)

        self._status = QLabel("Ready")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

        self._speed_sens.valueChanged.connect(self._on_sens_changed)
        self._yaw_sens.valueChanged.connect(self._on_sens_changed)
        self._btn_stop.clicked.connect(self._emergency_stop)
        self._btn_heading_reset.clicked.connect(self._do_heading_reset)

        self._tx_timer = QTimer(self)
        self._tx_timer.setInterval(_TX_INTERVAL_MS)
        self._tx_timer.timeout.connect(self._flush_commands)
        self._tx_timer.start()

        self._on_sens_changed()
        self._update_cmd_label()
        threading.Thread(target=self._probe_robot, name="remote-probe", daemon=True).start()

    def _mps_to_motor_turns_s(self, v_mps: float) -> float:
        wheel_m_per_motor_turn = self._gear * 2.0 * math.pi * self._wheel_radius_m
        if wheel_m_per_motor_turn <= 1e-12:
            return 0.0
        return v_mps / wheel_m_per_motor_turn

    def _max_speed_mps(self) -> float:
        return self._speed_sens.value() / 1000.0

    def _max_yaw_rate_deg_s(self) -> float:
        return float(self._yaw_sens.value())

    def _on_sens_changed(self, *_args) -> None:
        self._speed_sens_label.setText(f"±{self._max_speed_mps():.2f} m/s")
        self._yaw_sens_label.setText(f"±{self._yaw_sens.value()}°/s")
        self._pending_send = True
        self._update_cmd_label()

    def _on_stick(self, nx: float, ny: float) -> None:
        self._nx = nx
        self._ny = ny
        self._pending_send = True
        self._update_cmd_label()

    def _vel_turns_s(self) -> float:
        return self._mps_to_motor_turns_s(self._ny * self._max_speed_mps())

    def _heading_nudge_rad(self) -> float:
        if abs(self._nx) < _STICK_DEADZONE:
            return 0.0
        delta_deg = (self._max_yaw_rate_deg_s() * self._nx) / float(_TX_HZ)
        return math.radians(delta_deg)

    @staticmethod
    def _wrap_pi(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _update_cmd_label(self) -> None:
        turns_s = self._vel_turns_s()
        nudge = self._heading_nudge_rad()
        v_mps = self._ny * self._max_speed_mps()
        rate_deg_s = self._max_yaw_rate_deg_s() * self._nx
        self._cmd_label.setText(
            f"cmd  v={v_mps:+.2f} m/s  ({turns_s:+.2f} turn/s)   "
            f"yaw_rate={rate_deg_s:+.1f}°/s  "
            f"nudge={math.degrees(nudge):+.2f}°/{_TX_HZ}Hz  "
            f"ψ_ref={math.degrees(self._heading_ref_rad):+.1f}°"
        )

    def _probe_robot(self) -> None:
        try:
            with self._rpc_lock:
                snap = self._client.get_params()
            if snap.wheel_radius_m > 1e-6:
                self._wheel_radius_m = float(snap.wheel_radius_m)
            self._heading_ref_rad = float(snap.heading_ref_rad)
            self._last_heading = self._heading_ref_rad
            msg = (
                f"Connected  version={snap.version}  strategy={snap.strategy_id}  "
                f"ψ_ref={math.degrees(self._heading_ref_rad):+.1f}°"
            )
            QTimer.singleShot(0, self._update_cmd_label)
        except Exception as exc:
            msg = f"GET failed (teleop still sends): {exc}"
        QTimer.singleShot(0, lambda: self._status.setText(msg))

    def _flush_commands(self) -> None:
        nudge = self._heading_nudge_rad()
        if abs(nudge) > 1e-9:
            self._heading_ref_rad = self._wrap_pi(self._heading_ref_rad + nudge)
            self._pending_send = True
            self._update_cmd_label()

        # Always drain RX even when idle — leftover stream floods kill the socket.
        try:
            with self._rpc_lock:
                self._client.drain_rx(128)
        except Exception:
            pass

        if not self._pending_send:
            return

        turns_s = self._vel_turns_s()
        heading = self._heading_ref_rad
        vel_changed = self._last_vel is None or abs(turns_s - self._last_vel) > 1e-4
        heading_changed = self._last_heading is None or abs(heading - self._last_heading) > 1e-5
        if not vel_changed and not heading_changed and abs(self._nx) < 1e-6 and abs(self._ny) < 1e-6:
            vel_changed = self._last_vel is None or abs(self._last_vel) > 1e-4

        if not vel_changed and not heading_changed:
            self._pending_send = False
            return

        self._pending_send = False
        try:
            with self._rpc_lock:
                if vel_changed:
                    self._client.set_param_noreply("vel_ref_turns_s", turns_s)
                    self._last_vel = turns_s
                if heading_changed:
                    self._client.set_param_noreply("heading_ref_rad", heading)
                    self._last_heading = heading
            self._status.setText("TX ok")
        except Exception as exc:
            self._status.setText(f"TX failed: {exc}")
            self._pending_send = True

    def _emergency_stop(self) -> None:
        self._pad.zero()
        self._pending_send = False
        try:
            with self._rpc_lock:
                self._client.set_param_noreply("vel_ref_turns_s", 0.0)
            self._last_vel = 0.0
            self._status.setText("STOP sent (v=0)")
        except Exception as exc:
            self._status.setText(f"STOP failed: {exc}")
            self._pending_send = True

    def _do_heading_reset(self) -> None:
        try:
            with self._rpc_lock:
                self._client.set_param_noreply("heading_reset", 1.0)
                self._client.set_param_noreply("heading_ref_rad", 0.0)
            self._heading_ref_rad = 0.0
            self._last_heading = 0.0
            self._status.setText("heading_reset + ψ_ref=0")
            self._update_cmd_label()
        except Exception as exc:
            self._status.setText(f"heading_reset failed: {exc}")

    def closeEvent(self, event) -> None:
        try:
            with self._rpc_lock:
                self._client.set_param_noreply("vel_ref_turns_s", 0.0)
        except Exception:
            pass
        super().closeEvent(event)


def run_remote_control(client: ControlParamsClient) -> int:
    app = QApplication.instance()
    owns = app is None
    if owns:
        app = QApplication([])
    win = RemoteControlWindow(client)
    win.show()
    code = app.exec_() if hasattr(app, "exec_") else app.exec()
    return int(code)
