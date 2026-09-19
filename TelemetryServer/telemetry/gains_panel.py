"""Qt gains editor: simple line edits, grouped by control category."""

from __future__ import annotations

from typing import Dict, Optional, Sequence, Tuple

from telemetry.ctrl_params import PARAM_NAMES
from telemetry.rpc_mux import SharedRpcClient

try:
    from PyQt5.QtCore import Qt
    from PyQt5.QtWidgets import (
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QPushButton,
        QScrollArea,
        QVBoxLayout,
        QWidget,
    )
except ImportError:  # pragma: no cover
    from PySide2.QtCore import Qt
    from PySide2.QtWidgets import (
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QPushButton,
        QScrollArea,
        QVBoxLayout,
        QWidget,
    )

# One-shot / aliases: buttons or hidden (same store as heading_ref_rad).
_ACTION_PARAMS = frozenset({"pos_reset", "heading_reset", "heading_dec"})
_HIDDEN_ALIASES = frozenset({"heading_inc"})

# Proposed categories for ff_cascade (name stays the SET key).
CATEGORIES: Sequence[Tuple[str, Sequence[str]]] = (
    (
        "Consignes",
        (
            "pitch_ref_rad",
            "vel_ref_turns_s",
            "heading_ref_rad",
            "pos_x_ref_m",
            "outer_mode",
        ),
    ),
    (
        "Limites",
        (
            "pitch_failsafe_rad",
            "cmd_max_torque_nm",
            "cascade_pitch_ref_max_rad",
            "vel_ref_slew_turns_s2",
        ),
    ),
    (
        "Cascade vitesse / pitch",
        (
            "cascade_vel_kp",
            "cascade_vel_kd",
            "cascade_vel_err_ema_alpha",
            "cascade_vel_ema_kp",
            "cascade_vel_accel_kp",
        ),
    ),
    (
        "Équilibre (FF + PD)",
        (
            "ff_grav_k",
            "ff_fb_k_pitch",
            "ff_fb_k_rate",
            "ff_output_alpha",
        ),
    ),
    (
        "Lacet",
        (
            "heading_kp",
            "heading_kd",
            "heading_torque_max_nm",
        ),
    ),
    (
        "Position",
        (
            "pos_kp",
            "pos_kd",
            "pos_v_max_turns_s",
            "pos_err_ema_alpha",
            "pos_ema_kp",
            "wheel_radius_m",
        ),
    ),
    (
        "Friction / deadband",
        (
            "friction_mode",
            "friction_static_nm",
            "friction_kinetic_nm",
            "friction_vel_eps_turns_s",
            "torque_deadband_nm",
            "torque_deadband_pitch_max_rad",
            "torque_deadband_rate_max_rads",
        ),
    ),
    (
        "Accélération moteur",
        (
            "alpha_kp",
            "alpha_max_nm",
            "motor_J",
            "motor_friction_c",
            "alpha_pitch_max_rad",
            "alpha_rate_max_rads",
            "alpha_vel_max_turns_s",
            "alpha_lpf",
        ),
    ),
    (
        "Système",
        (
            "strategy",
            "wheel_encoder_vel_lpf_alpha",
        ),
    ),
)

_DIRTY_SS = "QLineEdit { background: #fff3cd; }"
_CLEAN_SS = ""


class GainsPanel(QWidget):
    def __init__(
        self,
        rpc: Optional[SharedRpcClient],
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._rpc = rpc
        self._edits: Dict[str, QLineEdit] = {}
        self._loaded: Dict[str, str] = {}
        self._status = QLabel("")
        self._status.setWordWrap(True)

        root = QVBoxLayout(self)
        btns = QHBoxLayout()
        self._btn_refresh = QPushButton("Refresh from robot")
        self._btn_apply = QPushButton("Apply changed")
        self._btn_apply_all = QPushButton("Apply all")
        self._btn_reset_x = QPushButton("pos_reset")
        self._btn_reset_heading = QPushButton("heading_reset")
        btns.addWidget(self._btn_refresh)
        btns.addWidget(self._btn_apply)
        btns.addWidget(self._btn_apply_all)
        btns.addWidget(self._btn_reset_x)
        btns.addWidget(self._btn_reset_heading)
        btns.addStretch(1)
        root.addLayout(btns)
        root.addWidget(self._status)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        host = QWidget()
        grid = QGridLayout(host)
        grid.setContentsMargins(8, 8, 8, 8)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)

        used = set()
        row = 0
        col = 0
        for title, keys in CATEGORIES:
            visible = [n for n in keys if n in PARAM_NAMES]
            if not visible:
                continue
            used.update(visible)
            box = self._make_group(title, visible)
            grid.addWidget(box, row, col)
            col += 1
            if col >= 2:
                col = 0
                row += 1

        extras = [
            n
            for n in sorted(PARAM_NAMES.keys(), key=lambda k: PARAM_NAMES[k])
            if n not in used and n not in _ACTION_PARAMS and n not in _HIDDEN_ALIASES
        ]
        if extras:
            if col != 0:
                col = 0
                row += 1
            grid.addWidget(self._make_group("Autres", extras), row, 0, 1, 2)

        scroll.setWidget(host)
        root.addWidget(scroll)

        self._btn_refresh.clicked.connect(self.refresh)
        self._btn_apply.clicked.connect(lambda: self.apply(changed_only=True))
        self._btn_apply_all.clicked.connect(lambda: self.apply(changed_only=False))
        self._btn_reset_x.clicked.connect(self._on_pos_reset)
        self._btn_reset_heading.clicked.connect(self._on_heading_reset)

        if rpc is None:
            self._set_status("RPC unavailable (pass --esp32-host).")
            self._btn_refresh.setEnabled(False)
            self._btn_apply.setEnabled(False)
            self._btn_apply_all.setEnabled(False)
            self._btn_reset_x.setEnabled(False)
            self._btn_reset_heading.setEnabled(False)

    def _make_group(self, title: str, names: Sequence[str]) -> QGroupBox:
        box = QGroupBox(title)
        form = QFormLayout(box)
        form.setLabelAlignment(Qt.AlignRight)
        form.setHorizontalSpacing(8)
        form.setVerticalSpacing(4)
        for name in names:
            edit = QLineEdit()
            edit.setPlaceholderText(name)
            edit.setClearButtonEnabled(True)
            edit.textChanged.connect(lambda _t, n=name: self._paint_dirty(n))
            edit.returnPressed.connect(lambda n=name: self._apply_one(n))
            self._edits[name] = edit
            form.addRow(name, edit)
        return box

    def _paint_dirty(self, name: str) -> None:
        edit = self._edits.get(name)
        if edit is None:
            return
        loaded = self._loaded.get(name)
        text = edit.text().strip()
        edit.setStyleSheet(_DIRTY_SS if loaded is not None and text != loaded else _CLEAN_SS)

    def _set_status(self, text: str, error: bool = False) -> None:
        self._status.setStyleSheet("color: #b00020;" if error else "")
        self._status.setText(text)

    def refresh(self) -> None:
        if self._rpc is None:
            return
        try:
            snap = self._rpc.get_params()
            data = snap.as_dict()
            for name, edit in self._edits.items():
                val = data.get(name)
                text = f"{val:.6g}" if isinstance(val, float) else str(val)
                edit.blockSignals(True)
                edit.setText(text)
                edit.blockSignals(False)
                self._loaded[name] = text
                edit.setStyleSheet(_CLEAN_SS)
            self._set_status(
                f"Loaded snapshot version={snap.version}  strategy={snap.strategy_id}"
            )
        except Exception as exc:
            self._set_status(f"Refresh failed: {exc}", error=True)

    def _collect_updates(self, changed_only: bool, only: Optional[str] = None) -> Optional[list[tuple[str, float]]]:
        updates: list[tuple[str, float]] = []
        names = (only,) if only is not None else tuple(self._edits.keys())
        for name in names:
            edit = self._edits[name]
            text = edit.text().strip()
            if not text:
                continue
            if changed_only and self._loaded.get(name) == text:
                continue
            try:
                value = float(text)
            except ValueError:
                self._set_status(f"Invalid number for {name!r}: {text!r}", error=True)
                return None
            updates.append((name, value))
        return updates

    def apply(self, changed_only: bool) -> None:
        self._send_updates(self._collect_updates(changed_only))

    def _apply_one(self, name: str) -> None:
        self._send_updates(self._collect_updates(changed_only=True, only=name))

    def _send_updates(self, updates: Optional[list[tuple[str, float]]]) -> None:
        if self._rpc is None or updates is None:
            return
        if not updates:
            self._set_status("Nothing to apply.")
            return

        ok = 0
        try:
            for name, value in updates:
                _id, _n, applied = self._rpc.set_param(name, value)
                text = f"{applied:.6g}"
                edit = self._edits[name]
                edit.blockSignals(True)
                edit.setText(text)
                edit.blockSignals(False)
                self._loaded[name] = text
                edit.setStyleSheet(_CLEAN_SS)
                ok += 1
            self._set_status(f"Applied {ok} param(s).")
        except Exception as exc:
            self._set_status(f"Apply stopped after {ok}: {exc}", error=True)

    def _on_pos_reset(self) -> None:
        if self._rpc is None:
            return
        try:
            self._rpc.set_param("pos_reset", 1.0)
            self._set_status("pos_reset pulsed.")
        except Exception as exc:
            self._set_status(f"pos_reset failed: {exc}", error=True)

    def _on_heading_reset(self) -> None:
        if self._rpc is None:
            return
        try:
            self._rpc.set_param("heading_reset", 1.0)
            self._set_status("heading_reset pulsed.")
        except Exception as exc:
            self._set_status(f"heading_reset failed: {exc}", error=True)
