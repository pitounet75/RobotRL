"""Qt gains editor panel (GET/SET via SharedRpcClient)."""

from __future__ import annotations

from typing import Dict, Optional

from telemetry.ctrl_params import PARAM_NAMES
from telemetry.rpc_mux import SharedRpcClient

try:
    from PyQt5.QtCore import Qt
    from PyQt5.QtWidgets import (
        QFormLayout,
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
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QPushButton,
        QScrollArea,
        QVBoxLayout,
        QWidget,
    )

# One-shot / action params: button instead of free edit apply-all.
_ACTION_PARAMS = frozenset({"pos_reset", "heading_reset", "heading_inc", "heading_dec"})


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
        form_host = QWidget()
        form = QFormLayout(form_host)
        form.setLabelAlignment(Qt.AlignRight)
        # Stable order by param id.
        for name in sorted(PARAM_NAMES.keys(), key=lambda n: PARAM_NAMES[n]):
            if name in _ACTION_PARAMS:
                continue
            edit = QLineEdit()
            edit.setPlaceholderText(name)
            self._edits[name] = edit
            form.addRow(name, edit)
        scroll.setWidget(form_host)
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
                edit.setText(text)
                self._loaded[name] = text
            self._set_status(
                f"Loaded snapshot version={snap.version}  strategy={snap.strategy_id}"
            )
        except Exception as exc:
            self._set_status(f"Refresh failed: {exc}", error=True)

    def apply(self, changed_only: bool) -> None:
        if self._rpc is None:
            return
        updates: list[tuple[str, float]] = []
        for name, edit in self._edits.items():
            text = edit.text().strip()
            if not text:
                continue
            if changed_only and self._loaded.get(name) == text:
                continue
            try:
                value = float(text)
            except ValueError:
                self._set_status(f"Invalid number for {name!r}: {text!r}", error=True)
                return
            updates.append((name, value))

        if not updates:
            self._set_status("Nothing to apply.")
            return

        ok = 0
        try:
            for name, value in updates:
                _id, _n, applied = self._rpc.set_param(name, value)
                self._edits[name].setText(f"{applied:.6g}")
                self._loaded[name] = f"{applied:.6g}"
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
