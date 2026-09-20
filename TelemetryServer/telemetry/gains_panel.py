"""Qt gains editor: simple line edits, grouped by control category."""

from __future__ import annotations

from typing import Dict, Optional, Sequence

from telemetry.ctrl_params import PARAM_NAMES
from telemetry.gains_catalog import HIDDEN_FROM_GAINS, PANELS, TOOLTIPS, panel_field_names
from telemetry.presets import list_presets, load_preset, save_preset
from telemetry.rpc_mux import SharedRpcClient

try:
    from PyQt5.QtCore import Qt
    from PyQt5.QtWidgets import (
        QDialog,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QListWidget,
        QPushButton,
        QScrollArea,
        QVBoxLayout,
        QWidget,
    )
except ImportError:  # pragma: no cover
    from PySide2.QtCore import Qt
    from PySide2.QtWidgets import (
        QDialog,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QListWidget,
        QPushButton,
        QScrollArea,
        QVBoxLayout,
        QWidget,
    )

# Buttons / aliases / Drive sliders stay out of the gain groups.
_ACTION_PARAMS = frozenset({"pos_reset", "heading_reset", "heading_dec"})
_HIDDEN_ALIASES = frozenset({"heading_inc"}) | HIDDEN_FROM_GAINS

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
        self._btn_save = QPushButton("Save")
        self._btn_load = QPushButton("Load")
        btns.addWidget(self._btn_refresh)
        btns.addWidget(self._btn_apply)
        btns.addWidget(self._btn_apply_all)
        btns.addWidget(self._btn_reset_x)
        btns.addWidget(self._btn_reset_heading)
        btns.addStretch(1)
        btns.addWidget(self._btn_save)
        btns.addWidget(self._btn_load)
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
        index = 0
        for panel in PANELS:
            visible = [n for n in panel_field_names(panel) if n in PARAM_NAMES]
            if not visible:
                continue
            used.update(visible)
            common = [n for n in (panel.get("fields") or ()) if n in PARAM_NAMES]
            grid.addWidget(
                self._make_group(
                    panel["title"],
                    common,
                    panel["legend"],
                    panel.get("sections") or (),
                ),
                index // 2,
                index % 2,
            )
            index += 1

        extras = [
            n
            for n in sorted(PARAM_NAMES.keys(), key=lambda k: PARAM_NAMES[k])
            if n not in used and n not in _ACTION_PARAMS and n not in _HIDDEN_ALIASES
        ]
        if extras:
            grid.addWidget(self._make_group("Autres", extras, ""), index // 2, index % 2)
        grid.setRowStretch(grid.rowCount(), 1)

        scroll.setWidget(host)
        root.addWidget(scroll)

        self._btn_refresh.clicked.connect(self.refresh)
        self._btn_apply.clicked.connect(lambda: self.apply(changed_only=True))
        self._btn_apply_all.clicked.connect(lambda: self.apply(changed_only=False))
        self._btn_reset_x.clicked.connect(self._on_pos_reset)
        self._btn_reset_heading.clicked.connect(self._on_heading_reset)
        self._btn_save.clicked.connect(lambda: self._open_preset_dialog("save"))
        self._btn_load.clicked.connect(lambda: self._open_preset_dialog("load"))

        if rpc is None:
            self._set_status("RPC unavailable (pass --esp32-host).")
            self._btn_refresh.setEnabled(False)
            self._btn_apply.setEnabled(False)
            self._btn_apply_all.setEnabled(False)
            self._btn_reset_x.setEnabled(False)
            self._btn_reset_heading.setEnabled(False)

    def _add_field_grid(self, layout: QVBoxLayout, names: Sequence[str]) -> None:
        fields = QGridLayout()
        fields.setHorizontalSpacing(12)
        fields.setVerticalSpacing(4)
        for i, name in enumerate(names):
            edit = QLineEdit()
            edit.setPlaceholderText(name)
            edit.setClearButtonEnabled(True)
            edit.setMaximumWidth(110)
            edit.textChanged.connect(lambda _t, n=name: self._paint_dirty(n))
            edit.returnPressed.connect(lambda n=name: self._apply_one(n))
            tip = TOOLTIPS.get(name, "")
            if tip:
                edit.setToolTip(tip)
            label = QLabel(name)
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            if tip:
                label.setToolTip(tip)
            self._edits[name] = edit
            row, col = divmod(i, 2)
            fields.addWidget(label, row, col * 2)
            fields.addWidget(edit, row, col * 2 + 1)
        fields.setColumnStretch(0, 1)
        fields.setColumnStretch(2, 1)
        layout.addLayout(fields)

    def _make_group(
        self,
        title: str,
        names: Sequence[str],
        legend: str = "",
        sections: Sequence[object] = (),
    ) -> QGroupBox:
        box = QGroupBox(title)
        layout = QVBoxLayout(box)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        if legend:
            legend_lbl = QLabel(legend)
            legend_lbl.setWordWrap(True)
            legend_lbl.setStyleSheet("color: #666666; font-size: 10px;")
            layout.addWidget(legend_lbl)
        if names:
            self._add_field_grid(layout, names)
        for section in sections:
            sec = section if isinstance(section, dict) else {}
            visible = [n for n in sec.get("fields", ()) if n in PARAM_NAMES]
            if not visible:
                continue
            layout.addWidget(
                self._make_group(str(sec.get("title", "")), visible, str(sec.get("legend", "")))
            )
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

    def apply(self, changed_only: bool) -> bool:
        return self._send_updates(self._collect_updates(changed_only))

    def _apply_one(self, name: str) -> None:
        self._send_updates(self._collect_updates(changed_only=True, only=name))

    def _send_updates(self, updates: Optional[list[tuple[str, float]]]) -> bool:
        if self._rpc is None or updates is None:
            return False
        if not updates:
            self._set_status("Nothing to apply.")
            return True

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
            return True
        except Exception as exc:
            self._set_status(f"Apply stopped after {ok}: {exc}", error=True)
            return False

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

    def _collect_form_values(self) -> Optional[Dict[str, float]]:
        values: Dict[str, float] = {}
        for name, edit in self._edits.items():
            text = edit.text().strip()
            if not text:
                continue
            try:
                values[name] = float(text)
            except ValueError:
                self._set_status(f"Invalid number for {name!r}: {text!r}", error=True)
                return None
        return values

    def _open_preset_dialog(self, mode: str) -> None:
        dialog = _PresetDialog(mode, list_presets(), self)
        exec_dialog = getattr(dialog, "exec_", dialog.exec)
        if exec_dialog() != QDialog.Accepted:
            return
        name, overwrite = dialog.result_choice()
        if not name:
            return
        if mode == "save":
            self._save_preset(name, overwrite)
        else:
            self._load_preset(name)

    def _save_preset(self, name: str, overwrite: bool) -> None:
        values = self._collect_form_values()
        if values is None:
            return
        if not values:
            self._set_status("Refresh gains before saving a preset.", error=True)
            return
        try:
            saved = save_preset(name, values, overwrite=overwrite)
            self._set_status(f"Saved preset {saved!r}.")
        except Exception as exc:
            self._set_status(f"Save failed: {exc}", error=True)

    def _load_preset(self, name: str) -> None:
        try:
            saved, _groups, params = load_preset(name)
            for key, value in params.items():
                edit = self._edits.get(key)
                if edit is None:
                    continue
                text = f"{value:.6g}"
                edit.blockSignals(True)
                edit.setText(text)
                edit.blockSignals(False)
                self._paint_dirty(key)
            if self.apply(changed_only=False):
                self._set_status(f"Loaded preset {saved!r}.")
        except Exception as exc:
            self._set_status(f"Load failed: {exc}", error=True)


class _PresetDialog(QDialog):
    def __init__(self, mode: str, names: Sequence[str], parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._mode = mode
        self._chosen = ""
        self._overwrite = False
        self.setWindowTitle("Save preset" if mode == "save" else "Load preset")
        self.setModal(True)
        layout = QVBoxLayout(self)
        self._list = QListWidget()
        self._list.addItems(list(names))
        if not names:
            empty = QLabel("No presets yet.")
            layout.addWidget(empty)
        layout.addWidget(self._list)
        if mode == "save":
            hint = QLabel("Click a name to overwrite, or create a new one.")
            hint.setWordWrap(True)
            layout.addWidget(hint)
            row = QHBoxLayout()
            self._name = QLineEdit()
            self._name.setPlaceholderText("New preset name")
            create = QPushButton("Create")
            create.clicked.connect(self._on_create)
            self._name.returnPressed.connect(self._on_create)
            row.addWidget(self._name)
            row.addWidget(create)
            layout.addLayout(row)
            self._list.itemClicked.connect(self._on_overwrite)
        else:
            hint = QLabel("Click a preset to load it.")
            hint.setWordWrap(True)
            layout.addWidget(hint)
            self._list.itemClicked.connect(self._on_load)
        close = QPushButton("Close")
        close.clicked.connect(self.reject)
        layout.addWidget(close)

    def result_choice(self) -> tuple[str, bool]:
        return self._chosen, self._overwrite

    def _on_create(self) -> None:
        self._chosen = self._name.text().strip()
        self._overwrite = False
        if self._chosen:
            self.accept()

    def _on_overwrite(self, item) -> None:
        self._chosen = item.text()
        self._overwrite = True
        self.accept()

    def _on_load(self, item) -> None:
        self._chosen = item.text()
        self._overwrite = False
        self.accept()
