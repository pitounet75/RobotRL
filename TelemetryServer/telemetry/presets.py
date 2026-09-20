"""Gain presets: JSON files in ./presets relative to the process cwd."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Dict, Mapping, Sequence, Tuple

from telemetry.ctrl_params import PARAM_NAMES
from telemetry.gains_catalog import HIDDEN_FROM_GAINS, PANELS, panel_field_names

PRESETS_DIRNAME = "presets"
_NAME_OK = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 ._+\-]{0,79}$")


def presets_dir() -> Path:
    """Directory next to wherever the Python app was launched."""
    return Path.cwd() / PRESETS_DIRNAME


def sanitize_preset_name(name: str) -> str:
    cleaned = " ".join(str(name).strip().split())
    if not _NAME_OK.match(cleaned):
        raise ValueError(
            "preset name must start with a letter or digit and use only "
            "letters, digits, spaces, '.', '_', '+', '-' (max 80 chars)"
        )
    if cleaned in {".", ".."}:
        raise ValueError("invalid preset name")
    return cleaned


def list_presets() -> Sequence[str]:
    folder = presets_dir()
    if not folder.is_dir():
        return ()
    names = [path.stem for path in folder.glob("*.json") if path.is_file()]
    return tuple(sorted(names, key=str.casefold))


def group_values(values: Mapping[str, object]) -> Dict[str, Dict[str, float]]:
    """Pack a flat name→value map into the same groups as the Gains UI."""
    grouped: Dict[str, Dict[str, float]] = {}
    used = set()
    for panel in PANELS:
        group: Dict[str, float] = {}
        for name in panel_field_names(panel):
            if name not in values:
                continue
            group[name] = _as_float(name, values[name])
            used.add(name)
        if group:
            grouped[panel["title"]] = group
    extras: Dict[str, float] = {}
    for name, raw in values.items():
        if name in used or name in HIDDEN_FROM_GAINS or name not in PARAM_NAMES:
            continue
        extras[name] = _as_float(name, raw)
    if extras:
        grouped["Autres"] = extras
    return grouped


def flatten_groups(groups: Mapping[str, object]) -> Dict[str, float]:
    """Flatten a UI-grouped preset back to name→value."""
    flat: Dict[str, float] = {}
    if not isinstance(groups, Mapping):
        raise ValueError("preset root must be a JSON object")
    for title, group in groups.items():
        if not isinstance(title, str) or not isinstance(group, Mapping):
            raise ValueError(f"preset group {title!r} must be an object")
        for name, raw in group.items():
            if name not in PARAM_NAMES or name in HIDDEN_FROM_GAINS:
                continue
            flat[str(name)] = _as_float(str(name), raw)
    return flat


def save_preset(
    name: str, values: Mapping[str, object], *, overwrite: bool = False
) -> str:
    cleaned = sanitize_preset_name(name)
    grouped = group_values(values)
    if not grouped:
        raise ValueError("nothing to save")
    folder = presets_dir()
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"{cleaned}.json"
    if path.exists() and not overwrite:
        raise FileExistsError(f"preset {cleaned!r} already exists")
    path.write_text(
        json.dumps(grouped, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return cleaned


def load_preset(name: str) -> Tuple[str, Dict[str, Dict[str, float]], Dict[str, float]]:
    cleaned = sanitize_preset_name(name)
    path = presets_dir() / f"{cleaned}.json"
    if not path.is_file():
        raise FileNotFoundError(f"preset {cleaned!r} not found")
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"preset {cleaned!r} is not valid JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError(f"preset {cleaned!r} root must be a JSON object")
    flat = flatten_groups(raw)
    if not flat:
        raise ValueError(f"preset {cleaned!r} has no gain fields")
    grouped = {
        title: {k: float(v) for k, v in group.items()}
        for title, group in raw.items()
        if isinstance(group, Mapping)
    }
    return cleaned, grouped, flat


def _as_float(name: str, raw: object) -> float:
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise ValueError(f"{name} must be a number")
    return float(raw)
