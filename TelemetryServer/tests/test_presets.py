import json
import os
import tempfile
import unittest
from pathlib import Path

from telemetry.gains_catalog import PANELS
from telemetry.presets import (
    flatten_groups,
    group_values,
    list_presets,
    load_preset,
    save_preset,
    sanitize_preset_name,
)
from telemetry.web_server import handle_control_message


class PresetTests(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self._old_cwd = os.getcwd()
        os.chdir(self._tmp.name)

    def tearDown(self) -> None:
        os.chdir(self._old_cwd)
        self._tmp.cleanup()

    def test_sanitize_rejects_paths(self) -> None:
        with self.assertRaises(ValueError):
            sanitize_preset_name("../secret")
        with self.assertRaises(ValueError):
            sanitize_preset_name("a/b")

    def test_group_values_follows_ui_panels(self) -> None:
        values = {
            "meca_k_grav": 0.14,
            "heading_kp": 0.08,
            "heading_d_ema": 0.96,
            "antipat_enable": 0.0,
            "vel_ref_turns_s": 1.0,
        }
        grouped = group_values(values)
        self.assertEqual(list(grouped), ["Équilibre", "Heading", "Antipatinage"])
        self.assertEqual(grouped["Équilibre"]["meca_k_grav"], 0.14)
        self.assertNotIn("vel_ref_turns_s", flatten_groups(grouped))

    def test_save_and_load_roundtrip(self) -> None:
        values = {"meca_k_grav": 0.14, "heading_kp": 0.08}
        name = save_preset("baseline heading", values)
        self.assertEqual(name, "baseline heading")
        self.assertEqual(list_presets(), ("baseline heading",))
        path = Path.cwd() / "presets" / "baseline heading.json"
        raw = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(list(raw), ["Équilibre", "Heading"])
        self.assertEqual(raw["Équilibre"]["meca_k_grav"], 0.14)
        loaded_name, groups, flat = load_preset("baseline heading")
        self.assertEqual(loaded_name, "baseline heading")
        self.assertEqual(groups["Heading"]["heading_kp"], 0.08)
        self.assertEqual(flat["heading_kp"], 0.08)

    def test_save_refuses_overwrite_unless_asked(self) -> None:
        save_preset("keep", {"meca_k_grav": 1.0})
        with self.assertRaises(FileExistsError):
            save_preset("keep", {"meca_k_grav": 2.0})
        save_preset("keep", {"meca_k_grav": 2.0}, overwrite=True)
        _name, _groups, flat = load_preset("keep")
        self.assertEqual(flat["meca_k_grav"], 2.0)

    def test_control_actions_work_without_rpc(self) -> None:
        listed = handle_control_message(None, {"action": "list_presets"})
        self.assertEqual(listed, {"ok": True, "presets": []})
        saved = handle_control_message(
            None,
            {
                "action": "save_preset",
                "name": "web",
                "values": {"heading_kp": 0.08, "heading_d_ema": 0.96},
            },
        )
        self.assertTrue(saved["ok"])
        loaded = handle_control_message(None, {"action": "load_preset", "name": "web"})
        self.assertTrue(loaded["ok"])
        self.assertEqual(loaded["params"]["heading_kp"], 0.08)
        self.assertIn("Heading", loaded["groups"])
        titles = [panel["title"] for panel in PANELS]
        self.assertTrue(set(loaded["groups"]).issubset(set(titles) | {"Autres"}))
