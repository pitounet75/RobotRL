import unittest

from telemetry.ctrl_params import PARAM_NAMES, SNAPSHOT_STRUCT, SNAPSHOT_VERSION, ControlParamsSnapshot
from telemetry.gains_catalog import HIDDEN_FROM_GAINS, PANELS, TOOLTIPS, panel_field_names


class GainsCatalogTests(unittest.TestCase):
    def test_screen_order(self) -> None:
        self.assertEqual(
            [panel["title"] for panel in PANELS],
            [
                "Équilibre",
                "Friction / deadband / motor correction",
                "Vitesse",
                "Heading",
                "Antipatinage",
                "Position",
                "Système",
            ],
        )

    def test_every_listed_field_exists(self) -> None:
        for panel in PANELS:
            for name in panel_field_names(panel):
                self.assertIn(name, PARAM_NAMES)
                self.assertIn(name, TOOLTIPS)
                self.assertNotIn(name, HIDDEN_FROM_GAINS)

    def test_no_duplicate_fields(self) -> None:
        seen = []
        for panel in PANELS:
            seen.extend(panel_field_names(panel))
        self.assertEqual(len(seen), len(set(seen)))

    def test_catalog_covers_all_visible_params(self) -> None:
        listed = {name for panel in PANELS for name in panel_field_names(panel)}
        visible = set(PARAM_NAMES) - HIDDEN_FROM_GAINS
        self.assertEqual(listed, visible)

    def test_snapshot_v19_new_names(self) -> None:
        self.assertEqual(SNAPSHOT_VERSION, 19)
        heading = next(p for p in PANELS if p["title"] == "Heading")
        antipat = next(p for p in PANELS if p["title"] == "Antipatinage")
        self.assertIn("heading_d_ema", heading["fields"])
        self.assertIn("antipat_enable", panel_field_names(antipat))
        self.assertIn("antipat_tau_ema", panel_field_names(antipat))
        self.assertIn("antipat_u_fade_ms", antipat["fields"])
        self.assertIn("antipat_sync_kd", panel_field_names(antipat))
        self.assertIn("antipat_both_enable", panel_field_names(antipat))
        section_titles = [s["title"] for s in antipat.get("sections") or ()]
        self.assertEqual(section_titles, ["Sync", "Both"])
        sync = next(s for s in antipat["sections"] if s["title"] == "Sync")
        both = next(s for s in antipat["sections"] if s["title"] == "Both")
        for name in antipat["fields"]:
            self.assertNotIn("_sync_", name)
            self.assertNotIn("_both_", name)
        for name in sync["fields"]:
            self.assertTrue(name.startswith("antipat_sync_"), name)
        for name in both["fields"]:
            self.assertTrue(name.startswith("antipat_both_"), name)
        self.assertEqual(PARAM_NAMES["heading_d_ema"], 50)
        self.assertEqual(PARAM_NAMES["antipat_enable"], 51)
        self.assertEqual(PARAM_NAMES["antipat_both_tau_max_nm"], 75)
        self.assertEqual(PARAM_NAMES["antipat_both_enable"], 76)
        self.assertEqual(PARAM_NAMES["antipat_tau_ema"], 77)
        self.assertEqual(PARAM_NAMES["antipat_u_fade_ms"], 78)
        self.assertEqual(PARAM_NAMES["antipat_sync_kd"], 79)
        self.assertEqual(SNAPSHOT_STRUCT.size, 8 + 79 * 4)
        self.assertEqual(len(ControlParamsSnapshot.__dataclass_fields__), 81)

    def test_decode_snapshot_matches_dataclass(self) -> None:
        from telemetry.ctrl_params import decode_snapshot

        n_unpack = SNAPSHOT_STRUCT.size // 4
        n_fields = len(ControlParamsSnapshot.__dataclass_fields__)
        self.assertEqual(n_unpack, n_fields)
        snap = decode_snapshot(b"\x00" * SNAPSHOT_STRUCT.size)
        self.assertEqual(snap.version, 0)
        self.assertEqual(snap.antipat_sync_kd, 0.0)
        data = snap.as_dict()
        self.assertEqual(set(data.keys()), set(PARAM_NAMES))

    def test_param_names_fit_telemetry_field_name_limit(self) -> None:
        # Keep in sync with TELEMETRY_MAX_FIELD_NAME_LEN in telemetry.h.
        # A longer name silently drops GetControlParams (STM32 error 4).
        max_len = 48
        too_long = [name for name in PARAM_NAMES if len(name) >= max_len]
        self.assertEqual(too_long, [])
