import unittest

from telemetry.vbus_alert import vbus_alert_level


class VbusAlertTests(unittest.TestCase):
    def test_missing_is_off(self) -> None:
        self.assertEqual(vbus_alert_level(0.0, 0.0), "off")
        self.assertEqual(vbus_alert_level(0.4, 0.3), "off")

    def test_nominal_is_ok(self) -> None:
        self.assertEqual(vbus_alert_level(24.0, 23.8), "ok")

    def test_warn_below_11_4(self) -> None:
        self.assertEqual(vbus_alert_level(11.3, 24.0), "warn")
        self.assertEqual(vbus_alert_level(11.4, 11.4), "ok")

    def test_crit_below_11(self) -> None:
        self.assertEqual(vbus_alert_level(10.9, 24.0), "crit")
        self.assertEqual(vbus_alert_level(10.0, 10.5), "crit")

    def test_one_missing_uses_the_other(self) -> None:
        self.assertEqual(vbus_alert_level(0.0, 11.2), "warn")
        self.assertEqual(vbus_alert_level(10.0, 0.0), "crit")


if __name__ == "__main__":
    unittest.main()
