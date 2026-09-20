import unittest

from telemetry.drive_stick import analog_step, hold_dir


class DriveStickTests(unittest.TestCase):
    def test_hold_dir_cancels_opposites(self) -> None:
        self.assertEqual(hold_dir(True, False), 1)
        self.assertEqual(hold_dir(False, True), -1)
        self.assertEqual(hold_dir(True, True), 0)
        self.assertEqual(hold_dir(False, False), 0)

    def test_hold_ramps_toward_max(self) -> None:
        value = 0.0
        for _ in range(10):
            value = analog_step(value, 1, 0.5, 0.07)
        self.assertGreater(value, 0.2)
        self.assertLessEqual(value, 0.5)
        for _ in range(20):
            value = analog_step(value, 1, 0.5, 0.07)
        self.assertAlmostEqual(value, 0.5, places=6)

    def test_release_returns_to_zero_faster_than_accel(self) -> None:
        value = 0.5
        for _ in range(4):
            value = analog_step(value, 0, 0.5, 0.07)
        self.assertLess(value, 0.2)
        for _ in range(10):
            value = analog_step(value, 0, 0.5, 0.07)
        self.assertEqual(value, 0.0)
