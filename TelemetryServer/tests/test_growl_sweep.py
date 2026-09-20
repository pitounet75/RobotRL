import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from growl_sweep import peak_in_band  # noqa: E402


FS = 500.0
T = np.arange(7000) / FS


class PeakInBandTest(unittest.TestCase):
    """The sweep compares amplitudes across runs, so the scale must be right.

    A relative measure would be enough to rank gains, but the amplitude is
    also read as 'how much acceleration does the growl inject' (A*2*pi*f) and
    compared against the 14 turn/s^2 free-wheel signature. That comparison is
    only meaningful if the number is calibrated.
    """

    def test_pure_sine_reads_its_own_amplitude(self) -> None:
        f, a = peak_in_band(0.66 * np.sin(2 * np.pi * 42 * T), FS)
        self.assertAlmostEqual(f, 42.0, delta=0.5)
        self.assertAlmostEqual(a, 0.66, delta=0.02)

    def test_broad_noise_does_not_look_like_a_peak(self) -> None:
        noise = 0.1 * np.random.RandomState(0).randn(len(T))
        _f, a = peak_in_band(noise, FS)
        # Noise of std 0.1 spread over the band must not be mistaken for an
        # oscillation: that would make every sweep row look identical.
        self.assertLess(a, 0.05)

    def test_wandering_frequency_keeps_its_amplitude(self) -> None:
        """The real growl drifts; reading one bin would undercount it 4x."""
        x = 0.66 * np.sin(2 * np.pi * 42 * T + 2 * np.sin(2 * np.pi * 0.7 * T))
        _f, a = peak_in_band(x, FS)
        self.assertAlmostEqual(a, 0.66, delta=0.05)

    def test_amplitude_scales_linearly(self) -> None:
        _f, half = peak_in_band(0.33 * np.sin(2 * np.pi * 42 * T), FS)
        _f, full = peak_in_band(0.66 * np.sin(2 * np.pi * 42 * T), FS)
        self.assertAlmostEqual(full / half, 2.0, delta=0.05)

    def test_short_capture_is_refused_not_guessed(self) -> None:
        _f, a = peak_in_band(np.sin(2 * np.pi * 42 * np.arange(100) / FS), FS)
        self.assertTrue(np.isnan(a))


if __name__ == "__main__":
    unittest.main()
