import sys
import unittest
from contextlib import redirect_stderr
from io import StringIO
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts import run_server


class RunServerCliTests(unittest.TestCase):
    def test_plot_and_web_are_mutually_exclusive(self) -> None:
        with redirect_stderr(StringIO()):
            with self.assertRaises(SystemExit):
                run_server.main(["--plot", "--web"])


if __name__ == "__main__":
    unittest.main()
