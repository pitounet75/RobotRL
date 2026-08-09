"""Select a matplotlib GUI backend that works when system Tcl/Tk is broken."""

from __future__ import annotations

import os


def configure_matplotlib() -> str:
    """Return backend name. Prefer Qt over TkAgg on Windows. Never silent-Agg for live."""
    import matplotlib

    if os.environ.get("MPLBACKEND"):
        return matplotlib.get_backend()

    for backend in ("QtAgg", "Qt5Agg", "TkAgg"):
        try:
            matplotlib.use(backend, force=True)
            # Force import of the canvas; use() alone can succeed then fail later.
            import matplotlib.pyplot as plt  # noqa: F401

            fig = plt.figure()
            plt.close(fig)
            return backend
        except Exception:
            continue

    matplotlib.use("Agg", force=True)
    return "Agg"


def backend_is_interactive() -> bool:
    import matplotlib

    name = matplotlib.get_backend().lower()
    return name not in ("agg", "pdf", "svg", "ps", "cairo", "template")
