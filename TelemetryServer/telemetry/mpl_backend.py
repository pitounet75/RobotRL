"""Select a matplotlib GUI backend that works when system Tcl/Tk is broken."""

from __future__ import annotations

import os


def configure_matplotlib() -> str:
    """Return backend name. Prefer Qt5Agg over TkAgg on Windows."""
    import matplotlib

    if os.environ.get("MPLBACKEND"):
        return matplotlib.get_backend()

    for backend in ("Qt5Agg", "TkAgg", "Agg"):
        try:
            matplotlib.use(backend, force=True)
            return backend
        except Exception:
            continue

    matplotlib.use("Agg", force=True)
    return "Agg"
