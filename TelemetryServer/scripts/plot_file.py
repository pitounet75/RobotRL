#!/usr/bin/env python3
"""Offline plot from CSV recorded by run_server.py."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from telemetry.mpl_backend import configure_matplotlib

configure_matplotlib()

import matplotlib.pyplot as plt
import pandas as pd


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("csv", type=Path)
    p.add_argument("--channels", default="pitch_rad,cmd_torque_nm,u_ff_nm,u_fb_nm")
    args = p.parse_args()

    df = pd.read_csv(args.csv)
    t = df["host_time_s"] - df["host_time_s"].iloc[0]
    channels = [c.strip() for c in args.channels.split(",") if c.strip()]

    fig, axes = plt.subplots(len(channels), 1, figsize=(10, 2.2 * len(channels)), sharex=True)
    if len(channels) == 1:
        axes = [axes]

    for ax, ch in zip(axes, channels):
        ax.plot(t, df[ch], label=ch)
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("time [s]")
    fig.suptitle(args.csv.name)
    plt.tight_layout()
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
