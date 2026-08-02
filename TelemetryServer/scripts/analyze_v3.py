#!/usr/bin/env python3
"""Compare balance sessions and diagnose v3."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]


def load(name: str) -> pd.DataFrame:
    df = pd.read_csv(ROOT / "logs" / name)
    df["t"] = df["host_time_s"] - df["host_time_s"].iloc[0]
    return df


def summarize(df: pd.DataFrame, label: str) -> None:
    dur = df["t"].iloc[-1]
    print(f"\n{'='*60}\n{label}  ({len(df)} rows, {dur:.1f}s, {len(df)/dur:.0f} Hz)")
    cols = ["pitch_rad", "pitch_rate_rads", "cmd_torque_nm", "u_ff_nm", "u_fb_nm", "u_ff_nm"]
    for col in ["pitch_rad", "pitch_rate_rads", "cmd_torque_nm", "u_ff_nm", "u_fb_nm"]:
        v = df[col]
        print(f"  {col:20s} mean={v.mean():+.5f} std={v.std():.5f}  [{v.min():+.4f}, {v.max():+.4f}]")
    sat = (df["cmd_torque_nm"].abs() >= 0.034).sum()
    sat_old = (df["cmd_torque_nm"].abs() >= 0.01870).sum()
    print(f"  saturation |cmd|>=0.01875: {sat_old}/{len(df)} ({100*sat_old/len(df):.0f}%)")
    print(f"  saturation |cmd|>=0.034:   {sat}/{len(df)} ({100*sat/len(df):.0f}%)")
    slow = df[df["pitch_rate_rads"].abs() < 0.03]
    if len(slow) > 20:
        print(f"  static |rate|<0.03: pitch={slow['pitch_rad'].mean():+.4f} cmd={slow['cmd_torque_nm'].mean():+.5f}")
        print(f"  corr(pitch,cmd) static: {np.corrcoef(slow['pitch_rad'], slow['cmd_torque_nm'])[0,1]:+.3f}")
    # recovery: pitch going backward (negative) with positive rate trying to recover?
    back = df[(df["pitch_rad"] < -0.05) & (df["pitch_rate_rads"] > 0.05)]
    print(f"  backward tilt recovering (pitch<-0.05, rate>0.05): {len(back)} samples")
    if len(back) > 5:
        print(f"    mean cmd={back['cmd_torque_nm'].mean():+.5f} u_fb={back['u_fb_nm'].mean():+.5f}")


def main() -> int:
    v3 = load("balance_v3.csv")
    try:
        v2 = load("balance_v2.csv")
        summarize(v2, "balance_v2 (old gains)")
    except FileNotFoundError:
        v2 = None
    summarize(v3, "balance_v3 (new gains)")

    # time segments v3
    print("\n--- balance_v3 time segments ---")
    dur = v3["t"].iloc[-1]
    for i, (a, b, name) in enumerate(
        [(0, min(5, dur), "0-5s"), (5, min(10, dur), "5-10s"), (10, dur, "10s-end")]
    ):
        if a >= dur:
            continue
        seg = v3[(v3["t"] >= a) & (v3["t"] < b)]
        if len(seg) < 5:
            continue
        print(
            f"{name}: pitch=[{seg['pitch_rad'].min():+.3f},{seg['pitch_rad'].max():+.3f}] "
            f"cmd=[{seg['cmd_torque_nm'].min():+.4f},{seg['cmd_torque_nm'].max():+.4f}] "
            f"|pitch|max={seg['pitch_rad'].abs().max():.3f} rad"
        )

    # cmd vs pitch error implied K
    act = v3[v3["pitch_rad"].abs() > 0.03]
    if len(act) > 20:
        ratio = (act["u_fb_nm"] / (0.0 - act["pitch_rad"])).replace([np.inf, -np.inf], np.nan).dropna()
        print(f"\nEffective K_pitch (u_fb / -pitch): median={ratio.median():.3f}  (target ~0.22)")

    # still soft? cmd lag: corr cmd with pitch vs pitch_rate
    print(f"corr(pitch, cmd) all: {np.corrcoef(v3['pitch_rad'], v3['cmd_torque_nm'])[0,1]:+.3f}")
    print(f"corr(pitch_rate, cmd) all: {np.corrcoef(v3['pitch_rate_rads'], v3['cmd_torque_nm'])[0,1]:+.3f}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
