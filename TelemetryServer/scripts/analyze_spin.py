#!/usr/bin/env python3
"""Quick analysis of spin_left / spin_right CSV logs."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]


def analyze(name: str, path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    dur = df["host_time_s"].iloc[-1] - df["host_time_s"].iloc[0]
    print("=" * 60)
    print(f"{name} | rows: {len(df)} | duration: {dur:.1f}s | rate: {len(df) / dur:.0f} Hz")

    for col in ("vel_wheel_l_turns_s", "vel_wheel_r_turns_s", "vel_wheel_turns_s"):
        v = df[col].values
        print(f"\n  {col}:")
        print(f"    mean={np.mean(v):+.3f}  std={np.std(v):.3f}  min={np.min(v):+.3f}  max={np.max(v):+.3f}")
        print(f"    |v|>0.05: {(np.abs(v) > 0.05).sum()} ({100 * np.mean(np.abs(v) > 0.05):.0f}%)")
        print(f"    |v|>0.5:  {(np.abs(v) > 0.5).sum()} ({100 * np.mean(np.abs(v) > 0.5):.0f}%)")
        print(f"    |v|>2.0:  {(np.abs(v) > 2.0).sum()}")

    vl = df["vel_wheel_l_turns_s"]
    vr = df["vel_wheel_r_turns_s"]
    for label, s in (("vel_l", vl), ("vel_r", vr)):
        near02 = (np.abs(np.abs(s) - 0.2) < 0.02).sum()
        near04 = (np.abs(np.abs(s) - 0.4) < 0.02).sum()
        print(f"  {label} spikes ~+-0.2: {near02}  ~+-0.4: {near04}")

    active_l = np.abs(vl) > 0.3
    active_r = np.abs(vr) > 0.3
    print(f"  active |vel_l|>0.3: {active_l.sum()} samples")
    print(f"  active |vel_r|>0.3: {active_r.sum()} samples")
    if active_l.sum() > 10:
        seg = vl[active_l]
        print(f"    vel_l during motion: mean={seg.mean():+.2f} std={seg.std():.2f}")
    if active_r.sum() > 10:
        seg = vr[active_r]
        print(f"    vel_r during motion: mean={seg.mean():+.2f} std={seg.std():.2f}")

    print(f"  pitch std: {df['pitch_rad'].std():.4f}  torque std: {df['cmd_torque_nm'].std():.6f}")
    return df


def main() -> int:
    logs = ROOT / "logs"
    left = analyze("spin_left.csv", logs / "spin_left.csv")
    right = analyze("spin_right.csv", logs / "spin_right.csv")

    print("\n" + "=" * 60)
    print("COMPARISON")
    print(f"spin_left:  max |vel_l|={left['vel_wheel_l_turns_s'].abs().max():.2f}  max |vel_r|={left['vel_wheel_r_turns_s'].abs().max():.2f}")
    print(f"spin_right: max |vel_l|={right['vel_wheel_l_turns_s'].abs().max():.2f}  max |vel_r|={right['vel_wheel_r_turns_s'].abs().max():.2f}")

    l_act = left[left["vel_wheel_l_turns_s"].abs() > 0.5]["vel_wheel_l_turns_s"]
    r_act = right[right["vel_wheel_r_turns_s"].abs() > 0.5]["vel_wheel_r_turns_s"]
    print(f"\nDuring |vel|>0.5 turn/s:")
    print(f"  left session  vel_l std={l_act.std():.2f}  n={len(l_act)}")
    print(f"  right session vel_r std={r_act.std():.2f}  n={len(r_act)}")
    if len(l_act) > 0 and len(r_act) > 0:
        print(f"  noise ratio vel_r/vel_l std: {r_act.std() / max(l_act.std(), 1e-6):.2f}x")

    print("\nspin_right vel_r value counts (top 12):")
    print(right["vel_wheel_r_turns_s"].value_counts().head(12).to_string())

    # segment with clear right-wheel motion
    r_move = right[right["vel_wheel_r_turns_s"].abs() > 1.0]
    if len(r_move) > 0:
        t0 = r_move["host_time_s"].iloc[0]
        t1 = r_move["host_time_s"].iloc[-1]
        seg = right[(right["host_time_s"] >= t0 - 0.5) & (right["host_time_s"] <= t1 + 0.5)]
        print(f"\nspin_right motion window ({t1 - t0:.1f}s, {len(seg)} samples):")
        print(f"  vel_r std={seg['vel_wheel_r_turns_s'].std():.2f}  vel_l std={seg['vel_wheel_l_turns_s'].std():.2f}")
        print(f"  pitch std={seg['pitch_rad'].std():.4f}  torque std={seg['cmd_torque_nm'].std():.6f}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
