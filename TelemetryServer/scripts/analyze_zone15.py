#!/usr/bin/env python3
"""Max cmd torque for |pitch| < 15 deg."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "logs" / "balance_v3.csv"

df = pd.read_csv(path)
df["t"] = df["host_time_s"] - df["host_time_s"].iloc[0]
deg15 = np.radians(15.0)
zone = df[df["pitch_rad"].abs() < deg15]

print(f"File: {path.name}")
print(f"Total samples: {len(df)}, duration: {df['t'].iloc[-1]:.1f} s")
print(f"Zone |pitch| < 15 deg: {len(zone)} samples ({100 * len(zone) / len(df):.1f}%)")
print()

if len(zone) == 0:
    sys.exit(0)

cmd = zone["cmd_torque_nm"]
print("cmd_torque_nm in zone:")
print(f"  max        = {cmd.max():+.6f} Nm")
print(f"  min        = {cmd.min():+.6f} Nm")
print(f"  max |cmd|   = {cmd.abs().max():.6f} Nm")
print(f"  mean       = {cmd.mean():+.6f} Nm")
print(f"  std        = {cmd.std():.6f} Nm")
print(f"  p95 |cmd|  = {cmd.abs().quantile(0.95):.6f} Nm")
print(f"  p99 |cmd|  = {cmd.abs().quantile(0.99):.6f} Nm")
print()
for thr, label in [
    (0.01875, "limite ODrive bring-up"),
    (0.030, "ancien plafond STM32"),
    (0.035, "plafond STM32 actuel"),
]:
    n = int((cmd.abs() >= thr - 1e-9).sum())
    print(f"  |cmd| >= {thr:.5f} Nm ({label}): {n}/{len(zone)} ({100 * n / len(zone):.1f}%)")

idx = cmd.abs().idxmax()
row = zone.loc[idx]
print()
print("Echantillon |cmd| max dans la zone:")
print(f"  t={row['t']:.2f} s")
print(f"  pitch={np.degrees(row['pitch_rad']):+.2f} deg  rate={row['pitch_rate_rads']:+.3f} rad/s")
print(f"  cmd={row['cmd_torque_nm']:+.6f} Nm")
print(f"  u_ff={row['u_ff_nm']:+.6f}  u_fb={row['u_fb_nm']:+.6f}")
print()
pitch = zone["pitch_rad"]
print(
    f"Pitch dans la zone: [{np.degrees(pitch.min()):+.2f}, {np.degrees(pitch.max()):+.2f}] deg, "
    f"std={np.degrees(pitch.std()):.2f} deg"
)
