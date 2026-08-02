#!/usr/bin/env python3
"""Analyze balance CSV for ff_cascade gain tuning."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]


def dominant_freq_hz(t: np.ndarray, y: np.ndarray, fmin: float = 0.3, fmax: float = 15.0) -> float | None:
    if len(y) < 64:
        return None
    y = y - np.mean(y)
    dt = np.median(np.diff(t))
    if dt <= 0:
        return None
    n = len(y)
    win = np.hanning(n)
    spec = np.fft.rfft(y * win)
    freqs = np.fft.rfftfreq(n, dt)
    mag = np.abs(spec)
    band = (freqs >= fmin) & (freqs <= fmax)
    if not np.any(band):
        return None
    idx = np.argmax(mag[band])
    return float(freqs[band][idx])


def segment_stats(df: pd.DataFrame, t0: float, t1: float, label: str, cmd_max: float) -> None:
    seg = df[(df["t_rel"] >= t0) & (df["t_rel"] <= t1)]
    if len(seg) < 5:
        print(f"\n{label}: too few samples")
        return
    print(f"\n--- {label} [{t0:.1f}-{t1:.1f}s, n={len(seg)}] ---")
    cols = [
        "pitch_rad",
        "pitch_rate_rads",
        "cmd_torque_nm",
        "u_fb_nm",
        "u_ff_nm",
        "vel_wheel_turns_s",
    ]
    if "cmd_torque_left_nm" in seg.columns:
        cols.extend(["cmd_torque_left_nm", "cmd_torque_right_nm"])
    for col in cols:
        if col not in seg.columns:
            continue
        v = seg[col]
        print(f"  {col:24s} mean={v.mean():+.5f}  std={v.std():.5f}  min={v.min():+.5f}  max={v.max():+.5f}")

    sat_thresh = cmd_max * 0.95
    sat = (seg["cmd_torque_nm"].abs() >= sat_thresh).sum()
    print(f"  torque saturated: {sat}/{len(seg)} ({100 * sat / len(seg):.0f}%)")

    dt = np.median(np.diff(seg["t_rel"].values))
    if dt > 0 and len(seg) > 2:
        dcmd = np.diff(seg["cmd_torque_nm"].values) / dt
        print(f"  |d(cmd)/dt| mean={np.mean(np.abs(dcmd)):.5f}  max={np.max(np.abs(dcmd)):.5f} Nm/s  (jerk proxy)")

    f_pitch = dominant_freq_hz(seg["t_rel"].values, seg["pitch_rad"].values)
    f_torq = dominant_freq_hz(seg["t_rel"].values, seg["cmd_torque_nm"].values)
    if f_pitch:
        print(f"  dominant pitch freq: {f_pitch:.2f} Hz")
    if f_torq:
        print(f"  dominant torque freq: {f_torq:.2f} Hz")
    c = np.corrcoef(seg["pitch_rad"], seg["u_fb_nm"])[0, 1]
    print(f"  corr(pitch, u_fb): {c:+.3f}  (expect negative for stabilizing FB)")


def gain_hints(df: pd.DataFrame, cmd_max: float) -> None:
    pitch = df["pitch_rad"]
    rate = df["pitch_rate_rads"]
    cmd = df["cmd_torque_nm"]
    u_fb = df["u_fb_nm"]
    u_ff = df["u_ff_nm"]

    small = df[pitch.abs() < np.radians(5.0)]
    print("\n=== GAIN HINTS (strategy 3 ff_cascade) ===")

    if len(small) > 20:
        cmd_std = small["cmd_torque_nm"].std()
        rate_std = small["pitch_rate_rads"].std()
        print(f"Small |pitch|<5 deg: cmd_std={cmd_std:.5f} Nm  pitch_rate_std={rate_std:.4f} rad/s")
        if cmd_std > 0.004:
            print("  -> Jerky at small angle: try  set ff_fb_k_rate 0.001  or  set ff_output_alpha 0.20")
        if rate_std > 0.15 and cmd_std > 0.003:
            print("  -> Noisy rate driving torque: lower ff_fb_k_rate first, not ff_fb_k_pitch")

    if u_ff.abs().max() < 1e-6:
        print("  -> u_ff always 0: ff_grav_k is 0 or strategy != ff_cascade")

    mp = pitch.mean()
    print(f"Session mean pitch: {mp:+.4f} rad ({np.degrees(mp):+.1f} deg)")
    if abs(mp) > 0.025:
        print("  -> Static tilt: tune pitch_ref_rad or ff_grav_k / ff_fb_k_pitch balance")

    valid = df[pitch.abs() > 0.003]
    if len(valid) > 10:
        ratio = (valid["u_fb_nm"] / valid["pitch_rad"]).median()
        print(f"Effective u_fb/pitch (median): {ratio:.4f}  (nominal ~ -ff_fb_k_pitch)")

    sat_pct = 100 * (cmd.abs() >= cmd_max * 0.95).mean()
    if sat_pct > 8:
        print(f"  -> Saturated {sat_pct:.0f}%: reduce ff_fb_k_pitch before raising cmd_max_torque_nm")

    release = df[(pitch.abs() > np.radians(2.0)) & (pitch.abs() < np.radians(12.0))]
    if len(release) > 30:
        lag = release["u_ff_nm"].mean() - release["u_fb_nm"].mean()
        print(f"Mid-fall window: mean u_ff={release['u_ff_nm'].mean():+.5f}  u_fb={release['u_fb_nm'].mean():+.5f}")
        if release["cmd_torque_nm"].abs().mean() < 0.004 and pitch.abs().mean() > 0.05:
            print("  -> Weak catch on release: increase |ff_grav_k| (e.g. -0.07) or ff_fb_k_pitch slightly")


def main() -> int:
    ap = argparse.ArgumentParser(description="Analyze balance CSV for gain tuning")
    ap.add_argument(
        "csv",
        nargs="?",
        type=Path,
        default=ROOT / "logs" / "balance_disturb.csv",
        help="CSV from run_server.py --record",
    )
    ap.add_argument("--cmd-max", type=float, default=0.035, help="cmd_max_torque_nm for sat detection")
    args = ap.parse_args()

    path = args.csv
    if not path.is_file():
        print(f"Missing {path}", file=sys.stderr)
        return 1

    df = pd.read_csv(path)
    df["t_rel"] = df["host_time_s"] - df["host_time_s"].iloc[0]
    dur = df["t_rel"].iloc[-1]

    print("=" * 60)
    print(f"{path.name} | {len(df)} rows | {dur:.1f}s | {len(df) / max(dur, 0.001):.0f} Hz")
    if "strategy_id" in df.columns:
        print(
            f"strategy_id: {df['strategy_id'].unique()}  estop: {df['estop'].unique()}  "
            f"imu_valid: {df['imu_valid'].unique()}"
        )

    print("\nFull session overview:")
    for col in ("pitch_rad", "pitch_rate_rads", "cmd_torque_nm", "u_fb_nm", "u_ff_nm"):
        if col not in df.columns:
            continue
        v = df[col]
        print(f"  {col:22s} mean={v.mean():+.5f}  std={v.std():.5f}  range=[{v.min():+.4f}, {v.max():+.4f}]")

    pitch = df["pitch_rad"].abs()
    active = (pitch > 0.008) | (df["cmd_torque_nm"].abs() > 0.002)
    if active.sum() > 20:
        idx = np.where(active.values)[0]
        runs: list[tuple[int, int]] = []
        start = idx[0]
        prev = idx[0]
        for i in idx[1:]:
            if i - prev > 15:
                runs.append((start, prev))
                start = i
            prev = i
        runs.append((start, prev))
        print(f"\nDetected {len(runs)} active region(s):")
        for i, (a, b) in enumerate(runs):
            t0, t1 = df["t_rel"].iloc[a], df["t_rel"].iloc[b]
            segment_stats(df, t0, t1, f"active region {i + 1}", args.cmd_max)

    third = dur / 3
    segment_stats(df, 0, third, "first third", args.cmd_max)
    segment_stats(df, third, 2 * third, "middle third", args.cmd_max)
    segment_stats(df, 2 * third, dur, "last third", args.cmd_max)

    gain_hints(df, args.cmd_max)
    return 0


if __name__ == "__main__":
    sys.exit(main())
