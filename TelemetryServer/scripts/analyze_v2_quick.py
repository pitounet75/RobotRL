#!/usr/bin/env python3
import sys
from pathlib import Path
import numpy as np
import pandas as pd

path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / "logs" / "balance_v2.csv"
df = pd.read_csv(path)
df["t"] = df["host_time_s"] - df["host_time_s"].iloc[0]
dur = df["t"].iloc[-1]
print(f"{path.name}: {len(df)} rows, {dur:.1f}s, {len(df)/dur:.0f} Hz")
print(f"u_ff range: [{df['u_ff_nm'].min():+.5f}, {df['u_ff_nm'].max():+.5f}] mean={df['u_ff_nm'].mean():+.5f}")
print(f"u_fb range: [{df['u_fb_nm'].min():+.5f}, {df['u_fb_nm'].max():+.5f}]")
print(f"cmd  range: [{df['cmd_torque_nm'].min():+.5f}, {df['cmd_torque_nm'].max():+.5f}]")

# find release events: pitch_rate spike
pr = df["pitch_rate_rads"].abs()
for i in range(1, len(df)-1):
    pass

# first 15s in detail
early = df[df["t"] <= min(15, dur)]
print("\n=== First 15s (held -> release) ===")
for col in ["pitch_rad","pitch_rate_rads","u_ff_nm","u_fb_nm","cmd_torque_nm"]:
    v = early[col]
    print(f"  {col:20s} mean={v.mean():+.5f} std={v.std():.5f}")

# when pitch going backward - user said falls backward
# find segments where pitch decreasing fast (negative rate sustained) and pitch negative or becoming more negative
fall = early[(early["pitch_rate_rads"] < -0.05)]
if len(fall):
    print(f"\nFalling samples (pitch_rate<-0.05): n={len(fall)}")
    print(f"  pitch mean={fall['pitch_rad'].mean():+.4f} rate mean={fall['pitch_rate_rads'].mean():+.4f}")
    print(f"  u_ff mean={fall['u_ff_nm'].mean():+.5f} u_fb mean={fall['u_fb_nm'].mean():+.5f} cmd mean={fall['cmd_torque_nm'].mean():+.5f}")
    # does cmd help or hurt? if falling backward with negative pitch, stabilizing torque should...
    c = np.corrcoef(fall["pitch_rate_rads"], fall["cmd_torque_nm"])[0,1]
    print(f"  corr(pitch_rate, cmd)={c:+.3f}  (positive cmd should oppose negative pitch rate if signs correct)")

rise = early[(early["pitch_rate_rads"] > 0.05)]
if len(rise):
    print(f"\nRising pitch samples (pitch_rate>0.05): n={len(rise)}")
    print(f"  pitch mean={rise['pitch_rad'].mean():+.4f} u_ff mean={rise['u_ff_nm'].mean():+.5f} cmd mean={rise['cmd_torque_nm'].mean():+.5f}")

# equilibrium when held: low rate
held = early[early["pitch_rate_rads"].abs() < 0.02]
if len(held) > 20:
    print(f"\nNearly static (|rate|<0.02): n={len(held)}")
    print(f"  pitch mean={held['pitch_rad'].mean():+.5f} ({np.degrees(held['pitch_rad'].mean()):+.2f} deg)")
    print(f"  u_ff mean={held['u_ff_nm'].mean():+.5f} u_fb mean={held['u_fb_nm'].mean():+.5f} cmd mean={held['cmd_torque_nm'].mean():+.5f}")
    # expected at equilibrium pitch=p0: u_ff = -K*sin(p0), u_fb = Kp*(0-p0)
    p0 = held['pitch_rad'].mean()
    print(f"  If pitch_ref=0 but equilibrium pitch={p0:+.4f}, FB pushes wrong unless pitch_ref trimmed")

# time series 4-9s
for t in np.arange(4, 9, 0.5):
    seg = df[(df["t"] >= t) & (df["t"] < t + 0.5)]
    if len(seg) == 0:
        continue
    print(
        f"t={t:.1f}-{t+0.5:.1f}s pitch={seg['pitch_rad'].mean():+.3f} "
        f"rate={seg['pitch_rate_rads'].mean():+.3f} cmd={seg['cmd_torque_nm'].mean():+.4f}"
    )
slow = df[df["pitch_rate_rads"].abs() < 0.03]
print("slow samples corr(pitch,cmd)=", np.corrcoef(slow["pitch_rad"], slow["cmd_torque_nm"])[0, 1])
print("expected stabilizing: negative corr (positive pitch -> negative cmd)")
