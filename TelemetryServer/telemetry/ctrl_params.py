"""Runtime control parameter IDs (must match app_ctrl_params.h)."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Dict, Iterable, Tuple

PARAM_NAMES: Dict[str, int] = {
    "strategy": 0,
    "pitch_ref_rad": 1,
    "vel_ref_turns_s": 2,
    "pitch_failsafe_rad": 3,
    "pitch_kp": 4,
    "pitch_ki": 5,
    "pitch_kd": 6,
    "vel_kp": 7,
    "vel_ki": 8,
    "vel_kd": 9,
    "cmd_max_torque_nm": 10,
    "linear_theta_func": 11,
    "linear_k_pitch": 12,
    "linear_k_pitch_rate": 13,
    "linear_k_vel": 14,
    "linear_output_alpha": 15,
    "cascade_vel_kp": 16,
    "cascade_vel_ki": 17,
    "cascade_vel_kd": 18,
    "cascade_pitch_ref_max_rad": 19,
    "ff_grav_k": 20,
    "ff_fb_k_pitch": 21,
    "ff_fb_k_rate": 22,
    "ff_output_alpha": 23,
    "wheel_encoder_vel_lpf_alpha": 24,
}

NAME_BY_ID = {v: k for k, v in PARAM_NAMES.items()}

SNAPSHOT_STRUCT = struct.Struct(
    "<"
    "II"  # version, strategy_id
    "fff"  # pitch_ref, vel_ref, pitch_failsafe
    "fffffff"  # pitch/vel pid + cmd_max
    "Iffff"  # linear
    "ffff"  # cascade vel
    "ffff"  # ff
    "f"  # wheel lpf
)

SET_PARAM_STRUCT = struct.Struct("<Hf")


@dataclass(frozen=True)
class ControlParamsSnapshot:
    version: int
    strategy_id: int
    pitch_ref_rad: float
    vel_ref_turns_s: float
    pitch_failsafe_rad: float
    pitch_kp: float
    pitch_ki: float
    pitch_kd: float
    vel_kp: float
    vel_ki: float
    vel_kd: float
    cmd_max_torque_nm: float
    linear_theta_func: int
    linear_k_pitch: float
    linear_k_pitch_rate: float
    linear_k_vel: float
    linear_output_alpha: float
    cascade_vel_kp: float
    cascade_vel_ki: float
    cascade_vel_kd: float
    cascade_pitch_ref_max_rad: float
    ff_grav_k: float
    ff_fb_k_pitch: float
    ff_fb_k_rate: float
    ff_output_alpha: float
    wheel_encoder_vel_lpf_alpha: float

    def as_dict(self) -> Dict[str, float | int]:
        return {
            "strategy": self.strategy_id,
            "pitch_ref_rad": self.pitch_ref_rad,
            "vel_ref_turns_s": self.vel_ref_turns_s,
            "pitch_failsafe_rad": self.pitch_failsafe_rad,
            "pitch_kp": self.pitch_kp,
            "pitch_ki": self.pitch_ki,
            "pitch_kd": self.pitch_kd,
            "vel_kp": self.vel_kp,
            "vel_ki": self.vel_ki,
            "vel_kd": self.vel_kd,
            "cmd_max_torque_nm": self.cmd_max_torque_nm,
            "linear_theta_func": self.linear_theta_func,
            "linear_k_pitch": self.linear_k_pitch,
            "linear_k_pitch_rate": self.linear_k_pitch_rate,
            "linear_k_vel": self.linear_k_vel,
            "linear_output_alpha": self.linear_output_alpha,
            "cascade_vel_kp": self.cascade_vel_kp,
            "cascade_vel_ki": self.cascade_vel_ki,
            "cascade_vel_kd": self.cascade_vel_kd,
            "cascade_pitch_ref_max_rad": self.cascade_pitch_ref_max_rad,
            "ff_grav_k": self.ff_grav_k,
            "ff_fb_k_pitch": self.ff_fb_k_pitch,
            "ff_fb_k_rate": self.ff_fb_k_rate,
            "ff_output_alpha": self.ff_output_alpha,
            "wheel_encoder_vel_lpf_alpha": self.wheel_encoder_vel_lpf_alpha,
        }


def resolve_param(name_or_id: str | int) -> Tuple[int, str]:
    if isinstance(name_or_id, int):
        if name_or_id not in NAME_BY_ID:
            raise KeyError(f"unknown param id {name_or_id}")
        return name_or_id, NAME_BY_ID[name_or_id]
    key = name_or_id.strip()
    if key not in PARAM_NAMES:
        raise KeyError(f"unknown param {name_or_id!r}")
    return PARAM_NAMES[key], key


def decode_snapshot(payload: bytes) -> ControlParamsSnapshot:
    if len(payload) < SNAPSHOT_STRUCT.size:
        raise ValueError(f"snapshot too short: {len(payload)} < {SNAPSHOT_STRUCT.size}")
    values = SNAPSHOT_STRUCT.unpack_from(payload)
    return ControlParamsSnapshot(*values)


def encode_set_param(param_id: int, value: float) -> bytes:
    return SET_PARAM_STRUCT.pack(param_id, float(value))


def decode_set_ack(payload: bytes) -> Tuple[int, float]:
    if len(payload) < SET_PARAM_STRUCT.size:
        raise ValueError("set ack too short")
    param_id, value = SET_PARAM_STRUCT.unpack_from(payload)
    return param_id, value


def format_snapshot(snap: ControlParamsSnapshot, keys: Iterable[str] | None = None) -> str:
    data = snap.as_dict()
    if keys is None:
        keys = data.keys()
    lines = []
    for key in keys:
        val = data[key]
        if isinstance(val, float):
            lines.append(f"{key:32s} {val:.6g}")
        else:
            lines.append(f"{key:32s} {val}")
    return "\n".join(lines)
