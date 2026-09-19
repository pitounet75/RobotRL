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
    "torque_deadband_nm": 25,
    "torque_deadband_pitch_max_rad": 26,
    "torque_deadband_rate_max_rads": 27,
    "alpha_kp": 28,
    "alpha_max_nm": 29,
    "motor_J": 30,
    "motor_friction_c": 31,
    "alpha_pitch_max_rad": 32,
    "alpha_rate_max_rads": 33,
    "alpha_vel_max_turns_s": 34,
    "alpha_lpf": 35,
    "pos_kp": 36,
    "pos_kd": 37,
    "pos_pitch_kp": 38,
    "pos_x_ref_m": 39,
    "pos_v_max_turns_s": 40,
    "pos_pitch_max_rad": 41,
    "wheel_radius_m": 42,
    "pos_reset": 43,
    "pos_err_ema_alpha": 44,
    "pos_ema_kp": 45,
    "outer_mode": 46,
    "heading_kp": 47,
    "heading_kd": 48,
    "heading_ref_rad": 49,
    "heading_torque_max_nm": 50,
    "heading_reset": 51,
    "cascade_vel_err_ema_alpha": 52,
    "cascade_vel_ema_kp": 53,
    "vel_ref_slew_turns_s2": 54,
    "cascade_vel_accel_kp": 55,
    "heading_inc": 56,
    "heading_dec": 57,
    "friction_mode": 58,
    "friction_static_nm": 59,
    "friction_kinetic_nm": 60,
    "friction_vel_eps_turns_s": 61,
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
    "fff"  # torque deadband + gates
    "ffffffff"  # alpha P loop (v3)
    "ffffffff"  # position hold (v4)
    "ff"  # pos EMA (v5)
    "f"  # outer_mode (v6)
    "fffff"  # heading (v7)
    "ff"  # cascade vel EMA (v8)
    "ff"  # vel_ref slew + accel FF (v9)
    "ff"  # heading_inc + heading_dec (v10)
    "ffff"  # friction two-level (v11)
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
    torque_deadband_nm: float
    torque_deadband_pitch_max_rad: float
    torque_deadband_rate_max_rads: float
    alpha_kp: float
    alpha_max_nm: float
    motor_J: float
    motor_friction_c: float
    alpha_pitch_max_rad: float
    alpha_rate_max_rads: float
    alpha_vel_max_turns_s: float
    alpha_lpf: float
    pos_kp: float
    pos_kd: float
    pos_pitch_kp: float
    pos_x_ref_m: float
    pos_v_max_turns_s: float
    pos_pitch_max_rad: float
    wheel_radius_m: float
    pos_reset: float
    pos_err_ema_alpha: float
    pos_ema_kp: float
    outer_mode: float
    heading_kp: float
    heading_kd: float
    heading_ref_rad: float
    heading_torque_max_nm: float
    heading_reset: float
    cascade_vel_err_ema_alpha: float
    cascade_vel_ema_kp: float
    vel_ref_slew_turns_s2: float
    cascade_vel_accel_kp: float
    heading_inc: float
    heading_dec: float
    friction_mode: float
    friction_static_nm: float
    friction_kinetic_nm: float
    friction_vel_eps_turns_s: float

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
            "torque_deadband_nm": self.torque_deadband_nm,
            "torque_deadband_pitch_max_rad": self.torque_deadband_pitch_max_rad,
            "torque_deadband_rate_max_rads": self.torque_deadband_rate_max_rads,
            "alpha_kp": self.alpha_kp,
            "alpha_max_nm": self.alpha_max_nm,
            "motor_J": self.motor_J,
            "motor_friction_c": self.motor_friction_c,
            "alpha_pitch_max_rad": self.alpha_pitch_max_rad,
            "alpha_rate_max_rads": self.alpha_rate_max_rads,
            "alpha_vel_max_turns_s": self.alpha_vel_max_turns_s,
            "alpha_lpf": self.alpha_lpf,
            "pos_kp": self.pos_kp,
            "pos_kd": self.pos_kd,
            "pos_pitch_kp": self.pos_pitch_kp,
            "pos_x_ref_m": self.pos_x_ref_m,
            "pos_v_max_turns_s": self.pos_v_max_turns_s,
            "pos_pitch_max_rad": self.pos_pitch_max_rad,
            "wheel_radius_m": self.wheel_radius_m,
            "pos_reset": self.pos_reset,
            "pos_err_ema_alpha": self.pos_err_ema_alpha,
            "pos_ema_kp": self.pos_ema_kp,
            "outer_mode": self.outer_mode,
            "heading_kp": self.heading_kp,
            "heading_kd": self.heading_kd,
            "heading_ref_rad": self.heading_ref_rad,
            "heading_torque_max_nm": self.heading_torque_max_nm,
            "heading_reset": self.heading_reset,
            "cascade_vel_err_ema_alpha": self.cascade_vel_err_ema_alpha,
            "cascade_vel_ema_kp": self.cascade_vel_ema_kp,
            "vel_ref_slew_turns_s2": self.vel_ref_slew_turns_s2,
            "cascade_vel_accel_kp": self.cascade_vel_accel_kp,
            "heading_inc": self.heading_inc,
            "heading_dec": self.heading_dec,
            "friction_mode": self.friction_mode,
            "friction_static_nm": self.friction_static_nm,
            "friction_kinetic_nm": self.friction_kinetic_nm,
            "friction_vel_eps_turns_s": self.friction_vel_eps_turns_s,
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
    if len(payload) < 8:
        raise ValueError(f"snapshot too short: {len(payload)} B (need >= 8)")
    if len(payload) < SNAPSHOT_STRUCT.size:
        # Older firmware: pad trailing floats so PC can still Refresh.
        payload = payload + b"\x00" * (SNAPSHOT_STRUCT.size - len(payload))
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
