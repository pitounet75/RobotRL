"""Runtime control parameter IDs (must match app_ctrl_params.h)."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Dict, Iterable, Tuple

# Must match APP_CTRL_PARAMS_SNAPSHOT_VERSION. GET pad/truncate on mismatch;
# SET has no version — unknown ids fail on the flashed firmware.
SNAPSHOT_VERSION = 19

PARAM_NAMES: Dict[str, int] = {
    "strategy": 0,
    "pitch_ref_rad": 1,
    "vel_ref_turns_s": 2,
    "pitch_failsafe_rad": 3,
    "cmd_max_torque_nm": 4,
    "cascade_vel_kp": 5,
    "cascade_vel_kd": 6,
    "cascade_pitch_ref_max_rad": 7,
    "meca_k_grav": 8,
    "err_k_pitch": 9,
    "meca_k_pitch_damp": 10,
    "balance_output_alpha": 11,
    "wheel_encoder_vel_lpf_alpha": 12,
    "torque_deadband_nm": 13,
    "torque_deadband_pitch_max_rad": 14,
    "torque_deadband_rate_max_rads": 15,
    "motor_torque_correction_kp": 16,
    "motor_torque_correction_max_nm": 17,
    "motor_J": 18,
    "motor_friction_c": 19,
    "motor_torque_correction_gate_pitch_max_rad": 20,
    "motor_torque_correction_gate_rate_max_rads": 21,
    "motor_torque_correction_gate_vel_max_turns_s": 22,
    "motor_accel_lpf": 23,
    "pos_kp": 24,
    "pos_kd": 25,
    "pos_x_ref_m": 26,
    "pos_v_max_turns_s": 27,
    "wheel_radius_m": 28,
    "pos_reset": 29,
    "pos_err_ema_alpha": 30,
    "pos_ema_kp": 31,
    "outer_mode": 32,
    "heading_kp": 33,
    "heading_kd": 34,
    "heading_ref_rad": 35,
    "heading_torque_max_nm": 36,
    "heading_reset": 37,
    "cascade_vel_err_ema_alpha": 38,
    "cascade_vel_ema_kp": 39,
    "vel_ref_slew_turns_s2": 40,
    "cascade_vel_accel_kp": 41,
    "heading_inc": 42,
    "heading_dec": 43,
    "friction_mode": 44,
    "friction_static_nm": 45,
    "friction_kinetic_nm": 46,
    "friction_vel_eps_turns_s": 47,
    "err_k_vel": 48,
    "heading_ema": 49,
    "heading_d_ema": 50,
    "antipat_enable": 51,
    "antipat_sync_enable": 52,
    "antipat_sync_track_width_m": 53,
    "antipat_tau_min_nm": 54,
    "antipat_eta_on": 55,
    "antipat_both_eta_off": 56,
    "antipat_both_alpha_contact_max_rads2": 57,
    "antipat_both_tau_steady_air_nm": 58,
    "antipat_omega_air_min_turns_s": 59,
    "antipat_sync_k_dom": 60,
    "antipat_sync_eps_abs_rads": 61,
    "antipat_sync_k_rel": 62,
    "antipat_sync_k_off": 63,
    "antipat_sync_t_on_ms": 64,
    "antipat_sync_t_off_ms": 65,
    "antipat_both_t_on_ms": 66,
    "antipat_both_t_off_ms": 67,
    "antipat_both_t_ma_ms": 68,
    "antipat_t_recover_ms": 69,
    "antipat_both_u_min_nm": 70,
    "antipat_both_pitch_rate_min_rads": 71,
    "antipat_sync_k": 72,
    "antipat_sync_tau_max_nm": 73,
    "antipat_both_k_v": 74,
    "antipat_both_tau_max_nm": 75,
    "antipat_both_enable": 76,
    "antipat_tau_ema": 77,
    "antipat_u_fade_ms": 78,
    "antipat_sync_kd": 79,
}

NAME_BY_ID = {v: k for k, v in PARAM_NAMES.items()}

SNAPSHOT_STRUCT = struct.Struct(
    "<"
    "II"  # version, strategy_id
    + "f" * 79
)

SET_PARAM_STRUCT = struct.Struct("<Hf")


@dataclass(frozen=True)
class ControlParamsSnapshot:
    version: int
    strategy_id: int
    pitch_ref_rad: float
    vel_ref_turns_s: float
    pitch_failsafe_rad: float
    cmd_max_torque_nm: float
    cascade_vel_kp: float
    cascade_vel_kd: float
    cascade_pitch_ref_max_rad: float
    meca_k_grav: float
    err_k_pitch: float
    meca_k_pitch_damp: float
    balance_output_alpha: float
    wheel_encoder_vel_lpf_alpha: float
    torque_deadband_nm: float
    torque_deadband_pitch_max_rad: float
    torque_deadband_rate_max_rads: float
    motor_torque_correction_kp: float
    motor_torque_correction_max_nm: float
    motor_J: float
    motor_friction_c: float
    motor_torque_correction_gate_pitch_max_rad: float
    motor_torque_correction_gate_rate_max_rads: float
    motor_torque_correction_gate_vel_max_turns_s: float
    motor_accel_lpf: float
    pos_kp: float
    pos_kd: float
    pos_x_ref_m: float
    pos_v_max_turns_s: float
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
    err_k_vel: float
    heading_ema: float
    heading_d_ema: float
    antipat_enable: float
    antipat_sync_enable: float
    antipat_sync_track_width_m: float
    antipat_tau_min_nm: float
    antipat_eta_on: float
    antipat_both_eta_off: float
    antipat_both_alpha_contact_max_rads2: float
    antipat_both_tau_steady_air_nm: float
    antipat_omega_air_min_turns_s: float
    antipat_sync_k_dom: float
    antipat_sync_eps_abs_rads: float
    antipat_sync_k_rel: float
    antipat_sync_k_off: float
    antipat_sync_t_on_ms: float
    antipat_sync_t_off_ms: float
    antipat_both_t_on_ms: float
    antipat_both_t_off_ms: float
    antipat_both_t_ma_ms: float
    antipat_t_recover_ms: float
    antipat_both_u_min_nm: float
    antipat_both_pitch_rate_min_rads: float
    antipat_sync_k: float
    antipat_sync_tau_max_nm: float
    antipat_both_k_v: float
    antipat_both_tau_max_nm: float
    antipat_both_enable: float
    antipat_tau_ema: float
    antipat_u_fade_ms: float
    antipat_sync_kd: float

    def as_dict(self) -> Dict[str, float | int]:
        return {
            "strategy": self.strategy_id,
            "pitch_ref_rad": self.pitch_ref_rad,
            "vel_ref_turns_s": self.vel_ref_turns_s,
            "pitch_failsafe_rad": self.pitch_failsafe_rad,
            "cmd_max_torque_nm": self.cmd_max_torque_nm,
            "cascade_vel_kp": self.cascade_vel_kp,
            "cascade_vel_kd": self.cascade_vel_kd,
            "cascade_pitch_ref_max_rad": self.cascade_pitch_ref_max_rad,
            "meca_k_grav": self.meca_k_grav,
            "err_k_pitch": self.err_k_pitch,
            "meca_k_pitch_damp": self.meca_k_pitch_damp,
            "balance_output_alpha": self.balance_output_alpha,
            "wheel_encoder_vel_lpf_alpha": self.wheel_encoder_vel_lpf_alpha,
            "torque_deadband_nm": self.torque_deadband_nm,
            "torque_deadband_pitch_max_rad": self.torque_deadband_pitch_max_rad,
            "torque_deadband_rate_max_rads": self.torque_deadband_rate_max_rads,
            "motor_torque_correction_kp": self.motor_torque_correction_kp,
            "motor_torque_correction_max_nm": self.motor_torque_correction_max_nm,
            "motor_J": self.motor_J,
            "motor_friction_c": self.motor_friction_c,
            "motor_torque_correction_gate_pitch_max_rad": self.motor_torque_correction_gate_pitch_max_rad,
            "motor_torque_correction_gate_rate_max_rads": self.motor_torque_correction_gate_rate_max_rads,
            "motor_torque_correction_gate_vel_max_turns_s": self.motor_torque_correction_gate_vel_max_turns_s,
            "motor_accel_lpf": self.motor_accel_lpf,
            "pos_kp": self.pos_kp,
            "pos_kd": self.pos_kd,
            "pos_x_ref_m": self.pos_x_ref_m,
            "pos_v_max_turns_s": self.pos_v_max_turns_s,
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
            "err_k_vel": self.err_k_vel,
            "heading_ema": self.heading_ema,
            "heading_d_ema": self.heading_d_ema,
            "antipat_enable": self.antipat_enable,
            "antipat_sync_enable": self.antipat_sync_enable,
            "antipat_sync_track_width_m": self.antipat_sync_track_width_m,
            "antipat_tau_min_nm": self.antipat_tau_min_nm,
            "antipat_eta_on": self.antipat_eta_on,
            "antipat_both_eta_off": self.antipat_both_eta_off,
            "antipat_both_alpha_contact_max_rads2": self.antipat_both_alpha_contact_max_rads2,
            "antipat_both_tau_steady_air_nm": self.antipat_both_tau_steady_air_nm,
            "antipat_omega_air_min_turns_s": self.antipat_omega_air_min_turns_s,
            "antipat_sync_k_dom": self.antipat_sync_k_dom,
            "antipat_sync_eps_abs_rads": self.antipat_sync_eps_abs_rads,
            "antipat_sync_k_rel": self.antipat_sync_k_rel,
            "antipat_sync_k_off": self.antipat_sync_k_off,
            "antipat_sync_t_on_ms": self.antipat_sync_t_on_ms,
            "antipat_sync_t_off_ms": self.antipat_sync_t_off_ms,
            "antipat_both_t_on_ms": self.antipat_both_t_on_ms,
            "antipat_both_t_off_ms": self.antipat_both_t_off_ms,
            "antipat_both_t_ma_ms": self.antipat_both_t_ma_ms,
            "antipat_t_recover_ms": self.antipat_t_recover_ms,
            "antipat_both_u_min_nm": self.antipat_both_u_min_nm,
            "antipat_both_pitch_rate_min_rads": self.antipat_both_pitch_rate_min_rads,
            "antipat_sync_k": self.antipat_sync_k,
            "antipat_sync_tau_max_nm": self.antipat_sync_tau_max_nm,
            "antipat_both_k_v": self.antipat_both_k_v,
            "antipat_both_tau_max_nm": self.antipat_both_tau_max_nm,
            "antipat_both_enable": self.antipat_both_enable,
            "antipat_tau_ema": self.antipat_tau_ema,
            "antipat_u_fade_ms": self.antipat_u_fade_ms,
            "antipat_sync_kd": self.antipat_sync_kd,
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
