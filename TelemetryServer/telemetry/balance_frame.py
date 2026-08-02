"""Decode BalanceFrame (message type 0x0100)."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

BALANCE_FRAME_STRUCT_V1 = struct.Struct("<II9f4B")
BALANCE_FRAME_STRUCT_V2 = struct.Struct("<II11f4B")
BALANCE_FRAME_PAYLOAD_LEN_V1 = 48
BALANCE_FRAME_PAYLOAD_LEN_V2 = 56
BALANCE_FRAME_PAYLOAD_LEN = BALANCE_FRAME_PAYLOAD_LEN_V2

# Motor-side Nm; reject corrupted sync frames (typ. |torque| < 0.5 Nm at bring-up).
_MAX_TORQUE_NM = 2.0
_MAX_PITCH_RAD = 3.5
_MAX_WHEEL_TURN_S = 50.0


@dataclass
class BalanceFrame:
    frame_number: int
    time_us: int
    pitch_rad: float
    pitch_rate_rads: float
    vel_wheel_turns_s: float
    vel_wheel_l_turns_s: float
    vel_wheel_r_turns_s: float
    cmd_torque_nm: float
    cmd_torque_left_nm: float
    cmd_torque_right_nm: float
    u_ff_nm: float
    u_fb_nm: float
    pitch_ref_rad: float
    imu_valid: int
    estop: int
    strategy_id: int

    @classmethod
    def decode(cls, payload: bytes) -> "BalanceFrame":
        if len(payload) >= BALANCE_FRAME_PAYLOAD_LEN_V2:
            values = BALANCE_FRAME_STRUCT_V2.unpack(payload[:BALANCE_FRAME_PAYLOAD_LEN_V2])
            return cls(
                frame_number=values[0],
                time_us=values[1],
                pitch_rad=values[2],
                pitch_rate_rads=values[3],
                vel_wheel_turns_s=values[4],
                vel_wheel_l_turns_s=values[5],
                vel_wheel_r_turns_s=values[6],
                cmd_torque_nm=values[7],
                cmd_torque_left_nm=values[8],
                cmd_torque_right_nm=values[9],
                u_ff_nm=values[10],
                u_fb_nm=values[11],
                pitch_ref_rad=values[12],
                imu_valid=values[13],
                estop=values[14],
                strategy_id=values[15],
            )

        if len(payload) >= BALANCE_FRAME_PAYLOAD_LEN_V1:
            values = BALANCE_FRAME_STRUCT_V1.unpack(payload[:BALANCE_FRAME_PAYLOAD_LEN_V1])
            cmd = values[7]
            return cls(
                frame_number=values[0],
                time_us=values[1],
                pitch_rad=values[2],
                pitch_rate_rads=values[3],
                vel_wheel_turns_s=values[4],
                vel_wheel_l_turns_s=values[5],
                vel_wheel_r_turns_s=values[6],
                cmd_torque_nm=cmd,
                cmd_torque_left_nm=cmd,
                cmd_torque_right_nm=cmd,
                u_ff_nm=values[8],
                u_fb_nm=values[9],
                pitch_ref_rad=values[10],
                imu_valid=values[11],
                estop=values[12],
                strategy_id=values[13],
            )

        raise ValueError(f"BalanceFrame payload too short: {len(payload)}")

    def is_sane(self) -> bool:
        """Drop UART/UDP mis-sync frames that still pass CRC-8."""
        if self.imu_valid not in (0, 1) or self.estop not in (0, 1):
            return False
        if self.strategy_id > 32:
            return False
        floats = (
            self.pitch_rad,
            self.pitch_rate_rads,
            self.vel_wheel_turns_s,
            self.vel_wheel_l_turns_s,
            self.vel_wheel_r_turns_s,
            self.cmd_torque_nm,
            self.cmd_torque_left_nm,
            self.cmd_torque_right_nm,
            self.u_ff_nm,
            self.u_fb_nm,
            self.pitch_ref_rad,
        )
        if not all(math.isfinite(v) for v in floats):
            return False
        if abs(self.pitch_rad) > _MAX_PITCH_RAD:
            return False
        torque_vals = (
            self.cmd_torque_nm,
            self.cmd_torque_left_nm,
            self.cmd_torque_right_nm,
            self.u_ff_nm,
            self.u_fb_nm,
        )
        if any(abs(v) > _MAX_TORQUE_NM for v in torque_vals):
            return False
        if any(abs(v) > _MAX_WHEEL_TURN_S for v in (self.vel_wheel_l_turns_s, self.vel_wheel_r_turns_s)):
            return False
        return True
