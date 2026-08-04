"""Decode BalanceFrame (message type 0x0100)."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Optional, Tuple

BALANCE_FRAME_STRUCT_V1 = struct.Struct("<II9f4B")
BALANCE_FRAME_STRUCT_V2 = struct.Struct("<II11f4B")
BALANCE_FRAME_PAYLOAD_LEN_V1 = 48
BALANCE_FRAME_PAYLOAD_LEN_V2 = 56
BALANCE_FRAME_PAYLOAD_LEN = BALANCE_FRAME_PAYLOAD_LEN_V2

@dataclass(frozen=True)
class BalanceFrameSanityLimits:
    """Optional operational limits, separate from encoding-validity checks."""

    max_torque_nm: Optional[float] = 2.0
    max_pitch_rad: Optional[float] = 3.5
    max_wheel_turns_s: Optional[float] = 50.0
    max_strategy_id: Optional[int] = 32


@dataclass(frozen=True)
class SanityIssue:
    category: str
    reason: str


DEFAULT_SANITY_LIMITS = BalanceFrameSanityLimits()


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
    source_drop_count_mod256: int = 0

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
                source_drop_count_mod256=values[16],
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
                source_drop_count_mod256=values[14],
            )

        raise ValueError(f"BalanceFrame payload too short: {len(payload)}")

    def sanity_issues(
        self, limits: BalanceFrameSanityLimits = DEFAULT_SANITY_LIMITS
    ) -> Tuple[SanityIssue, ...]:
        """Return encoding and configurable physical-limit rejection reasons."""
        issues = []
        if self.imu_valid not in (0, 1) or self.estop not in (0, 1):
            issues.append(SanityIssue("encoding", "invalid boolean flag"))
        if limits.max_strategy_id is not None and self.strategy_id > limits.max_strategy_id:
            issues.append(
                SanityIssue(
                    "physical_limit",
                    f"strategy_id={self.strategy_id} exceeds {limits.max_strategy_id}",
                )
            )

        float_fields = (
            ("pitch_rad", self.pitch_rad),
            ("pitch_rate_rads", self.pitch_rate_rads),
            ("vel_wheel_turns_s", self.vel_wheel_turns_s),
            ("vel_wheel_l_turns_s", self.vel_wheel_l_turns_s),
            ("vel_wheel_r_turns_s", self.vel_wheel_r_turns_s),
            ("cmd_torque_nm", self.cmd_torque_nm),
            ("cmd_torque_left_nm", self.cmd_torque_left_nm),
            ("cmd_torque_right_nm", self.cmd_torque_right_nm),
            ("u_ff_nm", self.u_ff_nm),
            ("u_fb_nm", self.u_fb_nm),
            ("pitch_ref_rad", self.pitch_ref_rad),
        )
        for name, value in float_fields:
            if not math.isfinite(value):
                issues.append(SanityIssue("encoding", f"{name} is not finite"))

        if limits.max_pitch_rad is not None and abs(self.pitch_rad) > limits.max_pitch_rad:
            issues.append(
                SanityIssue(
                    "physical_limit",
                    f"abs(pitch_rad)={abs(self.pitch_rad):g} exceeds {limits.max_pitch_rad:g}",
                )
            )
        torque_fields = (
            ("cmd_torque_nm", self.cmd_torque_nm),
            ("cmd_torque_left_nm", self.cmd_torque_left_nm),
            ("cmd_torque_right_nm", self.cmd_torque_right_nm),
            ("u_ff_nm", self.u_ff_nm),
            ("u_fb_nm", self.u_fb_nm),
        )
        if limits.max_torque_nm is not None:
            for name, value in torque_fields:
                if abs(value) > limits.max_torque_nm:
                    issues.append(
                        SanityIssue(
                            "physical_limit",
                            f"abs({name})={abs(value):g} exceeds {limits.max_torque_nm:g}",
                        )
                    )
        if limits.max_wheel_turns_s is not None:
            for name, value in (
                ("vel_wheel_l_turns_s", self.vel_wheel_l_turns_s),
                ("vel_wheel_r_turns_s", self.vel_wheel_r_turns_s),
            ):
                if abs(value) > limits.max_wheel_turns_s:
                    issues.append(
                        SanityIssue(
                            "physical_limit",
                            f"abs({name})={abs(value):g} exceeds {limits.max_wheel_turns_s:g}",
                        )
                    )
        return tuple(issues)

    def sanity_rejection_reasons(
        self, limits: BalanceFrameSanityLimits = DEFAULT_SANITY_LIMITS
    ) -> Tuple[str, ...]:
        return tuple(issue.reason for issue in self.sanity_issues(limits))

    def is_sane(self, limits: BalanceFrameSanityLimits = DEFAULT_SANITY_LIMITS) -> bool:
        """Return whether encoding checks and configured operational limits pass."""
        return not self.sanity_issues(limits)
