import unittest

from telemetry.balance_frame import (
    BALANCE_FRAME_STRUCT_V2,
    BalanceFrame,
    BalanceFrameSanityLimits,
)


def make_frame(**overrides: object) -> BalanceFrame:
    values = {
        "frame_number": 1,
        "time_us": 1000,
        "pitch_rad": 0.1,
        "pitch_rate_rads": 0.2,
        "vel_wheel_turns_s": 0.3,
        "vel_wheel_l_turns_s": 0.3,
        "vel_wheel_r_turns_s": 0.3,
        "cmd_torque_nm": 0.1,
        "cmd_torque_left_nm": 0.1,
        "cmd_torque_right_nm": 0.1,
        "u_ff_nm": 0.0,
        "u_fb_nm": 0.1,
        "pitch_ref_rad": 0.0,
        "imu_valid": 1,
        "estop": 0,
        "strategy_id": 1,
    }
    values.update(overrides)
    return BalanceFrame(**values)


class BalanceFrameSanityTests(unittest.TestCase):
    def test_v2_decodes_source_drop_counter(self) -> None:
        payload = BALANCE_FRAME_STRUCT_V2.pack(
            10,
            20,
            *([0.0] * 11),
            1,
            0,
            2,
            37,
        )
        frame = BalanceFrame.decode(payload)
        self.assertEqual(frame.source_drop_count_mod256, 37)

    def test_physical_limit_reason_is_diagnosable(self) -> None:
        frame = make_frame(cmd_torque_left_nm=3.0)

        issues = frame.sanity_issues()

        self.assertEqual(len(issues), 1)
        self.assertEqual(issues[0].category, "physical_limit")
        self.assertIn("cmd_torque_left_nm", issues[0].reason)
        self.assertFalse(frame.is_sane())

    def test_physical_limits_are_configurable(self) -> None:
        frame = make_frame(cmd_torque_left_nm=3.0, pitch_rad=4.0)
        limits = BalanceFrameSanityLimits(
            max_torque_nm=5.0,
            max_pitch_rad=5.0,
            max_wheel_turns_s=None,
            max_strategy_id=None,
        )

        self.assertTrue(frame.is_sane(limits))
        self.assertEqual(frame.sanity_rejection_reasons(limits), ())

    def test_nonfinite_value_is_encoding_issue(self) -> None:
        frame = make_frame(pitch_rate_rads=float("nan"))

        issues = frame.sanity_issues()

        self.assertEqual([(issue.category, issue.reason) for issue in issues], [("encoding", "pitch_rate_rads is not finite")])


if __name__ == "__main__":
    unittest.main()
