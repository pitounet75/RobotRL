import unittest

from telemetry.balance_frame import (
    BALANCE_FRAME_STRUCT_V2,
    BALANCE_FRAME_STRUCT_V3,
    BALANCE_FRAME_STRUCT_V4,
    BALANCE_FRAME_STRUCT_V5,
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
        "u_meca_nm": 0.0,
        "u_err_nm": 0.1,
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
        self.assertEqual(frame.vbus_l_v, 0.0)

    def test_v3_decodes_vbus(self) -> None:
        payload = BALANCE_FRAME_STRUCT_V3.pack(
            10,
            20,
            *([0.0] * 11),
            1,
            0,
            2,
            37,
            23.5,
            24.1,
        )
        frame = BalanceFrame.decode(payload)
        self.assertAlmostEqual(frame.vbus_l_v, 23.5, places=5)
        self.assertAlmostEqual(frame.vbus_r_v, 24.1, places=5)
        self.assertEqual(frame.sync_l, 0)
        self.assertEqual(frame.sync_r, 0)

    def test_v4_decodes_sync_flags(self) -> None:
        payload = BALANCE_FRAME_STRUCT_V4.pack(
            10,
            20,
            *([0.0] * 11),
            1,
            0,
            2,
            37,
            23.5,
            24.1,
            1,
            1,
            0,
            0,
        )
        frame = BalanceFrame.decode(payload)
        self.assertEqual(frame.wc_mode, 1)
        self.assertEqual(frame.sync_l, 1)
        self.assertEqual(frame.sync_r, 0)
        self.assertAlmostEqual(frame.vbus_l_v, 23.5, places=5)

    def test_v5_decodes_per_wheel_fit(self) -> None:
        payload = BALANCE_FRAME_STRUCT_V5.pack(
            10,
            20,
            *([0.0] * 11),
            1,
            0,
            2,
            37,
            23.5,
            24.1,
            1,
            1,
            0,
            0,
            1.25,
            -1.5,
            14.0,
            -14.0,
        )
        frame = BalanceFrame.decode(payload)
        self.assertAlmostEqual(frame.vel_fit_l_turns_s, 1.25, places=5)
        self.assertAlmostEqual(frame.vel_fit_r_turns_s, -1.5, places=5)
        self.assertAlmostEqual(frame.acc_fit_l_turns_s2, 14.0, places=5)
        self.assertAlmostEqual(frame.acc_fit_r_turns_s2, -14.0, places=5)
        # V4 fields must survive the layout growth.
        self.assertEqual(frame.wc_mode, 1)
        self.assertAlmostEqual(frame.vbus_l_v, 23.5, places=5)

    def test_v4_payload_still_decodes_with_fit_zeroed(self) -> None:
        """A robot running pre-V5 firmware must still stream, not crash."""
        payload = BALANCE_FRAME_STRUCT_V4.pack(
            10, 20, *([0.0] * 11), 1, 0, 2, 37, 23.5, 24.1, 3, 1, 1, 0
        )
        frame = BalanceFrame.decode(payload)
        self.assertEqual(frame.wc_mode, 3)
        self.assertEqual(frame.vel_fit_l_turns_s, 0.0)
        self.assertEqual(frame.acc_fit_r_turns_s2, 0.0)

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

    def test_encode_round_trips_through_decode(self) -> None:
        # Float fields are packed as float32 on the wire, so a double like 0.1
        # loses precision on the round trip (0.1 -> 0.10000000149011612) --
        # compare floats with assertAlmostEqual, not full dataclass equality.
        frame = make_frame(frame_number=42, time_us=123456, source_drop_count_mod256=7, sync_l=1, wc_mode=1,
                           vel_fit_l_turns_s=1.25, acc_fit_r_turns_s2=-14.0)

        decoded = BalanceFrame.decode(frame.encode())

        self.assertEqual(decoded.frame_number, frame.frame_number)
        self.assertEqual(decoded.time_us, frame.time_us)
        self.assertEqual(decoded.imu_valid, frame.imu_valid)
        self.assertEqual(decoded.estop, frame.estop)
        self.assertEqual(decoded.strategy_id, frame.strategy_id)
        self.assertEqual(decoded.source_drop_count_mod256, frame.source_drop_count_mod256)
        self.assertAlmostEqual(decoded.vbus_l_v, frame.vbus_l_v, places=6)
        self.assertAlmostEqual(decoded.vbus_r_v, frame.vbus_r_v, places=6)
        self.assertEqual(decoded.wc_mode, frame.wc_mode)
        self.assertEqual(decoded.sync_l, frame.sync_l)
        self.assertEqual(decoded.sync_r, frame.sync_r)
        for field in (
            "pitch_rad",
            "pitch_rate_rads",
            "vel_wheel_turns_s",
            "vel_wheel_l_turns_s",
            "vel_wheel_r_turns_s",
            "cmd_torque_nm",
            "cmd_torque_left_nm",
            "cmd_torque_right_nm",
            "u_meca_nm",
            "u_err_nm",
            "pitch_ref_rad",
            "vbus_l_v",
            "vbus_r_v",
            "vel_fit_l_turns_s",
            "vel_fit_r_turns_s",
            "acc_fit_l_turns_s2",
            "acc_fit_r_turns_s2",
        ):
            self.assertAlmostEqual(getattr(decoded, field), getattr(frame, field), places=6)


if __name__ == "__main__":
    unittest.main()
