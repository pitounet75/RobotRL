import unittest
from typing import List, Tuple

from telemetry.ctrl_params import ControlParamsSnapshot
from telemetry.web_server import handle_control_message


class _FakeRpc:
    def __init__(self) -> None:
        self.set_calls: List[Tuple[str, float]] = []

    def get_params(self) -> ControlParamsSnapshot:
        n_fields = len(ControlParamsSnapshot.__dataclass_fields__)
        values = [10, 1] + [0.0] * (n_fields - 2)
        return ControlParamsSnapshot(*values)

    def set_param(self, name_or_id, value):
        name = name_or_id if isinstance(name_or_id, str) else str(name_or_id)
        self.set_calls.append((name, float(value)))
        return 99, name, float(value)


class HandleControlMessageTests(unittest.TestCase):
    def test_rpc_unavailable(self) -> None:
        resp = handle_control_message(None, {"action": "get_params"})
        self.assertEqual(
            resp, {"ok": False, "error": "RPC unavailable (pass --esp32-host)."}
        )

    def test_get_params_returns_snapshot_dict(self) -> None:
        rpc = _FakeRpc()
        resp = handle_control_message(rpc, {"action": "get_params"})
        self.assertTrue(resp["ok"])
        self.assertEqual(resp["version"], 10)
        self.assertIn("cascade_vel_kd", resp["params"])

    def test_set_param_forwards_name_and_value(self) -> None:
        rpc = _FakeRpc()
        resp = handle_control_message(
            rpc, {"action": "set_param", "name": "cascade_vel_kd", "value": "0.5"}
        )
        self.assertEqual(
            resp, {"ok": True, "id": 99, "name": "cascade_vel_kd", "applied": 0.5}
        )
        self.assertEqual(rpc.set_calls, [("cascade_vel_kd", 0.5)])

    def test_pos_reset_pulses_param(self) -> None:
        rpc = _FakeRpc()
        resp = handle_control_message(rpc, {"action": "pos_reset"})
        self.assertEqual(
            resp, {"ok": True, "id": 99, "name": "pos_reset", "applied": 1.0}
        )

    def test_unknown_action_reports_error(self) -> None:
        rpc = _FakeRpc()
        resp = handle_control_message(rpc, {"action": "nope"})
        self.assertEqual(resp, {"ok": False, "error": "unknown action 'nope'"})

    def test_rpc_exception_is_captured(self) -> None:
        class _RaisingRpc:
            def get_params(self):
                raise RuntimeError("no reply for seq=1")

        resp = handle_control_message(_RaisingRpc(), {"action": "get_params"})
        self.assertEqual(resp, {"ok": False, "error": "no reply for seq=1"})


if __name__ == "__main__":
    unittest.main()
