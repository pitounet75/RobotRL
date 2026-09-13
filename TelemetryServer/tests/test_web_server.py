import asyncio
import json
import unittest
from pathlib import Path
from typing import List, Tuple

from aiohttp.test_utils import TestClient, TestServer

from telemetry.balance_frame import BalanceFrame
from telemetry.ctrl_params import ControlParamsSnapshot
from telemetry.web_server import TelemetryWebServer, handle_control_message

STATIC_DIR = Path(__file__).resolve().parents[1] / "telemetry" / "web"


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


def _make_frame() -> BalanceFrame:
    return BalanceFrame(
        frame_number=1,
        time_us=1000,
        pitch_rad=0.1,
        pitch_rate_rads=0.0,
        vel_wheel_turns_s=0.0,
        vel_wheel_l_turns_s=0.0,
        vel_wheel_r_turns_s=0.0,
        cmd_torque_nm=0.0,
        cmd_torque_left_nm=0.0,
        cmd_torque_right_nm=0.0,
        u_ff_nm=0.0,
        u_fb_nm=0.0,
        pitch_ref_rad=0.0,
        imu_valid=1,
        estop=0,
        strategy_id=0,
    )


class TelemetryWebServerTests(unittest.TestCase):
    def test_broadcast_reaches_connected_client(self) -> None:
        asyncio.run(self._check_broadcast())

    async def _check_broadcast(self) -> None:
        server = TelemetryWebServer(rpc=None, static_dir=STATIC_DIR)
        client = TestClient(TestServer(server.build_app()))
        await client.start_server()
        server.bind_loop(asyncio.get_running_loop())
        try:
            ws = await client.ws_connect("/ws/telemetry")
            bf = _make_frame()
            await server._broadcast(bf)
            msg = await ws.receive(timeout=1.0)
            self.assertEqual(msg.data, bf.encode())
        finally:
            await client.close()

    def test_control_endpoint_reports_rpc_unavailable(self) -> None:
        asyncio.run(self._check_control_unavailable())

    async def _check_control_unavailable(self) -> None:
        server = TelemetryWebServer(rpc=None, static_dir=STATIC_DIR)
        client = TestClient(TestServer(server.build_app()))
        await client.start_server()
        try:
            ws = await client.ws_connect("/ws/control")
            await ws.send_json({"action": "get_params"})
            msg = await ws.receive(timeout=1.0)
            self.assertEqual(
                json.loads(msg.data),
                {"ok": False, "error": "RPC unavailable (pass --esp32-host)."},
            )
        finally:
            await client.close()


if __name__ == "__main__":
    unittest.main()
