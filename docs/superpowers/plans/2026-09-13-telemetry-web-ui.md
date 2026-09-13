# Telemetry Web UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a browser-based live telemetry dashboard (`--web` mode) to `TelemetryServer`, replacing Matplotlib's per-frame full-redraw approach with a WebSocket + `uPlot` pipeline that comfortably handles the STM32's native 500Hz `BalanceFrame` rate across many channels.

**Architecture:** A new `aiohttp`-based `TelemetryWebServer` serves a no-build HTML/JS frontend and two WebSocket endpoints: `/ws/telemetry` (binary `BalanceFrame` broadcast, server does no throttling) and `/ws/control` (JSON request/response wrapping the existing `SharedRpcClient`). `scripts/run_server.py` gains a `--web` flag that runs this server on the main thread (via `asyncio.run`) instead of the Qt event loop, while the existing UDP receive thread pushes decoded frames into it via `loop.call_soon_threadsafe`. All existing decode/RPC/CSV-recording code is reused unchanged except one additive method on `BalanceFrame`.

**Tech Stack:** Python 3.9, `aiohttp` (asyncio HTTP + WebSocket server, incl. `aiohttp.test_utils` for tests), vendored `uPlot` 1.6.31 (canvas-based charting, no build step), vanilla HTML/CSS/JS, `unittest`/`pytest` (matches existing test style).

**Spec:** `docs/superpowers/specs/2026-09-12-telemetry-web-ui-design.md`

## Global Constraints

- No changes to the wire protocol between ESP32 and `TelemetryServer`, or to `BalanceFrame`/`ControlParamsSnapshot` encodings (spec: Non-goals).
- `udp_receiver.py`, `protocol.py`, `udp_envelope.py`, `ctrl_params.py`, `rpc_mux.py`, `recorder.py` stay behaviorally unchanged (spec: Reused unchanged). `balance_frame.py` gets exactly one additive method (`encode()`).
- `--plot` and `--web` are mutually exclusive within one process; run two processes for both (spec: Architecture).
- No auth/TLS — LAN-local trust model, same as today's UDP link (spec: Non-goals).
- Frontend has no build step: vendored/CDN-pinned libraries and plain JS files only (spec: Frontend).
- Retiring the PyQt5/Matplotlib path is explicitly out of scope for this plan (spec: Non-goals / Follow-up).

---

### Task 1: `BalanceFrame.encode()`

**Files:**
- Modify: `TelemetryServer/telemetry/balance_frame.py`
- Test: `TelemetryServer/tests/test_balance_frame.py`

**Interfaces:**
- Produces: `BalanceFrame.encode() -> bytes` — canonical `BALANCE_FRAME_STRUCT_V2`-layout bytes (56 bytes), used by `TelemetryWebServer` (Task 3) to build the `/ws/telemetry` binary payload, and mirrored byte-for-byte by the browser-side decoder (Task 7).

- [ ] **Step 1: Write the failing test**

Add to `TelemetryServer/tests/test_balance_frame.py` (uses the existing `make_frame` helper already in that file):

```python
    def test_encode_round_trips_through_decode(self) -> None:
        frame = make_frame(frame_number=42, time_us=123456, source_drop_count_mod256=7)

        decoded = BalanceFrame.decode(frame.encode())

        self.assertEqual(decoded, frame)
```

- [ ] **Step 2: Run test to verify it fails**

Run (from `TelemetryServer/`): `python -m pytest tests/test_balance_frame.py::BalanceFrameSanityTests::test_encode_round_trips_through_decode -v`
Expected: FAIL with `AttributeError: 'BalanceFrame' object has no attribute 'encode'`

- [ ] **Step 3: Write minimal implementation**

In `TelemetryServer/telemetry/balance_frame.py`, add this method to the `BalanceFrame` dataclass, directly after `is_sane`:

```python
    def encode(self) -> bytes:
        """Canonical V2-layout bytes (up-converts V1-sourced frames) for the web UI."""
        return BALANCE_FRAME_STRUCT_V2.pack(
            self.frame_number,
            self.time_us,
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
            self.imu_valid,
            self.estop,
            self.strategy_id,
            self.source_drop_count_mod256,
        )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/test_balance_frame.py -v`
Expected: PASS (all tests in the file, including the new one)

- [ ] **Step 5: Commit**

```bash
git add TelemetryServer/telemetry/balance_frame.py TelemetryServer/tests/test_balance_frame.py
git commit -m "feat(telemetry): add BalanceFrame.encode() for the web UI wire format"
```

---

### Task 2: `handle_control_message()` (pure control-bridge logic)

**Files:**
- Create: `TelemetryServer/telemetry/web_server.py`
- Test: `TelemetryServer/tests/test_web_server.py`

**Interfaces:**
- Consumes: `SharedRpcClient.get_params() -> ControlParamsSnapshot` and `SharedRpcClient.set_param(name_or_id: str | int, value: float) -> Tuple[int, str, float]` (both already defined in `telemetry/rpc_mux.py`); `ControlParamsSnapshot.as_dict() -> Dict[str, float | int]` and `.version: int` (already defined in `telemetry/ctrl_params.py`).
- Produces: `handle_control_message(rpc: Optional[SharedRpcClient], msg: Dict[str, Any]) -> Dict[str, Any]`, used by `TelemetryWebServer._control_handler` (Task 3). Response shapes:
  - success get_params: `{"ok": True, "params": {...}, "version": int}`
  - success set_param/pos_reset/heading_reset: `{"ok": True, "id": int, "name": str, "applied": float}`
  - failure (any reason): `{"ok": False, "error": str}`

- [ ] **Step 1: Write the failing tests**

Create `TelemetryServer/tests/test_web_server.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/test_web_server.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'telemetry.web_server'`

- [ ] **Step 3: Write minimal implementation**

Create `TelemetryServer/telemetry/web_server.py`:

```python
"""Web dashboard server: binary telemetry broadcast + JSON control bridge."""

from __future__ import annotations

from typing import Any, Dict, Optional

from telemetry.rpc_mux import SharedRpcClient


def handle_control_message(
    rpc: Optional[SharedRpcClient], msg: Dict[str, Any]
) -> Dict[str, Any]:
    """Run one /ws/control request against rpc; never raises."""
    if rpc is None:
        return {"ok": False, "error": "RPC unavailable (pass --esp32-host)."}

    action = msg.get("action")
    try:
        if action == "get_params":
            snap = rpc.get_params()
            return {"ok": True, "params": snap.as_dict(), "version": snap.version}
        if action == "set_param":
            param_id, name, applied = rpc.set_param(msg["name"], float(msg["value"]))
            return {"ok": True, "id": param_id, "name": name, "applied": applied}
        if action in ("pos_reset", "heading_reset"):
            param_id, name, applied = rpc.set_param(action, 1.0)
            return {"ok": True, "id": param_id, "name": name, "applied": applied}
        return {"ok": False, "error": f"unknown action {action!r}"}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/test_web_server.py -v`
Expected: PASS (6 tests)

- [ ] **Step 5: Commit**

```bash
git add TelemetryServer/telemetry/web_server.py TelemetryServer/tests/test_web_server.py
git commit -m "feat(telemetry): add pure /ws/control message handler"
```

---

### Task 3: `TelemetryWebServer` (aiohttp static + WebSocket server)

**Files:**
- Modify: `TelemetryServer/telemetry/web_server.py`
- Modify: `TelemetryServer/tests/test_web_server.py`
- Modify: `TelemetryServer/requirements.txt`
- Create: `TelemetryServer/telemetry/web/index.html` (minimal placeholder; fully built out in Task 6)

**Interfaces:**
- Consumes: `handle_control_message` (Task 2); `BalanceFrame.encode() -> bytes` (Task 1).
- Produces:
  - `TelemetryWebServer(rpc: Optional[SharedRpcClient], static_dir: Path)`
  - `TelemetryWebServer.build_app() -> aiohttp.web.Application`
  - `TelemetryWebServer.bind_loop(loop: asyncio.AbstractEventLoop) -> None`
  - `TelemetryWebServer.push_balance_frame_threadsafe(bf: BalanceFrame) -> None` — call from any thread; used by `scripts/run_server.py`'s rx thread (Task 5).
  - `async TelemetryWebServer.run(host: str, port: int) -> None` — blocks until cancelled; used by `scripts/run_server.py`'s main dispatch (Task 5).

- [ ] **Step 1: Write the failing tests**

Add `aiohttp>=3.9,<4` to `TelemetryServer/requirements.txt` (this is a dependency-only change, not a test, but the tests below import it so install it now: `pip install -r requirements.txt`).

Create `TelemetryServer/telemetry/web/index.html` (placeholder; Task 6 replaces this with the real page — `aiohttp`'s static-file route requires the directory to exist when the app is built):

```html
<!doctype html>
<title>RobotRL Telemetry</title>
<p>Web UI coming soon.</p>
```

Replace the contents of `TelemetryServer/tests/test_web_server.py` with (keeps the Task 2 tests, adds server-level ones):

```python
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
```

- [ ] **Step 2: Run tests to verify the new ones fail**

Run: `python -m pytest tests/test_web_server.py -v`
Expected: the 6 `HandleControlMessageTests` PASS (unchanged from Task 2); the 2 new `TelemetryWebServerTests` FAIL with `ImportError: cannot import name 'TelemetryWebServer'`

- [ ] **Step 3: Write minimal implementation**

Append to `TelemetryServer/telemetry/web_server.py` (keep `handle_control_message` from Task 2 unchanged; add these imports at the top and this class at the bottom):

```python
import asyncio
import json
from pathlib import Path
from typing import Set

from aiohttp import WSMsgType, web

from telemetry.balance_frame import BalanceFrame
```

```python
class TelemetryWebServer:
    """Serves the web UI, broadcasts BalanceFrames, and bridges control RPC."""

    def __init__(self, rpc: Optional[SharedRpcClient], static_dir: Path) -> None:
        self._rpc = rpc
        self._static_dir = static_dir
        self._telemetry_clients: Set[web.WebSocketResponse] = set()
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def build_app(self) -> web.Application:
        app = web.Application()
        app.router.add_get("/ws/telemetry", self._telemetry_handler)
        app.router.add_get("/ws/control", self._control_handler)
        app.router.add_static("/", self._static_dir, show_index=True)
        return app

    async def _telemetry_handler(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._telemetry_clients.add(ws)
        try:
            async for _ in ws:
                pass  # telemetry socket is broadcast-only
        finally:
            self._telemetry_clients.discard(ws)
        return ws

    async def _control_handler(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        loop = asyncio.get_running_loop()
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                req = json.loads(msg.data)
            except ValueError:
                await ws.send_json({"ok": False, "error": "invalid JSON"})
                continue
            resp = await loop.run_in_executor(
                None, handle_control_message, self._rpc, req
            )
            await ws.send_json(resp)
        return ws

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    def push_balance_frame_threadsafe(self, bf: BalanceFrame) -> None:
        """Call from any thread; schedules a broadcast on the server's own loop."""
        if self._loop is None:
            return
        self._loop.call_soon_threadsafe(self._schedule_broadcast, bf)

    def _schedule_broadcast(self, bf: BalanceFrame) -> None:
        asyncio.ensure_future(self._broadcast(bf))

    async def _broadcast(self, bf: BalanceFrame) -> None:
        if not self._telemetry_clients:
            return
        data = bf.encode()
        dead = []
        for ws in list(self._telemetry_clients):
            try:
                await ws.send_bytes(data)
            except ConnectionResetError:
                dead.append(ws)
        for ws in dead:
            self._telemetry_clients.discard(ws)

    async def run(self, host: str, port: int) -> None:
        self.bind_loop(asyncio.get_running_loop())
        runner = web.AppRunner(self.build_app())
        await runner.setup()
        site = web.TCPSite(runner, host, port)
        await site.start()
        display_host = "localhost" if host == "0.0.0.0" else host
        print(f"Web UI: http://{display_host}:{port}/")
        try:
            await asyncio.Event().wait()
        finally:
            await runner.cleanup()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/test_web_server.py -v`
Expected: PASS (8 tests)

- [ ] **Step 5: Commit**

```bash
git add TelemetryServer/telemetry/web_server.py TelemetryServer/tests/test_web_server.py TelemetryServer/requirements.txt TelemetryServer/telemetry/web/index.html
git commit -m "feat(telemetry): add TelemetryWebServer (aiohttp static + WS broadcast/control)"
```

---

### Task 4: Synthetic `BalanceFrame` generator (test traffic without hardware)

**Files:**
- Create: `TelemetryServer/telemetry/synthetic_balance_frame.py`
- Create: `TelemetryServer/scripts/fake_telemetry_feed.py`
- Test: `TelemetryServer/tests/test_synthetic_balance_frame.py`

**Interfaces:**
- Consumes: `BALANCE_FRAME_STRUCT_V2` (`telemetry/balance_frame.py`); `build_frame`, `TELEM_MSG_BALANCE_FRAME` (`telemetry/protocol.py`); `encode_udp_datagram` (`telemetry/udp_envelope.py`).
- Produces: `make_payload(frame_number: int, t: float) -> bytes` — a `BalanceFrame`-shaped, physically-plausible payload, used by `scripts/fake_telemetry_feed.py` and by manual verification in Tasks 5-9 (this is how the web UI is exercised end-to-end without the real robot).

- [ ] **Step 1: Write the failing test**

Create `TelemetryServer/tests/test_synthetic_balance_frame.py`:

```python
import unittest

from telemetry.balance_frame import BalanceFrame
from telemetry.protocol import TELEM_MSG_BALANCE_FRAME, FrameParser, build_frame
from telemetry.synthetic_balance_frame import make_payload
from telemetry.udp_envelope import decode_udp_datagram, encode_udp_datagram


class SyntheticBalanceFrameTests(unittest.TestCase):
    def test_generated_payload_decodes_end_to_end(self) -> None:
        payload = make_payload(frame_number=5, t=1.25)
        tm_frame = build_frame(1, TELEM_MSG_BALANCE_FRAME, payload)
        datagram = encode_udp_datagram(tm_frame, sequence=0, frame_count=1)

        envelope = decode_udp_datagram(datagram)
        frames = list(FrameParser().feed(envelope.payload))

        self.assertEqual(len(frames), 1)
        self.assertTrue(frames[0].ok)
        bf = BalanceFrame.decode(frames[0].payload)
        self.assertEqual(bf.frame_number, 5)
        self.assertTrue(bf.is_sane())


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_synthetic_balance_frame.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'telemetry.synthetic_balance_frame'`

- [ ] **Step 3: Write minimal implementation**

Create `TelemetryServer/telemetry/synthetic_balance_frame.py`:

```python
"""Synthetic BalanceFrame payloads for exercising the web UI without hardware."""

from __future__ import annotations

import math

from telemetry.balance_frame import BALANCE_FRAME_STRUCT_V2


def make_payload(frame_number: int, t: float) -> bytes:
    """A slowly oscillating, physically-plausible BalanceFrame payload at time t (s)."""
    pitch = 0.05 * math.sin(t)
    return BALANCE_FRAME_STRUCT_V2.pack(
        frame_number & 0xFFFFFFFF,
        int(t * 1_000_000) & 0xFFFFFFFF,
        pitch,
        0.05 * math.cos(t),
        0.2,
        0.2,
        0.2,
        0.01 * math.sin(t),
        0.01 * math.sin(t),
        0.01 * math.sin(t),
        0.0,
        0.01,
        0.0,
        1,
        0,
        0,
        0,
    )
```

Create `TelemetryServer/scripts/fake_telemetry_feed.py`:

```python
#!/usr/bin/env python3
"""CLI: send synthetic BalanceFrame UDP traffic to a local TelemetryServer."""

from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from telemetry.protocol import TELEM_MSG_BALANCE_FRAME, build_frame
from telemetry.synthetic_balance_frame import make_payload
from telemetry.udp_envelope import encode_udp_datagram


def main() -> int:
    p = argparse.ArgumentParser(description="Synthetic BalanceFrame UDP source")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--hz", type=float, default=500.0)
    p.add_argument("--duration-s", type=float, default=0.0, help="0 = run forever")
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / args.hz
    t_start = time.time()
    frame_number = 0
    try:
        while args.duration_s <= 0 or time.time() - t_start < args.duration_s:
            t = time.time() - t_start
            payload = make_payload(frame_number, t)
            tm_frame = build_frame(1, TELEM_MSG_BALANCE_FRAME, payload)
            datagram = encode_udp_datagram(tm_frame, sequence=frame_number, frame_count=1)
            sock.sendto(datagram, (args.host, args.port))
            frame_number += 1
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/test_synthetic_balance_frame.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add TelemetryServer/telemetry/synthetic_balance_frame.py TelemetryServer/scripts/fake_telemetry_feed.py TelemetryServer/tests/test_synthetic_balance_frame.py
git commit -m "feat(telemetry): add synthetic BalanceFrame UDP generator for hardware-free testing"
```

---

### Task 5: Wire `--web` into `scripts/run_server.py`

**Files:**
- Modify: `TelemetryServer/scripts/run_server.py`
- Test: `TelemetryServer/tests/test_run_server_cli.py`

**Interfaces:**
- Consumes: `TelemetryWebServer` (Task 3); `SharedRpcClient` (existing).
- Produces: `main(argv: Optional[List[str]] = None) -> int` (widened signature, still callable with no args as before); CLI flags `--web`, `--web-host`, `--web-port`.

- [ ] **Step 1: Write the failing test**

Create `TelemetryServer/tests/test_run_server_cli.py`:

```python
import sys
import unittest
from contextlib import redirect_stderr
from io import StringIO
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts import run_server


class RunServerCliTests(unittest.TestCase):
    def test_plot_and_web_are_mutually_exclusive(self) -> None:
        with redirect_stderr(StringIO()):
            with self.assertRaises(SystemExit):
                run_server.main(["--plot", "--web"])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_run_server_cli.py -v`
Expected: FAIL — either a `TypeError` (`main() takes 0 positional arguments but 1 was given`) or the test's `assertRaises(SystemExit)` not being raised, since `main()` doesn't accept `argv` yet and has no mutual-exclusion check.

- [ ] **Step 3: Write minimal implementation**

In `TelemetryServer/scripts/run_server.py`:

Add near the top, alongside the existing imports:

```python
import asyncio
```

and

```python
from telemetry.web_server import TelemetryWebServer
```

Change the function signature and parse call:

```python
def main(argv: Optional[list[str]] = None) -> int:
```

(`Optional` is already imported at the top of this file.)

```python
    args = p.parse_args(argv)
```

Add these three arguments right after the existing `p.add_argument("--plot", ...)` line:

```python
    p.add_argument("--web", action="store_true", help="Live web dashboard (browser)")
    p.add_argument("--web-host", default="0.0.0.0", help="Web UI bind host")
    p.add_argument("--web-port", type=int, default=8765, help="Web UI bind port")
```

Immediately after `args = p.parse_args(argv)`, add:

```python
    if args.plot and args.web:
        p.error("--plot and --web are mutually exclusive; run two processes for both.")
```

Change the RPC-creation condition (currently `if plotter is not None:` inside the `if args.esp32_host:` block) to also cover `--web`:

```python
    if args.esp32_host:
        receiver.subscribe()
        print(f"Subscribe ping sent to {args.esp32_host}:{args.esp32_port} (repeats every 5 s)")
        if plotter is not None or args.web:
            rpc = SharedRpcClient(receiver)
```

Right after that `if args.esp32_host:` block, add:

```python
    web_server: Optional[TelemetryWebServer] = None
    if args.web:
        web_server = TelemetryWebServer(rpc=rpc, static_dir=ROOT / "telemetry" / "web")
```

In `rx_loop`, change:

```python
                    host_t = time.time()
                    with lock:
                        if recorder is not None:
                            recorder.write(host_t, bf)
                        if plotter is not None:
                            plotter.add(host_t, bf)
```

to:

```python
                    host_t = time.time()
                    with lock:
                        if recorder is not None:
                            recorder.write(host_t, bf)
                        if plotter is not None:
                            plotter.add(host_t, bf)
                    if web_server is not None:
                        web_server.push_balance_frame_threadsafe(bf)
```

Finally, change the main dispatch block:

```python
    try:
        if plotter is not None:
            from telemetry.app_window import run_telemetry_window

            run_telemetry_window(plotter, rpc=rpc)
        else:
            while True:
                time.sleep(1.0)
```

to:

```python
    try:
        if plotter is not None:
            from telemetry.app_window import run_telemetry_window

            run_telemetry_window(plotter, rpc=rpc)
        elif web_server is not None:
            asyncio.run(web_server.run(args.web_host, args.web_port))
        else:
            while True:
                time.sleep(1.0)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/test_run_server_cli.py -v`
Expected: PASS

Then run the full suite to confirm nothing else broke: `python -m pytest -q`
Expected: all tests PASS

- [ ] **Step 5: Commit**

```bash
git add TelemetryServer/scripts/run_server.py TelemetryServer/tests/test_run_server_cli.py
git commit -m "feat(telemetry): wire --web mode into run_server.py"
```

---

### Task 6: Frontend skeleton (vendored uPlot, page layout, styling)

**Files:**
- Create: `TelemetryServer/telemetry/web/vendor/uplot/uPlot.iife.min.js`
- Create: `TelemetryServer/telemetry/web/vendor/uplot/uPlot.min.css`
- Modify: `TelemetryServer/telemetry/web/index.html` (replaces Task 3's placeholder)
- Create: `TelemetryServer/telemetry/web/style.css`

**Interfaces:**
- Produces: the DOM elements Task 7 (`app.js`) and Task 8 (`control.js`) attach to: `#connection-status`, `#chart-pitch`, `#chart-torque`, `#chart-velocity`, `#chart-flags`, `#speed-slider`/`#speed-label`/`#speed-zero`, `#heading-slider`/`#heading-label`/`#heading-zero`, `#gains-refresh`/`#gains-apply-changed`/`#gains-apply-all`/`#gains-pos-reset`/`#gains-heading-reset`/`#gains-status`/`#gains-form`. Also loads the global `uPlot` class from the vendored script.

- [ ] **Step 1: Vendor uPlot**

Run (from `TelemetryServer/`):

```bash
mkdir -p telemetry/web/vendor/uplot
curl -sf -o telemetry/web/vendor/uplot/uPlot.iife.min.js https://cdn.jsdelivr.net/npm/uplot@1.6.31/dist/uPlot.iife.min.js
curl -sf -o telemetry/web/vendor/uplot/uPlot.min.css https://cdn.jsdelivr.net/npm/uplot@1.6.31/dist/uPlot.min.css
```

Expected: both files download successfully (non-empty; `uPlot.iife.min.js` is tens of KB).

- [ ] **Step 2: Replace `index.html`**

Replace the contents of `TelemetryServer/telemetry/web/index.html`:

```html
<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <title>RobotRL Telemetry</title>
    <link rel="stylesheet" href="vendor/uplot/uPlot.min.css" />
    <link rel="stylesheet" href="style.css" />
  </head>
  <body>
    <header>
      <h1>RobotRL Telemetry</h1>
      <span id="connection-status" class="status-bad">disconnected</span>
    </header>

    <main>
      <section id="charts">
        <div id="chart-pitch" class="chart"></div>
        <div id="chart-torque" class="chart"></div>
        <div id="chart-velocity" class="chart"></div>
        <div id="chart-flags" class="chart"></div>
      </section>

      <section id="controls">
        <div class="control-block">
          <label for="speed-slider">Speed</label>
          <input type="range" id="speed-slider" min="-4000" max="4000" value="0" step="10" />
          <span id="speed-label">+0.00 m/s (+0.000 motor turn/s)</span>
          <button id="speed-zero">0</button>
        </div>
        <div class="control-block">
          <label for="heading-slider">Heading</label>
          <input type="range" id="heading-slider" min="-180" max="180" value="0" step="1" />
          <span id="heading-label">+0&deg; (+0.000 rad)</span>
          <button id="heading-zero">0</button>
        </div>

        <div class="control-block">
          <button id="gains-refresh">Refresh from robot</button>
          <button id="gains-apply-changed">Apply changed</button>
          <button id="gains-apply-all">Apply all</button>
          <button id="gains-pos-reset">pos_reset</button>
          <button id="gains-heading-reset">heading_reset</button>
          <span id="gains-status"></span>
        </div>
        <div id="gains-form"></div>
      </section>
    </main>

    <script src="vendor/uplot/uPlot.iife.min.js"></script>
    <script src="app.js" defer></script>
    <script src="control.js" defer></script>
  </body>
</html>
```

- [ ] **Step 3: Create `style.css`**

Create `TelemetryServer/telemetry/web/style.css`:

```css
:root {
  color-scheme: dark;
  --bg: #14161a;
  --panel: #1d2026;
  --text: #e8e8ea;
  --muted: #9aa0a8;
  --ok: #33c17a;
  --bad: #e0555a;
  --border: #2c3038;
}

* {
  box-sizing: border-box;
}

body {
  margin: 0;
  background: var(--bg);
  color: var(--text);
  font-family: system-ui, sans-serif;
  padding: 16px;
}

header {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-bottom: 16px;
}

header h1 {
  font-size: 1.1rem;
  margin: 0;
}

.status-ok {
  color: var(--ok);
}

.status-bad {
  color: var(--bad);
}

#charts {
  display: grid;
  grid-template-columns: repeat(2, minmax(280px, 1fr));
  gap: 12px;
}

.chart {
  background: var(--panel);
  border: 1px solid var(--border);
  border-radius: 8px;
  padding: 8px;
  overflow-x: auto;
}

#controls {
  margin-top: 16px;
  background: var(--panel);
  border: 1px solid var(--border);
  border-radius: 8px;
  padding: 12px;
}

.control-block {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 10px;
  flex-wrap: wrap;
}

.control-block input[type="range"] {
  flex: 1;
  min-width: 160px;
}

#gains-form {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(240px, 1fr));
  gap: 6px;
  max-height: 320px;
  overflow-y: auto;
}

.gains-row {
  display: flex;
  justify-content: space-between;
  gap: 6px;
  font-size: 0.85rem;
}

.gains-row input {
  width: 100px;
  background: var(--bg);
  color: var(--text);
  border: 1px solid var(--border);
  border-radius: 4px;
  padding: 2px 4px;
}
```

- [ ] **Step 4: Verify the page is served**

Run (from `TelemetryServer/`, in the background): `python scripts/run_server.py --web &`
Then: `curl -sf -o /dev/null -w "%{http_code}\n" http://127.0.0.1:8765/` — expected `200`
Then: `curl -sf -o /dev/null -w "%{http_code}\n" http://127.0.0.1:8765/vendor/uplot/uPlot.iife.min.js` — expected `200`
Then: `curl -sf -o /dev/null -w "%{http_code}\n" http://127.0.0.1:8765/style.css` — expected `200`
Stop the server (`kill %1` or Ctrl-C).

(A human should also open `http://localhost:8765/` in a browser at this point and confirm the dark-themed layout renders with 4 empty chart panels and the control section below — no chart data yet, that's Tasks 7-8.)

- [ ] **Step 5: Commit**

```bash
git add TelemetryServer/telemetry/web/
git commit -m "feat(telemetry): add web UI page skeleton and vendored uPlot"
```

---

### Task 7: `app.js` — telemetry WebSocket client and live charts

**Files:**
- Create: `TelemetryServer/telemetry/web/app.js`

**Interfaces:**
- Consumes: `/ws/telemetry` binary messages (56-byte `BalanceFrame` layout from `BalanceFrame.encode()`, Task 1); the global `uPlot` class (vendored, Task 6); DOM elements from Task 6 (`#connection-status`, `#chart-pitch`, `#chart-torque`, `#chart-velocity`, `#chart-flags`).
- Produces: nothing consumed by later tasks (control.js, Task 8, is independent).

- [ ] **Step 1: Create `app.js`**

Create `TelemetryServer/telemetry/web/app.js`:

```js
"use strict";

const MAX_POINTS = 5000;
const WS_TELEMETRY_PATH = "/ws/telemetry";
const FRAME_BYTES = 56;

function decodeBalanceFrame(buf) {
  const dv = new DataView(buf);
  return {
    frame_number: dv.getUint32(0, true),
    time_us: dv.getUint32(4, true),
    pitch_rad: dv.getFloat32(8, true),
    pitch_rate_rads: dv.getFloat32(12, true),
    vel_wheel_turns_s: dv.getFloat32(16, true),
    vel_wheel_l_turns_s: dv.getFloat32(20, true),
    vel_wheel_r_turns_s: dv.getFloat32(24, true),
    cmd_torque_nm: dv.getFloat32(28, true),
    cmd_torque_left_nm: dv.getFloat32(32, true),
    cmd_torque_right_nm: dv.getFloat32(36, true),
    u_ff_nm: dv.getFloat32(40, true),
    u_fb_nm: dv.getFloat32(44, true),
    pitch_ref_rad: dv.getFloat32(48, true),
    imu_valid: dv.getUint8(52),
    estop: dv.getUint8(53),
    strategy_id: dv.getUint8(54),
    source_drop_count_mod256: dv.getUint8(55),
  };
}

const BUFFER_KEYS = [
  "pitch_rad", "pitch_deg", "pitch_rate",
  "cmd_torque", "cmd_torque_l", "cmd_torque_r", "u_ff", "u_fb",
  "vel_l", "vel_r", "estop", "imu_valid",
];

class ChannelBuffers {
  constructor(keys) {
    this.keys = keys;
    this.t = [];
    this.y = {};
    for (const k of keys) this.y[k] = [];
    this.t0 = null;
  }

  push(frame, tRecvSec) {
    if (this.t0 === null) this.t0 = tRecvSec;
    this.t.push(tRecvSec - this.t0);
    this.y.pitch_rad.push(frame.pitch_rad);
    this.y.pitch_deg.push((frame.pitch_rad * 180) / Math.PI);
    this.y.pitch_rate.push(frame.pitch_rate_rads);
    this.y.cmd_torque.push(frame.cmd_torque_nm);
    this.y.cmd_torque_l.push(frame.cmd_torque_left_nm);
    this.y.cmd_torque_r.push(frame.cmd_torque_right_nm);
    this.y.u_ff.push(frame.u_ff_nm);
    this.y.u_fb.push(frame.u_fb_nm);
    this.y.vel_l.push(frame.vel_wheel_l_turns_s);
    this.y.vel_r.push(frame.vel_wheel_r_turns_s);
    this.y.estop.push(frame.estop);
    this.y.imu_valid.push(frame.imu_valid);
    if (this.t.length > MAX_POINTS) {
      this.t.shift();
      for (const k of this.keys) this.y[k].shift();
    }
  }

  series(keys) {
    return [this.t, ...keys.map((k) => this.y[k])];
  }
}

const buffers = new ChannelBuffers(BUFFER_KEYS);

function makeChart(containerId, title, seriesKeys, labels) {
  const opts = {
    title,
    width: 600,
    height: 220,
    series: [
      {},
      ...seriesKeys.map((k, i) => ({
        label: labels[i],
        stroke: `hsl(${(i * 67) % 360},70%,55%)`,
      })),
    ],
    scales: { x: { time: false } },
    axes: [{ label: "s" }, {}],
  };
  const plot = new uPlot(opts, buffers.series(seriesKeys), document.getElementById(containerId));
  return { plot, seriesKeys };
}

const charts = [
  makeChart("chart-pitch", "Pitch", ["pitch_rad", "pitch_rate"], ["pitch_rad", "pitch_rate"]),
  makeChart(
    "chart-torque",
    "Torque (Nm)",
    ["cmd_torque", "cmd_torque_l", "cmd_torque_r", "u_ff", "u_fb"],
    ["cmd_torque", "cmd_torque_l", "cmd_torque_r", "u_ff", "u_fb"]
  ),
  makeChart("chart-velocity", "Wheel velocity (turn/s)", ["vel_l", "vel_r"], ["vel_l", "vel_r"]),
  makeChart("chart-flags", "Flags", ["estop", "imu_valid"], ["estop", "imu_valid"]),
];

let redrawScheduled = false;

function scheduleRedraw() {
  if (redrawScheduled) return;
  redrawScheduled = true;
  requestAnimationFrame(() => {
    redrawScheduled = false;
    for (const { plot, seriesKeys } of charts) {
      plot.setData(buffers.series(seriesKeys));
    }
  });
}

function setStatus(connected) {
  const el = document.getElementById("connection-status");
  if (!el) return;
  el.textContent = connected ? "connected" : "disconnected";
  el.className = connected ? "status-ok" : "status-bad";
}

function connectTelemetry() {
  const url = `ws://${window.location.host}${WS_TELEMETRY_PATH}`;
  const ws = new WebSocket(url);
  ws.binaryType = "arraybuffer";
  ws.onopen = () => setStatus(true);
  ws.onclose = () => {
    setStatus(false);
    setTimeout(connectTelemetry, 1000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (event) => {
    if (event.data.byteLength !== FRAME_BYTES) return;
    const frame = decodeBalanceFrame(event.data);
    buffers.push(frame, performance.now() / 1000);
    scheduleRedraw();
  };
}

connectTelemetry();
```

- [ ] **Step 2: Verify end-to-end with the synthetic generator**

Run (from `TelemetryServer/`, three separate processes):

```bash
python scripts/run_server.py --web &
python scripts/fake_telemetry_feed.py --hz 500 &
```

Then check the telemetry socket is actually delivering correctly-shaped frames (no browser needed for this check — confirms the wire format end-to-end):

```bash
python -c "
import asyncio
from aiohttp import ClientSession

async def main():
    async with ClientSession() as session:
        async with session.ws_connect('http://127.0.0.1:8765/ws/telemetry') as ws:
            msg = await ws.receive(timeout=2.0)
            assert len(msg.data) == 56, len(msg.data)
            print('OK', len(msg.data), 'bytes')

asyncio.run(main())
"
```

Expected output: `OK 56 bytes`

Stop both background processes (`kill %1 %2`).

A human should also open `http://localhost:8765/` in a browser while `run_server.py --web` and `fake_telemetry_feed.py` are running, and confirm the 4 charts animate smoothly with the oscillating synthetic signal and the connection badge reads "connected".

- [ ] **Step 3: Commit**

```bash
git add TelemetryServer/telemetry/web/app.js
git commit -m "feat(telemetry): add live telemetry WebSocket client and uPlot charts"
```

---

### Task 8: `control.js` — Gains panel + speed/heading sliders

**Files:**
- Create: `TelemetryServer/telemetry/web/control.js`

**Interfaces:**
- Consumes: `/ws/control` JSON protocol (Task 2/3: `get_params`, `set_param`, `pos_reset`, `heading_reset` actions and their response shapes); DOM elements from Task 6.

- [ ] **Step 1: Create `control.js`**

Create `TelemetryServer/telemetry/web/control.js`:

```js
"use strict";

const ACTION_PARAMS = new Set(["pos_reset", "heading_reset", "heading_inc", "heading_dec"]);
const GEAR_WHEEL_PER_MOTOR = 3.0 / 16.0;

let controlWs = null;
let loadedValues = {};
let wheelRadiusM = 0.04;
let speedSendTimer = null;
let headingSendTimer = null;

function connectControl() {
  const url = `ws://${window.location.host}/ws/control`;
  controlWs = new WebSocket(url);
  controlWs.onopen = () => refreshGains();
  controlWs.onclose = () => setTimeout(connectControl, 1000);
}

function sendControl(action, extra) {
  return new Promise((resolve, reject) => {
    if (!controlWs || controlWs.readyState !== WebSocket.OPEN) {
      reject(new Error("control socket not connected"));
      return;
    }
    const handler = (event) => {
      controlWs.removeEventListener("message", handler);
      const resp = JSON.parse(event.data);
      if (resp.ok) resolve(resp);
      else reject(new Error(resp.error));
    };
    controlWs.addEventListener("message", handler);
    controlWs.send(JSON.stringify(Object.assign({ action }, extra)));
  });
}

function setGainsStatus(text, isError) {
  const el = document.getElementById("gains-status");
  el.textContent = text;
  el.className = isError ? "status-bad" : "";
}

function buildGainsForm(params) {
  const form = document.getElementById("gains-form");
  form.innerHTML = "";
  loadedValues = {};
  const names = Object.keys(params).sort();
  for (const name of names) {
    if (ACTION_PARAMS.has(name)) continue;
    const row = document.createElement("div");
    row.className = "gains-row";
    const label = document.createElement("label");
    label.textContent = name;
    label.htmlFor = `gain-${name}`;
    const input = document.createElement("input");
    input.type = "text";
    input.id = `gain-${name}`;
    input.dataset.name = name;
    const val = params[name];
    input.value =
      typeof val === "number" && !Number.isInteger(val) ? val.toPrecision(6) : String(val);
    loadedValues[name] = input.value;
    row.appendChild(label);
    row.appendChild(input);
    form.appendChild(row);
  }
}

async function refreshGains() {
  try {
    const resp = await sendControl("get_params");
    buildGainsForm(resp.params);
    setGainsStatus(`Loaded snapshot version=${resp.version}`, false);
  } catch (err) {
    setGainsStatus(`Refresh failed: ${err.message}`, true);
  }
}

async function applyGains(changedOnly) {
  const inputs = document.querySelectorAll("#gains-form input");
  let applied = 0;
  for (const input of inputs) {
    const name = input.dataset.name;
    const text = input.value.trim();
    if (!text) continue;
    if (changedOnly && loadedValues[name] === text) continue;
    const value = Number(text);
    if (Number.isNaN(value)) {
      setGainsStatus(`Invalid number for ${name}: ${text}`, true);
      return;
    }
    try {
      const resp = await sendControl("set_param", { name, value });
      input.value = String(resp.applied);
      loadedValues[name] = input.value;
      applied += 1;
    } catch (err) {
      setGainsStatus(`Apply stopped after ${applied}: ${err.message}`, true);
      return;
    }
  }
  setGainsStatus(applied ? `Applied ${applied} param(s).` : "Nothing to apply.", false);
}

async function pulseAction(action) {
  try {
    await sendControl(action);
    setGainsStatus(`${action} pulsed.`, false);
  } catch (err) {
    setGainsStatus(`${action} failed: ${err.message}`, true);
  }
}

document.getElementById("gains-refresh").addEventListener("click", refreshGains);
document.getElementById("gains-apply-changed").addEventListener("click", () => applyGains(true));
document.getElementById("gains-apply-all").addEventListener("click", () => applyGains(false));
document.getElementById("gains-pos-reset").addEventListener("click", () => pulseAction("pos_reset"));
document
  .getElementById("gains-heading-reset")
  .addEventListener("click", () => pulseAction("heading_reset"));

function motorTurnsPerSToMps(turnsPerS) {
  return turnsPerS * GEAR_WHEEL_PER_MOTOR * 2 * Math.PI * wheelRadiusM;
}

function mpsToMotorTurnsPerS(mps) {
  const wheelMPerMotorTurn = GEAR_WHEEL_PER_MOTOR * 2 * Math.PI * wheelRadiusM;
  return wheelMPerMotorTurn <= 1e-12 ? 0 : mps / wheelMPerMotorTurn;
}

function formatSpeedLabel(mps) {
  return `${mps.toFixed(2)} m/s (${mpsToMotorTurnsPerS(mps).toFixed(3)} motor turn/s)`;
}

function formatHeadingLabel(deg) {
  return `${deg.toFixed(0)}\u00b0 (${((deg * Math.PI) / 180).toFixed(3)} rad)`;
}

const speedSlider = document.getElementById("speed-slider");
const speedLabel = document.getElementById("speed-label");
speedSlider.addEventListener("input", () => {
  const mmS = Number(speedSlider.value);
  speedLabel.textContent = formatSpeedLabel(mmS / 1000);
  clearTimeout(speedSendTimer);
  speedSendTimer = setTimeout(() => flushSpeed(mmS), 80);
});
document.getElementById("speed-zero").addEventListener("click", () => {
  speedSlider.value = "0";
  speedSlider.dispatchEvent(new Event("input"));
});

async function flushSpeed(mmS) {
  try {
    const turnsPerS = mpsToMotorTurnsPerS(mmS / 1000);
    const resp = await sendControl("set_param", { name: "vel_ref_turns_s", value: turnsPerS });
    speedLabel.textContent = formatSpeedLabel(motorTurnsPerSToMps(resp.applied));
  } catch (err) {
    speedLabel.textContent = `SET failed: ${err.message}`;
  }
}

const headingSlider = document.getElementById("heading-slider");
const headingLabel = document.getElementById("heading-label");
headingSlider.addEventListener("input", () => {
  const deg = Number(headingSlider.value);
  headingLabel.textContent = formatHeadingLabel(deg);
  clearTimeout(headingSendTimer);
  headingSendTimer = setTimeout(() => flushHeading(deg), 80);
});
document.getElementById("heading-zero").addEventListener("click", () => {
  headingSlider.value = "0";
  headingSlider.dispatchEvent(new Event("input"));
});

async function flushHeading(deg) {
  try {
    const rad = (deg * Math.PI) / 180;
    const resp = await sendControl("set_param", { name: "heading_ref_rad", value: rad });
    const appliedDeg = (resp.applied * 180) / Math.PI;
    headingLabel.textContent = formatHeadingLabel(appliedDeg);
  } catch (err) {
    headingLabel.textContent = `SET failed: ${err.message}`;
  }
}

connectControl();
```

- [ ] **Step 2: Verify the control channel end-to-end (no hardware needed for the unavailable-RPC path)**

Run (from `TelemetryServer/`, background): `python scripts/run_server.py --web &`

```bash
python -c "
import asyncio, json
from aiohttp import ClientSession

async def main():
    async with ClientSession() as session:
        async with session.ws_connect('http://127.0.0.1:8765/ws/control') as ws:
            await ws.send_json({'action': 'get_params'})
            msg = await ws.receive(timeout=2.0)
            resp = json.loads(msg.data)
            assert resp == {'ok': False, 'error': 'RPC unavailable (pass --esp32-host).'}, resp
            print('OK', resp)

asyncio.run(main())
"
```

Expected output: `OK {'ok': False, 'error': 'RPC unavailable (pass --esp32-host).'}` (matches — no `--esp32-host` was passed to `run_server.py`).

Stop the background server (`kill %1`).

Full validation of `set_param`/`pos_reset`/`heading_reset`/Gains-form-population against a real `ControlParamsSnapshot` requires the ESP32/STM32 link (`--esp32-host`), same as the existing Qt Gains panel does today — verify that manually against real hardware, or skip if hardware isn't available in this environment; the message-shape logic itself is already covered by Task 2/3's automated tests.

- [ ] **Step 3: Commit**

```bash
git add TelemetryServer/telemetry/web/control.js
git commit -m "feat(telemetry): add web Gains panel and speed/heading controls"
```

---

### Task 9: Documentation and final end-to-end check

**Files:**
- Modify: `TelemetryServer/README.md`

**Interfaces:**
- None (documentation only).

- [ ] **Step 1: Update the README**

In `TelemetryServer/README.md`, add a new section right after the existing `## Run (live plot + CSV)` section (before `## Control params (GET / SET)`):

```markdown
## Run (web dashboard)

```powershell
cd H:\Projects\RobotRL\TelemetryServer
.\venv\Scripts\Activate.ps1
python scripts/run_server.py --esp32-host 192.168.1.7 --web
```

Open `http://localhost:8765/` in a browser. `--web` and `--plot` are mutually
exclusive per process — run two `run_server.py` processes if you want both.

- `--web-host` / `--web-port`: bind address/port for the web UI (default
  `0.0.0.0:8765` — reachable from other devices on the LAN).
- Without `--esp32-host`, telemetry streaming still works; the Gains panel and
  speed/heading sliders report "RPC unavailable" (same as `--plot` without
  `--esp32-host`).
- No hardware needed to try it: `python scripts/fake_telemetry_feed.py --hz 500`
  sends synthetic `BalanceFrame` UDP traffic to `127.0.0.1:5000`, which
  `run_server.py --web` (run without `--esp32-host`) will pick up and display.
```

- [ ] **Step 2: Run the full test suite**

Run (from `TelemetryServer/`): `python -m pytest -q`
Expected: all tests PASS (existing tests plus every test added in Tasks 1-5)

- [ ] **Step 3: Manual end-to-end smoke test**

Run, each in the background:

```bash
python scripts/run_server.py --web &
python scripts/fake_telemetry_feed.py --hz 500 &
```

Open `http://localhost:8765/` in a browser and confirm:
- Connection badge reads "connected"
- All 4 charts animate smoothly with no visible stutter
- The Gains panel shows "RPC unavailable (pass --esp32-host)." (expected, since no `--esp32-host` was passed) and the speed/heading sliders report `SET failed: ...` if moved (also expected)

Stop both background processes (`kill %1 %2`).

- [ ] **Step 4: Commit**

```bash
git add TelemetryServer/README.md
git commit -m "docs(telemetry): document --web dashboard mode"
```

---

## Deliberately not in this plan

- **Visual design pass with `superdesign`.** Per the spec, that happens once this functional skeleton is running against live data — it's an interactive design workflow the user drives, not a mechanical TDD task. Task 6 leaves a clean, styled dark-theme baseline as the starting point.
- **Retiring `app_window.py`/`plot_live.py`/`gains_panel.py`/`mpl_backend.py` and the `PyQt5`/`matplotlib`/`pyqtgraph` dependencies.** Per the spec's Non-goals/Follow-up, this is a separate future task once the web UI has been run against real hardware and is preferred in practice.
