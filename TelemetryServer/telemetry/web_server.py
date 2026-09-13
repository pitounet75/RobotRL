"""Web dashboard server: binary telemetry broadcast + JSON control bridge."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Any, Dict, Optional, Set

from aiohttp import WSMsgType, web

from telemetry.balance_frame import BalanceFrame
from telemetry.rpc_mux import SharedRpcClient

# Bounded broadcast queue: at up to 500Hz this is ~0.2s of buffering before
# the drop-oldest policy kicks in -- small enough that a stalled client
# doesn't accumulate a large stale backlog, large enough to absorb jitter.
_BROADCAST_QUEUE_MAX = 100


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


class TelemetryWebServer:
    """Serves the web UI, broadcasts BalanceFrames, and bridges control RPC."""

    def __init__(self, rpc: Optional[SharedRpcClient], static_dir: Path) -> None:
        self._rpc = rpc
        self._static_dir = static_dir
        self._telemetry_clients: Set[web.WebSocketResponse] = set()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._broadcast_queue: Optional["asyncio.Queue[BalanceFrame]"] = None
        self._drain_task: Optional["asyncio.Future[None]"] = None

    def build_app(self) -> web.Application:
        app = web.Application()
        app.router.add_get("/ws/telemetry", self._telemetry_handler)
        app.router.add_get("/ws/control", self._control_handler)
        app.router.add_get("/", self._index_handler)
        app.router.add_static("/", self._static_dir, show_index=False)
        return app

    async def _index_handler(self, request: web.Request) -> web.FileResponse:
        return web.FileResponse(self._static_dir / "index.html")

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
        """Bind the broadcast loop and start the single drain coroutine.

        Must be called from inside `loop` (it creates the queue and the drain
        task on it). Both `run()` and tests use this as the one entry point.
        """
        self._loop = loop
        if self._broadcast_queue is None:
            self._broadcast_queue = asyncio.Queue(maxsize=_BROADCAST_QUEUE_MAX)
        if self._drain_task is None:
            self._drain_task = asyncio.ensure_future(self._drain_broadcast_queue())

    def close_loop_binding(self) -> None:
        """Cancel the drain coroutine (idempotent); counterpart of bind_loop."""
        if self._drain_task is not None:
            self._drain_task.cancel()
            self._drain_task = None
        self._broadcast_queue = None
        self._loop = None

    def push_balance_frame_threadsafe(self, bf: BalanceFrame) -> None:
        """Call from any thread; enqueues onto the server's own loop for broadcast."""
        loop = self._loop
        queue = self._broadcast_queue
        if loop is None or queue is None:
            return
        try:
            loop.call_soon_threadsafe(self._enqueue_frame, bf)
        except RuntimeError:
            pass  # loop already closed during shutdown

    def _enqueue_frame(self, bf: BalanceFrame) -> None:
        """Runs on the server loop. Drops the oldest frame when backed up."""
        queue = self._broadcast_queue
        if queue is None:
            return
        if queue.full():
            # A stalled/backgrounded browser must not grow this without bound;
            # for live telemetry the newest frame is the one worth keeping.
            try:
                queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
        queue.put_nowait(bf)

    async def _drain_broadcast_queue(self) -> None:
        assert self._broadcast_queue is not None
        queue = self._broadcast_queue
        while True:
            bf = await queue.get()
            try:
                await self._broadcast(bf)
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # never let one bad frame kill the drain
                print(f"web broadcast error: {exc!r}")

    async def _broadcast(self, bf: BalanceFrame) -> None:
        if not self._telemetry_clients:
            return
        data = bf.encode()
        dead = []
        for ws in list(self._telemetry_clients):
            try:
                await asyncio.wait_for(ws.send_bytes(data), timeout=1.0)
            except (ConnectionResetError, ConnectionError, OSError, asyncio.TimeoutError):
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
            self.close_loop_binding()
            await runner.cleanup()
