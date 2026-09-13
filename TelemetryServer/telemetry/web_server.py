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
