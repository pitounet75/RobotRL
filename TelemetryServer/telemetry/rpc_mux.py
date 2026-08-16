"""Control-params RPC multiplexed on the live telemetry UDP socket."""

from __future__ import annotations

import queue
import threading
from typing import Tuple

from telemetry.ctrl_params import (
    ControlParamsSnapshot,
    decode_set_ack,
    decode_snapshot,
    encode_set_param,
    resolve_param,
)
from telemetry.protocol import (
    TELEM_MSG_GET_CONTROL_PARAMS,
    TELEM_MSG_SET_CONTROL_PARAM,
    TelemetryErrorCode,
    TelemetryFrame,
    build_frame,
)
from telemetry.udp_receiver import UdpTelemetryReceiver


class SharedRpcClient:
    """GET/SET control params using the same socket as BalanceFrame RX."""

    def __init__(
        self,
        receiver: UdpTelemetryReceiver,
        timeout_s: float = 1.5,
    ) -> None:
        self._receiver = receiver
        self.timeout_s = timeout_s
        self._seq = 1
        self._lock = threading.Lock()
        self._pending: dict[tuple[int, int], queue.Queue[TelemetryFrame]] = {}

    def feed_reply(self, frame: TelemetryFrame) -> bool:
        """Return True if this frame was consumed as an RPC reply."""
        key = (frame.sequence_id, frame.message_type)
        with self._lock:
            q = self._pending.get(key)
        if q is None:
            return False
        q.put(frame)
        return True

    def _next_seq(self) -> int:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        if self._seq == 0:
            self._seq = 1
        return seq

    def _transact(self, msg_type: int, payload: bytes = b"") -> TelemetryFrame:
        if not self._receiver.esp32_host:
            raise RuntimeError("esp32-host required for control-params RPC")

        last_err: Exception | None = None
        for _ in range(5):
            seq = self._next_seq()
            key = (seq, msg_type)
            q: queue.Queue[TelemetryFrame] = queue.Queue(maxsize=1)
            with self._lock:
                self._pending[key] = q
            try:
                self._receiver.send_raw(build_frame(seq, msg_type, payload))
                try:
                    frame = q.get(timeout=self.timeout_s)
                except queue.Empty as exc:
                    last_err = TimeoutError(
                        f"no reply for seq={seq} type=0x{msg_type:04x}"
                    )
                    continue
                return frame
            finally:
                with self._lock:
                    self._pending.pop(key, None)

        raise TimeoutError(
            f"no reply for type=0x{msg_type:04x} after 5 attempts ({last_err})"
        )

    def get_params(self) -> ControlParamsSnapshot:
        frame = self._transact(TELEM_MSG_GET_CONTROL_PARAMS)
        if not frame.ok:
            raise RuntimeError(f"GET failed: error_code={frame.error_code}")
        return decode_snapshot(frame.payload)

    def set_param(self, name_or_id: str | int, value: float) -> Tuple[int, str, float]:
        param_id, name = resolve_param(name_or_id)
        frame = self._transact(
            TELEM_MSG_SET_CONTROL_PARAM, encode_set_param(param_id, value)
        )
        if not frame.ok:
            raise RuntimeError(
                f"SET {name} failed: error_code={TelemetryErrorCode(frame.error_code).name}"
            )
        ack_id, applied = decode_set_ack(frame.payload)
        return ack_id, name, applied
