"""Bidirectional telemetry client for runtime gain tuning."""

from __future__ import annotations

import socket
import time
from typing import Optional, Tuple

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
    FrameParser,
    TelemetryErrorCode,
    TelemetryFrame,
    build_frame,
)


class ControlParamsClient:
    def __init__(
        self,
        esp32_host: str,
        esp32_port: int = 5000,
        bind_host: str = "0.0.0.0",
        bind_port: int = 5000,
        timeout_s: float = 1.0,
    ) -> None:
        self.esp32_host = esp32_host
        self.esp32_port = esp32_port
        self.timeout_s = timeout_s
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((bind_host, bind_port))
        self._sock.settimeout(0.05)
        self._parser = FrameParser()
        self._seq = 1
        self._subscribe()

    def close(self) -> None:
        self._sock.close()

    def _next_seq(self) -> int:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        if self._seq == 0:
            self._seq = 1
        return seq

    def _subscribe(self) -> None:
        self._sock.sendto(b"subscribe", (self.esp32_host, self.esp32_port))

    def _send(self, frame: bytes) -> None:
        self._sock.sendto(frame, (self.esp32_host, self.esp32_port))

    def _wait_reply(self, seq: int, msg_type: int, deadline: float) -> TelemetryFrame:
        while time.time() < deadline:
            try:
                data, _addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            for frame in self._parser.feed(data):
                if frame.sequence_id == seq and frame.message_type == msg_type:
                    return frame
        raise TimeoutError(f"no reply for seq={seq} type=0x{msg_type:04x}")

    def get_params(self) -> ControlParamsSnapshot:
        last_err: Exception | None = None
        for attempt in range(5):
            seq = self._next_seq()
            self._send(build_frame(seq, TELEM_MSG_GET_CONTROL_PARAMS))
            deadline = time.time() + self.timeout_s
            try:
                frame = self._wait_reply(seq, TELEM_MSG_GET_CONTROL_PARAMS, deadline)
            except TimeoutError as exc:
                last_err = exc
                continue
            if not frame.ok:
                raise RuntimeError(f"GET failed: error_code={frame.error_code}")
            return decode_snapshot(frame.payload)
        raise TimeoutError(
            f"no reply for GetControlParams after 5 attempts ({last_err}). "
            "BalanceFrame may work while RPC RX is broken — reflash latest STM32 firmware."
        )

    def set_param(self, name_or_id: str | int, value: float) -> Tuple[int, str, float]:
        param_id, name = resolve_param(name_or_id)
        last_err: Exception | None = None
        for _attempt in range(5):
            seq = self._next_seq()
            self._send(build_frame(seq, TELEM_MSG_SET_CONTROL_PARAM, encode_set_param(param_id, value)))
            deadline = time.time() + self.timeout_s
            try:
                frame = self._wait_reply(seq, TELEM_MSG_SET_CONTROL_PARAM, deadline)
            except TimeoutError as exc:
                last_err = exc
                continue
            if not frame.ok:
                raise RuntimeError(
                    f"SET {name} failed: error_code={TelemetryErrorCode(frame.error_code).name}"
                )
            ack_id, applied = decode_set_ack(frame.payload)
            return ack_id, name, applied
        raise TimeoutError(f"no reply for SetControlParam after 5 attempts ({last_err})")
