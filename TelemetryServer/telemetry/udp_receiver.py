"""UDP receiver for ESP32 telemetry bridge."""

from __future__ import annotations

import socket
import time
from typing import Callable, Optional, Tuple


class UdpTelemetryReceiver:
    def __init__(
        self,
        bind_host: str = "0.0.0.0",
        bind_port: int = 5000,
        esp32_host: Optional[str] = None,
        esp32_port: int = 5000,
    ) -> None:
        self.bind_host = bind_host
        self.bind_port = bind_port
        self.esp32_host = esp32_host
        self.esp32_port = esp32_port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((bind_host, bind_port))
        self._sock.settimeout(0.25)
        self._remote: Optional[Tuple[str, int]] = None

    def subscribe(self, payload: bytes = b"subscribe") -> None:
        """Ping ESP32 so learn-remote mode knows our address."""
        if not self.esp32_host:
            return
        self._sock.sendto(payload, (self.esp32_host, self.esp32_port))

    def recv_chunk(self) -> Optional[bytes]:
        try:
            data, addr = self._sock.recvfrom(4096)
        except socket.timeout:
            return None
        self._remote = addr
        return data

    def run_forever(self, on_chunk: Callable[[bytes, float], None]) -> None:
        while True:
            chunk = self.recv_chunk()
            if chunk is None:
                continue
            on_chunk(chunk, time.time())

    def close(self) -> None:
        self._sock.close()
