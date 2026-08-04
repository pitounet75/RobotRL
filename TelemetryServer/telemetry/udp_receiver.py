"""UDP receiver for ESP32 telemetry bridge."""

from __future__ import annotations

import select
import socket
import time
from typing import Callable, Iterator, Optional, Tuple

DEFAULT_RECEIVE_BUFFER_SIZE = 1024 * 1024
MAX_UDP_DATAGRAM_SIZE = 65535


class UdpTelemetryReceiver:
    def __init__(
        self,
        bind_host: str = "0.0.0.0",
        bind_port: int = 5000,
        esp32_host: Optional[str] = None,
        esp32_port: int = 5000,
        receive_buffer_size: int = DEFAULT_RECEIVE_BUFFER_SIZE,
    ) -> None:
        self.bind_host = bind_host
        self.bind_port = bind_port
        self.esp32_host = esp32_host
        self.esp32_port = esp32_port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, receive_buffer_size)
        self._sock.bind((bind_host, bind_port))
        self._sock.settimeout(0.25)
        self.receive_buffer_size = self._sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        self._remote: Optional[Tuple[str, int]] = None

    def subscribe(self, payload: bytes = b"subscribe") -> None:
        """Ping ESP32 so learn-remote mode knows our address."""
        if not self.esp32_host:
            return
        self._sock.sendto(payload, (self.esp32_host, self.esp32_port))

    def recv_chunk(self) -> Optional[bytes]:
        try:
            data, addr = self._sock.recvfrom(MAX_UDP_DATAGRAM_SIZE)
        except socket.timeout:
            return None
        self._remote = addr
        return data

    def recv_chunks(self) -> Iterator[bytes]:
        """Wait for one datagram, then drain all currently queued datagrams."""
        first = self.recv_chunk()
        if first is None:
            return
        yield first

        while select.select([self._sock], [], [], 0)[0]:
            try:
                data, addr = self._sock.recvfrom(MAX_UDP_DATAGRAM_SIZE)
            except (BlockingIOError, socket.timeout):
                return
            self._remote = addr
            yield data

    def run_forever(self, on_chunk: Callable[[bytes, float], None]) -> None:
        while True:
            for chunk in self.recv_chunks():
                on_chunk(chunk, time.time())

    def close(self) -> None:
        self._sock.close()
