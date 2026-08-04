"""Self-contained UDP envelope used by the ESP32 telemetry bridge."""

from __future__ import annotations

import struct
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Set

UDP_ENVELOPE_MAGIC = b"RT"
UDP_ENVELOPE_VERSION = 1
UDP_ENVELOPE_HEADER = struct.Struct("<2sBBIBBH")
UDP_ENVELOPE_CRC = struct.Struct("<H")


class UdpEnvelopeError(ValueError):
    """Raised when an RT datagram is present but malformed."""


@dataclass(frozen=True)
class UdpEnvelope:
    payload: bytes
    sequence: Optional[int]
    frame_count: Optional[int]
    flags: int = 0

    @property
    def enveloped(self) -> bool:
        return self.sequence is not None


@dataclass
class DatagramSequenceCounters:
    received: int = 0
    missing: int = 0
    duplicates: int = 0
    out_of_order: int = 0
    resets: int = 0


class DatagramSequenceTracker:
    """Track loss while tolerating duplicates, wraparound and limited reordering."""

    def __init__(self, history_size: int = 4096) -> None:
        if history_size < 2:
            raise ValueError("history_size must be at least 2")
        self.history_size = history_size
        self.highest: Optional[int] = None
        self.counters = DatagramSequenceCounters()
        self._history: Deque[int] = deque()
        self._seen: Set[int] = set()

    def observe(self, sequence: int) -> None:
        sequence &= 0xFFFFFFFF
        self.counters.received += 1
        if self.highest is None:
            self.highest = sequence
            self._remember(sequence)
            return

        distance = (sequence - self.highest) & 0xFFFFFFFF
        if distance == 0 or sequence in self._seen:
            self.counters.duplicates += 1
            return
        if 0 < distance < 0x80000000:
            self.counters.missing += distance - 1
            self.highest = sequence
            self._remember(sequence)
            return

        # A small sequence after a long-running stream is an ESP32 reboot.
        if sequence < 1024 and self.highest > 1_000_000:
            self.counters.resets += 1
            self.highest = sequence
            self._history.clear()
            self._seen.clear()
            self._remember(sequence)
            return

        self.counters.out_of_order += 1
        if self.counters.missing > 0:
            self.counters.missing -= 1
        self._remember(sequence)

    def _remember(self, sequence: int) -> None:
        self._history.append(sequence)
        self._seen.add(sequence)
        while len(self._history) > self.history_size:
            old = self._history.popleft()
            self._seen.discard(old)


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xffff."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def decode_udp_datagram(data: bytes) -> UdpEnvelope:
    """Decode an RT envelope, or pass through a legacy raw-TM datagram."""
    if not data.startswith(UDP_ENVELOPE_MAGIC):
        return UdpEnvelope(payload=data, sequence=None, frame_count=None)
    minimum = UDP_ENVELOPE_HEADER.size + UDP_ENVELOPE_CRC.size
    if len(data) < minimum:
        raise UdpEnvelopeError("truncated UDP envelope")

    magic, version, flags, sequence, frame_count, _reserved, payload_len = (
        UDP_ENVELOPE_HEADER.unpack_from(data)
    )
    if magic != UDP_ENVELOPE_MAGIC:
        raise UdpEnvelopeError("invalid UDP envelope magic")
    if version != UDP_ENVELOPE_VERSION:
        raise UdpEnvelopeError(f"unsupported UDP envelope version {version}")
    if frame_count == 0:
        raise UdpEnvelopeError("UDP envelope contains no frames")

    expected_len = UDP_ENVELOPE_HEADER.size + payload_len + UDP_ENVELOPE_CRC.size
    if len(data) != expected_len:
        raise UdpEnvelopeError(
            f"UDP envelope length mismatch: got {len(data)}, expected {expected_len}"
        )
    expected_crc = UDP_ENVELOPE_CRC.unpack_from(data, expected_len - 2)[0]
    actual_crc = crc16_ccitt(data[:-2])
    if actual_crc != expected_crc:
        raise UdpEnvelopeError(
            f"UDP envelope CRC mismatch: got 0x{expected_crc:04x}, "
            f"expected 0x{actual_crc:04x}"
        )
    payload = data[UDP_ENVELOPE_HEADER.size : -UDP_ENVELOPE_CRC.size]
    return UdpEnvelope(payload=payload, sequence=sequence, frame_count=frame_count, flags=flags)


def encode_udp_datagram(
    payload: bytes,
    sequence: int,
    frame_count: int,
    flags: int = 0,
) -> bytes:
    """Reference encoder used by tests and fault-injection tools."""
    if not 1 <= frame_count <= 0xFF:
        raise ValueError("frame_count must be in range 1..255")
    if len(payload) > 0xFFFF:
        raise ValueError("payload is too large")
    header = UDP_ENVELOPE_HEADER.pack(
        UDP_ENVELOPE_MAGIC,
        UDP_ENVELOPE_VERSION,
        flags & 0xFF,
        sequence & 0xFFFFFFFF,
        frame_count,
        0,
        len(payload),
    )
    without_crc = header + payload
    return without_crc + UDP_ENVELOPE_CRC.pack(crc16_ccitt(without_crc))
