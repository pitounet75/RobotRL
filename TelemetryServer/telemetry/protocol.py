"""STM32 telemetry wire protocol (protocol v1, CRC-8 poly 0x07)."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable, Iterator, Optional

TELEMETRY_MAGIC = 0x544D
TELEMETRY_PROTOCOL_VERSION = 1
TELEMETRY_PREFIX_SIZE = 4
TELEMETRY_BODY_HEADER_SIZE = 6
TELEMETRY_PAYLOAD_OFFSET = 10
TELEMETRY_MIN_BODY_LEN = 7
TELEMETRY_MIN_FRAME_SIZE = 11
TELEMETRY_MAX_FRAME_SIZE = 512
TELEMETRY_MAX_BODY_LEN = TELEMETRY_MAX_FRAME_SIZE - TELEMETRY_PREFIX_SIZE

TELEM_MSG_BALANCE_FRAME = 0x0100
TELEM_MSG_GET_CONTROL_PARAMS = 0x0101
TELEM_MSG_SET_CONTROL_PARAM = 0x0102


class TelemetryErrorCode(IntEnum):
    INVALID_MESSAGE = 0
    NONE = 1
    PROTOCOL_VERSION = 2
    INVALID_PAYLOAD = 3
    UNKNOWN_MESSAGE_TYPE = 4
    UNKNOWN_MESSAGE_KEY = 5
    TX_FAILED = 6


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


@dataclass(frozen=True)
class TelemetryFrame:
    sequence_id: int
    message_type: int
    error_code: int
    payload: bytes

    @property
    def ok(self) -> bool:
        return self.error_code == TelemetryErrorCode.NONE


@dataclass
class ParserCounters:
    bytes_discarded: int = 0
    length_errors: int = 0
    version_errors: int = 0
    crc_errors: int = 0
    valid_frames: int = 0


class FrameParser:
    """Incremental parser for TM frames from a byte stream."""

    def __init__(self, max_body_len: int = TELEMETRY_MAX_BODY_LEN) -> None:
        if max_body_len < TELEMETRY_MIN_BODY_LEN or max_body_len > 0xFFFF:
            raise ValueError(
                f"max_body_len must be in [{TELEMETRY_MIN_BODY_LEN}, 65535]"
            )
        self.max_body_len = max_body_len
        self.max_frame_len = TELEMETRY_PREFIX_SIZE + max_body_len
        self.counters = ParserCounters()
        self._buf = bytearray()

    def feed(self, data: bytes) -> Iterator[TelemetryFrame]:
        self._buf.extend(data)
        while True:
            frame = self._try_extract()
            if frame is None:
                break
            yield frame

    def _try_extract(self) -> Optional[TelemetryFrame]:
        while True:
            while len(self._buf) >= 2 and struct.unpack_from("<H", self._buf, 0)[0] != TELEMETRY_MAGIC:
                self._discard_one()

            if len(self._buf) < TELEMETRY_PREFIX_SIZE:
                return None

            body_len = struct.unpack_from("<H", self._buf, 2)[0]
            if body_len < TELEMETRY_MIN_BODY_LEN or body_len > self.max_body_len:
                self.counters.length_errors += 1
                self._discard_one()
                continue

            frame_len = TELEMETRY_PREFIX_SIZE + body_len
            if len(self._buf) < frame_len:
                return None

            raw = bytes(self._buf[:frame_len])
            if raw[4] != TELEMETRY_PROTOCOL_VERSION:
                self.counters.version_errors += 1
                self._discard_one()
                continue

            rx_crc = raw[-1]
            if crc8(raw[:-1]) != rx_crc:
                self.counters.crc_errors += 1
                self._discard_one()
                continue

            del self._buf[:frame_len]
            self.counters.valid_frames += 1
            sequence_id = struct.unpack_from("<H", raw, 5)[0]
            message_type = struct.unpack_from("<H", raw, 7)[0]
            error_code = raw[9]
            payload = raw[TELEMETRY_PAYLOAD_OFFSET:-1]
            return TelemetryFrame(sequence_id, message_type, error_code, payload)

    def _discard_one(self) -> None:
        del self._buf[0]
        self.counters.bytes_discarded += 1


def build_frame(sequence_id: int, message_type: int, payload: bytes = b"", error_code: int = TelemetryErrorCode.NONE) -> bytes:
    """Build a host->device TM frame (little-endian, CRC-8)."""
    body_len = TELEMETRY_BODY_HEADER_SIZE + len(payload) + 1
    header = struct.pack(
        "<HHBHHB",
        TELEMETRY_MAGIC,
        body_len,
        TELEMETRY_PROTOCOL_VERSION,
        sequence_id & 0xFFFF,
        message_type & 0xFFFF,
        error_code & 0xFF,
    )
    frame = header + payload
    return frame + bytes([crc8(frame)])


def parse_frames(chunks: Iterable[bytes]) -> Iterator[TelemetryFrame]:
    parser = FrameParser()
    for chunk in chunks:
        yield from parser.feed(chunk)
