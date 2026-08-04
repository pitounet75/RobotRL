from __future__ import annotations

import pytest

from telemetry.udp_envelope import (
    DatagramSequenceTracker,
    UdpEnvelopeError,
    decode_udp_datagram,
    encode_udp_datagram,
)


def test_envelope_round_trip() -> None:
    payload = b"TM" + bytes(range(65))
    raw = encode_udp_datagram(payload, sequence=0x12345678, frame_count=1, flags=3)

    decoded = decode_udp_datagram(raw)

    assert decoded.enveloped
    assert decoded.sequence == 0x12345678
    assert decoded.frame_count == 1
    assert decoded.flags == 3
    assert decoded.payload == payload


def test_legacy_datagram_passes_through() -> None:
    raw = b"TM legacy"
    decoded = decode_udp_datagram(raw)
    assert not decoded.enveloped
    assert decoded.payload == raw


@pytest.mark.parametrize("cut", range(2, 14))
def test_truncated_envelope_is_rejected(cut: int) -> None:
    raw = encode_udp_datagram(b"TM payload", sequence=1, frame_count=1)
    with pytest.raises(UdpEnvelopeError):
        decode_udp_datagram(raw[:cut])


def test_crc_error_is_rejected() -> None:
    raw = bytearray(encode_udp_datagram(b"TM payload", sequence=1, frame_count=1))
    raw[12] ^= 0x80
    with pytest.raises(UdpEnvelopeError, match="CRC mismatch"):
        decode_udp_datagram(bytes(raw))


def test_length_error_is_rejected() -> None:
    raw = bytearray(encode_udp_datagram(b"TM payload", sequence=1, frame_count=1))
    raw[10] += 1
    with pytest.raises(UdpEnvelopeError, match="length mismatch"):
        decode_udp_datagram(bytes(raw))


def test_sequence_tracker_repairs_late_packet_gap() -> None:
    tracker = DatagramSequenceTracker()
    for sequence in (10, 12, 11, 13, 13):
        tracker.observe(sequence)
    assert tracker.counters.received == 5
    assert tracker.counters.missing == 0
    assert tracker.counters.out_of_order == 1
    assert tracker.counters.duplicates == 1


def test_sequence_tracker_handles_wraparound() -> None:
    tracker = DatagramSequenceTracker()
    for sequence in (0xFFFFFFFE, 0xFFFFFFFF, 0, 1):
        tracker.observe(sequence)
    assert tracker.counters.missing == 0
    assert tracker.counters.out_of_order == 0


def test_transport_stress_preserves_complete_payloads() -> None:
    tracker = DatagramSequenceTracker()
    expected_missing = 0
    for sequence in range(10_000):
        if sequence != 0 and sequence % 137 == 0:
            expected_missing += 1
            continue
        payload = (b"TM" + sequence.to_bytes(4, "little")) * 5
        raw = encode_udp_datagram(payload, sequence=sequence, frame_count=5)
        decoded = decode_udp_datagram(raw)
        tracker.observe(decoded.sequence or 0)
        assert decoded.payload == payload
        assert decoded.frame_count == 5
    assert tracker.counters.missing == expected_missing
