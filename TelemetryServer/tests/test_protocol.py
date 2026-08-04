import struct
import unittest

from telemetry.protocol import (
    TELEMETRY_MAGIC,
    TELEMETRY_MAX_BODY_LEN,
    FrameParser,
    build_frame,
    crc8,
)


class FrameParserTests(unittest.TestCase):
    def test_arbitrary_fragmentation(self) -> None:
        wire = build_frame(1, 0x0100, b"first") + build_frame(2, 0x0101, b"second")
        parser = FrameParser()

        frames = []
        for byte in wire:
            frames.extend(parser.feed(bytes((byte,))))

        self.assertEqual([(frame.sequence_id, frame.payload) for frame in frames], [(1, b"first"), (2, b"second")])
        self.assertEqual(parser.counters.valid_frames, 2)

    def test_invalid_lengths_resync_and_recover(self) -> None:
        too_short = struct.pack("<HH", TELEMETRY_MAGIC, 6) + b"garbage"
        too_long = struct.pack("<HH", TELEMETRY_MAGIC, TELEMETRY_MAX_BODY_LEN + 1)
        expected = build_frame(9, 0x0100, b"recovered")
        parser = FrameParser()

        frames = list(parser.feed(too_short + too_long + expected))

        self.assertEqual([frame.sequence_id for frame in frames], [9])
        self.assertEqual(parser.counters.length_errors, 2)
        self.assertGreater(parser.counters.bytes_discarded, 0)

    def test_invalid_crc_counts_crc_only_and_recovers(self) -> None:
        damaged = bytearray(build_frame(3, 0x0100, b"bad"))
        damaged[-1] ^= 0x80
        parser = FrameParser()

        frames = list(parser.feed(bytes(damaged) + build_frame(4, 0x0100, b"good")))

        self.assertEqual([frame.sequence_id for frame in frames], [4])
        self.assertEqual(parser.counters.crc_errors, 1)
        self.assertEqual(parser.counters.valid_frames, 1)

    def test_invalid_version_resyncs_one_byte(self) -> None:
        damaged = bytearray(build_frame(3, 0x0100, b"bad-version"))
        damaged[4] = 2
        damaged[-1] = crc8(damaged[:-1])
        parser = FrameParser()

        frames = list(parser.feed(bytes(damaged) + build_frame(5, 0x0100)))

        self.assertEqual([frame.sequence_id for frame in frames], [5])
        self.assertEqual(parser.counters.version_errors, 1)
        self.assertEqual(parser.counters.crc_errors, 0)

    def test_false_magic_in_garbage_and_payload_does_not_hide_frame(self) -> None:
        nested = build_frame(12, 0x0100, b"nested")
        wrapper = bytearray(build_frame(11, 0x0100, b"prefix" + nested + b"suffix"))
        wrapper[-1] ^= 1
        false_magic_garbage = b"noise" + struct.pack("<HH", TELEMETRY_MAGIC, 7) + b"\x01" * 7
        parser = FrameParser()

        frames = list(parser.feed(false_magic_garbage + bytes(wrapper)))

        self.assertEqual([(frame.sequence_id, frame.payload) for frame in frames], [(12, b"nested")])
        self.assertEqual(parser.counters.crc_errors, 2)
        self.assertGreater(parser.counters.bytes_discarded, 0)

    def test_recovery_when_corruption_and_valid_frame_are_fragmented(self) -> None:
        damaged = bytearray(build_frame(20, 0x0100, b"corrupt"))
        damaged[-1] ^= 1
        wire = b"junk" + bytes(damaged) + build_frame(21, 0x0100, b"ok")
        parser = FrameParser(max_body_len=128)

        frames = []
        positions = (1, 3, 8, 9, 17, len(wire))
        start = 0
        for end in positions:
            frames.extend(parser.feed(wire[start:end]))
            start = end

        self.assertEqual([frame.sequence_id for frame in frames], [21])
        self.assertEqual(parser.counters.crc_errors, 1)


if __name__ == "__main__":
    unittest.main()
