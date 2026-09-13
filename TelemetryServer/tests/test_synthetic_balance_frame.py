import unittest

from telemetry.balance_frame import BalanceFrame
from telemetry.protocol import TELEM_MSG_BALANCE_FRAME, FrameParser, build_frame
from telemetry.synthetic_balance_frame import make_payload
from telemetry.udp_envelope import decode_udp_datagram, encode_udp_datagram


class SyntheticBalanceFrameTests(unittest.TestCase):
    def test_generated_payload_decodes_end_to_end(self) -> None:
        payload = make_payload(frame_number=5, t=1.25)
        tm_frame = build_frame(1, TELEM_MSG_BALANCE_FRAME, payload)
        datagram = encode_udp_datagram(tm_frame, sequence=0, frame_count=1)

        envelope = decode_udp_datagram(datagram)
        frames = list(FrameParser().feed(envelope.payload))

        self.assertEqual(len(frames), 1)
        self.assertTrue(frames[0].ok)
        bf = BalanceFrame.decode(frames[0].payload)
        self.assertEqual(bf.frame_number, 5)
        self.assertTrue(bf.is_sane())


if __name__ == "__main__":
    unittest.main()
