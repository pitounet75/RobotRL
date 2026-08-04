import socket
import unittest
from unittest.mock import MagicMock, call, patch

from telemetry.udp_receiver import UdpTelemetryReceiver


class UdpTelemetryReceiverTests(unittest.TestCase):
    @patch("telemetry.udp_receiver.select.select")
    @patch("telemetry.udp_receiver.socket.socket")
    def test_receive_buffer_and_drain_until_empty(self, socket_factory: MagicMock, select_mock: MagicMock) -> None:
        sock = socket_factory.return_value
        sock.getsockopt.return_value = 2_000_000
        sock.recvfrom.side_effect = [
            (b"one", ("192.0.2.1", 5000)),
            (b"two", ("192.0.2.1", 5000)),
            (b"three", ("192.0.2.1", 5000)),
        ]
        select_mock.side_effect = [
            ([sock], [], []),
            ([sock], [], []),
            ([], [], []),
        ]

        receiver = UdpTelemetryReceiver(bind_port=0, receive_buffer_size=1_000_000)

        self.assertEqual(list(receiver.recv_chunks()), [b"one", b"two", b"three"])
        self.assertEqual(receiver.receive_buffer_size, 2_000_000)
        self.assertIn(
            call(socket.SOL_SOCKET, socket.SO_RCVBUF, 1_000_000),
            sock.setsockopt.call_args_list,
        )
        sock.recvfrom.assert_has_calls([call(65535), call(65535), call(65535)])


if __name__ == "__main__":
    unittest.main()
