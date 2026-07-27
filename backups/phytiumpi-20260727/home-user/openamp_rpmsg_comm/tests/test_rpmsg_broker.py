import os
import json
from pathlib import Path
import pty
import select
import socket
import subprocess
import tempfile
import threading
import time
import tty
import unittest


ROOT = Path(__file__).parents[1]
BROKER = ROOT / "build" / "rpmsg-broker"


def checksum(data: bytes) -> int:
    return (-sum(data)) & 0xFF


def encode(command: int, sequence: int, payload: bytes = b"") -> bytes:
    header = bytes((0xA5, command, sequence, len(payload))) + payload
    return header + bytes((checksum(header),))


def decode(frame: bytes) -> tuple[int, int, bytes]:
    if len(frame) < 5 or frame[0] != 0xA5 or frame[3] + 5 != len(frame):
        raise AssertionError("invalid frame")
    if checksum(frame[:-1]) != frame[-1]:
        raise AssertionError("invalid checksum")
    return frame[1], frame[2], frame[4:-1]


class BrokerIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.socket_path = str(Path(self.tempdir.name) / "rpmsg.sock")
        self.monitor_socket_path = str(Path(self.tempdir.name) / "monitor.sock")
        self.master_fd, slave_fd = pty.openpty()
        tty.setraw(slave_fd)
        device_path = os.ttyname(slave_fd)
        self.process = subprocess.Popen(
            [str(BROKER), "--device", device_path,
             "--socket", self.socket_path,
             "--monitor-socket", self.monitor_socket_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        os.close(slave_fd)
        deadline = time.monotonic() + 2.0
        while (not Path(self.socket_path).exists() or
               not Path(self.monitor_socket_path).exists()):
            if self.process.poll() is not None:
                stdout, stderr = self.process.communicate()
                self.fail(f"broker exited: {stdout} {stderr}")
            if time.monotonic() >= deadline:
                self.fail("broker socket was not created")
            time.sleep(0.01)

    def tearDown(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=2.0)
        self.process.communicate()
        os.close(self.master_fd)
        self.tempdir.cleanup()

    def test_sequence_is_owned_by_broker_and_restored_for_client(self):
        client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        client.connect(self.socket_path)
        client.send(encode(52, 77))

        readable, _, _ = select.select([self.master_fd], [], [], 1.0)
        self.assertTrue(readable)
        command, broker_sequence, payload = decode(os.read(self.master_fd, 128))
        self.assertEqual((command, payload), (52, b""))
        self.assertNotEqual(broker_sequence, 77)

        os.write(self.master_fd, encode(58, broker_sequence, b"ok"))
        command, client_sequence, payload = decode(client.recv(128))
        self.assertEqual((command, client_sequence, payload), (58, 77, b"ok"))
        client.close()

    def test_two_clients_are_serialized(self):
        clients = [socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
                   for _ in range(2)]
        for client in clients:
            client.connect(self.socket_path)

        results = [None, None]

        def request(index: int) -> None:
            clients[index].send(encode(40 + index, 10 + index))
            results[index] = decode(clients[index].recv(128))

        threads = [threading.Thread(target=request, args=(index,))
                   for index in range(2)]
        for thread in threads:
            thread.start()
        seen = []
        for _ in range(2):
            readable, _, _ = select.select([self.master_fd], [], [], 1.0)
            self.assertTrue(readable)
            command, sequence, _ = decode(os.read(self.master_fd, 128))
            seen.append(command)
            os.write(self.master_fd, encode(command, sequence, bytes((command,))))
        for thread in threads:
            thread.join(timeout=1.0)
        self.assertCountEqual(seen, [40, 41])
        self.assertEqual(results[0], (40, 10, b"("))
        self.assertEqual(results[1], (41, 11, b")"))
        for client in clients:
            client.close()

    def test_monitor_observes_wire_frames_without_sending_requests(self):
        monitor = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        monitor.settimeout(1.0)
        monitor.connect(self.monitor_socket_path)
        time.sleep(0.05)

        client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        client.connect(self.socket_path)
        client.send(encode(52, 91))
        command, broker_sequence, _ = decode(os.read(self.master_fd, 128))
        self.assertEqual(command, 52)
        tx_event = json.loads(monitor.recv(2048))
        self.assertEqual(tx_event["direction"], "tx")
        self.assertEqual(tx_event["type"], 52)
        self.assertEqual(tx_event["client_seq"], 91)
        self.assertEqual(tx_event["wire_seq"], broker_sequence)

        os.write(self.master_fd, encode(58, broker_sequence, b"telemetry"))
        decode(client.recv(128))
        rx_event = json.loads(monitor.recv(2048))
        self.assertEqual(rx_event["direction"], "rx")
        self.assertEqual(rx_event["type"], 58)
        self.assertEqual(rx_event["client_seq"], 91)
        self.assertGreaterEqual(rx_event["latency_ms"], 0)
        self.assertEqual(rx_event["totals"]["tx_frames"], 1)
        self.assertEqual(rx_event["totals"]["rx_frames"], 1)
        client.close()
        monitor.close()


if __name__ == "__main__":
    unittest.main()
