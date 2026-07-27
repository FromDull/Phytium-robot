import importlib.util
from pathlib import Path
import struct
import unittest


MODULE_PATH = Path(__file__).parents[1] / "linux_user" / "chassis_state_bridge.py"
SPEC = importlib.util.spec_from_file_location("chassis_state_bridge", MODULE_PATH)
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


class ChassisStateBridgeTests(unittest.TestCase):
    def test_status_request_contains_no_motion_payload(self):
        frame = bridge.encode_status_request(7)
        self.assertEqual(frame[:4], bytes((0xA5, 63, 7, 0)))
        self.assertEqual(len(frame), 5)
        self.assertEqual(bridge.checksum(frame[:-1]), frame[-1])

    def test_decode_telemetry(self):
        payload = struct.pack(
            ">BBBBIiiiiiiiii",
            1, 0, 2, 0, 123,
            100000, -200000, 90000, -180000, 80000, -160000,
            1200000, 300000, 250000,
        )
        state = bridge.state_from_telemetry(
            bridge.decode_telemetry(payload), 9, 1234.5)
        self.assertTrue(state["state_valid"])
        self.assertEqual(state["balance_state"], 2)
        self.assertAlmostEqual(state["measured_linear_m_s"], 0.08)
        self.assertAlmostEqual(state["wheel_track_m"], 0.25)
        self.assertEqual(state["updated_at"], 1234.5)

    def test_rejects_mismatched_sequence(self):
        payload = bytes(44)
        header = bytes((0xA5, 63, 4, len(payload))) + payload
        frame = header + bytes((bridge.checksum(header),))
        with self.assertRaisesRegex(ValueError, "mismatched sequence"):
            bridge.decode_status_reply(frame, 5)


if __name__ == "__main__":
    unittest.main()
