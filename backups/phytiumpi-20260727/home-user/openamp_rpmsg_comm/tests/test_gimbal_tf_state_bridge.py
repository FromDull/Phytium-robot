import importlib.util
import json
from pathlib import Path
import socket
import tempfile
import threading
import unittest


MODULE_PATH = Path(__file__).parents[1] / "linux_user" / "gimbal_tf_state_bridge.py"
SPEC = importlib.util.spec_from_file_location("gimbal_tf_state_bridge", MODULE_PATH)
assert SPEC and SPEC.loader
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


class GimbalTfStateBridgeTests(unittest.TestCase):
    def response(self, **updates):
        telemetry = {
            "state": 3,
            "fault": 0,
            "feedback_valid_mask": 3,
            "yaw_feedback_age_ms": 4,
            "pitch_feedback_age_ms": 5,
            "yaw_deg": 12.5,
            "pitch_deg": -3.25,
        }
        telemetry.update(updates)
        return {"ok": True, "telemetry": telemetry}

    def test_valid_pose_is_independent_of_motor_enable_state(self):
        state = bridge.state_from_response(
            self.response(state=0), sequence=7,
            maximum_feedback_age_ms=500, sampled_at=123.0
        )
        self.assertTrue(state["pose_valid"])
        self.assertFalse(state["active"])
        self.assertEqual(state["sequence"], 7)

    def test_stale_or_incomplete_feedback_is_rejected(self):
        stale = bridge.state_from_response(
            self.response(yaw_feedback_age_ms=501), 1, 500, 1.0
        )
        incomplete = bridge.state_from_response(
            self.response(feedback_valid_mask=1), 2, 500, 2.0
        )
        self.assertFalse(stale["pose_valid"])
        self.assertFalse(incomplete["pose_valid"])

    def test_direct_unix_socket_request(self):
        with tempfile.TemporaryDirectory() as directory:
            socket_path = str(Path(directory) / "gimbal.sock")
            ready = threading.Event()

            def server():
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
                    listener.bind(socket_path)
                    listener.listen(1)
                    ready.set()
                    connection, _ = listener.accept()
                    with connection:
                        self.assertIn(b'"status"', connection.recv(1024))
                        connection.sendall(
                            (json.dumps(self.response()) + "\n").encode("utf-8")
                        )

            worker = threading.Thread(target=server)
            worker.start()
            self.assertTrue(ready.wait(1.0))
            response = bridge.request_status(socket_path, 1.0)
            worker.join(1.0)
            self.assertFalse(worker.is_alive())
            self.assertEqual(response["telemetry"]["yaw_deg"], 12.5)


if __name__ == "__main__":
    unittest.main()
