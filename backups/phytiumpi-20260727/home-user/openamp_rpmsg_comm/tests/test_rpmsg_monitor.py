import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).parents[1] / "linux_user" / "rpmsg_monitor.py"
SPEC = importlib.util.spec_from_file_location("rpmsg_monitor", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class MonitorStateTests(unittest.TestCase):
    def test_event_is_named_and_aggregated(self):
        state = MODULE.MonitorState()
        state.set_connection(True)
        state.add_event({
            "event_id": 1,
            "epoch_ms": 1234,
            "direction": "tx",
            "type": 52,
            "client_seq": 9,
            "wire_seq": 1,
            "payload_len": 0,
            "wire_bytes": 5,
            "latency_ms": -1,
            "status": "ok",
            "frame_hex": "A534010026",
            "totals": {"tx_frames": 1, "rx_frames": 0, "errors": 0,
                       "tx_bytes": 5, "rx_bytes": 0},
        })
        snapshot = state.snapshot()
        self.assertTrue(snapshot["connected"])
        self.assertEqual(snapshot["events"][0]["command"], "BALANCE_STATUS")
        self.assertEqual(snapshot["rates"]["tx_fps"], 1)
        self.assertEqual(snapshot["totals"]["tx_bytes"], 5)

    def test_receive_latency_statistics(self):
        state = MODULE.MonitorState()
        for latency in (1, 2, 8, 3):
            state.add_event({"direction": "rx", "type": 58,
                             "latency_ms": latency, "totals": {}})
        snapshot = state.snapshot()
        self.assertEqual(snapshot["latency"]["latest_ms"], 3.0)
        self.assertEqual(snapshot["latency"]["average_ms"], 3.5)
        self.assertGreaterEqual(snapshot["latency"]["p95_ms"], 3.0)


if __name__ == "__main__":
    unittest.main()
