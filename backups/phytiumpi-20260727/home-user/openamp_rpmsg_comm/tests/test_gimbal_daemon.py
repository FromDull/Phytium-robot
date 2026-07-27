import importlib.util
from dataclasses import replace
from pathlib import Path
import struct
import sys
import tempfile
import unittest


MODULE_PATH = Path(__file__).parents[1] / "linux_user" / "gimbal_daemon.py"
SPEC = importlib.util.spec_from_file_location("gimbal_daemon", MODULE_PATH)
assert SPEC and SPEC.loader
gimbal_daemon = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = gimbal_daemon
SPEC.loader.exec_module(gimbal_daemon)


class ProtocolTests(unittest.TestCase):
    def test_frame_round_trip(self):
        frame = gimbal_daemon.encode_frame(72, 9, b"abc")
        self.assertEqual(sum(frame) & 0xFF, 0)
        self.assertEqual(gimbal_daemon.decode_frame(frame), (72, 9, b"abc"))

    def test_bad_checksum_is_rejected(self):
        frame = bytearray(gimbal_daemon.encode_frame(73, 1))
        frame[-1] ^= 1
        with self.assertRaises(gimbal_daemon.GimbalError):
            gimbal_daemon.decode_frame(bytes(frame))

    def test_telemetry_offsets(self):
        payload = bytearray(68)
        payload[0:7] = bytes((1, 0, 3, 0, 0x0F, 0x03, 50))
        struct.pack_into(">i", payload, 8, 123)
        struct.pack_into(">i", payload, 12, -231)
        struct.pack_into(">H", payload, 48, 5)
        struct.pack_into(">I", payload, 52, 12)
        struct.pack_into(">I", payload, 56, 13)
        parsed = gimbal_daemon.Telemetry.parse(bytes(payload))
        self.assertEqual(parsed.state, 3)
        self.assertEqual(parsed.yaw_deg, 1.23)
        self.assertEqual(parsed.pitch_deg, -2.31)
        self.assertEqual(parsed.command_speed_rpm, 5)
        self.assertEqual(parsed.feedback_valid_mask, 0x03)


class ConfigTests(unittest.TestCase):
    CONFIG = """\
yaw_min_deg=-37.94
yaw_max_deg=213.97
pitch_min_deg=-50.13
pitch_max_deg=97.50
home_speed_rpm=5
home_torque_percent=50
return_speed_rpm=5
return_torque_percent=50
move_speed_rpm=20
move_torque_percent=50
"""

    def test_config_loads(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gimbal.conf"
            path.write_text(self.CONFIG, encoding="utf-8")
            config = gimbal_daemon.GimbalConfig.load(str(path))
        self.assertEqual(config.move_torque_percent, 50)
        self.assertEqual(config.pitch_min_deg, -50.13)

    def test_incomplete_config_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gimbal.conf"
            path.write_text("yaw_min_deg=-10\n", encoding="utf-8")
            with self.assertRaises(gimbal_daemon.GimbalError):
                gimbal_daemon.GimbalConfig.load(str(path))


class FakeGimbal:
    def __init__(self, telemetry):
        self.telemetry = telemetry
        self.config = type("Config", (), {
            "move_speed_rpm": 20,
            "move_torque_percent": 50,
            "yaw_min_deg": -37.94,
            "yaw_max_deg": 213.97,
            "pitch_min_deg": -50.13,
            "pitch_max_deg": 97.50,
        })()
        self.targets = []
        self.estop_count = 0
        self.disable_count = 0

    def status(self):
        return self.telemetry

    def set_target(self, yaw, pitch, speed_rpm, torque_percent):
        self.targets.append((yaw, pitch, speed_rpm, torque_percent))
        self.telemetry = replace(
            self.telemetry,
            yaw_deg=yaw,
            pitch_deg=pitch,
            yaw_target_deg=yaw,
            pitch_target_deg=pitch,
        )
        return self.telemetry

    def enable(self):
        return self.telemetry

    def disable(self):
        self.disable_count += 1
        return self.telemetry

    def estop(self):
        self.estop_count += 1
        return self.telemetry


class ServiceSafetyTests(unittest.TestCase):
    def active_telemetry(self, **overrides):
        values = dict(
            command_status=0, state=3, fault=0, limits_valid_mask=0x0F,
            feedback_valid_mask=0x03, torque_percent=50, yaw_deg=0.0,
            pitch_deg=0.0, yaw_speed_rpm=0, pitch_speed_rpm=0,
            yaw_current_a=0.0, pitch_current_a=0.0, yaw_target_deg=0.0,
            pitch_target_deg=0.0, yaw_min_deg=-37.94,
            yaw_max_deg=213.97, pitch_min_deg=-50.13,
            pitch_max_deg=97.5, command_speed_rpm=5,
            yaw_feedback_age_ms=10, pitch_feedback_age_ms=10,
            timeout_remaining_ms=0, startup_pitch_deg=0.0,
        )
        values.update(overrides)
        return gimbal_daemon.Telemetry(**values)

    def test_set_uses_verified_speed_and_torque(self):
        fake = FakeGimbal(self.active_telemetry())
        response = gimbal_daemon.GimbalService(fake, 10.0).handle(
            {"command": "set", "yaw_deg": 2, "pitch_deg": -2}
        )
        self.assertTrue(response["ok"])
        self.assertEqual(fake.targets, [(2.0, -2.0, 20, 50)])

    def test_workspace_violation_is_rejected(self):
        service = gimbal_daemon.GimbalService(
            FakeGimbal(self.active_telemetry()), 3.0
        )
        with self.assertRaises(gimbal_daemon.GimbalError):
            service.handle({"command": "set", "yaw_deg": 212, "pitch_deg": 0})

    def test_asymmetric_workspace_accepts_large_safe_yaw(self):
        fake = FakeGimbal(self.active_telemetry())
        response = gimbal_daemon.GimbalService(fake, 3.0).handle(
            {"command": "set", "yaw_deg": 180, "pitch_deg": 0}
        )
        self.assertTrue(response["ok"])
        self.assertEqual(fake.targets, [(180.0, 0.0, 20, 50)])

    def test_fault_is_reported_before_inactive_state(self):
        service = gimbal_daemon.GimbalService(
            FakeGimbal(self.active_telemetry(state=6, fault=0x80)), 3.0
        )
        with self.assertRaisesRegex(
            gimbal_daemon.GimbalError, "state=6 fault=0x80"
        ):
            service.handle({"command": "set", "yaw_deg": 2, "pitch_deg": 0})

    def test_stale_feedback_is_rejected(self):
        service = gimbal_daemon.GimbalService(
            FakeGimbal(self.active_telemetry(yaw_feedback_age_ms=500)), 10.0
        )
        with self.assertRaises(gimbal_daemon.GimbalError):
            service.handle({"command": "center"})

    def test_unsafe_enable_is_emergency_stopped(self):
        fake = FakeGimbal(self.active_telemetry(startup_pitch_deg=98.0))
        service = gimbal_daemon.GimbalService(fake, 10.0)
        with self.assertRaises(gimbal_daemon.GimbalError):
            service.handle({"command": "enable", "confirm": True})
        self.assertEqual(fake.estop_count, 1)

    def test_calibrated_resting_pitch_is_accepted(self):
        fake = FakeGimbal(self.active_telemetry(startup_pitch_deg=97.35))
        service = gimbal_daemon.GimbalService(fake, 10.0)
        response = service.handle({"command": "enable", "confirm": True})
        self.assertTrue(response["ok"])
        self.assertEqual(fake.estop_count, 0)

    def test_unsafe_disable_is_rejected(self):
        fake = FakeGimbal(self.active_telemetry(startup_pitch_deg=98.0))
        service = gimbal_daemon.GimbalService(fake, 10.0)
        with self.assertRaises(gimbal_daemon.GimbalError):
            service.handle({"command": "disable", "confirm": True})
        self.assertEqual(fake.disable_count, 0)

    def test_shutdown_estops_when_return_position_is_unsafe(self):
        fake = FakeGimbal(self.active_telemetry(startup_pitch_deg=98.0))
        service = gimbal_daemon.GimbalService(fake, 10.0)
        service.controlled_shutdown()
        self.assertEqual(fake.estop_count, 1)

    def test_shutdown_returns_to_calibrated_resting_pitch(self):
        fake = FakeGimbal(self.active_telemetry(startup_pitch_deg=97.35))
        service = gimbal_daemon.GimbalService(fake, 10.0)
        service.controlled_shutdown()
        self.assertEqual(fake.disable_count, 1)
        self.assertEqual(fake.estop_count, 0)


if __name__ == "__main__":
    unittest.main()
