import math
import unittest

from gimbal_camera_tf_ros2.node import (
    calibration_from_payload,
    quaternion_from_rpy,
    state_validity,
)


class GimbalCameraTfStateTests(unittest.TestCase):
    def test_calibration_supports_frames_axes_and_offsets(self):
        calibration = calibration_from_payload({
            "base_frame": "base_custom",
            "yaw_sign": -1,
            "pitch_offset_deg": 2.5,
            "base_to_yaw_xyz": [1, 2, 3],
        })
        self.assertEqual(calibration["base_frame"], "base_custom")
        self.assertEqual(calibration["yaw_sign"], -1.0)
        self.assertEqual(calibration["pitch_offset_deg"], 2.5)
        self.assertEqual(calibration["base_to_yaw_xyz"], [1.0, 2.0, 3.0])

    def test_state_validity_rejects_stale_and_invalid_pose(self):
        self.assertTrue(state_validity(
            {"updated_at": 9.8, "pose_valid": True}, 10.0, 0.5
        )[0])
        self.assertFalse(state_validity(
            {"updated_at": 9.0, "pose_valid": True}, 10.0, 0.5
        )[0])
        self.assertFalse(state_validity(
            {"updated_at": 9.9, "pose_valid": False}, 10.0, 0.5
        )[0])

    def test_yaw_quaternion_is_normalized(self):
        quaternion = quaternion_from_rpy(0.0, 0.0, math.pi / 2)
        self.assertTrue(math.isclose(
            sum(value * value for value in quaternion), 1.0
        ))
        self.assertTrue(math.isclose(quaternion[2], math.sqrt(0.5)))
        self.assertTrue(math.isclose(quaternion[3], math.sqrt(0.5)))


if __name__ == "__main__":
    unittest.main()
