from __future__ import annotations

import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


DEFAULTS = {
    "state_file": "/root/robot_data/gimbal_tf_state.json",
    "base_frame": "base_link",
    "yaw_frame": "gimbal_yaw_link",
    "pitch_frame": "gimbal_pitch_link",
    "camera_frame": "camera_link",
    "optical_frame": "camera_color_optical_frame",
    "base_to_yaw_xyz": [0.0, 0.0, 0.1455],
    "pitch_to_camera_xyz": [0.00295, 0.0, 0.038],
    "yaw_sign": 1.0,
    "pitch_sign": -1.0,
    "yaw_offset_deg": 0.0,
    "pitch_offset_deg": 0.0,
    "maximum_state_age_s": 0.5,
}


def quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def calibration_from_payload(payload: object) -> dict:
    calibration = dict(DEFAULTS)
    if not isinstance(payload, dict):
        return calibration
    for name in (
        "state_file", "base_frame", "yaw_frame", "pitch_frame",
        "camera_frame", "optical_frame",
    ):
        if isinstance(payload.get(name), str) and payload[name]:
            calibration[name] = payload[name]
    for name in ("base_to_yaw_xyz", "pitch_to_camera_xyz"):
        value = payload.get(name)
        if isinstance(value, list) and len(value) == 3:
            calibration[name] = [float(item) for item in value]
    for name in (
        "yaw_sign", "pitch_sign", "yaw_offset_deg", "pitch_offset_deg",
        "maximum_state_age_s",
    ):
        if name in payload:
            calibration[name] = float(payload[name])
    return calibration


def state_validity(state: object, now: float, maximum_age_s: float) -> tuple[bool, str, float]:
    if not isinstance(state, dict):
        return False, "state is not an object", math.inf
    try:
        age = max(0.0, now - float(state["updated_at"]))
    except (KeyError, TypeError, ValueError):
        return False, "state has no valid timestamp", math.inf
    if age > maximum_age_s:
        return False, f"state is stale ({age:.3f}s)", age
    if not state.get("pose_valid"):
        return False, str(state.get("error", "gimbal pose is invalid")), age
    return True, "ok", age


class GimbalCameraTf(Node):
    def __init__(self):
        super().__init__("gimbal_camera_tf")
        self.declare_parameter("calibration_file", "/root/robot_data/gimbal_tf_calibration.json")
        self.declare_parameter("status_topic", "/gimbal/tf_status")
        self.declare_parameter("publish_rate_hz", 10.0)
        calibration_path = Path(str(self.get_parameter("calibration_file").value))
        payload = None
        if calibration_path.is_file():
            try:
                payload = json.loads(calibration_path.read_text(encoding="utf-8"))
            except (OSError, ValueError) as error:
                self.get_logger().warning(
                    f"failed to read calibration file {calibration_path}: {error}"
                )
        self.calibration = calibration_from_payload(payload)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self.state_path = Path(self.calibration["state_file"])
        self.dynamic = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.last_warning = 0.0
        self.last_sequence = None
        self._publish_static()
        self.timer = self.create_timer(1.0 / rate_hz, self._update)
        self.get_logger().info(
            f"Gimbal camera TF ready: {self.calibration['base_frame']} -> "
            f"{self.calibration['optical_frame']}"
        )

    def _transform(self, parent, child, xyz, quaternion, stamp=None):
        message = TransformStamped()
        message.header.stamp = stamp or self.get_clock().now().to_msg()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x = float(xyz[0])
        message.transform.translation.y = float(xyz[1])
        message.transform.translation.z = float(xyz[2])
        (
            message.transform.rotation.x,
            message.transform.rotation.y,
            message.transform.rotation.z,
            message.transform.rotation.w,
        ) = quaternion
        return message

    def _publish_static(self):
        identity = (0.0, 0.0, 0.0, 1.0)
        optical_rotation = quaternion_from_rpy(-math.pi / 2, 0.0, -math.pi / 2)
        self.static.sendTransform([
            self._transform(
                self.calibration["pitch_frame"], self.calibration["camera_frame"],
                self.calibration["pitch_to_camera_xyz"], identity,
            ),
            self._transform(
                self.calibration["camera_frame"], self.calibration["optical_frame"],
                (0.0, 0.0, 0.0), optical_rotation,
            ),
        ])

    @staticmethod
    def _stamp_from_seconds(value: float):
        from builtin_interfaces.msg import Time

        seconds = int(value)
        nanoseconds = int((value - seconds) * 1_000_000_000)
        return Time(sec=seconds, nanosec=max(0, min(999_999_999, nanoseconds)))

    def _read_state(self):
        return json.loads(self.state_path.read_text(encoding="utf-8"))

    def _publish_status(self, state, valid: bool, reason: str, age: float, published: bool):
        payload = {
            "valid": valid,
            "reason": reason,
            "state_age_s": None if not math.isfinite(age) else round(age, 4),
            "transform_published": published,
            "sequence": state.get("sequence") if isinstance(state, dict) else None,
            "active": bool(state.get("active")) if isinstance(state, dict) else False,
            "yaw_deg": state.get("yaw_deg") if isinstance(state, dict) else None,
            "pitch_deg": state.get("pitch_deg") if isinstance(state, dict) else None,
        }
        self.status_publisher.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )

    def _update(self):
        try:
            state = self._read_state()
        except (OSError, ValueError) as error:
            state = None
            valid, reason, age = False, str(error), math.inf
        else:
            valid, reason, age = state_validity(
                state, time.time(), float(self.calibration["maximum_state_age_s"])
            )

        published = False
        if valid:
            sequence = state.get("sequence")
            if sequence != self.last_sequence:
                yaw = math.radians(
                    float(self.calibration["yaw_sign"])
                    * (float(state["yaw_deg"]) + float(self.calibration["yaw_offset_deg"]))
                )
                pitch = math.radians(
                    float(self.calibration["pitch_sign"])
                    * (float(state["pitch_deg"]) + float(self.calibration["pitch_offset_deg"]))
                )
                stamp = self._stamp_from_seconds(float(state["updated_at"]))
                self.dynamic.sendTransform([
                    self._transform(
                        self.calibration["base_frame"], self.calibration["yaw_frame"],
                        self.calibration["base_to_yaw_xyz"],
                        quaternion_from_rpy(0.0, 0.0, yaw), stamp,
                    ),
                    self._transform(
                        self.calibration["yaw_frame"], self.calibration["pitch_frame"],
                        (0.0, 0.0, 0.0), quaternion_from_rpy(0.0, pitch, 0.0), stamp,
                    ),
                ])
                self.last_sequence = sequence
                published = True
        else:
            now = time.monotonic()
            if now - self.last_warning > 10.0:
                self.get_logger().warning(reason)
                self.last_warning = now
        self._publish_status(state, valid, reason, age, published)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalCameraTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
