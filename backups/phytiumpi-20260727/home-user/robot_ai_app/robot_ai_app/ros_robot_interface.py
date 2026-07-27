"""ROS 2 interface used by the AI control layer."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover - lets pure unit tests import this module without Nav2.
    NavigateToPose = None
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .safety import MotionLimits


@dataclass(frozen=True)
class RosTopics:
    cmd_vel: str = "/cmd_vel"
    status: str = "/wheel_leg/status"
    camera: str = "/camera/image_raw"
    odom: str = "/odom"
    navigate_action: str = "navigate_to_pose"


class RobotRosInterface(Node):
    """Small synchronous facade over ROS topics/actions.

    The class is a ROS node, but its public methods are blocking helpers so CLI
    tools and AI policies can be written like normal application code.
    """

    def __init__(
        self,
        node_name: str = "robot_ai_control",
        topics: RosTopics | None = None,
        limits: MotionLimits | None = None,
    ):
        super().__init__(node_name)
        self.topics = topics or RosTopics()
        self.limits = limits or MotionLimits()

        self._last_status: dict[str, Any] = {}
        self._last_status_time: float | None = None
        self._last_image: Image | None = None
        self._last_image_time: float | None = None
        self._last_odom: Odometry | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cmd_pub = self.create_publisher(Twist, self.topics.cmd_vel, 10)
        self._status_sub = self.create_subscription(String, self.topics.status, self._on_status, 10)
        self._odom_sub = self.create_subscription(Odometry, self.topics.odom, self._on_odom, 10)

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._camera_sub = self.create_subscription(Image, self.topics.camera, self._on_image, camera_qos)

        self._nav_client = None
        if NavigateToPose is not None:
            self._nav_client = ActionClient(self, NavigateToPose, self.topics.navigate_action)

    def spin_for(self, duration: float) -> None:
        deadline = time.monotonic() + max(0.0, duration)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def state(self) -> dict[str, Any]:
        self.spin_for(0.20)
        odom = odom_to_dict(self._last_odom)
        now = time.monotonic()
        nav_server_available = False
        if self._nav_client is not None:
            nav_server_available = self._nav_client.wait_for_server(timeout_sec=0.0)
        return {
            "ok": True,
            "mode": "ros2",
            "status": self._last_status,
            "status_age_sec": None if self._last_status_time is None else now - self._last_status_time,
            "command": {
                "cmd_vel_topic": self.topics.cmd_vel,
                "status_topic": self.topics.status,
            },
            "camera": {
                "topic": self.topics.camera,
                "available": self._last_image is not None,
                "age_sec": None if self._last_image_time is None else now - self._last_image_time,
                "width": None if self._last_image is None else self._last_image.width,
                "height": None if self._last_image is None else self._last_image.height,
                "encoding": None if self._last_image is None else self._last_image.encoding,
            },
            "odom": odom,
            "pose": self.current_pose("map", "base_link", timeout=0.0),
            "nav": {
                "navigate_action": self.topics.navigate_action,
                "available": nav_server_available,
                "odom_available": odom is not None,
                "status_available": bool(self._last_status),
            },
            "limits": self.limits.__dict__,
        }

    def move_base(self, vx: float, wz: float, duration: float) -> dict[str, Any]:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)

        deadline = time.monotonic() + max(0.0, float(duration))
        publish_count = 0
        while time.monotonic() < deadline:
            self._cmd_pub.publish(msg)
            publish_count += 1
            rclpy.spin_once(self, timeout_sec=0.02)

        self.stop()
        return {"ok": True, "vx": float(vx), "wz": float(wz), "duration": float(duration), "published": publish_count}

    def rotate_in_place(self, angle_rad: float, timeout: float = 30.0, tolerance_rad: float = 0.035) -> dict[str, Any]:
        start = self.current_pose("map", "base_link", timeout=1.0)
        if start is None:
            return {"ok": False, "error": "cannot resolve TF map->base_link"}

        target_yaw = normalize_angle(float(start["yaw"]) + float(angle_rad))
        direction = 1.0 if angle_rad >= 0.0 else -1.0
        max_wz = min(abs(self.limits.max_wz), 0.70)
        min_wz = 0.18
        deadline = time.monotonic() + max(0.1, float(timeout))
        publish_count = 0

        while time.monotonic() < deadline:
            pose = self.current_pose("map", "base_link", timeout=0.05)
            if pose is None:
                self.stop()
                return {"ok": False, "error": "lost TF map->base_link during rotation"}

            remaining = normalize_angle(target_yaw - float(pose["yaw"]))
            if abs(remaining) <= tolerance_rad:
                self.stop()
                return {
                    "ok": True,
                    "angle_rad": float(angle_rad),
                    "target_yaw": target_yaw,
                    "final_yaw": float(pose["yaw"]),
                    "remaining_rad": remaining,
                    "published": publish_count,
                    "mode": "in_place_tf_feedback",
                }

            msg = Twist()
            speed = max(min_wz, min(max_wz, abs(remaining) * 1.15))
            msg.angular.z = speed * direction
            self._cmd_pub.publish(msg)
            publish_count += 1
            rclpy.spin_once(self, timeout_sec=0.03)

        self.stop()
        return {"ok": False, "error": "rotation timeout", "angle_rad": float(angle_rad), "target_yaw": target_yaw}

    def settle_to_position(
        self,
        x: float,
        y: float,
        timeout: float = 12.0,
        tolerance_m: float = 0.02,
    ) -> dict[str, Any]:
        target_x = float(x)
        target_y = float(y)
        deadline = time.monotonic() + max(0.1, float(timeout))
        corrections = 0

        while time.monotonic() < deadline:
            pose = self.current_pose("map", "base_link", timeout=0.1)
            if pose is None:
                self.stop()
                return {"ok": False, "error": "cannot resolve TF map->base_link during final settle"}

            dx = target_x - float(pose["x"])
            dy = target_y - float(pose["y"])
            distance = math.hypot(dx, dy)
            if distance <= tolerance_m:
                self.stop()
                return {
                    "ok": True,
                    "target": {"x": target_x, "y": target_y},
                    "final": pose,
                    "position_error_m": distance,
                    "corrections": corrections,
                    "mode": "tf_position_settle",
                }

            target_heading = math.atan2(dy, dx)
            heading_error = normalize_angle(target_heading - float(pose["yaw"]))
            if abs(heading_error) > 0.08:
                rotate = self.rotate_in_place(heading_error, timeout=min(4.0, max(1.0, deadline - time.monotonic())))
                if not rotate.get("ok"):
                    return rotate
                continue

            msg = Twist()
            msg.linear.x = min(0.014, max(0.003, distance * 0.35))
            self._cmd_pub.publish(msg)
            corrections += 1
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop()
        pose = self.current_pose("map", "base_link", timeout=0.1)
        error = None
        if pose is not None:
            error = math.hypot(target_x - float(pose["x"]), target_y - float(pose["y"]))
        return {"ok": False, "error": "final position settle timeout", "position_error_m": error}

    def stop(self) -> dict[str, Any]:
        msg = Twist()
        for _ in range(3):
            self._cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        return {"ok": True, "stopped": True}

    def capture_image(self, output_dir: str | Path = "captures", prefix: str = "frame", timeout: float = 2.0) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        while self._last_image is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._last_image is None:
            raise RuntimeError(f"no image received on {self.topics.camera}")

        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        image_path = output_path / f"{prefix}_{int(time.time() * 1000)}.jpg"
        path, fmt = write_image_file(self._last_image, image_path)
        return {"ok": True, "path": str(path), "format": fmt, "width": self._last_image.width, "height": self._last_image.height}

    def navigate_to_pose(self, x: float, y: float, yaw: float | None = None, timeout: float = 60.0) -> dict[str, Any]:
        if self._nav_client is None or NavigateToPose is None:
            return {"ok": False, "error": "nav2_msgs is unavailable"}

        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            return {"ok": False, "error": f"Nav2 action server not available: {self.topics.navigate_action}"}

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        yaw_value = self._default_goal_yaw() if yaw is None else float(yaw)
        goal.pose.pose.orientation = yaw_to_quaternion(yaw_value)

        send_future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {"ok": False, "error": "navigation goal rejected"}

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=float(timeout))
        result = result_future.result()
        if result is None:
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            return {"ok": False, "error": "navigation timeout; goal cancelled"}
        status = int(result.status)
        ok = status == GoalStatus.STATUS_SUCCEEDED
        return {
            "ok": ok,
            "status": status,
            "status_text": goal_status_text(status),
            "x": float(x),
            "y": float(y),
            "yaw": yaw_value,
            "orientation_policy": "position_first",
        }

    def navigate_relative(self, distance_m: float = 0.0, yaw_delta: float = 0.0, timeout: float = 60.0) -> dict[str, Any]:
        pose = self.current_pose("map", "base_link", timeout=1.0)
        if pose is None:
            return {"ok": False, "error": "cannot resolve TF map->base_link"}

        target_yaw = normalize_angle(float(pose["yaw"]) + float(yaw_delta))
        target_x = float(pose["x"]) + float(distance_m) * math.cos(float(pose["yaw"]))
        target_y = float(pose["y"]) + float(distance_m) * math.sin(float(pose["yaw"]))

        result = self.navigate_to_pose(target_x, target_y, target_yaw, timeout)
        result["relative"] = {
            "distance_m": float(distance_m),
            "yaw_delta": float(yaw_delta),
            "start": pose,
            "target": {"x": target_x, "y": target_y, "yaw": target_yaw},
        }
        return result

    def _default_goal_yaw(self) -> float:
        pose = self.current_pose("map", "base_link", timeout=0.2)
        if pose is not None:
            return float(pose["yaw"])
        return 0.0

    def current_pose(self, target_frame: str = "map", source_frame: str = "base_link", timeout: float = 0.2) -> dict[str, float] | None:
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            try:
                transform = self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
                t = transform.transform.translation
                q = transform.transform.rotation
                return {
                    "x": float(t.x),
                    "y": float(t.y),
                    "z": float(t.z),
                    "yaw": quaternion_to_yaw(q.x, q.y, q.z, q.w),
                    "target_frame": target_frame,
                    "source_frame": source_frame,
                }
            except TransformException:
                if time.monotonic() >= deadline:
                    return None
                rclpy.spin_once(self, timeout_sec=0.02)

    def _on_status(self, msg: String) -> None:
        try:
            self._last_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self._last_status = {"raw": msg.data, "parse_error": True}
        self._last_status_time = time.monotonic()

    def _on_image(self, msg: Image) -> None:
        self._last_image = msg
        self._last_image_time = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def odom_to_dict(msg: Odometry | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return {
        "frame_id": msg.header.frame_id,
        "child_frame_id": msg.child_frame_id,
        "position": {"x": p.x, "y": p.y, "z": p.z},
        "orientation": {"x": q.x, "y": q.y, "z": q.z, "w": q.w},
    }


def goal_status_text(status: int) -> str:
    names = {
        GoalStatus.STATUS_UNKNOWN: "unknown",
        GoalStatus.STATUS_ACCEPTED: "accepted",
        GoalStatus.STATUS_EXECUTING: "executing",
        GoalStatus.STATUS_CANCELING: "canceling",
        GoalStatus.STATUS_SUCCEEDED: "succeeded",
        GoalStatus.STATUS_CANCELED: "canceled",
        GoalStatus.STATUS_ABORTED: "aborted",
    }
    return names.get(status, f"status_{status}")


def write_image_file(msg: Image, preferred_path: Path) -> tuple[Path, str]:
    try:
        return write_jpeg_with_cv2(msg, preferred_path), "jpeg"
    except Exception:
        return write_portable_image(msg, preferred_path.with_suffix(".ppm")), "ppm"


def write_jpeg_with_cv2(msg: Image, path: Path) -> Path:
    import cv2  # type: ignore
    import numpy as np  # type: ignore

    channels = channels_for_encoding(msg.encoding)
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.step))
    pixels = data[:, : msg.width * channels].reshape((msg.height, msg.width, channels))
    if msg.encoding.lower() == "rgb8":
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    elif msg.encoding.lower() == "rgba8":
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
    elif msg.encoding.lower() == "bgra8":
        pixels = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
    elif msg.encoding.lower() == "mono8":
        pass
    elif msg.encoding.lower() != "bgr8":
        raise ValueError(f"unsupported image encoding for jpeg: {msg.encoding}")
    ok, encoded = cv2.imencode(".jpg", pixels)
    if not ok:
        raise RuntimeError("cv2 failed to encode jpeg")
    path.write_bytes(encoded.tobytes())
    return path


def write_portable_image(msg: Image, path: Path) -> Path:
    encoding = msg.encoding.lower()
    if encoding not in {"rgb8", "bgr8", "rgba8", "bgra8", "mono8"}:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")

    channels = channels_for_encoding(encoding)
    rows: list[bytes] = []
    raw = bytes(msg.data)
    for row_index in range(msg.height):
        row = raw[row_index * msg.step : row_index * msg.step + msg.width * channels]
        if encoding == "bgr8":
            row = b"".join(bytes((row[i + 2], row[i + 1], row[i])) for i in range(0, len(row), 3))
        elif encoding in {"rgba8", "bgra8"}:
            converted = []
            for i in range(0, len(row), 4):
                if encoding == "rgba8":
                    converted.append(bytes((row[i], row[i + 1], row[i + 2])))
                else:
                    converted.append(bytes((row[i + 2], row[i + 1], row[i])))
            row = b"".join(converted)
        rows.append(row)

    if encoding == "mono8":
        path = path.with_suffix(".pgm")
        header = f"P5\n{msg.width} {msg.height}\n255\n".encode("ascii")
    else:
        header = f"P6\n{msg.width} {msg.height}\n255\n".encode("ascii")
    path.write_bytes(header + b"".join(rows))
    return path


def channels_for_encoding(encoding: str) -> int:
    value = encoding.lower()
    if value in {"rgb8", "bgr8"}:
        return 3
    if value in {"rgba8", "bgra8"}:
        return 4
    if value == "mono8":
        return 1
    raise ValueError(f"unsupported image encoding: {encoding}")
