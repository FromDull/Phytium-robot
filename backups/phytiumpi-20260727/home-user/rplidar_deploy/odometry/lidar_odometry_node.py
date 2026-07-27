#!/usr/bin/env python3
"""ROS 2 node publishing scan-matched odometry and odom/base TF."""

from __future__ import annotations

import json
import math
import socket
import struct
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from scan_matcher import LaserOdometryCore, MatcherConfig


class LidarOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_odometry")
        defaults = {
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            "status_topic": "/lidar_odometry/status",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "laser_frame": "laser",
            "laser_x": 0.0,
            "laser_y": 0.0,
            "laser_z": 0.0,
            "laser_roll": 0.0,
            "laser_pitch": 0.0,
            "laser_yaw": 0.0,
            "processing_interval": 0.08,
            "min_range": 0.20,
            "max_range": 12.0,
            "sample_count": 360,
            "imu_enabled": True,
            "imu_yaw_fusion_weight": 0.35,
            "imu_max_age_ms": 300,
            "imu_tilt_covariance_gain": 3.0,
        }
        matcher_defaults = MatcherConfig()
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name, value in matcher_defaults.__dict__.items():
            self.declare_parameter(name, value)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.laser_frame = str(self.get_parameter("laser_frame").value)
        self.processing_interval = float(self.get_parameter("processing_interval").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.sample_count = int(self.get_parameter("sample_count").value)
        matcher_config = MatcherConfig(
            **{
                name: self.get_parameter(name).value
                for name in matcher_defaults.__dict__
            }
        )
        self.laser_translation = (
            float(self.get_parameter("laser_x").value),
            float(self.get_parameter("laser_y").value),
            float(self.get_parameter("laser_z").value),
        )
        self.laser_rpy = (
            float(self.get_parameter("laser_roll").value),
            float(self.get_parameter("laser_pitch").value),
            float(self.get_parameter("laser_yaw").value),
        )
        self.scan_to_base_xy = self.planar_rotation(*self.laser_rpy)
        # Feed scan points expressed in base_link to the 2D matcher.  This
        # preserves a physically inverted laser (roll=pi), whose planar
        # transform is a reflection and cannot be represented by yaw alone.
        self.core = LaserOdometryCore(matcher_config)
        self.last_processed_stamp: float | None = None
        self.last_scan_monotonic: float | None = None
        self.scan_frame_warned = False
        self.previous_ranges: dict[int, float] | None = None
        self.imu_socket = "/run/rpmsg-broker/rpmsg.sock"
        self.imu_last_time: float | None = None
        self.imu_tilt = (0.0, 0.0)
        self.imu_online = False

        self.odom_publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.on_scan,
            scan_qos,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform(self.laser_translation, self.laser_rpy)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info(
            f"scan odometry ready: {self.odom_frame}->{self.base_frame}->{self.laser_frame}"
        )

    @staticmethod
    def stamp_seconds(msg: LaserScan) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    @staticmethod
    def quaternion_from_rpy(
        roll: float, pitch: float, yaw: float
    ) -> tuple[float, float, float, float]:
        half_roll, half_pitch, half_yaw = roll / 2.0, pitch / 2.0, yaw / 2.0
        cr, sr = math.cos(half_roll), math.sin(half_roll)
        cp, sp = math.cos(half_pitch), math.sin(half_pitch)
        cy, sy = math.cos(half_yaw), math.sin(half_yaw)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    @staticmethod
    def planar_rotation(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        qx, qy, qz, qw = LidarOdometryNode.quaternion_from_rpy(roll, pitch, yaw)
        return (
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy - qz * qw),
            2.0 * (qx * qy + qz * qw),
            1.0 - 2.0 * (qx * qx + qz * qz),
        )

    def publish_static_transform(
        self,
        translation: tuple[float, float, float],
        rpy: tuple[float, float, float],
    ) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.laser_frame
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        qx, qy, qz, qw = self.quaternion_from_rpy(*rpy)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.static_tf_broadcaster.sendTransform(transform)

    def laser_point_to_base(self, x_value: float, y_value: float) -> tuple[float, float]:
        r00, r01, r10, r11 = self.scan_to_base_xy
        return (
            self.laser_translation[0] + r00 * x_value + r01 * y_value,
            self.laser_translation[1] + r10 * x_value + r11 * y_value,
        )

    def scan_points(self, msg: LaserScan) -> tuple[list[tuple[float, float]], dict[int, float]]:
        usable_min = max(float(msg.range_min), self.min_range)
        usable_max = min(float(msg.range_max), self.max_range)
        stride = max(1, len(msg.ranges) // max(1, self.sample_count))
        points = []
        ranges = {}
        for index in range(0, len(msg.ranges), stride):
            distance = float(msg.ranges[index])
            if not math.isfinite(distance) or distance < usable_min or distance > usable_max:
                continue
            angle = float(msg.angle_min) + index * float(msg.angle_increment)
            points.append(
                self.laser_point_to_base(
                    math.cos(angle) * distance, math.sin(angle) * distance
                )
            )
            ranges[index] = distance
        return points, ranges

    def raw_scan_rms(self, ranges: dict[int, float]) -> float | None:
        if self.previous_ranges is None:
            self.previous_ranges = ranges
            return None
        differences = [
            abs(distance - self.previous_ranges[index])
            for index, distance in ranges.items()
            if index in self.previous_ranges
        ]
        self.previous_ranges = ranges
        if len(differences) < 40:
            return None
        differences.sort()
        trim_ratio = float(self.get_parameter("static_trim_ratio").value)
        keep = max(40, int(len(differences) * trim_ratio))
        differences = differences[:keep]
        return math.sqrt(sum(value * value for value in differences) / len(differences))

    def read_imu_delta(self) -> float | None:
        if not bool(self.get_parameter("imu_enabled").value):
            return None
        header = bytes((0xA5, 42, 1, 0))
        request = header + bytes(((-sum(header)) & 0xFF,))
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as connection:
                connection.settimeout(0.03)
                connection.connect(self.imu_socket)
                connection.sendall(request)
                response = connection.recv(128)
            payload = response[4:-1]
            if (len(response) != response[3] + 5 or sum(response) & 0xFF or len(payload) < 52 or payload[0] != 1 or payload[1] != 0 or not payload[2]):
                return None
            age_ms = struct.unpack_from(">I", payload, 48)[0]
            if age_ms > int(self.get_parameter("imu_max_age_ms").value):
                return None
            now = time.monotonic()
            gyro_z = struct.unpack_from(">i", payload, 40)[0] / 1_000_000.0
            self.imu_tilt = (struct.unpack_from(">i", payload, 4)[0] / 1_000_000.0, struct.unpack_from(">i", payload, 12)[0] / 1_000_000.0)
            self.imu_online = True
            delta = None if self.imu_last_time is None else gyro_z * min(0.2, now - self.imu_last_time)
            self.imu_last_time = now
            return delta
        except (OSError, IndexError, struct.error):
            self.imu_online = False
            return None

    def on_scan(self, msg: LaserScan) -> None:
        self.last_scan_monotonic = time.monotonic()
        stamp = self.stamp_seconds(msg)
        if self.last_processed_stamp is not None and stamp - self.last_processed_stamp < self.processing_interval:
            return
        self.last_processed_stamp = stamp
        if msg.header.frame_id and msg.header.frame_id != self.laser_frame and not self.scan_frame_warned:
            self.scan_frame_warned = True
            self.get_logger().warning(
                f"scan frame is {msg.header.frame_id}, configured laser_frame is {self.laser_frame}"
            )

        points, ranges = self.scan_points(msg)
        imu_delta = self.read_imu_delta()
        quality = self.core.update(points, stamp, self.raw_scan_rms(ranges), imu_delta, float(self.get_parameter("imu_yaw_fusion_weight").value))
        self.publish_odometry(msg)
        if not quality.accepted and quality.reason not in {"initialized", "reference_reset"}:
            self.get_logger().warning(
                f"scan match rejected: {quality.reason}", throttle_duration_sec=5.0
            )

    def publish_odometry(self, scan: LaserScan) -> None:
        x_value, y_value, yaw = self.core.base_pose
        vx, vy, wz = self.core.velocity
        accepted = self.core.quality.accepted
        tilt = math.hypot(*self.imu_tilt)
        tilt_scale = 1.0 + float(self.get_parameter("imu_tilt_covariance_gain").value) * tilt * tilt
        position_variance = (0.003 if accepted else 0.25) * tilt_scale
        yaw_variance = (0.008 if accepted else 0.50) * tilt_scale

        odom = Odometry()
        odom.header.stamp = scan.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x_value
        odom.pose.pose.position.y = y_value
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.pose.covariance[0] = position_variance
        odom.pose.covariance[7] = position_variance * 2.0
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = yaw_variance
        odom.twist.covariance = list(odom.pose.covariance)
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = x_value
        transform.transform.translation.y = y_value
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def publish_status(self) -> None:
        age = None
        if self.last_scan_monotonic is not None:
            age = time.monotonic() - self.last_scan_monotonic
        payload = {
            "online": age is not None and age < 1.0,
            "scan_age_ms": None if age is None else round(age * 1000.0),
            "pose": {
                "x": self.core.base_pose[0],
                "y": self.core.base_pose[1],
                "yaw": self.core.base_pose[2],
            },
            "velocity": {
                "x": self.core.velocity[0],
                "y": self.core.velocity[1],
                "yaw": self.core.velocity[2],
            },
            "quality": self.core.quality.to_dict(),
            "accepted": self.core.accepted_count,
            "rejected": self.core.rejected_count,
            "consecutive_rejections": self.core.consecutive_rejections,
            "imu": {"online": self.imu_online, "roll_rad": self.imu_tilt[0], "pitch_rad": self.imu_tilt[1]},
        }
        self.status_publisher.publish(String(data=json.dumps(payload, separators=(",", ":"))))


def main() -> None:
    rclpy.init()
    node = LidarOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
