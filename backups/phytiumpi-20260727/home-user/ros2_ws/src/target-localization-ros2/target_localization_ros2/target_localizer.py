#!/usr/bin/env python3
import json
import math
import statistics
import struct
import threading
import time
import urllib.error
import urllib.request
from collections import defaultdict, deque

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def rotate_vector(vector, quaternion):
    """Rotate a 3D vector by a geometry_msgs Quaternion."""
    x, y, z = vector
    qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def observation_quality(confidence, depth_m, sample_count, depth_mad, minimum_samples):
    """Return a conservative 0..1 fusion score for one RGB-D observation."""
    confidence = max(0.0, min(1.0, float(confidence)))
    sample_score = min(1.0, max(0.0, float(sample_count)) / max(1.0, minimum_samples * 3.0))
    mad_limit = max(0.03, min(0.12, 0.04 * max(1.0, float(depth_m))))
    mad_score = max(0.0, 1.0 - max(0.0, float(depth_mad)) / mad_limit)
    return max(
        0.0,
        min(
            1.0,
            confidence
            * (0.70 + 0.30 * sample_score)
            * (0.60 + 0.40 * mad_score),
        ),
    )


class TargetLocalizer(Node):
    def __init__(self):
        super().__init__("target_localizer")
        self.declare_parameter(
            "aligned_depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter(
            "camera_info_topic", "/camera/aligned_depth_to_color/camera_info"
        )
        self.declare_parameter("detections_url", "http://127.0.0.1:8091/detections")
        self.declare_parameter("target_frames", ["base_link", "map"])
        self.declare_parameter("gimbal_tf_status_topic", "/gimbal/tf_status")
        self.declare_parameter("maximum_tf_status_age_s", 1.5)
        self.declare_parameter("maximum_transform_age_s", 1.5)
        self.declare_parameter("minimum_map_tf_stable_s", 0.8)
        self.declare_parameter("minimum_depth_m", 0.20)
        self.declare_parameter("maximum_depth_m", 8.0)
        self.declare_parameter("minimum_samples", 12)
        self.declare_parameter("poll_period_s", 0.25)
        self.declare_parameter("maximum_data_age_s", 2.0)

        self.depth_topic = self.get_parameter("aligned_depth_topic").value
        self.info_topic = self.get_parameter("camera_info_topic").value
        self.detections_url = self.get_parameter("detections_url").value
        self.target_frames = list(self.get_parameter("target_frames").value)
        self.maximum_tf_status_age = float(
            self.get_parameter("maximum_tf_status_age_s").value
        )
        self.maximum_transform_age = float(
            self.get_parameter("maximum_transform_age_s").value
        )
        self.minimum_map_tf_stable = float(
            self.get_parameter("minimum_map_tf_stable_s").value
        )
        self.minimum_depth = float(self.get_parameter("minimum_depth_m").value)
        self.maximum_depth = float(self.get_parameter("maximum_depth_m").value)
        self.minimum_samples = int(self.get_parameter("minimum_samples").value)
        self.maximum_data_age = float(self.get_parameter("maximum_data_age_s").value)

        self.lock = threading.Lock()
        self.latest_info = None
        self.last_detection_update = None
        self.pending_detection = None
        self.depth_history = defaultdict(lambda: deque(maxlen=5))
        self.last_http_warning = 0.0
        self.last_tf_warning = {}
        self.gimbal_tf_status = None
        self.gimbal_tf_status_received = None
        self.map_tf_valid_since = None

        self.depth_subscription = None
        self.info_subscription = self.create_subscription(
            CameraInfo, self.info_topic, self.on_camera_info, qos_profile_sensor_data
        )
        self.gimbal_tf_status_subscription = self.create_subscription(
            String,
            str(self.get_parameter("gimbal_tf_status_topic").value),
            self.on_gimbal_tf_status,
            10,
        )
        self.targets_publisher = self.create_publisher(String, "/vision/targets_3d", 10)
        self.tracking_publisher = self.create_publisher(
            PointStamped, "/vision/tracking_target", 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(
            float(self.get_parameter("poll_period_s").value), self.process_detections
        )
        self.get_logger().info(
            f"Target localization ready: {self.depth_topic} + {self.detections_url}"
        )

    def on_depth(self, message):
        with self.lock:
            result = self.pending_detection
            info_state = self.latest_info
            self.pending_detection = None
            subscription = self.depth_subscription
            self.depth_subscription = None
        if subscription is not None:
            self.destroy_subscription(subscription)
        if result is None or info_state is None:
            return
        info, info_received_at = info_state
        self.localize(result, message, info, info_received_at)

    def on_camera_info(self, message):
        with self.lock:
            self.latest_info = (message, time.monotonic())

    def on_gimbal_tf_status(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(status, dict):
            return
        with self.lock:
            self.gimbal_tf_status = status
            self.gimbal_tf_status_received = time.monotonic()

    def fetch_detections(self):
        request = urllib.request.Request(
            self.detections_url, headers={"Accept": "application/json"}
        )
        with urllib.request.urlopen(request, timeout=0.35) as response:
            return json.loads(response.read())

    def depth_at(self, message, u, v):
        offset = v * message.step
        if message.encoding == "16UC1":
            fmt = ">H" if message.is_bigendian else "<H"
            return struct.unpack_from(fmt, message.data, offset + u * 2)[0] * 0.001
        if message.encoding == "32FC1":
            fmt = ">f" if message.is_bigendian else "<f"
            return float(struct.unpack_from(fmt, message.data, offset + u * 4)[0])
        return math.nan

    def stable_depth(self, message, box, source_width, source_height, class_id):
        scale_x = message.width / max(1.0, float(source_width))
        scale_y = message.height / max(1.0, float(source_height))
        x1, y1, x2, y2 = box
        x1, x2 = sorted((x1 * scale_x, x2 * scale_x))
        y1, y2 = sorted((y1 * scale_y, y2 * scale_y))
        width, height = max(1.0, x2 - x1), max(1.0, y2 - y1)
        left = max(0, int(x1 + width * 0.25))
        right = min(message.width - 1, int(x2 - width * 0.25))
        top = max(0, int(y1 + height * 0.25))
        bottom = min(message.height - 1, int(y2 - height * 0.20))
        stride = max(1, int(min(width, height) / 24))
        samples = []
        for v in range(top, bottom + 1, stride):
            for u in range(left, right + 1, stride):
                depth = self.depth_at(message, u, v)
                if math.isfinite(depth) and self.minimum_depth <= depth <= self.maximum_depth:
                    samples.append((depth, u, v))
        if len(samples) < self.minimum_samples:
            return None

        median_depth = statistics.median(sample[0] for sample in samples)
        deviations = [abs(sample[0] - median_depth) for sample in samples]
        mad = statistics.median(deviations)
        threshold = max(0.04, 2.5 * mad)
        inliers = [sample for sample in samples if abs(sample[0] - median_depth) <= threshold]
        if len(inliers) < self.minimum_samples:
            return None

        depth = statistics.median(sample[0] for sample in inliers)
        u = statistics.median(sample[1] for sample in inliers)
        v = statistics.median(sample[2] for sample in inliers)
        center_x = 0.5 * (x1 + x2)
        center_y = 0.5 * (y1 + y2)
        history_key = (
            int(class_id),
            round(center_x / max(20.0, message.width * 0.12)),
            round(center_y / max(20.0, message.height * 0.12)),
        )
        history = self.depth_history[history_key]
        history.append(depth)
        return statistics.median(history), float(u), float(v), len(inliers), mad

    def gimbal_tf_ready(self):
        with self.lock:
            status = self.gimbal_tf_status
            received = self.gimbal_tf_status_received
        if status is None or received is None:
            return False, "gimbal TF status unavailable", None
        age = time.monotonic() - received
        if age > self.maximum_tf_status_age:
            return False, f"gimbal TF status stale ({age:.3f}s)", status
        if not status.get("valid"):
            return False, str(status.get("reason", "gimbal TF invalid")), status
        return True, "ok", status

    def transform_is_fresh(self, transform):
        stamp = transform.header.stamp
        stamp_seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if stamp_seconds <= 0.0:
            return False
        now_seconds = self.get_clock().now().nanoseconds * 1e-9
        return abs(now_seconds - stamp_seconds) <= self.maximum_transform_age

    def lookup_transforms(self, source_frame):
        ready, reason, _status = self.gimbal_tf_ready()
        if not ready:
            now = time.monotonic()
            if now - self.last_tf_warning.get("gimbal_status", 0.0) > 30.0:
                self.get_logger().warning(f"camera TF disabled: {reason}")
                self.last_tf_warning["gimbal_status"] = now
            return {}
        transforms = {}
        for target_frame in self.target_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.03),
                )
                if not self.transform_is_fresh(transform):
                    raise TransformException(
                        f"latest transform to {target_frame} is stale"
                    )
                transforms[target_frame] = transform
            except TransformException as error:
                now = time.monotonic()
                if now - self.last_tf_warning.get(target_frame, 0.0) > 30.0:
                    self.get_logger().warning(
                        f"TF {target_frame} <- {source_frame} unavailable: {error}"
                    )
                    self.last_tf_warning[target_frame] = now
        return transforms

    @staticmethod
    def transform_point(position, transform):
        rotated = rotate_vector(position, transform.transform.rotation)
        translation = transform.transform.translation
        return (
            rotated[0] + translation.x,
            rotated[1] + translation.y,
            rotated[2] + translation.z,
        )

    def publish_tracking_target(self, targets, stamp, source_frame):
        if not targets:
            return
        people = [target for target in targets if target["label"] == "person"]
        candidates = people or targets
        target = min(
            candidates,
            key=lambda item: (-item["confidence"], item["depth_m"]),
        )
        frame = source_frame
        position = target["position_camera"]
        for preferred_frame in self.target_frames:
            if preferred_frame in target["positions"]:
                frame = preferred_frame
                position = target["positions"][preferred_frame]
                break
        message = PointStamped()
        message.header.stamp = stamp
        message.header.frame_id = frame
        message.point.x = position["x"]
        message.point.y = position["y"]
        message.point.z = position["z"]
        self.tracking_publisher.publish(message)

    def process_detections(self):
        try:
            result = self.fetch_detections()
        except (OSError, ValueError, urllib.error.URLError) as error:
            now = time.monotonic()
            if now - self.last_http_warning > 10.0:
                self.get_logger().warning(f"Detection endpoint unavailable: {error}")
                self.last_http_warning = now
            return
        if not result.get("online") or result.get("error"):
            return
        updated_at = result.get("updated_at")
        if updated_at is not None and updated_at == self.last_detection_update:
            return

        with self.lock:
            pending_update = (
                None
                if self.pending_detection is None
                else self.pending_detection.get("updated_at")
            )
            if updated_at is not None and updated_at == pending_update:
                return
            self.pending_detection = result
            if self.depth_subscription is None:
                self.depth_subscription = self.create_subscription(
                    Image, self.depth_topic, self.on_depth, qos_profile_sensor_data
                )

    def localize(self, result, depth, info, info_received_at):
        now = time.monotonic()
        updated_at = result.get("updated_at")
        if now - info_received_at > 5.0:
            return
        if len(info.k) < 9 or info.k[0] <= 0.0 or info.k[4] <= 0.0:
            return

        source_width = result.get("source_width") or info.width
        source_height = result.get("source_height") or info.height
        transforms = self.lookup_transforms(depth.header.frame_id)
        tf_ready, tf_reason, tf_status = self.gimbal_tf_ready()
        stamp_seconds = (
            float(depth.header.stamp.sec)
            + float(depth.header.stamp.nanosec) * 1e-9
        )
        ros_now_seconds = self.get_clock().now().nanoseconds * 1e-9
        observation_age = (
            math.inf
            if stamp_seconds <= 0.0
            else max(0.0, ros_now_seconds - stamp_seconds)
        )
        data_fresh = (
            math.isfinite(observation_age)
            and observation_age <= self.maximum_data_age
        )
        map_tf_available = tf_ready and "map" in transforms
        if map_tf_available:
            if self.map_tf_valid_since is None:
                self.map_tf_valid_since = now
            map_tf_stable_for = max(0.0, now - self.map_tf_valid_since)
        else:
            self.map_tf_valid_since = None
            map_tf_stable_for = 0.0
        map_position_valid = (
            map_tf_available
            and map_tf_stable_for >= self.minimum_map_tf_stable
            and data_fresh
        )
        if not tf_ready:
            fusion_reason = tf_reason
        elif not map_tf_available:
            fusion_reason = "map transform unavailable"
        elif not data_fresh:
            fusion_reason = f"depth frame stale ({observation_age:.3f}s)"
        elif not map_position_valid:
            fusion_reason = f"map transform settling ({map_tf_stable_for:.3f}s)"
        else:
            fusion_reason = "ok"
        targets = []
        for detection in result.get("detections", []):
            box = detection.get("box")
            if not isinstance(box, list) or len(box) != 4:
                continue
            sampled = self.stable_depth(
                depth,
                box,
                source_width,
                source_height,
                detection.get("class_id", -1),
            )
            if sampled is None:
                continue
            depth_m, u, v, sample_count, depth_mad = sampled
            confidence = max(
                0.0,
                min(1.0, float(detection.get("confidence", 0.0))),
            )
            quality = observation_quality(
                confidence,
                depth_m,
                sample_count,
                depth_mad,
                self.minimum_samples,
            )
            position = (
                (u - info.k[2]) * depth_m / info.k[0],
                (v - info.k[5]) * depth_m / info.k[4],
                depth_m,
            )
            positions = {}
            for frame, transform in transforms.items():
                transformed = self.transform_point(position, transform)
                positions[frame] = {
                    "x": round(transformed[0], 4),
                    "y": round(transformed[1], 4),
                    "z": round(transformed[2], 4),
                }
            targets.append(
                {
                    "class_id": int(detection.get("class_id", -1)),
                    "label": str(detection.get("label", "unknown")),
                    "confidence": round(confidence, 4),
                    "quality": round(quality, 4),
                    "observation_age_s": (
                        None
                        if not math.isfinite(observation_age)
                        else round(observation_age, 4)
                    ),
                    "map_position_valid": map_position_valid,
                    "box": [round(float(value), 1) for value in box],
                    "depth_m": round(depth_m, 4),
                    "depth_samples": sample_count,
                    "depth_mad_m": round(depth_mad, 4),
                    "pixel": {"u": round(u, 1), "v": round(v, 1)},
                    "position_camera": {
                        "x": round(position[0], 4),
                        "y": round(position[1], 4),
                        "z": round(position[2], 4),
                    },
                    "positions": positions,
                }
            )

        payload = {
            "stamp": {
                "sec": depth.header.stamp.sec,
                "nanosec": depth.header.stamp.nanosec,
            },
            "frame_id": depth.header.frame_id,
            "target_frames": self.target_frames,
            "tf_available": sorted(transforms),
            "tf_valid": tf_ready and bool(transforms),
            "tf_reason": fusion_reason,
            "map_tf_valid": map_position_valid,
            "map_tf_stable_for_s": round(map_tf_stable_for, 4),
            "observation_age_s": (
                None
                if not math.isfinite(observation_age)
                else round(observation_age, 4)
            ),
            "gimbal_tf_status": tf_status,
            "source_width": int(source_width),
            "source_height": int(source_height),
            "detections_updated_at": updated_at,
            "targets": targets,
        }
        output = String()
        output.data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
        self.targets_publisher.publish(output)
        self.publish_tracking_target(targets, depth.header.stamp, depth.header.frame_id)
        self.last_detection_update = updated_at


def main(args=None):
    rclpy.init(args=args)
    node = TargetLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
