#!/usr/bin/env python3

from array import array
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraSnapshot(Node):
    def __init__(self) -> None:
        super().__init__("camera_snapshot")
        output_dir = Path(
            self.declare_parameter("output_dir", "/root/robot_data").value
        )
        output_dir.mkdir(parents=True, exist_ok=True)
        self.color_path = output_dir / "astra_color.ppm"
        self.depth_path = output_dir / "astra_depth.pgm"
        self.color_saved = False
        self.depth_saved = False
        self.last_depth_check_ns = 0
        self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.save_color,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/depth/image_raw",
            self.save_depth,
            qos_profile_sensor_data,
        )
        self.timeout = self.create_timer(15.0, self.fail_on_timeout)
        self.get_logger().info("Waiting for one color frame and one depth frame...")

    def save_color(self, message: Image) -> None:
        if self.color_saved:
            return
        if message.encoding != "rgb8":
            self.get_logger().error(f"Expected rgb8, received {message.encoding}")
            return
        row_bytes = message.width * 3
        with self.color_path.open("wb") as output:
            output.write(f"P6\n{message.width} {message.height}\n255\n".encode("ascii"))
            for row in range(message.height):
                start = row * message.step
                output.write(message.data[start : start + row_bytes])
        self.color_saved = True
        self.get_logger().info(f"Saved {self.color_path}")
        self.finish_if_complete()

    def save_depth(self, message: Image) -> None:
        if self.depth_saved:
            return
        if message.encoding != "16UC1":
            self.get_logger().error(f"Expected 16UC1, received {message.encoding}")
            return
        row_bytes = message.width * 2
        values = array("H")
        for row in range(message.height):
            start = row * message.step
            values.frombytes(bytes(message.data[start : start + row_bytes]))
        source_is_big_endian = bool(message.is_bigendian)
        host_is_big_endian = sys.byteorder == "big"
        if source_is_big_endian != host_is_big_endian:
            values.byteswap()
        valid_count = sum(1 for value in values if 300 <= value <= 10000)
        minimum_valid = max(1, message.width * message.height // 100)
        if valid_count < minimum_valid:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_depth_check_ns >= 1_000_000_000:
                self.get_logger().info(
                    f"Waiting for valid depth: {valid_count}/{message.width * message.height} pixels"
                )
                self.last_depth_check_ns = now_ns
            return
        if not host_is_big_endian:
            values.byteswap()  # 16-bit PGM stores samples in big-endian order.
        with self.depth_path.open("wb") as output:
            output.write(f"P5\n{message.width} {message.height}\n65535\n".encode("ascii"))
            output.write(values.tobytes())
        self.depth_saved = True
        self.get_logger().info(f"Saved {self.depth_path}")
        self.finish_if_complete()

    def finish_if_complete(self) -> None:
        if self.color_saved and self.depth_saved:
            self.get_logger().info("Camera snapshots complete.")
            rclpy.shutdown()

    def fail_on_timeout(self) -> None:
        missing = []
        if not self.color_saved:
            missing.append("color")
        if not self.depth_saved:
            missing.append("depth")
        self.get_logger().error("Timed out waiting for: " + ", ".join(missing))
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = CameraSnapshot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
