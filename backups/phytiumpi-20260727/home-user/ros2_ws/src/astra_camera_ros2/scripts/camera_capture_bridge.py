#!/usr/bin/env python3

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraCaptureBridge(Node):
    def __init__(self) -> None:
        super().__init__("camera_capture_bridge")
        shared_dir = Path(self.declare_parameter("shared_dir", "/root/robot_data").value)
        shared_dir.mkdir(parents=True, exist_ok=True)
        self.request_path = shared_dir / "camera_capture.request"
        self.ready_path = shared_dir / "camera_capture.ready"
        self.output_path = shared_dir / "latest_color.ppm"
        self.temporary_path = shared_dir / ".latest_color.ppm.tmp"
        self.latest_image: Image | None = None
        self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.receive_image,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self.process_request)
        self.get_logger().info(f"On-demand camera bridge ready in {shared_dir}")

    def receive_image(self, message: Image) -> None:
        self.latest_image = message

    def process_request(self) -> None:
        if not self.request_path.exists() or self.latest_image is None:
            return
        try:
            request_token = self.request_path.read_text(encoding="ascii").strip()
        except OSError as error:
            self.get_logger().warning(f"Could not read capture request: {error}")
            return
        message = self.latest_image
        if message.encoding != "rgb8":
            self.get_logger().error(f"Expected rgb8, received {message.encoding}")
            self.request_path.unlink(missing_ok=True)
            return
        row_bytes = message.width * 3
        with self.temporary_path.open("wb") as output:
            output.write(f"P6\n{message.width} {message.height}\n255\n".encode("ascii"))
            for row in range(message.height):
                start = row * message.step
                output.write(message.data[start : start + row_bytes])
        self.temporary_path.replace(self.output_path)
        self.request_path.unlink(missing_ok=True)
        self.ready_path.write_text(request_token, encoding="ascii")
        self.get_logger().info(f"Captured {self.output_path}")


def main() -> None:
    rclpy.init()
    node = CameraCaptureBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
