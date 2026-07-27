"""Save one ROS camera frame."""

from __future__ import annotations

import argparse
from pathlib import Path

import rclpy

from .ros_robot_interface import RobotRosInterface
from .runtime import add_ros_robot_args, limits_from_args, topics_from_args


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Save one robot camera frame from ROS.")
    add_ros_robot_args(parser)
    parser.add_argument("--output", default="camera_frame.jpg")
    parser.add_argument("--timeout", type=float, default=3.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    robot = RobotRosInterface(topics=topics_from_args(args), limits=limits_from_args(args))
    try:
        result = robot.capture_image(Path(args.output).parent, Path(args.output).stem, timeout=args.timeout)
        generated = Path(result["path"])
        requested = Path(args.output)
        if generated != requested:
            requested.write_bytes(generated.read_bytes())
            result["path"] = str(requested)
        print(result)
        return 0
    except Exception as exc:
        print(f"camera request failed: {exc}")
        return 1
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
