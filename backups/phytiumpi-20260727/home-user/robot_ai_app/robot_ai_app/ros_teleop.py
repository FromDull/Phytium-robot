"""Keyboard teleoperation through ROS 2 /cmd_vel."""

from __future__ import annotations

import argparse
import sys

import rclpy

from .robot_tools import RobotTools
from .ros_robot_interface import RobotRosInterface
from .runtime import add_ros_robot_args, limits_from_args, topics_from_args
from .safety import SafetyGuard


COMMANDS = {
    "w": (0.03, 0.0, "forward"),
    "s": (-0.03, 0.0, "backward"),
    "a": (0.0, 0.3, "turn_left"),
    "d": (0.0, -0.3, "turn_right"),
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Teleoperate the robot through ROS 2.")
    add_ros_robot_args(parser)
    parser.add_argument("--duration", type=float, default=0.25)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    robot = RobotRosInterface(topics=topics_from_args(args), limits=limits_from_args(args))
    tools = RobotTools(robot, SafetyGuard(limits_from_args(args)))
    try:
        print("ROS robot state:", robot.state())
        print("Commands: w forward, s backward, a left, d right, x stop, state, q quit")
        while True:
            command = input("> ").strip().lower()
            if command == "q":
                tools.stop()
                return 0
            if command == "x":
                print(tools.stop())
                continue
            if command == "state":
                print(tools.get_status().data)
                continue
            if command in COMMANDS:
                vx, wz, label = COMMANDS[command]
                print(label, tools.move_base(vx, wz, args.duration))
                continue
            print("Unknown command. Use w/s/a/d/x/state/q.")
    except KeyboardInterrupt:
        tools.stop()
        return 1
    except Exception as exc:
        print(f"teleop failed: {exc}", file=sys.stderr)
        tools.stop()
        return 1
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
