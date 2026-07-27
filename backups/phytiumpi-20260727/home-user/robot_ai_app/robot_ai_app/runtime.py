"""Shared CLI helpers for ROS AI control executables."""

from __future__ import annotations

import argparse

from .robot_capabilities import RobotCapabilities
from .ros_robot_interface import RosTopics
from .safety import MotionLimits


def add_ros_robot_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--status-topic", default="/wheel_leg/status")
    parser.add_argument("--camera-topic", default="/camera/image_raw")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--navigate-action", default="navigate_to_pose")
    parser.add_argument("--max-vx", type=float, default=0.10)
    parser.add_argument("--max-wz", type=float, default=0.75)
    parser.add_argument("--max-duration", type=float, default=1.0)
    parser.add_argument("--enable-navigation", action="store_true", default=True)
    parser.add_argument("--disable-navigation", action="store_true")
    parser.add_argument("--enable-localization", action="store_true", default=True)
    parser.add_argument("--disable-localization", action="store_true")


def topics_from_args(args: argparse.Namespace) -> RosTopics:
    return RosTopics(
        cmd_vel=args.cmd_vel_topic,
        status=args.status_topic,
        camera=args.camera_topic,
        odom=args.odom_topic,
        navigate_action=args.navigate_action,
    )


def limits_from_args(args: argparse.Namespace) -> MotionLimits:
    return MotionLimits(max_vx=args.max_vx, max_wz=args.max_wz, max_duration=args.max_duration)


def capabilities_from_args(args: argparse.Namespace) -> RobotCapabilities:
    return RobotCapabilities(
        camera=True,
        localization=bool(args.enable_localization and not args.disable_localization),
        navigation=bool(args.enable_navigation and not args.disable_navigation),
        target_detection=False,
        semantic_map=False,
        basic_turn=True,
        basic_motion=True,
    )
