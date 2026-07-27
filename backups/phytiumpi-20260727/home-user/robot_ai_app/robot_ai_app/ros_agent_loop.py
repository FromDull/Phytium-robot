"""Run an agent policy loop against ROS robot tools."""

from __future__ import annotations

import argparse
import json
import time

import rclpy

from .agent_core import AgentRunner, ReactiveSafetyPolicy, ScriptedPolicy
from .qwen_policy import QwenPolicyError, QwenVisionPolicy
from .robot_tools import RobotTools, ToolResult
from .ros_robot_interface import RobotRosInterface
from .runtime import add_ros_robot_args, limits_from_args, topics_from_args


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a ROS-native AI agent loop.")
    add_ros_robot_args(parser)
    parser.add_argument("--policy", choices=["scripted", "reactive", "qwen"], default="scripted")
    parser.add_argument("--steps", type=int, default=8)
    parser.add_argument("--sleep", type=float, default=0.35)
    parser.add_argument("--capture-dir", default="captures")
    parser.add_argument("--llm-timeout", type=float, default=20.0)
    return parser


def make_policy(name: str, llm_timeout: float):
    if name == "qwen":
        return QwenVisionPolicy(timeout=llm_timeout)
    if name == "reactive":
        return ReactiveSafetyPolicy()
    return ScriptedPolicy()


def print_result(step: int, result: ToolResult) -> None:
    print(json.dumps({"step": step, "action": result.action, "ok": result.ok, "data": result.data}, ensure_ascii=False, indent=2))


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    robot = RobotRosInterface(topics=topics_from_args(args), limits=limits_from_args(args))
    tools = RobotTools(robot)
    try:
        runner = AgentRunner(tools, make_policy(args.policy, args.llm_timeout), capture_dir=args.capture_dir)
        print("state:", robot.state())
        for step in range(args.steps):
            result = runner.run_step(step)
            print_result(step, result)
            if result.action == "finish":
                break
            time.sleep(args.sleep)
        tools.stop()
        return 0
    except (QwenPolicyError, ValueError, KeyboardInterrupt) as exc:
        tools.stop()
        print(f"agent loop failed: {exc}")
        return 1
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
