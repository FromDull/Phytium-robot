"""Interactive natural-language control through ROS 2."""

from __future__ import annotations

import argparse

import rclpy

from .chat_policy import ChatDecision, QwenChatPolicy
from .intent_parser import parse_user_intent
from .qwen_policy import QwenPolicyError
from .robot_tools import RobotTools
from .ros_robot_interface import RobotRosInterface
from .runtime import add_ros_robot_args, capabilities_from_args, limits_from_args, topics_from_args
from .task_executor import TaskExecutor


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Chat with an AI agent to control the ROS robot.")
    add_ros_robot_args(parser)
    parser.add_argument("--capture-dir", default="captures_chat")
    parser.add_argument("--llm-timeout", type=float, default=30.0)
    parser.add_argument("--no-image", action="store_true")
    return parser


def execute_decision(executor: TaskExecutor, decision: ChatDecision):
    if decision.task is None:
        return None
    return executor.execute(decision.task)


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    robot = RobotRosInterface(topics=topics_from_args(args), limits=limits_from_args(args))
    tools = RobotTools(robot)
    executor = TaskExecutor(tools, capabilities=capabilities_from_args(args))
    policy: QwenChatPolicy | None = None
    history: list[dict[str, str]] = []

    try:
        print("state:", robot.state())
        print("输入中文指令控制机器人，例如：前进一点、左转、停下、看看前面、导航到坐标 x=1 y=0。输入 q 退出。")
        while True:
            user_text = input("你> ").strip()
            if user_text.lower() in {"q", "quit", "exit"}:
                tools.stop()
                return 0
            if not user_text:
                continue

            state = robot.state()
            intent = parse_user_intent(user_text)
            image_path = None
            if not args.no_image:
                look_result = tools.look(args.capture_dir, prefix="chat")
                image_path = look_result.data["path"]

            if policy is None:
                policy = QwenChatPolicy(timeout=args.llm_timeout)
            decision = policy.decide_from_user(user_text, state, image_path=image_path, history=history, intent=intent)
            print(f"AI> {decision.reply}")
            print(f"任务> {decision.task.type if decision.task else decision.action} | 安全> {decision.safety} | 原因> {decision.reason}")

            result = execute_decision(executor, decision)
            if result is not None:
                print(f"执行> {result.task_type}, ok={result.ok}, steps={len(result.steps)}, message={result.message}")

            history.append({"role": "user", "content": user_text})
            history.append({"role": "assistant", "content": decision.reply})
    except (QwenPolicyError, ValueError, KeyboardInterrupt, Exception) as exc:
        tools.stop()
        print(f"对话控制结束：{exc}")
        return 1
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
