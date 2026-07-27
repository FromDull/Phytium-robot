#!/usr/bin/env python3
"""Preview-only Nav2 bridge. It never publishes velocity or control commands."""

import json
import math
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String


class PreviewPlannerBridge(Node):
    def __init__(self):
        super().__init__("nav2_preview_bridge")
        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_publisher = self.create_publisher(Path, "/navigation/preview_path", transient)
        self.status_publisher = self.create_publisher(String, "/navigation/preview_status", transient)
        self.create_subscription(PoseStamped, "/navigation/preview_goal", self.on_goal, 10)
        self.create_subscription(Empty, "/navigation/preview_clear", self.on_clear, 10)
        self.client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.lock = threading.Lock()
        self.generation = 0
        self.active_goal = None
        self.publish_status("waiting", "等待网页选择目标")

    def publish_status(self, state, message, **extra):
        payload = {"state": state, "message": message, "preview_only": True, **extra}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.status_publisher.publish(msg)

    def clear_path(self):
        empty = Path()
        empty.header.frame_id = "map"
        empty.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(empty)

    def on_clear(self, _message):
        with self.lock:
            self.generation += 1
            active = self.active_goal
            self.active_goal = None
        if active is not None:
            active.cancel_goal_async()
        self.clear_path()
        self.publish_status("idle", "预览路线已清除")

    def on_goal(self, goal):
        if goal.header.frame_id not in ("", "map"):
            self.clear_path()
            self.publish_status("failed", "目标必须位于 map 坐标系")
            return
        if not math.isfinite(goal.pose.position.x) or not math.isfinite(goal.pose.position.y):
            self.clear_path()
            self.publish_status("failed", "目标坐标无效")
            return
        with self.lock:
            self.generation += 1
            generation = self.generation
            active = self.active_goal
            self.active_goal = None
        if active is not None:
            active.cancel_goal_async()
        self.clear_path()  # A new click always removes the previous preview immediately.
        self.publish_status("planning", "Nav2 正在计算安全预览路线")
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.publish_status("failed", "Nav2 planner_server 尚未就绪")
            return
        request = ComputePathToPose.Goal()
        request.goal = goal
        request.goal.header.frame_id = "map"
        request.goal.header.stamp = self.get_clock().now().to_msg()
        request.planner_id = "GridBased"
        request.use_start = False
        future = self.client.send_goal_async(request)
        future.add_done_callback(lambda result: self.on_goal_response(result, generation))

    def on_goal_response(self, future, generation):
        try:
            handle = future.result()
        except Exception as error:
            self.publish_status("failed", f"提交规划失败：{error}")
            return
        with self.lock:
            if generation != self.generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            self.active_goal = handle if handle.accepted else None
        if not handle.accepted:
            self.publish_status("failed", "Nav2 拒绝了规划目标")
            return
        result = handle.get_result_async()
        result.add_done_callback(lambda value: self.on_result(value, generation))

    def on_result(self, future, generation):
        with self.lock:
            if generation != self.generation:
                return
            self.active_goal = None
        try:
            wrapped = future.result()
            path = wrapped.result.path
        except Exception as error:
            self.clear_path()
            self.publish_status("failed", f"规划计算失败：{error}")
            return
        if len(path.poses) < 2:
            self.clear_path()
            self.publish_status("failed", "目标不可达或路径为空")
            return
        length = sum(
            math.hypot(
                right.pose.position.x - left.pose.position.x,
                right.pose.position.y - left.pose.position.y,
            )
            for left, right in zip(path.poses, path.poses[1:])
        )
        path.header.frame_id = "map"
        self.path_publisher.publish(path)
        self.publish_status(
            "ready", "仅预览，不会驱动底盘", length_m=round(length, 3), pose_count=len(path.poses)
        )


def main():
    rclpy.init()
    node = PreviewPlannerBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
