"""Prompt manual for AI policies."""

from __future__ import annotations

from .robot_capabilities import DEFAULT_CAPABILITIES, RobotCapabilities


def build_robot_manual(capabilities: RobotCapabilities = DEFAULT_CAPABILITIES) -> str:
    return f"""
机器人能力手册：

当前能力：
- camera: {capabilities.camera}
- localization: {capabilities.localization}
- navigation: {capabilities.navigation}
- target_detection: {capabilities.target_detection}
- semantic_map: {capabilities.semantic_map}
- basic_turn: {capabilities.basic_turn}
- basic_motion: {capabilities.basic_motion}

允许任务：
- look: 获取一帧摄像头图像。
- stop: 停止机器人。
- move_distance: 短距离直行，params={{"distance_m": 0.2, "avoid_obstacles": true}}。
- turn_angle: 指定角度旋转，params={{"angle_deg": 90}}，正数左转，负数右转。
- inspect_front: 观察前方。
- inspect_target: 观察指定目标，params={{"target": "door"}}。
- navigate_to_pose: 导航到地图坐标，默认只要求到点，params={{"x": 1.0, "y": 0.5}}；只有用户明确要求最终朝向时才带 yaw。
- navigate_to_object: 预留目标导航，当前仅 target_detection 或 semantic_map 可用时执行。

禁止事项：
- 不要直接输出连续底盘速度规划。
- 每轮只输出一个任务。
- 前方不清楚、姿态异常、定位不可靠时，选择 stop 或 look。
- 看到目标不等于能够到达目标；没有目标识别或语义地图时，不能声称能可靠导航到目标物。
""".strip()
