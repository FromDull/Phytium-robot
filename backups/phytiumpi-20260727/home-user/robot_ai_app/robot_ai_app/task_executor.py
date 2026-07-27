"""Execute high-level AI tasks as safe ROS-backed tool calls."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .robot_capabilities import DEFAULT_CAPABILITIES, RobotCapabilities
from .robot_tools import RobotTools, ToolResult
from .semantic_map import SemanticMap
from .target_detection import TargetDetector
from .task_schema import RobotTask


@dataclass
class TaskExecutionResult:
    ok: bool
    task_type: str
    steps: list[ToolResult] = field(default_factory=list)
    message: str = ""


class TaskExecutor:
    def __init__(
        self,
        tools: RobotTools,
        step_sleep: float = 0.15,
        capabilities: RobotCapabilities = DEFAULT_CAPABILITIES,
        target_detector: TargetDetector | None = None,
        semantic_map: SemanticMap | None = None,
    ):
        self.tools = tools
        self.step_sleep = step_sleep
        self.capabilities = capabilities
        self.target_detector = target_detector or TargetDetector()
        self.semantic_map = semantic_map or SemanticMap()

    def execute(self, task: RobotTask) -> TaskExecutionResult:
        try:
            if task.type == "none":
                return TaskExecutionResult(True, task.type, message="no action")
            if task.type == "stop":
                return self._single(task, self.tools.stop())
            if task.type in {"look", "inspect_front"}:
                return self._single(task, self.tools.look())
            if task.type == "inspect_target":
                return self._execute_inspect_target(task)
            if task.type == "turn_left":
                return self._execute_turn_angle(RobotTask("turn_angle", {"angle_deg": 90}, task.reason))
            if task.type == "turn_right":
                return self._execute_turn_angle(RobotTask("turn_angle", {"angle_deg": -90}, task.reason))
            if task.type == "move_base":
                return self._execute_move_base(task)
            if task.type == "move_distance":
                return self._execute_move_distance(task)
            if task.type == "turn_angle":
                return self._execute_turn_angle(task)
            if task.type == "navigate_to_pose":
                return self._execute_navigate_to_pose(task)
            if task.type == "navigate_to_object":
                return self._execute_navigate_to_object(task)
            raise ValueError(f"Unsupported task: {task.type}")
        except Exception as exc:
            stop = self._try_stop()
            steps = [stop] if stop is not None else []
            return TaskExecutionResult(False, task.type, steps, f"task failed: {exc}")

    def _single(self, task: RobotTask, result: ToolResult) -> TaskExecutionResult:
        return TaskExecutionResult(result.ok, task.type, [result], message=task.reason)

    def _execute_move_base(self, task: RobotTask) -> TaskExecutionResult:
        params = task.params
        vx = float(params.get("vx", 0.0))
        wz = float(params.get("wz", 0.0))
        duration = float(params.get("duration", 0.25))
        if abs(vx) < 1e-6 and abs(wz) > 1e-6:
            result = self.tools.rotate_in_place(wz * duration, timeout=float(params.get("timeout", 30.0)))
            message = "in-place rotation complete" if result.ok else self._failure_message(result)
            return TaskExecutionResult(result.ok, task.type, [result], message)

        if self._navigation_ready():
            result = self.tools.navigate_relative(
                distance_m=vx * duration,
                yaw_delta=0.0,
                timeout=float(params.get("timeout", 30.0)),
            )
            message = "navigation relative move complete" if result.ok else self._failure_message(result)
            return TaskExecutionResult(result.ok, task.type, [result], message)

        result = self.tools.move_base(
            vx,
            wz,
            duration,
            action="move_base",
        )
        return self._single(task, result)

    def _execute_move_distance(self, task: RobotTask) -> TaskExecutionResult:
        distance_m = float(task.params.get("distance_m", 0.0))
        if self._navigation_ready():
            result = self.tools.navigate_relative(
                distance_m=distance_m,
                yaw_delta=0.0,
                timeout=float(task.params.get("timeout", 60.0)),
            )
            message = f"navigation distance complete: {distance_m:.2f}m" if result.ok else self._failure_message(result)
            return TaskExecutionResult(result.ok, task.type, [result], message)

        return self._fallback_move_distance(task)

    def _execute_turn_angle(self, task: RobotTask) -> TaskExecutionResult:
        angle_deg = float(task.params.get("angle_deg", 0.0))
        result = self.tools.rotate_in_place(
            math.radians(angle_deg),
            timeout=float(task.params.get("timeout", 30.0)),
        )
        if result.ok:
            return TaskExecutionResult(True, task.type, [result], f"in-place turn complete: {angle_deg:.1f}deg")

        if self._navigation_ready():
            return TaskExecutionResult(False, task.type, [result], self._failure_message(result))
        return self._fallback_turn_angle(task)

    def _execute_inspect_target(self, task: RobotTask) -> TaskExecutionResult:
        look = self.tools.look()
        target = str(task.params.get("target", "target"))
        if self.capabilities.target_detection and self.target_detector.available:
            detection = self.target_detector.detect(target, look.data.get("path"))
            return TaskExecutionResult(
                detection is not None,
                task.type,
                [look],
                "target detected" if detection is not None else f"target not found: {target}",
            )
        return TaskExecutionResult(look.ok, task.type, [look], f"captured image for target inspection: {target}")

    def _execute_navigate_to_pose(self, task: RobotTask) -> TaskExecutionResult:
        if not (self.capabilities.navigation and self.capabilities.localization):
            look = self.tools.look()
            return TaskExecutionResult(False, task.type, [look], "navigation/localization disabled; only captured observation")

        state = self.tools.get_status().data
        nav = state.get("nav", {})
        if not nav.get("available"):
            return TaskExecutionResult(False, task.type, [], "Nav2 navigate_to_pose action server is not available")

        params = task.params
        requested_yaw = params.get("yaw")
        result = self.tools.navigate_to_pose(
            float(params.get("x", 0.0)),
            float(params.get("y", 0.0)),
            None,
            float(params.get("timeout", 60.0)),
        )
        message = task.reason
        if not result.ok:
            message = self._failure_message(result)

        steps = [result]
        if result.ok:
            settle = self.tools.settle_to_position(
                float(params.get("x", 0.0)),
                float(params.get("y", 0.0)),
                timeout=float(params.get("settle_timeout", 12.0)),
                tolerance_m=float(params.get("position_tolerance", 0.02)),
            )
            steps.append(settle)
            if not settle.ok:
                return TaskExecutionResult(False, task.type, steps, self._failure_message(settle))

        if result.ok and requested_yaw is not None:
            state_after_nav = self.tools.get_status().data
            pose_after_nav = state_after_nav.get("pose") or {}
            current_yaw = float(pose_after_nav.get("yaw", result.data.get("yaw", 0.0)))
            rotate = self.tools.rotate_in_place(float(requested_yaw) - current_yaw)
            steps.append(rotate)
            if not rotate.ok:
                return TaskExecutionResult(False, task.type, steps, self._failure_message(rotate))
            message = f"{message}; final yaw adjusted in place"
        return TaskExecutionResult(result.ok, task.type, steps, message=message)

    def _execute_navigate_to_object(self, task: RobotTask) -> TaskExecutionResult:
        target = str(task.params.get("target", "target"))
        if self.capabilities.semantic_map and self.semantic_map.available:
            place = self.semantic_map.lookup(target)
            if place is not None:
                return self._execute_navigate_to_pose(
                    RobotTask("navigate_to_pose", {"x": place.x, "y": place.y, "yaw": place.yaw}, task.reason)
                )

        look = self.tools.look()
        return TaskExecutionResult(
            False,
            task.type,
            [look],
            f"object navigation is reserved; no target detector/semantic map result for: {target}",
        )

    def _try_stop(self) -> ToolResult | None:
        try:
            return self.tools.stop()
        except Exception:
            return None

    def _navigation_ready(self) -> bool:
        if not (self.capabilities.navigation and self.capabilities.localization):
            return False
        state = self.tools.get_status().data
        nav = state.get("nav", {})
        return bool(nav.get("available"))

    def _failure_message(self, result: ToolResult) -> str:
        return str(result.data.get("error") or result.data.get("status_text") or "navigation failed")

    def _fallback_move_distance(self, task: RobotTask) -> TaskExecutionResult:
        distance_m = float(task.params.get("distance_m", 0.0))
        direction = 1.0 if distance_m >= 0 else -1.0
        remaining = abs(distance_m)
        step_distance = 0.03
        vx = 0.04 * direction
        duration = 0.35
        max_steps = min(80, max(1, math.ceil(remaining / step_distance)))
        results: list[ToolResult] = []

        for _ in range(max_steps):
            move = self.tools.move_base(vx, 0.0, duration, action="move_distance_step")
            results.append(move)
            if not move.ok:
                results.append(self.tools.stop())
                return TaskExecutionResult(False, task.type, results, "movement step failed")
            remaining -= step_distance
            if remaining <= 0:
                break

        results.append(self.tools.stop())
        return TaskExecutionResult(True, task.type, results, f"fallback estimated distance complete: {distance_m:.2f}m")

    def _fallback_turn_angle(self, task: RobotTask) -> TaskExecutionResult:
        angle_deg = float(task.params.get("angle_deg", 0.0))
        direction = 1.0 if angle_deg >= 0 else -1.0
        remaining = abs(angle_deg)
        step_angle = 15.0
        wz = 0.35 * direction
        duration = 0.35
        max_steps = min(24, max(1, math.ceil(remaining / step_angle)))
        results: list[ToolResult] = []

        for _ in range(max_steps):
            move = self.tools.move_base(0.0, wz, duration, action="turn_angle_step")
            results.append(move)
            if not move.ok:
                results.append(self.tools.stop())
                return TaskExecutionResult(False, task.type, results, "turn step failed")
            remaining -= step_angle
            if remaining <= 0:
                break

        results.append(self.tools.stop())
        return TaskExecutionResult(True, task.type, results, f"fallback estimated angle complete: {angle_deg:.1f}deg")
