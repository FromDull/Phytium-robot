"""High-level semantic tools backed by ROS 2."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .ros_robot_interface import RobotRosInterface
from .safety import SafetyGuard


@dataclass(frozen=True)
class ToolResult:
    ok: bool
    action: str
    data: dict


class RobotTools:
    def __init__(self, robot: RobotRosInterface, safety: SafetyGuard | None = None):
        self.robot = robot
        self.safety = safety or SafetyGuard()

    def get_status(self) -> ToolResult:
        state = self.robot.state()
        return ToolResult(ok=bool(state.get("ok", True)), action="get_status", data=state)

    def look(self, output_dir: str | Path = "captures", prefix: str = "frame") -> ToolResult:
        result = self.robot.capture_image(output_dir, prefix)
        return ToolResult(ok=bool(result.get("ok", True)), action="look", data=result)

    def move_forward_short(self) -> ToolResult:
        return self._move("move_forward_short", vx=0.03, wz=0.0, duration=0.25)

    def move_backward_short(self) -> ToolResult:
        return self._move("move_backward_short", vx=-0.03, wz=0.0, duration=0.25)

    def turn_left_small(self) -> ToolResult:
        return self._move("turn_left_small", vx=0.0, wz=0.3, duration=0.25)

    def turn_right_small(self) -> ToolResult:
        return self._move("turn_right_small", vx=0.0, wz=-0.3, duration=0.25)

    def move_base(self, vx: float, wz: float, duration: float, action: str = "move_base") -> ToolResult:
        return self._move(action, vx=vx, wz=wz, duration=duration)

    def navigate_to_pose(self, x: float, y: float, yaw: float | None = None, timeout: float = 60.0) -> ToolResult:
        response = self.robot.navigate_to_pose(x, y, yaw, timeout)
        return ToolResult(ok=bool(response.get("ok", False)), action="navigate_to_pose", data=response)

    def navigate_relative(self, distance_m: float = 0.0, yaw_delta: float = 0.0, timeout: float = 60.0) -> ToolResult:
        response = self.robot.navigate_relative(distance_m, yaw_delta, timeout)
        return ToolResult(ok=bool(response.get("ok", False)), action="navigate_relative", data=response)

    def rotate_in_place(self, angle_rad: float, timeout: float = 30.0) -> ToolResult:
        response = self.robot.rotate_in_place(angle_rad, timeout)
        return ToolResult(ok=bool(response.get("ok", False)), action="rotate_in_place", data=response)

    def settle_to_position(self, x: float, y: float, timeout: float = 12.0, tolerance_m: float = 0.03) -> ToolResult:
        response = self.robot.settle_to_position(x, y, timeout, tolerance_m)
        return ToolResult(ok=bool(response.get("ok", False)), action="settle_to_position", data=response)

    def stop(self) -> ToolResult:
        response = self.robot.stop()
        return ToolResult(ok=bool(response.get("ok", True)), action="stop", data=response)

    def _move(self, action: str, vx: float, wz: float, duration: float) -> ToolResult:
        safe = self.safety.clamp_motion(vx, wz, duration)
        response = self.robot.move_base(safe.vx, safe.wz, safe.duration)
        return ToolResult(
            ok=bool(response.get("ok", True)),
            action=action,
            data={"vx": safe.vx, "wz": safe.wz, "duration": safe.duration, "response": response},
        )
