"""Client-side motion limits for AI and teleoperation commands."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MotionLimits:
    max_vx: float = 0.10
    max_wz: float = 0.75
    max_duration: float = 1.0


@dataclass(frozen=True)
class MotionCommand:
    vx: float
    wz: float
    duration: float


class SafetyGuard:
    def __init__(self, limits: MotionLimits | None = None):
        self.limits = limits or MotionLimits()

    def clamp_motion(self, vx: float, wz: float, duration: float | None = 1.0) -> MotionCommand:
        safe_duration = self.limits.max_duration if duration is None else float(duration)
        return MotionCommand(
            vx=clamp(float(vx), -self.limits.max_vx, self.limits.max_vx),
            wz=clamp(float(wz), -self.limits.max_wz, self.limits.max_wz),
            duration=clamp(safe_duration, 0.05, self.limits.max_duration),
        )


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))
