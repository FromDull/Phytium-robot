"""Read and aggregate fixed AcousticEye direction-of-arrival samples."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import threading
import time


DEFAULT_ANGLE_OFFSET_DEG = -90.0


def normalize_angle(angle_deg: float) -> float:
    """Normalize an angle to (-180, 180]."""
    normalized = (float(angle_deg) + 180.0) % 360.0 - 180.0
    return 180.0 if normalized == -180.0 else normalized


@dataclass(frozen=True)
class DirectionSample:
    angle_deg: float
    stable: bool
    current_valid: bool
    device_ok: bool
    sequence: int
    timestamp: float

    @classmethod
    def from_mapping(cls, value: dict, angle_offset_deg: float = 0.0) -> "DirectionSample":
        return cls(
            angle_deg=normalize_angle(float(value["angle_deg"]) + float(angle_offset_deg)),
            stable=bool(value["stable"]),
            current_valid=bool(value["current_valid"]),
            device_ok=bool(value["device_ok"]),
            sequence=int(value["sequence"]),
            timestamp=float(value["timestamp"]),
        )

    @property
    def usable(self) -> bool:
        return self.stable and self.current_valid and self.device_ok


@dataclass(frozen=True)
class DirectionEstimate:
    valid: bool
    angle_deg: float | None
    valid_ratio: float
    circular_spread_deg: float | None
    sample_count: int
    valid_sample_count: int
    sequence_start: int | None
    sequence_end: int | None
    reason: str

    def gimbal_disposition(self, yaw_min_deg: float = -87.0, yaw_max_deg: float = 87.0) -> str:
        if not self.valid or self.angle_deg is None:
            return "invalid"
        if yaw_min_deg <= self.angle_deg <= yaw_max_deg:
            return "gimbal_reachable"
        return "base_assist_required"


class AcousticDirectionTracker:
    """Keep a bounded, de-duplicated history of AcousticEye JSON states."""

    def __init__(
        self,
        state_path: str | Path = "/run/acoustic-eye/angle.json",
        history_seconds: float = 2.0,
        poll_interval: float = 0.05,
        stale_after: float = 0.5,
        angle_offset_deg: float | None = None,
    ) -> None:
        self.state_path = Path(state_path)
        self.history_seconds = float(history_seconds)
        self.poll_interval = float(poll_interval)
        self.stale_after = float(stale_after)
        configured_offset = os.getenv("ACOUSTIC_DIRECTION_OFFSET_DEG")
        self.angle_offset_deg = (
            float(configured_offset)
            if angle_offset_deg is None and configured_offset is not None
            else DEFAULT_ANGLE_OFFSET_DEG if angle_offset_deg is None else float(angle_offset_deg)
        )
        self._samples: deque[DirectionSample] = deque()
        self._last_identity: tuple[int, float] | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._poll, name="acoustic-direction", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self.poll_interval * 4))

    def __enter__(self) -> "AcousticDirectionTracker":
        self.start()
        return self

    def __exit__(self, *_args) -> None:
        self.close()

    def read_once(self) -> DirectionSample:
        value = json.loads(self.state_path.read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError("AcousticEye state must be a JSON object")
        return DirectionSample.from_mapping(value, self.angle_offset_deg)

    def add_sample(self, sample: DirectionSample, now: float | None = None) -> bool:
        identity = (sample.sequence, sample.timestamp)
        current_time = time.time() if now is None else float(now)
        with self._lock:
            if identity == self._last_identity:
                return False
            self._last_identity = identity
            self._samples.append(sample)
            cutoff = current_time - self.history_seconds
            while self._samples and self._samples[0].timestamp < cutoff:
                self._samples.popleft()
        return True

    def estimate(
        self,
        end_time: float | None = None,
        window_seconds: float = 2.0,
        # The STM32 only marks a frame usable after its own GCC-PHAT and
        # multi-frame stability checks.  Do not add another multi-second
        # quorum here: one fresh hardware-stable sample is sufficient for
        # the short voice wake-word window.
        min_samples: int = 1,
        min_valid_ratio: float = 0.10,
        max_spread_deg: float = 25.0,
    ) -> DirectionEstimate:
        end = time.time() if end_time is None else float(end_time)
        start = end - float(window_seconds)
        with self._lock:
            samples = [sample for sample in self._samples if start <= sample.timestamp <= end]

        valid = [sample for sample in samples if sample.usable]
        ratio = len(valid) / len(samples) if samples else 0.0
        sequence_start = samples[0].sequence if samples else None
        sequence_end = samples[-1].sequence if samples else None
        common = {
            "valid_ratio": ratio,
            "sample_count": len(samples),
            "valid_sample_count": len(valid),
            "sequence_start": sequence_start,
            "sequence_end": sequence_end,
        }
        if not samples or end - samples[-1].timestamp > self.stale_after:
            return DirectionEstimate(False, None, circular_spread_deg=None, reason="data_stale", **common)
        if len(valid) < min_samples:
            return DirectionEstimate(False, None, circular_spread_deg=None, reason="insufficient_valid_samples", **common)
        if ratio < min_valid_ratio:
            return DirectionEstimate(False, None, circular_spread_deg=None, reason="low_valid_ratio", **common)

        radians = [math.radians(sample.angle_deg) for sample in valid]
        mean_sin = sum(math.sin(value) for value in radians) / len(radians)
        mean_cos = sum(math.cos(value) for value in radians) / len(radians)
        resultant = math.hypot(mean_sin, mean_cos)
        angle = normalize_angle(math.degrees(math.atan2(mean_sin, mean_cos)))
        spread = 180.0 if resultant <= 1e-9 else math.degrees(math.sqrt(max(0.0, -2.0 * math.log(resultant))))
        if spread > max_spread_deg:
            return DirectionEstimate(False, angle, circular_spread_deg=spread, reason="direction_unstable", **common)
        return DirectionEstimate(True, angle, circular_spread_deg=spread, reason="ok", **common)

    def _poll(self) -> None:
        while not self._stop.is_set():
            try:
                sample = self.read_once()
                self.add_sample(sample)
            except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
                pass
            self._stop.wait(self.poll_interval)
