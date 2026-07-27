"""Read the existing YOLO and aligned-depth perception services."""

from __future__ import annotations

from dataclasses import dataclass, replace
import json
import time
from typing import Any
import urllib.request


LABEL_NAMES = {
    "person": "人", "bicycle": "自行车", "car": "汽车", "motorcycle": "摩托车",
    "bus": "公交车", "truck": "卡车", "bench": "长椅", "cat": "猫", "dog": "狗",
    "backpack": "背包", "umbrella": "雨伞", "handbag": "手提包", "suitcase": "行李箱",
    "bottle": "瓶子", "cup": "杯子", "bowl": "碗", "banana": "香蕉", "apple": "苹果",
    "chair": "椅子", "couch": "沙发", "potted plant": "盆栽", "bed": "床",
    "dining table": "桌子", "tv": "电视", "laptop": "笔记本电脑", "mouse": "鼠标",
    "remote": "遥控器", "keyboard": "键盘", "cell phone": "手机", "book": "书",
    "clock": "时钟", "vase": "花瓶", "scissors": "剪刀",
}

ALIASES = {
    "人": "person", "人员": "person", "行人": "person", "自行车": "bicycle",
    "汽车": "car", "车": "car", "猫": "cat", "狗": "dog", "背包": "backpack",
    "雨伞": "umbrella", "瓶子": "bottle", "水瓶": "bottle", "杯子": "cup",
    "碗": "bowl", "香蕉": "banana", "苹果": "apple", "椅子": "chair",
    "沙发": "couch", "盆栽": "potted plant", "床": "bed", "桌子": "dining table",
    "电视": "tv", "电脑": "laptop", "笔记本电脑": "laptop", "鼠标": "mouse",
    "遥控器": "remote", "键盘": "keyboard", "手机": "cell phone", "书": "book",
    "时钟": "clock", "花瓶": "vase", "剪刀": "scissors",
}


@dataclass(frozen=True)
class DetectedTarget:
    label: str
    confidence: float
    depth_m: float | None = None
    camera_x: float | None = None
    camera_y: float | None = None
    image_path: str | None = None
    map_x: float | None = None
    map_y: float | None = None
    box: tuple[float, float, float, float] | None = None
    image_width: int | None = None
    image_height: int | None = None


@dataclass(frozen=True)
class PerceptionReply:
    recognized: bool
    reply: str = ""
    targets: tuple[DetectedTarget, ...] = ()


class TargetDetector:
    """Adapter for services already maintained by the perception stack."""

    available = True

    def __init__(
        self,
        state_url: str = "http://127.0.0.1:8080/api/state",
        detections_url: str = "http://127.0.0.1:8091/detections",
        timeout: float = 1.0,
        maximum_age_s: float = 2.0,
    ):
        self.state_url = state_url
        self.detections_url = detections_url
        self.timeout = timeout
        self.maximum_age_s = maximum_age_s

    def _read_json(self, url: str) -> dict[str, Any]:
        request = urllib.request.Request(url, headers={"Accept": "application/json"})
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            value = json.loads(response.read())
        if not isinstance(value, dict):
            raise ValueError("perception service returned non-object JSON")
        return value

    def snapshot(self) -> tuple[list[DetectedTarget], list[DetectedTarget]]:
        state = self._read_json(self.state_url)
        targets_state = state.get("targets_3d") or {}
        targets_3d: list[DetectedTarget] = []
        if targets_state.get("online") and float(targets_state.get("age_ms", 999999)) <= self.maximum_age_s * 1000:
            for item in targets_state.get("targets", []):
                position = item.get("position_camera") or {}
                map_position = (item.get("positions") or {}).get("map") or {}
                targets_3d.append(
                    DetectedTarget(
                        label=str(item.get("label", "unknown")),
                        confidence=float(item.get("confidence", 0.0)),
                        depth_m=float(item["depth_m"]) if item.get("depth_m") is not None else None,
                        camera_x=float(position["x"]) if position.get("x") is not None else None,
                        camera_y=float(position["y"]) if position.get("y") is not None else None,
                        map_x=float(map_position["x"]) if map_position.get("x") is not None else None,
                        map_y=float(map_position["y"]) if map_position.get("y") is not None else None,
                    )
                )

        raw = self._read_json(self.detections_url)
        detections: list[DetectedTarget] = []
        updated_at = raw.get("updated_at")
        fresh = updated_at is not None and time.time() - float(updated_at) <= self.maximum_age_s
        if raw.get("online") and not raw.get("error") and fresh:
            image_width = int(raw["source_width"]) if raw.get("source_width") is not None else None
            image_height = int(raw["source_height"]) if raw.get("source_height") is not None else None
            for item in raw.get("detections", []):
                raw_box = item.get("box")
                box = (
                    tuple(float(value) for value in raw_box)
                    if isinstance(raw_box, (list, tuple)) and len(raw_box) == 4
                    else None
                )
                detections.append(
                    DetectedTarget(
                        str(item.get("label", "unknown")),
                        float(item.get("confidence", 0.0)),
                        box=box,
                        image_width=image_width,
                        image_height=image_height,
                    )
                )
        return targets_3d, detections

    def detect(self, target: str, image_path: str | None = None) -> DetectedTarget | None:
        del image_path
        label = ALIASES.get(target, target)
        try:
            targets_3d, detections = self.snapshot()
        except (OSError, ValueError, json.JSONDecodeError):
            return None
        matches = [item for item in self._combine(targets_3d, detections) if item.label == label]
        return max(matches, key=lambda item: item.confidence, default=None)

    def answer(self, text: str) -> PerceptionReply:
        normalized = "".join(text.lower().split())
        target_label = next((label for alias, label in ALIASES.items() if alias in normalized), None)
        asks_scene = any(word in normalized for word in ("前面有什么", "看到了什么", "识别到什么", "有哪些东西", "有什么东西"))
        asks_target = target_label is not None and any(
            word in normalized for word in ("有没有", "有吗", "在哪", "哪里", "多远", "距离", "几个", "多少")
        )
        if not asks_scene and not asks_target:
            return PerceptionReply(False)

        try:
            targets_3d, detections = self.snapshot()
        except (OSError, ValueError, json.JSONDecodeError):
            return PerceptionReply(False)

        source = self._combine(targets_3d, detections)
        if target_label is not None:
            source = [item for item in source if item.label == target_label]
            name = LABEL_NAMES.get(target_label, target_label)
            if not source:
                return PerceptionReply(True, f"目前没有识别到{name}")
        elif not source:
            # An empty COCO result is not proof that the scene itself is empty.
            return PerceptionReply(False)

        source = sorted(source, key=lambda item: (item.depth_m is None, item.depth_m or 999, -item.confidence))
        descriptions = [self._describe_target(item) for item in source[:4]]
        prefix = "我识别到" if len(source) == 1 else f"我识别到{len(source)}个目标："
        reply = prefix + (descriptions[0] if len(source) == 1 else "、".join(descriptions))
        return PerceptionReply(True, reply, tuple(source))

    @staticmethod
    def _combine(
        targets_3d: list[DetectedTarget], detections: list[DetectedTarget]
    ) -> list[DetectedTarget]:
        """Merge depth metadata with matching YOLO image-space geometry."""
        combined: list[DetectedTarget] = []
        remaining = list(detections)
        for target in targets_3d:
            match_index = next(
                (index for index, item in enumerate(remaining) if item.label == target.label),
                None,
            )
            if match_index is None:
                combined.append(target)
                continue
            detection = remaining.pop(match_index)
            combined.append(
                replace(
                    target,
                    box=detection.box,
                    image_width=detection.image_width,
                    image_height=detection.image_height,
                )
            )
        combined.extend(remaining)
        return combined

    @staticmethod
    def _describe_target(target: DetectedTarget) -> str:
        name = LABEL_NAMES.get(target.label, target.label)
        direction = ""
        if target.camera_x is not None:
            if target.camera_x < -0.15:
                direction = "左侧"
            elif target.camera_x > 0.15:
                direction = "右侧"
            else:
                direction = "前方"
        distance = f"约{target.depth_m:.1f}米" if target.depth_m is not None else ""
        return f"{direction}{name}{distance}"
