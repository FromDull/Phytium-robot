#!/usr/bin/env python3
"""Persistent semantic landmark fusion for map-frame vision targets."""

import json
import math
import os
import threading
import time
from collections import deque
from pathlib import Path


class SemanticMapStore:
    SCHEMA_VERSION = 1

    def __init__(
        self,
        path,
        association_radius_m=0.8,
        movement_threshold_m=0.35,
        minimum_observations=3,
        candidate_timeout_s=30.0,
        active_timeout_s=3.0,
        clock=time.time,
    ):
        self.path = Path(path)
        self.association_radius_m = float(association_radius_m)
        self.movement_threshold_m = float(movement_threshold_m)
        self.minimum_observations = int(minimum_observations)
        self.candidate_timeout_s = float(candidate_timeout_s)
        self.active_timeout_s = float(active_timeout_s)
        self.clock = clock
        self.lock = threading.Lock()
        self.objects = {}
        self.changes = deque(maxlen=100)
        self.next_id = 1
        self.revision = 0
        self.dirty = False
        self.last_flush = 0.0
        self._load()

    @staticmethod
    def _position(target):
        positions = target.get("positions") or {}
        point = positions.get("map")
        if not isinstance(point, dict):
            return None
        try:
            values = {axis: float(point.get(axis, 0.0)) for axis in ("x", "y", "z")}
        except (TypeError, ValueError):
            return None
        return values if all(math.isfinite(value) for value in values.values()) else None

    @staticmethod
    def _distance(left, right):
        return math.sqrt(sum((left[axis] - right[axis]) ** 2 for axis in ("x", "y", "z")))

    @staticmethod
    def _safe_label(value):
        label = str(value or "unknown").strip()
        return label[:80] or "unknown"

    def _load(self):
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            if payload.get("schema_version") != self.SCHEMA_VERSION:
                return
            objects = payload.get("objects") or []
            self.objects = {
                item["id"]: item
                for item in objects
                if isinstance(item, dict) and isinstance(item.get("id"), str)
            }
            self.changes.extend(payload.get("changes") or [])
            self.next_id = max(1, int(payload.get("next_id", 1)))
            self.revision = max(0, int(payload.get("revision", 0)))
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return

    def _new_id(self, label):
        slug = "".join(character if character.isalnum() else "-" for character in label.lower())
        slug = "-".join(part for part in slug.split("-") if part)[:32] or "object"
        object_id = f"{slug}-{self.next_id:04d}"
        self.next_id += 1
        return object_id

    def _record_change(self, object_id, kind, now, previous=None, current=None):
        event = {"object_id": object_id, "kind": kind, "timestamp": now}
        if previous is not None:
            event["previous_position"] = dict(previous)
        if current is not None:
            event["position"] = dict(current)
        self.changes.appendleft(event)

    def _create(self, target, position, now):
        label = self._safe_label(target.get("label"))
        object_id = self._new_id(label)
        landmark = {
            "id": object_id,
            "class_id": int(target.get("class_id", -1)),
            "label": label,
            "position": dict(position),
            "confidence": round(float(target.get("confidence", 0.0)), 4),
            "observations": 1,
            "confirmed": self.minimum_observations <= 1,
            "first_seen": now,
            "last_seen": now,
            "last_changed": now,
            "movement_count": 0,
            "depth_m": float(target.get("depth_m", 0.0) or 0.0),
        }
        self.objects[object_id] = landmark
        self._record_change(object_id, "created", now, current=position)
        return landmark

    def _update(self, landmark, target, position, now):
        previous = dict(landmark["position"])
        displacement = self._distance(previous, position)
        moved = displacement >= self.movement_threshold_m
        alpha = 0.65 if moved else 0.25
        landmark["position"] = {
            axis: round(previous[axis] * (1.0 - alpha) + position[axis] * alpha, 4)
            for axis in ("x", "y", "z")
        }
        confidence = max(0.0, min(1.0, float(target.get("confidence", 0.0))))
        landmark["confidence"] = round(
            float(landmark.get("confidence", confidence)) * 0.8 + confidence * 0.2,
            4,
        )
        landmark["observations"] = int(landmark.get("observations", 0)) + 1
        landmark["last_seen"] = now
        landmark["depth_m"] = float(target.get("depth_m", 0.0) or 0.0)
        if not landmark.get("confirmed") and landmark["observations"] >= self.minimum_observations:
            landmark["confirmed"] = True
            self._record_change(landmark["id"], "confirmed", now, current=landmark["position"])
        if moved:
            landmark["last_changed"] = now
            landmark["movement_count"] = int(landmark.get("movement_count", 0)) + 1
            self._record_change(
                landmark["id"], "moved", now, previous=previous, current=landmark["position"]
            )

    def ingest(self, targets, observed_at=None):
        now = float(self.clock() if observed_at is None else observed_at)
        observations = []
        for target in targets or []:
            if not isinstance(target, dict):
                continue
            position = self._position(target)
            if position is None:
                continue
            try:
                confidence = float(target.get("confidence", 0.0))
            except (TypeError, ValueError):
                continue
            if not math.isfinite(confidence) or confidence < 0.25:
                continue
            observations.append((target, position))

        with self.lock:
            matched = set()
            for target, position in observations:
                class_id = int(target.get("class_id", -1))
                label = self._safe_label(target.get("label"))
                candidates = [
                    landmark
                    for landmark in self.objects.values()
                    if landmark["id"] not in matched
                    and int(landmark.get("class_id", -2)) == class_id
                    and landmark.get("label") == label
                ]
                nearest = min(
                    candidates,
                    key=lambda item: self._distance(item["position"], position),
                    default=None,
                )
                if nearest is None or self._distance(nearest["position"], position) > self.association_radius_m:
                    nearest = self._create(target, position, now)
                else:
                    self._update(nearest, target, position, now)
                matched.add(nearest["id"])

            expired = [
                object_id
                for object_id, landmark in self.objects.items()
                if not landmark.get("confirmed")
                and now - float(landmark.get("last_seen", now)) > self.candidate_timeout_s
            ]
            for object_id in expired:
                del self.objects[object_id]
            if observations or expired:
                self.revision += 1
                self.dirty = True
        self.flush()

    def snapshot(self):
        now = float(self.clock())
        with self.lock:
            objects = []
            for landmark in self.objects.values():
                item = dict(landmark)
                item["position"] = dict(landmark["position"])
                age = max(0.0, now - float(landmark.get("last_seen", now)))
                item["age_s"] = round(age, 1)
                item["status"] = (
                    "candidate"
                    if not landmark.get("confirmed")
                    else "active"
                    if age <= self.active_timeout_s
                    else "remembered"
                )
                objects.append(item)
            objects.sort(key=lambda item: (not item.get("confirmed"), -item.get("last_seen", 0.0)))
            confirmed = [item for item in objects if item.get("confirmed")]
            return {
                "schema_version": self.SCHEMA_VERSION,
                "revision": self.revision,
                "updated_at": now,
                "objects": objects,
                "changes": [dict(change) for change in self.changes],
                "stats": {
                    "total": len(objects),
                    "confirmed": len(confirmed),
                    "active": sum(item["status"] == "active" for item in confirmed),
                    "remembered": sum(item["status"] == "remembered" for item in confirmed),
                    "candidates": sum(not item.get("confirmed") for item in objects),
                    "moved": sum(int(item.get("movement_count", 0)) > 0 for item in confirmed),
                },
            }

    def remove(self, object_id):
        now = float(self.clock())
        with self.lock:
            if object_id not in self.objects:
                return False
            del self.objects[object_id]
            self._record_change(object_id, "removed", now)
            self.revision += 1
            self.dirty = True
        self.flush(force=True)
        return True

    def clear(self):
        now = float(self.clock())
        with self.lock:
            count = len(self.objects)
            self.objects.clear()
            self.changes.clear()
            self.changes.appendleft({"kind": "cleared", "timestamp": now, "count": count})
            self.revision += 1
            self.dirty = True
        self.flush(force=True)
        return count

    def flush(self, force=False):
        now = float(self.clock())
        with self.lock:
            if not self.dirty or (not force and now - self.last_flush < 2.0):
                return
            payload = {
                "schema_version": self.SCHEMA_VERSION,
                "revision": self.revision,
                "next_id": self.next_id,
                "objects": list(self.objects.values()),
                "changes": list(self.changes),
            }
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix(self.path.suffix + ".tmp")
            temporary.write_text(
                json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
                encoding="utf-8",
            )
            os.replace(temporary, self.path)
            self.last_flush = now
            self.dirty = False
