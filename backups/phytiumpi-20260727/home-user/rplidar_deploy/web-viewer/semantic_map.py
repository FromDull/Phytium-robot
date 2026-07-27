#!/usr/bin/env python3
"""Persistent, quality-gated semantic landmark fusion in the map frame."""

from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from collections import Counter, deque
from pathlib import Path


class SemanticMapStore:
    """Fuse vision observations without letting one bad TF sample move a landmark."""

    SCHEMA_VERSION = 2
    SUPPORTED_SCHEMA_VERSIONS = {1, 2}

    def __init__(
        self,
        path,
        association_radius_m=0.8,
        movement_threshold_m=0.35,
        minimum_observations=3,
        candidate_timeout_s=30.0,
        active_timeout_s=3.0,
        dynamic_timeout_s=20.0,
        deduplication_radius_m=0.35,
        minimum_confidence=0.35,
        minimum_quality=0.25,
        minimum_depth_samples=12,
        maximum_depth_mad_m=0.12,
        maximum_observation_age_s=2.5,
        confirmation_radius_m=0.25,
        confirmation_min_duration_s=0.4,
        relocation_observations=3,
        relocation_radius_m=0.25,
        relocation_timeout_s=4.0,
        maximum_static_jump_m=1.25,
        dynamic_labels=("person",),
        site_id=None,
        clock=time.time,
    ):
        self.path = Path(path)
        self.backup_path = self.path.with_suffix(self.path.suffix + ".bak")
        self.association_radius_m = float(association_radius_m)
        self.movement_threshold_m = float(movement_threshold_m)
        self.minimum_observations = max(1, int(minimum_observations))
        self.candidate_timeout_s = float(candidate_timeout_s)
        self.active_timeout_s = float(active_timeout_s)
        self.dynamic_timeout_s = float(dynamic_timeout_s)
        self.deduplication_radius_m = float(deduplication_radius_m)
        self.minimum_confidence = float(minimum_confidence)
        self.minimum_quality = float(minimum_quality)
        self.minimum_depth_samples = max(1, int(minimum_depth_samples))
        self.maximum_depth_mad_m = float(maximum_depth_mad_m)
        self.maximum_observation_age_s = float(maximum_observation_age_s)
        self.confirmation_radius_m = float(confirmation_radius_m)
        self.confirmation_min_duration_s = float(confirmation_min_duration_s)
        self.relocation_observations = max(2, int(relocation_observations))
        self.relocation_radius_m = float(relocation_radius_m)
        self.relocation_timeout_s = float(relocation_timeout_s)
        self.maximum_static_jump_m = float(maximum_static_jump_m)
        self.dynamic_labels = {self._safe_label(label) for label in dynamic_labels}
        self.configured_site_id = str(site_id).strip() if site_id else None
        self.site_id = self.configured_site_id or uuid.uuid4().hex
        self.clock = clock
        self.lock = threading.RLock()
        self.objects = {}
        self.changes = deque(maxlen=100)
        self.next_id = 1
        self.revision = 0
        self.dirty = False
        self.last_flush = 0.0
        self.persistence_error = None
        self.recovered_from_backup = False
        self.accepted_observations = 0
        self.rejected_observations = 0
        self.rejection_counts = Counter()
        self._load()

    @staticmethod
    def _finite_float(value, default=0.0):
        try:
            result = float(value)
        except (TypeError, ValueError):
            return float(default)
        return result if math.isfinite(result) else float(default)

    @classmethod
    def _position(cls, target):
        positions = target.get("positions") or {}
        point = positions.get("map")
        if not isinstance(point, dict):
            return None
        values = {axis: cls._finite_float(point.get(axis), math.nan) for axis in ("x", "y", "z")}
        return values if all(math.isfinite(value) for value in values.values()) else None

    @staticmethod
    def _distance(left, right):
        return math.sqrt(sum((left[axis] - right[axis]) ** 2 for axis in ("x", "y", "z")))

    @staticmethod
    def _safe_label(value):
        label = str(value or "unknown").strip().lower().replace(" ", "-")
        return label[:80] or "unknown"

    @classmethod
    def _normalized_position(cls, value):
        if not isinstance(value, dict):
            return None
        result = {axis: cls._finite_float(value.get(axis), math.nan) for axis in ("x", "y", "z")}
        return result if all(math.isfinite(item) for item in result.values()) else None

    def _read_payload(self, path):
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("semantic map root is not an object")
        if int(payload.get("schema_version", 0)) not in self.SUPPORTED_SCHEMA_VERSIONS:
            raise ValueError("unsupported semantic map schema")
        if not isinstance(payload.get("objects", []), list):
            raise ValueError("semantic map objects is not a list")
        return payload

    def _normalize_landmark(self, item, now):
        if not isinstance(item, dict) or not isinstance(item.get("id"), str):
            return None
        position = self._normalized_position(item.get("position"))
        if position is None:
            return None
        observations = max(1, int(self._finite_float(item.get("observations"), 1)))
        label = self._safe_label(item.get("label"))
        first_seen = self._finite_float(item.get("first_seen"), now)
        last_seen = self._finite_float(item.get("last_seen"), first_seen)
        landmark = {
            "id": item["id"][:96],
            "class_id": int(self._finite_float(item.get("class_id"), -1)),
            "label": label,
            "position": position,
            "confidence": round(max(0.0, min(1.0, self._finite_float(item.get("confidence")))), 4),
            "quality": round(max(0.0, min(1.0, self._finite_float(item.get("quality"), item.get("confidence", 0.0)))), 4),
            "observations": observations,
            "confirmed": bool(item.get("confirmed")),
            "first_seen": first_seen,
            "last_seen": last_seen,
            "last_changed": self._finite_float(item.get("last_changed"), first_seen),
            "movement_count": max(0, int(self._finite_float(item.get("movement_count"), 0))),
            "depth_m": max(0.0, self._finite_float(item.get("depth_m"), 0.0)),
            "position_spread_m": max(0.0, self._finite_float(item.get("position_spread_m"), 0.0)),
            "position_weight": max(0.1, min(100.0, self._finite_float(item.get("position_weight"), observations))),
            "confirmation_hits": max(1, int(self._finite_float(item.get("confirmation_hits"), observations))),
            "confirmation_first_seen": self._finite_float(item.get("confirmation_first_seen"), first_seen),
            "confirmation_anchor": self._normalized_position(item.get("confirmation_anchor")) or dict(position),
        }
        pending = item.get("relocation_candidate")
        if isinstance(pending, dict):
            pending_position = self._normalized_position(pending.get("position"))
            if pending_position is not None:
                landmark["relocation_candidate"] = {
                    "position": pending_position,
                    "observations": max(1, int(self._finite_float(pending.get("observations"), 1))),
                    "first_seen": self._finite_float(pending.get("first_seen"), last_seen),
                    "last_seen": self._finite_float(pending.get("last_seen"), last_seen),
                    "weight": max(0.1, self._finite_float(pending.get("weight"), 1.0)),
                }
        return landmark

    def _load(self):
        payload = None
        loaded_path = None
        for candidate in (self.path, self.backup_path):
            try:
                payload = self._read_payload(candidate)
                loaded_path = candidate
                break
            except (OSError, ValueError, TypeError, json.JSONDecodeError):
                continue
        if payload is None:
            return
        now = float(self.clock())
        self.objects = {}
        for item in payload.get("objects") or []:
            landmark = self._normalize_landmark(item, now)
            if landmark is not None:
                self.objects[landmark["id"]] = landmark
        self.changes.extend(change for change in (payload.get("changes") or []) if isinstance(change, dict))
        self.next_id = max(1, int(self._finite_float(payload.get("next_id"), 1)))
        self.revision = max(0, int(self._finite_float(payload.get("revision"), 0)))
        stored_site_id = str(payload.get("site_id") or "").strip()
        self.site_id = self.configured_site_id or stored_site_id or self.site_id
        fusion = payload.get("fusion_stats") or {}
        self.accepted_observations = max(0, int(self._finite_float(fusion.get("accepted"), 0)))
        self.rejected_observations = max(0, int(self._finite_float(fusion.get("rejected"), 0)))
        self.recovered_from_backup = loaded_path == self.backup_path
        if int(payload.get("schema_version", 0)) != self.SCHEMA_VERSION or self.recovered_from_backup:
            self.dirty = True

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

    def _is_dynamic(self, landmark):
        return self._safe_label(landmark.get("label")) in self.dynamic_labels

    def _reject(self, reason):
        with self.lock:
            self.rejected_observations += 1
            self.rejection_counts[str(reason)] += 1

    def _observation_quality(self, target):
        if target.get("map_position_valid") is False:
            return None, "map_tf_invalid"
        confidence = self._finite_float(target.get("confidence"), math.nan)
        if not math.isfinite(confidence) or confidence < self.minimum_confidence:
            return None, "confidence"
        age = target.get("observation_age_s")
        if age is not None:
            age = self._finite_float(age, math.inf)
            if age < 0.0 or age > self.maximum_observation_age_s:
                return None, "stale"
        samples = target.get("depth_samples")
        if samples is not None:
            samples = int(self._finite_float(samples, 0))
            if samples < self.minimum_depth_samples:
                return None, "depth_samples"
        mad = target.get("depth_mad_m")
        if mad is not None:
            mad = self._finite_float(mad, math.inf)
            if mad < 0.0 or mad > self.maximum_depth_mad_m:
                return None, "depth_spread"
        depth = max(0.0, self._finite_float(target.get("depth_m"), 0.0))
        sample_score = 1.0 if samples is None else min(1.0, samples / (self.minimum_depth_samples * 3.0))
        mad_limit = max(0.03, min(self.maximum_depth_mad_m, 0.04 * max(1.0, depth)))
        mad_score = 1.0 if mad is None else max(0.0, 1.0 - mad / mad_limit)
        computed = confidence * (0.70 + 0.30 * sample_score) * (0.60 + 0.40 * mad_score)
        supplied = target.get("quality")
        quality = computed if supplied is None else min(computed, self._finite_float(supplied, 0.0))
        quality = max(0.0, min(1.0, quality))
        if quality < self.minimum_quality:
            return None, "quality"
        return quality, None

    def _expire_stale(self, now):
        expired = []
        for object_id, landmark in self.objects.items():
            age = now - float(landmark.get("last_seen", now))
            if not landmark.get("confirmed") and age > self.candidate_timeout_s:
                expired.append(object_id)
            elif self._is_dynamic(landmark) and age > self.dynamic_timeout_s:
                expired.append(object_id)
        for object_id in expired:
            landmark = self.objects.pop(object_id)
            self._record_change(object_id, "expired", now, previous=landmark.get("position"))
        return len(expired)

    @staticmethod
    def _blend_position(left, right, alpha):
        return {
            axis: round(left[axis] * (1.0 - alpha) + right[axis] * alpha, 4)
            for axis in ("x", "y", "z")
        }

    def _merge_static_duplicates(self, now):
        merged = 0
        ordered = sorted(self.objects.values(), key=lambda item: int(item.get("observations", 0)), reverse=True)
        removed = set()
        for index, keeper in enumerate(ordered):
            if keeper["id"] in removed or self._is_dynamic(keeper):
                continue
            for duplicate in ordered[index + 1 :]:
                if duplicate["id"] in removed or self._is_dynamic(duplicate):
                    continue
                if int(keeper.get("class_id", -1)) != int(duplicate.get("class_id", -2)):
                    continue
                if keeper.get("label") != duplicate.get("label"):
                    continue
                if self._distance(keeper["position"], duplicate["position"]) > self.deduplication_radius_m:
                    continue
                left_weight = max(0.1, float(keeper.get("position_weight", 1.0)))
                right_weight = max(0.1, float(duplicate.get("position_weight", 1.0)))
                total_weight = left_weight + right_weight
                keeper["position"] = {
                    axis: round(
                        (keeper["position"][axis] * left_weight + duplicate["position"][axis] * right_weight)
                        / total_weight,
                        4,
                    )
                    for axis in ("x", "y", "z")
                }
                left_count = max(1, int(keeper.get("observations", 1)))
                right_count = max(1, int(duplicate.get("observations", 1)))
                total_count = left_count + right_count
                for field in ("confidence", "quality"):
                    keeper[field] = round(
                        (float(keeper.get(field, 0.0)) * left_count + float(duplicate.get(field, 0.0)) * right_count)
                        / total_count,
                        4,
                    )
                keeper["observations"] = total_count
                keeper["position_weight"] = min(100.0, total_weight)
                keeper["position_spread_m"] = round(max(
                    float(keeper.get("position_spread_m", 0.0)),
                    float(duplicate.get("position_spread_m", 0.0)),
                ), 4)
                keeper["confirmed"] = bool(keeper.get("confirmed") or duplicate.get("confirmed"))
                keeper["first_seen"] = min(keeper.get("first_seen", now), duplicate.get("first_seen", now))
                keeper["last_seen"] = max(keeper.get("last_seen", 0.0), duplicate.get("last_seen", 0.0))
                keeper["movement_count"] = int(keeper.get("movement_count", 0)) + int(duplicate.get("movement_count", 0))
                removed.add(duplicate["id"])
                self._record_change(keeper["id"], "merged", now, previous=duplicate["position"], current=keeper["position"])
                merged += 1
        for object_id in removed:
            self.objects.pop(object_id, None)
        return merged

    def _create(self, target, position, quality, now):
        label = self._safe_label(target.get("label"))
        object_id = self._new_id(label)
        confirmed = self.minimum_observations <= 1
        landmark = {
            "id": object_id,
            "class_id": int(self._finite_float(target.get("class_id"), -1)),
            "label": label,
            "position": dict(position),
            "confidence": round(max(0.0, min(1.0, self._finite_float(target.get("confidence")))), 4),
            "quality": round(quality, 4),
            "observations": 1,
            "confirmed": confirmed,
            "first_seen": now,
            "last_seen": now,
            "last_changed": now,
            "movement_count": 0,
            "depth_m": max(0.0, self._finite_float(target.get("depth_m"), 0.0)),
            "position_spread_m": 0.0,
            "position_weight": max(0.1, quality),
            "confirmation_hits": 1,
            "confirmation_first_seen": now,
            "confirmation_anchor": dict(position),
        }
        self.objects[object_id] = landmark
        self._record_change(object_id, "created", now, current=position)
        return landmark

    def _update_metrics(self, landmark, target, quality, displacement):
        confidence = max(0.0, min(1.0, self._finite_float(target.get("confidence"))))
        landmark["confidence"] = round(float(landmark.get("confidence", confidence)) * 0.8 + confidence * 0.2, 4)
        landmark["quality"] = round(float(landmark.get("quality", quality)) * 0.8 + quality * 0.2, 4)
        previous_spread = float(landmark.get("position_spread_m", 0.0))
        landmark["position_spread_m"] = round(math.sqrt(0.8 * previous_spread**2 + 0.2 * displacement**2), 4)
        landmark["position_weight"] = min(100.0, float(landmark.get("position_weight", 1.0)) + quality)
        landmark["depth_m"] = max(0.0, self._finite_float(target.get("depth_m"), 0.0))

    def _update_candidate(self, landmark, position, quality, now):
        anchor = landmark.get("confirmation_anchor") or dict(landmark["position"])
        if self._distance(anchor, position) <= self.confirmation_radius_m:
            hits = int(landmark.get("confirmation_hits", 1)) + 1
            alpha = max(0.15, min(0.45, quality / max(0.2, hits * 0.5)))
            anchor = self._blend_position(anchor, position, alpha)
        else:
            hits = 1
            anchor = dict(position)
            landmark["confirmation_first_seen"] = now
        landmark["confirmation_hits"] = hits
        landmark["confirmation_anchor"] = anchor
        landmark["position"] = dict(anchor)
        duration = now - float(landmark.get("confirmation_first_seen", now))
        if hits >= self.minimum_observations and duration >= self.confirmation_min_duration_s:
            landmark["confirmed"] = True
            landmark["last_changed"] = now
            self._record_change(landmark["id"], "confirmed", now, current=landmark["position"])

    def _observe_static_relocation(self, landmark, position, quality, now):
        displacement = self._distance(landmark["position"], position)
        if displacement > self.maximum_static_jump_m:
            landmark.pop("relocation_candidate", None)
            self._reject("static_jump")
            return False
        pending = landmark.get("relocation_candidate")
        if (
            not isinstance(pending, dict)
            or now - float(pending.get("last_seen", now)) > self.relocation_timeout_s
            or self._distance(pending.get("position", position), position) > self.relocation_radius_m
        ):
            pending = {
                "position": dict(position),
                "observations": 1,
                "first_seen": now,
                "last_seen": now,
                "weight": max(0.1, quality),
            }
            landmark["relocation_candidate"] = pending
            return False
        old_weight = max(0.1, float(pending.get("weight", 1.0)))
        new_weight = max(0.1, quality)
        alpha = new_weight / (old_weight + new_weight)
        pending["position"] = self._blend_position(pending["position"], position, alpha)
        pending["observations"] = int(pending.get("observations", 1)) + 1
        pending["last_seen"] = now
        pending["weight"] = old_weight + new_weight
        duration = now - float(pending.get("first_seen", now))
        if pending["observations"] < self.relocation_observations or duration < self.confirmation_min_duration_s:
            return False
        previous = dict(landmark["position"])
        landmark["position"] = dict(pending["position"])
        landmark["last_changed"] = now
        landmark["movement_count"] = int(landmark.get("movement_count", 0)) + 1
        landmark["position_spread_m"] = 0.0
        landmark.pop("relocation_candidate", None)
        self._record_change(landmark["id"], "moved", now, previous=previous, current=landmark["position"])
        return True

    def _update(self, landmark, target, position, quality, now):
        previous = dict(landmark["position"])
        displacement = self._distance(previous, position)
        landmark["observations"] = int(landmark.get("observations", 0)) + 1
        landmark["last_seen"] = now
        self._update_metrics(landmark, target, quality, displacement)
        if not landmark.get("confirmed"):
            self._update_candidate(landmark, position, quality, now)
            return
        if self._is_dynamic(landmark):
            landmark["position"] = self._blend_position(previous, position, 0.55)
            landmark.pop("relocation_candidate", None)
            return
        if displacement < self.movement_threshold_m:
            alpha = 0.08 + 0.20 * quality
            landmark["position"] = self._blend_position(previous, position, alpha)
            landmark.pop("relocation_candidate", None)
            return
        self._observe_static_relocation(landmark, position, quality, now)

    def ingest(self, targets, observed_at=None, source_context=None):
        now = float(self.clock())
        observations = []
        context = source_context if isinstance(source_context, dict) else {}
        context_valid = context.get("map_tf_valid") is not False
        if observed_at is not None:
            observed_at = self._finite_float(observed_at, math.nan)
            if not math.isfinite(observed_at) or abs(now - observed_at) > self.maximum_observation_age_s:
                context_valid = False
        for target in targets or []:
            if not isinstance(target, dict):
                self._reject("format")
                continue
            if not context_valid:
                self._reject("source_tf_or_age")
                continue
            position = self._position(target)
            if position is None:
                self._reject("map_position")
                continue
            if not -0.30 <= position["z"] <= 2.50:
                self._reject("height")
                continue
            quality, reason = self._observation_quality(target)
            if quality is None:
                self._reject(reason)
                continue
            observations.append((target, position, quality))

        with self.lock:
            matched = set()
            for target, position, quality in sorted(observations, key=lambda item: item[2], reverse=True):
                class_id = int(self._finite_float(target.get("class_id"), -1))
                label = self._safe_label(target.get("label"))
                candidates = [
                    landmark
                    for landmark in self.objects.values()
                    if landmark["id"] not in matched
                    and int(landmark.get("class_id", -2)) == class_id
                    and landmark.get("label") == label
                ]
                nearest = min(candidates, key=lambda item: self._distance(item["position"], position), default=None)
                if nearest is None or self._distance(nearest["position"], position) > self.association_radius_m:
                    nearest = self._create(target, position, quality, now)
                else:
                    self._update(nearest, target, position, quality, now)
                matched.add(nearest["id"])
                self.accepted_observations += 1

            merged = self._merge_static_duplicates(now)
            expired = self._expire_stale(now)
            if observations or expired or merged:
                self.revision += 1
                self.dirty = True
        self.flush()

    def snapshot(self):
        now = float(self.clock())
        expired = False
        with self.lock:
            if self._expire_stale(now):
                self.revision += 1
                self.dirty = True
                expired = True
            objects = []
            for landmark in self.objects.values():
                item = dict(landmark)
                item["position"] = dict(landmark["position"])
                item.pop("confirmation_anchor", None)
                pending = item.pop("relocation_candidate", None)
                if pending:
                    item["relocation_pending"] = {
                        "observations": int(pending.get("observations", 0)),
                        "position": dict(pending.get("position") or {}),
                    }
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
            result = {
                "schema_version": self.SCHEMA_VERSION,
                "site_id": self.site_id,
                "revision": self.revision,
                "updated_at": now,
                "objects": objects,
                "changes": [dict(change) for change in self.changes],
                "fusion": {
                    "accepted": self.accepted_observations,
                    "rejected": self.rejected_observations,
                    "rejection_counts": dict(self.rejection_counts),
                    "recovered_from_backup": self.recovered_from_backup,
                    "persistence_error": self.persistence_error,
                },
                "stats": {
                    "total": len(objects),
                    "confirmed": len(confirmed),
                    "active": sum(item["status"] == "active" for item in confirmed),
                    "remembered": sum(item["status"] == "remembered" for item in confirmed),
                    "candidates": sum(not item.get("confirmed") for item in objects),
                    "moved": sum(int(item.get("movement_count", 0)) > 0 for item in confirmed if not self._is_dynamic(item)),
                    "relocation_pending": sum(bool(item.get("relocation_pending")) for item in objects),
                    "dynamic": sum(self._is_dynamic(item) for item in objects),
                    "static": sum(not self._is_dynamic(item) for item in confirmed),
                },
            }
        if expired:
            self.flush(force=True)
        return result

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

    def clear(self, new_site=False):
        now = float(self.clock())
        with self.lock:
            count = len(self.objects)
            self.objects.clear()
            self.changes.clear()
            if new_site:
                self.site_id = self.configured_site_id or uuid.uuid4().hex
            self.changes.appendleft({"kind": "cleared", "timestamp": now, "count": count, "new_site": bool(new_site)})
            self.revision += 1
            self.dirty = True
        self.flush(force=True)
        return count

    @staticmethod
    def _atomic_write(path, text):
        temporary = path.with_suffix(path.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8") as output:
            output.write(text)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)

    def flush(self, force=False):
        now = float(self.clock())
        with self.lock:
            if not self.dirty or (not force and now - self.last_flush < 2.0):
                return False
            payload = {
                "schema_version": self.SCHEMA_VERSION,
                "site_id": self.site_id,
                "revision": self.revision,
                "next_id": self.next_id,
                "objects": list(self.objects.values()),
                "changes": list(self.changes),
                "fusion_stats": {
                    "accepted": self.accepted_observations,
                    "rejected": self.rejected_observations,
                },
            }
            serialized = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                self._atomic_write(self.path, serialized)
                self._atomic_write(self.backup_path, serialized)
            except OSError as error:
                self.persistence_error = str(error)
                return False
            self.last_flush = now
            self.dirty = False
            self.persistence_error = None
            self.recovered_from_backup = False
            return True
