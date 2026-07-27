"""Low-rate RGB-D voxel integration for semantic-map context."""

from __future__ import annotations

import math
import struct
import threading
import time


class DepthVoxelMap:
    def __init__(
        self,
        voxel_size_m=0.10,
        pixel_stride=8,
        minimum_depth_m=0.25,
        maximum_depth_m=4.0,
        minimum_z_m=0.04,
        maximum_z_m=1.8,
        minimum_observations=3,
        stale_after_s=120.0,
        maximum_voxels=12000,
        maximum_output_voxels=6000,
        depth_edge_threshold_m=0.12,
        robot_half_length_m=0.20,
        robot_half_width_m=0.25,
    ):
        self.voxel_size_m = float(voxel_size_m)
        self.pixel_stride = int(pixel_stride)
        self.minimum_depth_m = float(minimum_depth_m)
        self.maximum_depth_m = float(maximum_depth_m)
        self.minimum_z_m = float(minimum_z_m)
        self.maximum_z_m = float(maximum_z_m)
        self.minimum_observations = int(minimum_observations)
        self.stale_after_s = float(stale_after_s)
        self.maximum_voxels = int(maximum_voxels)
        self.maximum_output_voxels = int(maximum_output_voxels)
        self.depth_edge_threshold_m = float(depth_edge_threshold_m)
        self.robot_half_length_m = float(robot_half_length_m)
        self.robot_half_width_m = float(robot_half_width_m)
        self.lock = threading.RLock()
        self.voxels = {}
        self.frames_integrated = 0
        self.frames_rejected = 0
        self.last_integrated_at = None
        self.last_cleared_at = None

    @staticmethod
    def _rotation_matrix(quaternion):
        x = float(quaternion["x"])
        y = float(quaternion["y"])
        z = float(quaternion["z"])
        w = float(quaternion["w"])
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1e-9:
            raise ValueError("invalid transform quaternion")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        )

    def reject_frame(self):
        with self.lock:
            self.frames_rejected += 1

    def clear(self, now=None):
        stamp = time.monotonic() if now is None else float(now)
        with self.lock:
            count = len(self.voxels)
            self.voxels.clear()
            self.last_integrated_at = None
            self.last_cleared_at = stamp
            return count

    @staticmethod
    def _map_relation(world_x, world_y, map_context):
        if not map_context:
            return 0
        resolution = float(map_context.get("resolution", 0))
        width = int(map_context.get("width", 0))
        height = int(map_context.get("height", 0))
        data = map_context.get("data")
        if resolution <= 0 or width <= 0 or height <= 0 or data is None:
            return 0
        yaw = float(map_context.get("origin_yaw", 0))
        dx = world_x - float(map_context.get("origin_x", 0))
        dy = world_y - float(map_context.get("origin_y", 0))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        grid_x = math.floor((cos_yaw * dx + sin_yaw * dy) / resolution)
        grid_y = math.floor((-sin_yaw * dx + cos_yaw * dy) / resolution)
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return 0
        occupancy = data[grid_y * width + grid_x]
        if occupancy == 255:
            return 0
        return 2 if occupancy >= 55 else 1

    @classmethod
    def _point_in_robot_footprint(cls, world_x, world_y, robot_pose, half_length, half_width):
        if not robot_pose:
            return False
        position = robot_pose.get("position") or {}
        quaternion = robot_pose.get("quaternion") or {}
        rotation = cls._rotation_matrix(quaternion)
        dx = world_x - float(position.get("x", 0))
        dy = world_y - float(position.get("y", 0))
        local_x = rotation[0][0] * dx + rotation[1][0] * dy
        local_y = rotation[0][1] * dx + rotation[1][1] * dy
        return abs(local_x) <= half_length and abs(local_y) <= half_width

    def integrate(self, *args, **kwargs):
        with self.lock:
            return self._integrate(*args, **kwargs)

    def _integrate(
        self,
        depth_data,
        width,
        height,
        encoding,
        is_bigendian,
        intrinsics,
        transform,
        rgb_data=None,
        rgb_encoding="rgb8",
        map_context=None,
        robot_pose=None,
        lidar_points=None,
        now=None,
    ):
        if encoding not in {"16UC1", "32FC1"}:
            raise ValueError(f"unsupported depth encoding: {encoding}")
        width = int(width)
        height = int(height)
        if width <= 0 or height <= 0:
            raise ValueError("invalid depth dimensions")
        bytes_per_depth = 2 if encoding == "16UC1" else 4
        if len(depth_data) < width * height * bytes_per_depth:
            raise ValueError("short depth image")

        fx = float(intrinsics["fx"])
        fy = float(intrinsics["fy"])
        cx = float(intrinsics["cx"])
        cy = float(intrinsics["cy"])
        if fx <= 0 or fy <= 0:
            raise ValueError("invalid camera intrinsics")

        rotation = self._rotation_matrix(transform["quaternion"])
        translation = transform["translation"]
        tx = float(translation["x"])
        ty = float(translation["y"])
        tz = float(translation["z"])
        stamp = time.monotonic() if now is None else float(now)
        endian = ">" if is_bigendian else "<"
        depth_format = endian + ("H" if encoding == "16UC1" else "f")
        rgb_channels = 3 if rgb_encoding in {"rgb8", "bgr8"} else None
        if rgb_data is not None and (
            rgb_channels is None or len(rgb_data) < width * height * rgb_channels
        ):
            rgb_data = None

        def read_depth(pixel_index):
            raw = struct.unpack_from(
                depth_format, depth_data, pixel_index * bytes_per_depth
            )[0]
            return raw * 0.001 if encoding == "16UC1" else float(raw)

        seen = set()
        accepted = 0
        voxel_size = self.voxel_size_m
        lidar_cells = {}
        for lidar_point in lidar_points or ():
            lidar_x = float(lidar_point[0])
            lidar_y = float(lidar_point[1])
            cell = (
                math.floor(lidar_x / voxel_size),
                math.floor(lidar_y / voxel_size),
            )
            lidar_cells.setdefault(cell, []).append((lidar_x, lidar_y))
        for v in range(self.pixel_stride // 2, height, self.pixel_stride):
            row_offset = v * width
            for u in range(self.pixel_stride // 2, width, self.pixel_stride):
                pixel_index = row_offset + u
                depth_m = read_depth(pixel_index)
                if not math.isfinite(depth_m) or not (
                    self.minimum_depth_m <= depth_m <= self.maximum_depth_m
                ):
                    continue
                neighbor_depths = []
                for neighbor_u, neighbor_v in (
                    (u - 1, v), (u + 1, v), (u, v - 1), (u, v + 1)
                ):
                    if 0 <= neighbor_u < width and 0 <= neighbor_v < height:
                        neighbor = read_depth(neighbor_v * width + neighbor_u)
                        if math.isfinite(neighbor) and neighbor > 0:
                            neighbor_depths.append(neighbor)
                edge_limit = self.depth_edge_threshold_m + depth_m * 0.02
                if width >= 3 and height >= 3 and (
                    len(neighbor_depths) < 2 or sum(
                        abs(neighbor - depth_m) <= edge_limit
                        for neighbor in neighbor_depths
                    ) < 2
                ):
                    continue

                camera_x = (u - cx) * depth_m / fx
                camera_y = (v - cy) * depth_m / fy
                camera_z = depth_m
                world_x = (
                    rotation[0][0] * camera_x
                    + rotation[0][1] * camera_y
                    + rotation[0][2] * camera_z
                    + tx
                )
                world_y = (
                    rotation[1][0] * camera_x
                    + rotation[1][1] * camera_y
                    + rotation[1][2] * camera_z
                    + ty
                )
                world_z = (
                    rotation[2][0] * camera_x
                    + rotation[2][1] * camera_y
                    + rotation[2][2] * camera_z
                    + tz
                )
                if not self.minimum_z_m <= world_z <= self.maximum_z_m:
                    continue
                if self._point_in_robot_footprint(
                    world_x,
                    world_y,
                    robot_pose,
                    self.robot_half_length_m,
                    self.robot_half_width_m,
                ):
                    continue
                key = (
                    math.floor(world_x / voxel_size),
                    math.floor(world_y / voxel_size),
                    math.floor(world_z / voxel_size),
                )
                if key in seen:
                    continue
                seen.add(key)
                accepted += 1

                map_relation = self._map_relation(world_x, world_y, map_context)
                match_distance = max(0.14, voxel_size * 1.6)
                for cell_x in range(key[0] - 2, key[0] + 3):
                    for cell_y in range(key[1] - 2, key[1] + 3):
                        if any(
                            math.hypot(world_x - point_x, world_y - point_y)
                            <= match_distance
                            for point_x, point_y in lidar_cells.get((cell_x, cell_y), ())
                        ):
                            map_relation = 2
                            break
                    if map_relation == 2:
                        break

                color = (120, 190, 220)
                if rgb_data is not None:
                    offset = pixel_index * 3
                    first, green, third = rgb_data[offset : offset + 3]
                    color = (
                        (third, green, first)
                        if rgb_encoding == "bgr8"
                        else (first, green, third)
                    )
                current = self.voxels.get(key)
                if current is None:
                    self.voxels[key] = {
                        "hits": 1,
                        "last_seen": stamp,
                        "color": color,
                        "map_relation": map_relation,
                    }
                else:
                    hits = min(255, current["hits"] + 1)
                    old_color = current["color"]
                    current.update({
                        "hits": hits,
                        "last_seen": stamp,
                        "color": tuple(
                            round(old_color[index] * 0.8 + color[index] * 0.2)
                            for index in range(3)
                        ),
                        "map_relation": map_relation,
                    })

        self.frames_integrated += 1
        self.last_integrated_at = stamp
        self._prune(stamp)
        return accepted

    def prune(self, now=None):
        with self.lock:
            self._prune(now)

    def _prune(self, now=None):
        stamp = time.monotonic() if now is None else float(now)
        deadline = stamp - self.stale_after_s
        for key in [
            key for key, value in self.voxels.items() if value["last_seen"] < deadline
        ]:
            del self.voxels[key]
        if len(self.voxels) <= self.maximum_voxels:
            return
        ranked = sorted(
            self.voxels.items(),
            key=lambda item: (item[1]["hits"], item[1]["last_seen"]),
        )
        for key, _value in ranked[: len(self.voxels) - self.maximum_voxels]:
            del self.voxels[key]

    def snapshot(self, now=None, include_points=True):
        with self.lock:
            return self._snapshot(now, include_points)

    def _snapshot(self, now=None, include_points=True):
        stamp = time.monotonic() if now is None else float(now)
        self._prune(stamp)
        points = []
        columns = []
        relation_counts = {"unknown": 0, "lidar_free": 0, "lidar_occupied": 0}
        if include_points:
            visible = [
                (key, value)
                for key, value in self.voxels.items()
                if value["hits"] >= self.minimum_observations
            ]
            visible_count = min(len(visible), self.maximum_output_voxels)
            visible.sort(key=lambda item: (-item[1]["hits"], -item[1]["last_seen"]))
            visible = visible[: self.maximum_output_voxels]
            size = self.voxel_size_m
            points = [
                [
                    round((key[0] + 0.5) * size, 3),
                    round((key[1] + 0.5) * size, 3),
                    round((key[2] + 0.5) * size, 3),
                    *value["color"],
                    value["hits"],
                    value.get("map_relation", 0),
                ]
                for key, value in visible
            ]
            column_values = {}
            for key, value in visible:
                column_key = key[:2]
                current = column_values.get(column_key)
                top = (key[2] + 1) * size
                if current is None:
                    column_values[column_key] = {
                        "top": top,
                        "hits": value["hits"],
                        "relation": value.get("map_relation", 0),
                    }
                else:
                    current["top"] = max(current["top"], top)
                    current["hits"] = max(current["hits"], value["hits"])
                    current["relation"] = max(
                        current["relation"], value.get("map_relation", 0)
                    )
            relation_names = {0: "unknown", 1: "lidar_free", 2: "lidar_occupied"}
            for (key_x, key_y), value in sorted(
                column_values.items(), key=lambda item: -item[1]["hits"]
            )[: self.maximum_output_voxels]:
                relation = value["relation"]
                relation_counts[relation_names[relation]] += 1
                columns.append([
                    round((key_x + 0.5) * size, 3),
                    round((key_y + 0.5) * size, 3),
                    round(max(size, value["top"]), 3),
                    value["hits"],
                    relation,
                ])
        else:
            visible_count = min(
                sum(
                    1
                    for value in self.voxels.values()
                    if value["hits"] >= self.minimum_observations
                ),
                self.maximum_output_voxels,
            )
        age_ms = (
            None
            if self.last_integrated_at is None
            else round(max(0.0, stamp - self.last_integrated_at) * 1000)
        )
        result = {
            "online": age_ms is not None and age_ms < 5000,
            "age_ms": age_ms,
            "voxel_size_m": self.voxel_size_m,
            "stored_voxels": len(self.voxels),
            "visible_voxels": visible_count,
            "visible_columns": len(columns),
            "relation_counts": relation_counts,
            "frames_integrated": self.frames_integrated,
            "frames_rejected": self.frames_rejected,
            "last_cleared_at": self.last_cleared_at,
        }
        if include_points:
            result["points"] = points
            result["columns"] = columns
        return result
