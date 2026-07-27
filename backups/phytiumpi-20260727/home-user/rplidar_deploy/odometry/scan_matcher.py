#!/usr/bin/env python3
"""Dependency-free robust 2D scan matching and odometry core."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Iterable, Sequence

Pose2D = tuple[float, float, float]
Point2D = tuple[float, float]


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    ax, ay, ayaw = a
    bx, by, byaw = b
    cosine, sine = math.cos(ayaw), math.sin(ayaw)
    return (
        ax + cosine * bx - sine * by,
        ay + sine * bx + cosine * by,
        normalize_angle(ayaw + byaw),
    )


def inverse_pose(pose: Pose2D) -> Pose2D:
    x_value, y_value, yaw = pose
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (
        -cosine * x_value - sine * y_value,
        sine * x_value - cosine * y_value,
        -yaw,
    )


def relative_pose(origin: Pose2D, target: Pose2D) -> Pose2D:
    return compose(inverse_pose(origin), target)


def transform_points(points: Iterable[Point2D], pose: Pose2D) -> list[Point2D]:
    x_value, y_value, yaw = pose
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [
        (
            cosine * point_x - sine * point_y + x_value,
            sine * point_x + cosine * point_y + y_value,
        )
        for point_x, point_y in points
    ]


def rigid_transform(source: Sequence[Point2D], target: Sequence[Point2D]) -> Pose2D:
    source_x = sum(point[0] for point in source) / len(source)
    source_y = sum(point[1] for point in source) / len(source)
    target_x = sum(point[0] for point in target) / len(target)
    target_y = sum(point[1] for point in target) / len(target)
    cross = 0.0
    dot = 0.0
    for (point_x, point_y), (match_x, match_y) in zip(source, target):
        point_x -= source_x
        point_y -= source_y
        match_x -= target_x
        match_y -= target_y
        cross += point_x * match_y - point_y * match_x
        dot += point_x * match_x + point_y * match_y
    yaw = math.atan2(cross, dot)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (
        target_x - cosine * source_x + sine * source_y,
        target_y - sine * source_x - cosine * source_y,
        yaw,
    )


def build_grid(points: Sequence[Point2D], cell_size: float) -> dict[tuple[int, int], list[Point2D]]:
    grid: dict[tuple[int, int], list[Point2D]] = {}
    for point in points:
        key = (math.floor(point[0] / cell_size), math.floor(point[1] / cell_size))
        grid.setdefault(key, []).append(point)
    return grid


def nearest_from_grid(
    grid: dict[tuple[int, int], list[Point2D]],
    point: Point2D,
    cell_size: float,
    radius: int,
) -> tuple[Point2D | None, float]:
    cell_x = math.floor(point[0] / cell_size)
    cell_y = math.floor(point[1] / cell_size)
    nearest = None
    nearest_distance_sq = float("inf")
    for offset_x in range(-radius, radius + 1):
        for offset_y in range(-radius, radius + 1):
            for candidate in grid.get((cell_x + offset_x, cell_y + offset_y), ()):
                distance_sq = (
                    (candidate[0] - point[0]) ** 2
                    + (candidate[1] - point[1]) ** 2
                )
                if distance_sq < nearest_distance_sq:
                    nearest = candidate
                    nearest_distance_sq = distance_sq
    return nearest, nearest_distance_sq


def target_normals(points: Sequence[Point2D], max_neighbor_gap: float) -> list[Point2D | None]:
    """Estimate surface normals while rejecting scan discontinuities."""
    normals: list[Point2D | None] = [None] * len(points)
    for index in range(1, len(points) - 1):
        before = points[index - 1]
        point = points[index]
        after = points[index + 1]
        if (
            math.hypot(point[0] - before[0], point[1] - before[1]) > max_neighbor_gap
            or math.hypot(after[0] - point[0], after[1] - point[1]) > max_neighbor_gap
        ):
            continue
        tangent_x = after[0] - before[0]
        tangent_y = after[1] - before[1]
        length = math.hypot(tangent_x, tangent_y)
        if length > 1e-6:
            normals[index] = (-tangent_y / length, tangent_x / length)
    return normals


def build_index_grid(points: Sequence[Point2D], cell_size: float) -> dict[tuple[int, int], list[int]]:
    grid: dict[tuple[int, int], list[int]] = {}
    for index, point in enumerate(points):
        key = (math.floor(point[0] / cell_size), math.floor(point[1] / cell_size))
        grid.setdefault(key, []).append(index)
    return grid


def nearest_index_from_grid(
    grid: dict[tuple[int, int], list[int]],
    points: Sequence[Point2D],
    point: Point2D,
    cell_size: float,
    radius: int,
) -> tuple[int | None, float]:
    cell_x = math.floor(point[0] / cell_size)
    cell_y = math.floor(point[1] / cell_size)
    nearest_index = None
    nearest_distance_sq = float("inf")
    for offset_x in range(-radius, radius + 1):
        for offset_y in range(-radius, radius + 1):
            for candidate_index in grid.get((cell_x + offset_x, cell_y + offset_y), ()):
                candidate = points[candidate_index]
                distance_sq = (
                    (candidate[0] - point[0]) ** 2
                    + (candidate[1] - point[1]) ** 2
                )
                if distance_sq < nearest_distance_sq:
                    nearest_index = candidate_index
                    nearest_distance_sq = distance_sq
    return nearest_index, nearest_distance_sq


def solve_symmetric_3x3(matrix: list[list[float]], vector: list[float]) -> Pose2D | None:
    """Small Gaussian-elimination solver used by point-to-line ICP."""
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-9:
            return None
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        augmented[column] = [value / scale for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                value - factor * pivot_value
                for value, pivot_value in zip(augmented[row], augmented[column])
            ]
    return augmented[0][3], augmented[1][3], augmented[2][3]


def point_to_line_correction(
    matches: Sequence[tuple[float, Point2D, Point2D, Point2D | None]],
) -> Pose2D | None:
    hessian = [[0.0, 0.0, 0.0] for _ in range(3)]
    gradient = [0.0, 0.0, 0.0]
    usable = 0
    for _, point, target, normal in matches:
        if normal is None:
            continue
        normal_x, normal_y = normal
        residual = normal_x * (point[0] - target[0]) + normal_y * (point[1] - target[1])
        jacobian = (normal_x, normal_y, -normal_x * point[1] + normal_y * point[0])
        for row in range(3):
            gradient[row] -= jacobian[row] * residual
            for column in range(3):
                hessian[row][column] += jacobian[row] * jacobian[column]
        usable += 1
    if usable < 12:
        return None
    return solve_symmetric_3x3(hessian, gradient)


@dataclass
class MatcherConfig:
    cell_size: float = 0.08
    max_correspondence: float = 0.20
    min_matches: int = 40
    trim_ratio: float = 0.75
    max_rms: float = 0.08
    max_iterations: int = 6
    translation_deadband: float = 0.002
    rotation_deadband: float = 0.003
    max_linear_speed: float = 0.45
    max_angular_speed: float = 1.50
    frame_translation_margin: float = 0.015
    frame_rotation_margin: float = 0.035
    velocity_filter_alpha: float = 0.35
    max_reference_interval: float = 5.00
    static_scan_rms: float = 0.012
    static_trim_ratio: float = 0.70
    static_zero_rms: float = 0.025
    static_reference_refresh: float = 2.00


@dataclass
class MatchQuality:
    accepted: bool = False
    reason: str = "initializing"
    inliers: int = 0
    rms: float | None = None
    zero_rms: float | None = None
    dt: float = 0.0
    translation: float = 0.0
    rotation: float = 0.0
    raw_scan_rms: float | None = None

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def icp_2d(
    source: Sequence[Point2D],
    target: Sequence[Point2D],
    config: MatcherConfig,
    initial: Pose2D = (0.0, 0.0, 0.0),
) -> tuple[Pose2D, MatchQuality]:
    quality = MatchQuality(reason="insufficient_points")
    if len(source) < config.min_matches or len(target) < config.min_matches:
        return initial, quality

    transform = initial
    max_distance_sq = config.max_correspondence**2
    grid = build_grid(target, config.cell_size)
    index_grid = build_index_grid(target, config.cell_size)
    normals = target_normals(target, config.max_correspondence)
    search_radius = math.ceil(config.max_correspondence / config.cell_size)
    zero_distances = []
    for point in source:
        _, distance_sq = nearest_from_grid(grid, point, config.cell_size, search_radius)
        if distance_sq <= max_distance_sq:
            zero_distances.append(distance_sq)
    if len(zero_distances) >= config.min_matches:
        zero_distances.sort()
        keep = max(config.min_matches, int(len(zero_distances) * config.trim_ratio))
        zero_distances = zero_distances[:keep]
        quality.zero_rms = math.sqrt(sum(zero_distances) / len(zero_distances))

    for _ in range(config.max_iterations):
        transformed = transform_points(source, transform)
        matches = []
        for point in transformed:
            nearest_index, distance_sq = nearest_index_from_grid(
                index_grid, target, point, config.cell_size, search_radius
            )
            if nearest_index is not None and distance_sq <= max_distance_sq:
                matches.append(
                    (distance_sq, point, target[nearest_index], normals[nearest_index])
                )
        if len(matches) < config.min_matches:
            quality.reason = "insufficient_matches"
            return transform, quality

        matches.sort(key=lambda item: item[0])
        keep = max(config.min_matches, int(len(matches) * config.trim_ratio))
        matches = matches[:keep]
        quality.inliers = len(matches)
        quality.rms = math.sqrt(sum(item[0] for item in matches) / len(matches))
        if quality.rms > config.max_rms:
            quality.reason = "rms_too_high"
            return transform, quality

        correction = point_to_line_correction(matches)
        if correction is None:
            correction = rigid_transform(
                [item[1] for item in matches], [item[2] for item in matches]
            )
        transform = compose(correction, transform)
        if math.hypot(correction[0], correction[1]) < 1e-5 and abs(correction[2]) < 1e-5:
            break

    quality.accepted = True
    quality.reason = "ok"
    quality.translation = math.hypot(transform[0], transform[1])
    quality.rotation = abs(transform[2])
    return transform, quality


class LaserOdometryCore:
    """Integrate accepted laser-frame motion while exposing match quality."""

    def __init__(
        self,
        config: MatcherConfig | None = None,
        base_to_laser: Pose2D = (0.0, 0.0, 0.0),
    ) -> None:
        self.config = config or MatcherConfig()
        self.base_to_laser = base_to_laser
        self.reset()

    def reset(self) -> None:
        self.previous_points: list[Point2D] | None = None
        self.previous_stamp: float | None = None
        self.base_pose: Pose2D = (0.0, 0.0, 0.0)
        self.laser_pose: Pose2D = self.base_to_laser
        self.velocity: Pose2D = (0.0, 0.0, 0.0)
        self.last_sensor_delta: Pose2D = (0.0, 0.0, 0.0)
        self.accepted_count = 0
        self.rejected_count = 0
        self.consecutive_rejections = 0
        self.quality = MatchQuality()

    def update(
        self,
        points: Sequence[Point2D],
        stamp: float,
        raw_scan_rms: float | None = None,
        imu_yaw_delta: float | None = None,
        imu_yaw_weight: float = 0.0,
    ) -> MatchQuality:
        points = list(points)
        if self.previous_points is None or self.previous_stamp is None:
            self.previous_points = points
            self.previous_stamp = stamp
            self.quality = MatchQuality(reason="initialized")
            return self.quality

        dt = stamp - self.previous_stamp
        if dt <= 0.0 or dt > self.config.max_reference_interval:
            self.previous_points = points
            self.previous_stamp = stamp
            self.last_sensor_delta = (0.0, 0.0, 0.0)
            self.velocity = (0.0, 0.0, 0.0)
            self.quality = MatchQuality(reason="reference_reset", dt=dt)
            return self.quality

        if raw_scan_rms is not None and raw_scan_rms < self.config.static_scan_rms:
            if dt >= self.config.static_reference_refresh:
                self.previous_points = points
                self.previous_stamp = stamp
            self.last_sensor_delta = (0.0, 0.0, 0.0)
            self.velocity = (0.0, 0.0, 0.0)
            self.accepted_count += 1
            self.consecutive_rejections = 0
            self.quality = MatchQuality(
                accepted=True,
                reason="static",
                dt=dt,
                raw_scan_rms=raw_scan_rms,
            )
            return self.quality

        old_base_pose = self.base_pose
        sensor_delta, quality = icp_2d(
            self.previous_points,
            points,
            self.config,
            initial=self.last_sensor_delta,
        )
        quality.dt = dt
        quality.raw_scan_rms = raw_scan_rms
        if (
            quality.zero_rms is not None
            and quality.zero_rms < self.config.static_zero_rms
        ):
            if dt >= self.config.static_reference_refresh:
                self.previous_points = points
                self.previous_stamp = stamp
            self.last_sensor_delta = (0.0, 0.0, 0.0)
            self.velocity = (0.0, 0.0, 0.0)
            self.accepted_count += 1
            self.consecutive_rejections = 0
            quality.accepted = True
            quality.reason = "static_geometry"
            quality.translation = 0.0
            quality.rotation = 0.0
            self.quality = quality
            return quality
        max_translation = self.config.frame_translation_margin + self.config.max_linear_speed * dt
        max_rotation = self.config.frame_rotation_margin + self.config.max_angular_speed * dt
        if quality.accepted and quality.translation > max_translation:
            quality.accepted = False
            quality.reason = "translation_jump"
        if quality.accepted and quality.rotation > max_rotation:
            quality.accepted = False
            quality.reason = "rotation_jump"

        if quality.accepted:
            robot_delta_laser = inverse_pose(sensor_delta)
            if imu_yaw_delta is not None and math.isfinite(imu_yaw_delta):
                weight = min(1.0, max(0.0, imu_yaw_weight))
                robot_delta_laser = (
                    robot_delta_laser[0],
                    robot_delta_laser[1],
                    normalize_angle(
                        (1.0 - weight) * robot_delta_laser[2]
                        + weight * imu_yaw_delta
                    ),
                )
            if math.hypot(robot_delta_laser[0], robot_delta_laser[1]) < self.config.translation_deadband:
                robot_delta_laser = (0.0, 0.0, robot_delta_laser[2])
            if abs(robot_delta_laser[2]) < self.config.rotation_deadband:
                robot_delta_laser = (robot_delta_laser[0], robot_delta_laser[1], 0.0)
            self.laser_pose = compose(self.laser_pose, robot_delta_laser)
            self.base_pose = compose(self.laser_pose, inverse_pose(self.base_to_laser))
            base_delta = relative_pose(old_base_pose, self.base_pose)
            measured_velocity = (
                base_delta[0] / dt,
                base_delta[1] / dt,
                base_delta[2] / dt,
            )
            alpha = self.config.velocity_filter_alpha
            self.velocity = tuple(
                alpha * measured + (1.0 - alpha) * previous
                for measured, previous in zip(measured_velocity, self.velocity)
            )
            self.last_sensor_delta = inverse_pose(robot_delta_laser)
            self.accepted_count += 1
            self.consecutive_rejections = 0
        else:
            self.velocity = tuple(value * 0.5 for value in self.velocity)
            self.last_sensor_delta = (0.0, 0.0, 0.0)
            self.rejected_count += 1
            self.consecutive_rejections += 1
            if self.consecutive_rejections >= 3:
                self.previous_points = points
                self.previous_stamp = stamp
                self.consecutive_rejections = 0

        if quality.accepted:
            self.previous_points = points
            self.previous_stamp = stamp
        self.quality = quality
        return quality
