#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Iterable, Optional, Sequence, Tuple

import numpy as np


def _as_float32_vector3(values: Sequence[float]) -> np.ndarray:
    array = np.asarray(values, dtype=np.float32).reshape(3)
    return array


def extract_box_arrays_from_dicts(obstacles: Iterable[dict]) -> Tuple[np.ndarray, np.ndarray]:
    obstacle_list = list(obstacles)
    if not obstacle_list:
        empty = np.zeros((0, 3), dtype=np.float32)
        return empty, empty

    centers = np.asarray([obs["center"] for obs in obstacle_list], dtype=np.float32).reshape(-1, 3)
    sizes = np.asarray([obs["size"] for obs in obstacle_list], dtype=np.float32).reshape(-1, 3)
    return centers, sizes


def extract_box_arrays_from_env_info(env_info) -> Tuple[np.ndarray, np.ndarray]:
    if not getattr(env_info, "obstacles", None):
        empty = np.zeros((0, 3), dtype=np.float32)
        return empty, empty

    centers = np.asarray(
        [[box.center.x, box.center.y, box.center.z] for box in env_info.obstacles],
        dtype=np.float32,
    ).reshape(-1, 3)
    sizes = np.asarray(
        [[box.size.x, box.size.y, box.size.z] for box in env_info.obstacles],
        dtype=np.float32,
    ).reshape(-1, 3)
    return centers, sizes


def infer_bounds_from_boxes(
    centers: np.ndarray,
    sizes: np.ndarray,
    table_z: Optional[float] = None,
    xy_margin: float = 0.03,
    z_min_offset: float = -0.02,
    z_max_height: float = 0.60,
) -> Tuple[np.ndarray, np.ndarray]:
    centers = np.asarray(centers, dtype=np.float32).reshape(-1, 3)
    sizes = np.asarray(sizes, dtype=np.float32).reshape(-1, 3)
    if centers.size == 0:
        raise ValueError("Cannot infer workspace bounds without any obstacle bounding boxes.")

    half_sizes = 0.5 * sizes
    min_bounds = centers - half_sizes
    max_bounds = centers + half_sizes
    mins = min_bounds.min(axis=0)
    maxs = max_bounds.max(axis=0)

    z_reference = float(table_z) if table_z is not None and np.isfinite(table_z) else None
    if z_reference is None:
        z_reference = float(mins[2])
        z_upper = float(maxs[2] + max(z_max_height * 0.25, 0.2))
    else:
        z_upper = float(z_reference + z_max_height)

    lower = np.asarray(
        [
            float(mins[0] - xy_margin),
            float(mins[1] - xy_margin),
            float(z_reference + z_min_offset),
        ],
        dtype=np.float32,
    )
    upper = np.asarray(
        [
            float(maxs[0] + xy_margin),
            float(maxs[1] + xy_margin),
            z_upper,
        ],
        dtype=np.float32,
    )
    return lower, upper


def compute_workspace_bounds_from_segmentation(
    segmentation: dict,
    xy_margin: float,
    z_min_offset: float,
    z_max_height: float,
) -> Tuple[float, float, float, float, float, float]:
    table = segmentation.get("table_plane")
    obstacles = segmentation.get("obstacles", [])

    if table:
        return (
            float(table["min_bound"][0] - xy_margin),
            float(table["max_bound"][0] + xy_margin),
            float(table["min_bound"][1] - xy_margin),
            float(table["max_bound"][1] + xy_margin),
            float(table["top_z"] + z_min_offset),
            float(table["top_z"] + z_max_height),
        )

    centers, sizes = extract_box_arrays_from_dicts(obstacles)
    lower, upper = infer_bounds_from_boxes(
        centers=centers,
        sizes=sizes,
        table_z=None,
        xy_margin=xy_margin,
        z_min_offset=z_min_offset,
        z_max_height=z_max_height,
    )
    return float(lower[0]), float(upper[0]), float(lower[1]), float(upper[1]), float(lower[2]), float(upper[2])


def compute_workspace_bounds_from_env_info(
    env_info,
    xy_margin: float,
    z_min_offset: float,
    z_max_height: float,
) -> Tuple[np.ndarray, np.ndarray]:
    centers, sizes = extract_box_arrays_from_env_info(env_info)
    table_z = getattr(env_info, "table_z", None)
    if table_z is not None and not np.isfinite(table_z):
        table_z = None
    return infer_bounds_from_boxes(
        centers=centers,
        sizes=sizes,
        table_z=table_z,
        xy_margin=xy_margin,
        z_min_offset=z_min_offset,
        z_max_height=z_max_height,
    )


def make_grid_axes(min_bound: Sequence[float], max_bound: Sequence[float], resolution: float):
    if resolution <= 0.0:
        raise ValueError(f"Grid resolution must be positive, got {resolution}")

    lower = _as_float32_vector3(min_bound)
    upper = _as_float32_vector3(max_bound)
    if np.any(upper < lower):
        raise ValueError(f"Invalid bounds: min={lower.tolist()} max={upper.tolist()}")

    xs = np.arange(lower[0], upper[0] + 1e-9, resolution, dtype=np.float32)
    ys = np.arange(lower[1], upper[1] + 1e-9, resolution, dtype=np.float32)
    zs = np.arange(lower[2], upper[2] + 1e-9, resolution, dtype=np.float32)

    if xs.size == 0:
        xs = np.asarray([lower[0]], dtype=np.float32)
    if ys.size == 0:
        ys = np.asarray([lower[1]], dtype=np.float32)
    if zs.size == 0:
        zs = np.asarray([lower[2]], dtype=np.float32)
    return xs, ys, zs


def compute_aabb_signed_distance(points_xyz: np.ndarray, centers: np.ndarray, half_sizes: np.ndarray) -> np.ndarray:
    points_xyz = np.asarray(points_xyz, dtype=np.float32).reshape(-1, 3)
    centers = np.asarray(centers, dtype=np.float32).reshape(-1, 3)
    half_sizes = np.asarray(half_sizes, dtype=np.float32).reshape(-1, 3)

    if centers.size == 0:
        return np.full(points_xyz.shape[0], np.inf, dtype=np.float32)

    q = np.abs(points_xyz[:, None, :] - centers[None, :, :]) - half_sizes[None, :, :]
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=2)
    inside = np.minimum(np.max(q, axis=2), 0.0)
    signed = outside + inside
    return np.min(signed, axis=1).astype(np.float32)


def build_clearance_heatmap(
    centers: np.ndarray,
    sizes: np.ndarray,
    resolution: float,
    min_bound: Sequence[float],
    max_bound: Sequence[float],
    robot_radius: float,
    safety_margin: float,
    clearance_clip: float,
):
    xs, ys, zs = make_grid_axes(min_bound, max_bound, resolution)
    X, Y, Z = np.meshgrid(xs, ys, zs, indexing="xy")
    points_xyz = np.stack([X, Y, Z], axis=-1).reshape(-1, 3).astype(np.float32)

    centers = np.asarray(centers, dtype=np.float32).reshape(-1, 3)
    sizes = np.asarray(sizes, dtype=np.float32).reshape(-1, 3)

    if centers.size == 0:
        safe_value = float(clearance_clip) if clearance_clip > 0.0 else 1.0
        effective_clearance = np.full(points_xyz.shape[0], safe_value, dtype=np.float32)
    else:
        signed_distance = compute_aabb_signed_distance(points_xyz, centers, 0.5 * sizes)
        effective_clearance = signed_distance - float(robot_radius + safety_margin)
        if clearance_clip > 0.0:
            effective_clearance = np.clip(effective_clearance, -clearance_clip, clearance_clip)

    heatmap_zyx = effective_clearance.reshape(len(ys), len(xs), len(zs)).transpose(2, 0, 1).astype(np.float32)
    return xs, ys, zs, heatmap_zyx
