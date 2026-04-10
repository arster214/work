#!/usr/bin/env python3
import argparse
import json
import math
import os
import subprocess
import sys
import time
import shutil
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial import cKDTree

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# Heatmap / planner communication messages
try:
    from clearance_heatmap_msgs.msg import BoundingBox, EnvironmentInfo
except ImportError as e:
    rospy.logerr(f"Failed to import clearance_heatmap_msgs. Make sure it is compiled and sourced. {e}")



def str2bool(value):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {'true', '1', 'yes', 'y', 'on'}:
        return True
    if text in {'false', '0', 'no', 'n', 'off'}:
        return False
    raise argparse.ArgumentTypeError(f'Invalid boolean value: {value}')


@dataclass
class PlaneResult:
    normal: List[float]
    d: float
    center: List[float]
    extents_xy: List[float]
    thickness: float
    top_z: float
    min_bound: List[float]
    max_bound: List[float]
    voxel_count: int


@dataclass
class ObstacleResult:
    id: int
    center: List[float]
    size: List[float]
    min_bound: List[float]
    max_bound: List[float]
    voxel_count: int
    support_type: str
    support_id: str
    support_top_z: float
    stack_level: int


@dataclass
class ProcessingResult:
    resolution_hint: float
    table_plane: Optional[PlaneResult]
    obstacles: List[ObstacleResult]
    total_occupied_voxels: int
    tabletop_candidate_voxels: int
    non_table_voxels: int
    kept_voxel_indices: List[int]


def load_xyzs(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    data = np.loadtxt(str(path), dtype=float)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 4:
        raise ValueError(f"Expected 4 columns x y z size in {path}, got shape {data.shape}")
    return data[:, :3], data[:, 3]


def lightly_remove_flyers(
    points: np.ndarray,
    voxel_sizes: np.ndarray,
    min_neighbors: int = 1,
    radius_scale: float = 1.05,
) -> Tuple[np.ndarray, np.ndarray]:
    if len(points) < 3:
        return points, voxel_sizes

    resolution = float(np.median(voxel_sizes)) if len(voxel_sizes) else 0.01
    tree = cKDTree(points)
    radius = max(resolution * radius_scale, 1e-6)
    keep_mask = np.zeros(len(points), dtype=bool)

    for i, point in enumerate(points):
        neighbors = tree.query_ball_point(point, r=radius)
        neighbor_count = max(len(neighbors) - 1, 0)
        if neighbor_count >= min_neighbors:
            keep_mask[i] = True

    if not np.any(keep_mask):
        return points, voxel_sizes

    return points[keep_mask], voxel_sizes[keep_mask]


def fit_horizontal_plane_ransac(points: np.ndarray, distance_threshold: float, iterations: int, rng: np.random.Generator):
    if len(points) < 3:
        return None, np.array([], dtype=int)

    best_inliers = np.array([], dtype=int)
    best_model = None

    for _ in range(iterations):
        ids = rng.choice(len(points), size=3, replace=False)
        p1, p2, p3 = points[ids]
        normal = np.cross(p2 - p1, p3 - p1)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        if abs(normal[2]) < 0.9:
            continue
        if normal[2] < 0:
            normal = -normal
        d = -float(np.dot(normal, p1))
        distances = np.abs(points @ normal + d)
        inliers = np.flatnonzero(distances <= distance_threshold)
        if inliers.size > best_inliers.size:
            best_inliers = inliers
            best_model = (normal, d)

    if best_model is None or best_inliers.size < 3:
        return None, np.array([], dtype=int)

    inlier_points = points[best_inliers]
    centroid = inlier_points.mean(axis=0)
    centered = inlier_points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    if abs(normal[2]) < 0.9:
        normal = np.array([0.0, 0.0, 1.0])
    if normal[2] < 0:
        normal = -normal
    normal = normal / np.linalg.norm(normal)
    d = -float(np.dot(normal, centroid))
    distances = np.abs(points @ normal + d)
    refined_inliers = np.flatnonzero(distances <= distance_threshold)
    return (normal, d), refined_inliers


def largest_connected_subset_xy(points: np.ndarray, radius: float) -> np.ndarray:
    if len(points) == 0:
        return np.array([], dtype=int)
    if len(points) == 1:
        return np.array([0], dtype=int)

    xy = points[:, :2]
    tree = cKDTree(xy)
    visited = np.zeros(len(points), dtype=bool)
    best = []

    for start in range(len(points)):
        if visited[start]:
            continue
        stack = [start]
        visited[start] = True
        comp = []
        while stack:
            idx = stack.pop()
            comp.append(idx)
            for nb in tree.query_ball_point(xy[idx], r=radius):
                if not visited[nb]:
                    visited[nb] = True
                    stack.append(nb)
        if len(comp) > len(best):
            best = comp

    return np.array(best, dtype=int)


def detect_table_plane(points: np.ndarray, voxel_sizes: np.ndarray, min_z: float, max_z: float, distance_threshold: float):
    mask = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
    candidates = points[mask]
    candidate_sizes = voxel_sizes[mask]
    if len(candidates) < 30:
        return None, np.array([], dtype=int), 0

    resolution_hint = float(np.median(candidate_sizes)) if len(candidate_sizes) else 0.05
    model, inliers_local = fit_horizontal_plane_ransac(candidates, distance_threshold, 500, np.random.default_rng(42))
    if model is None or len(inliers_local) == 0:
        return None, np.array([], dtype=int), len(candidates)

    inlier_points = candidates[inliers_local]
    keep_local = largest_connected_subset_xy(inlier_points, radius=max(resolution_hint * 1.8, distance_threshold * 2.0))
    if len(keep_local) == 0:
        return None, np.array([], dtype=int), len(candidates)

    inliers_local = inliers_local[keep_local]
    global_indices = np.flatnonzero(mask)[inliers_local]
    plane_points = points[global_indices]
    normal, d = model
    min_bound = plane_points.min(axis=0)
    max_bound = plane_points.max(axis=0)
    thickness = float(max(max_bound[2] - min_bound[2], resolution_hint))
    top_z = float(max_bound[2] + resolution_hint * 0.5)
    center = np.array([
        float((min_bound[0] + max_bound[0]) * 0.5),
        float((min_bound[1] + max_bound[1]) * 0.5),
        float(top_z - thickness * 0.5),
    ])
    extents_xy = [float(max_bound[0] - min_bound[0]), float(max_bound[1] - min_bound[1])]
    plane = PlaneResult(
        normal=[float(x) for x in normal],
        d=float(d),
        center=[float(x) for x in center],
        extents_xy=extents_xy,
        thickness=thickness,
        top_z=top_z,
        min_bound=[float(x) for x in min_bound],
        max_bound=[float(x) for x in max_bound],
        voxel_count=int(len(global_indices)),
    )
    return plane, global_indices, len(candidates)

def split_cluster_by_z_layers(cluster: np.ndarray, resolution_hint: float) -> List[np.ndarray]:
    """
    根据Z轴横截面面积的突变来切割物体。
    例如：从 30x30 变为 10x10，面积缩小为原来的 1/9，这里就是切割点。
    """
    if len(cluster) < 20:
        return [cluster]

    z_values = cluster[:, 2]
    z_min, z_max = np.min(z_values), np.max(z_values)
    
    # 将高度划分为多个层 (每层 1-2 个体素高度)
    layer_height = resolution_hint * 1.5
    layers = []
    z_coords = np.arange(z_min, z_max + layer_height, layer_height)
    
    layer_areas = []
    for i in range(len(z_coords) - 1):
        mask = (z_values >= z_coords[i]) & (z_values < z_coords[i+1])
        points_in_layer = cluster[mask]
        if len(points_in_layer) > 0:
            # 计算这一层在 XY 平面的大致覆盖面积
            min_xy = points_in_layer.min(axis=0)[:2]
            max_xy = points_in_layer.max(axis=0)[:2]
            area = (max_xy[0] - min_xy[0]) * (max_xy[1] - min_xy[1])
            layer_areas.append((i, area, points_in_layer))

    if len(layer_areas) < 2:
        return [cluster]

    # 检测面积突变点
    split_indices = []
    for i in range(1, len(layer_areas)):
        prev_area = layer_areas[i-1][1]
        curr_area = layer_areas[i][1]
        
        # 如果面积变化超过 2 倍（比如从 30x30 变成 10x10），记录切割位置
        if prev_area > 0 and (prev_area / curr_area > 2.0 or curr_area / prev_area > 2.0):
            split_indices.append(layer_areas[i][0])

    if not split_indices:
        return [cluster]

    # 根据第一个明显的面积跳变点进行分割
    split_z = z_coords[split_indices[0]]
    lower = cluster[cluster[:, 2] < split_z]
    upper = cluster[cluster[:, 2] >= split_z]
    
    # 递归分割（处理多层堆叠）
    result = []
    if len(lower) > 10: result.append(lower)
    if len(upper) > 10: 
        # 对上层继续尝试分割（可能存在多个10cm方块并列）
        result.extend(split_by_xy_islands(upper, resolution_hint))
    
    return result

def split_by_xy_islands(points: np.ndarray, radius: float) -> List[np.ndarray]:
    """
    在同一个高度层内，如果物体分成了几个不相连的岛屿，则拆分成独立物体。
    """
    if len(points) < 10: return [points]
    
    from scipy.spatial import cKDTree
    tree = cKDTree(points[:, :2]) # 只看XY
    visited = np.zeros(len(points), dtype=bool)
    sub_clusters = []
    
    # 增大搜索半径，允许体素间有微小空隙 (1.5倍分辨率)
    search_r = radius * 1.5

    for i in range(len(points)):
        if visited[i]: continue
        
        # 找 2D 连通域
        idx_in_comp = []
        queue = [i]
        visited[i] = True
        
        while queue:
            curr = queue.pop(0)
            idx_in_comp.append(curr)
            neighbors = tree.query_ball_point(points[curr, :2], r=search_r)
            for nb in neighbors:
                if not visited[nb]:
                    visited[nb] = True
                    queue.append(nb)
        
        if len(idx_in_comp) > 5:
            sub_clusters.append(points[idx_in_comp])
            
    return sub_clusters


def xy_cluster_split(points: np.ndarray, resolution: float) -> List[np.ndarray]:
    """使用较小的半径在 XY 平面上拆分并列的物体。"""
    if len(points) < 10:
        return [points]

    xy_tree = cKDTree(points[:, :2])
    r = resolution * 1.5
    visited = np.zeros(len(points), dtype=bool)
    results = []

    for i in range(len(points)):
        if visited[i]:
            continue
        q = [i]
        visited[i] = True
        idx_list = []
        while q:
            curr = q.pop(0)
            idx_list.append(curr)
            nbs = xy_tree.query_ball_point(points[curr, :2], r=r)
            for nb in nbs:
                if not visited[nb]:
                    visited[nb] = True
                    q.append(nb)
        if len(idx_list) > 5:
            results.append(points[idx_list])

    return results if results else [points]


def split_hollow_structures(points: np.ndarray, resolution: float) -> List[np.ndarray]:
    """专门处理堆叠的空心方块。"""
    z_min = points[:, 2].min()
    z_max = points[:, 2].max()
    z_height = z_max - z_min

    if z_height > 0.15:
        z_step = 0.02
        z_levels = np.arange(z_min, z_max, z_step)
        densities = []
        for zl in z_levels:
            count = np.sum((points[:, 2] >= zl) & (points[:, 2] < zl + z_step))
            densities.append(count)

        for j in range(1, len(densities) - 1):
            if densities[j] < densities[j - 1] * 0.3:
                cut_z = z_levels[j]
                lower = points[points[:, 2] < cut_z]
                upper = points[points[:, 2] >= cut_z]

                if len(lower) > 10 and len(upper) > 10:
                    split_results = [lower]
                    split_results.extend(xy_cluster_split(upper, resolution))
                    return split_results

    return xy_cluster_split(points, resolution)


def ckdtree_clustering(points: np.ndarray, resolution: float, radius_scale: float = 2.5) -> List[np.ndarray]:
    if len(points) < 5:
        return []

    tree = cKDTree(points)
    radius = max(resolution * radius_scale, 1e-6)
    visited = np.zeros(len(points), dtype=bool)
    clusters: List[np.ndarray] = []

    for i in range(len(points)):
        if visited[i]:
            continue
        queue = [i]
        visited[i] = True
        indices = []

        while queue:
            curr = queue.pop(0)
            indices.append(curr)
            neighbors = tree.query_ball_point(points[curr], r=radius)
            for nb in neighbors:
                if not visited[nb]:
                    visited[nb] = True
                    queue.append(nb)

        if len(indices) >= 5:
            clusters.append(points[np.array(indices, dtype=int)])

    return clusters


def dynamic_footprint_scan(cluster: np.ndarray, resolution: float) -> List[np.ndarray]:
    if len(cluster) < 10:
        return [cluster]

    z_min = float(np.min(cluster[:, 2]))
    z_max = float(np.max(cluster[:, 2]))
    z_levels = np.arange(z_min, z_max + resolution, resolution)
    if len(z_levels) < 2:
        return [cluster]

    layers = []
    for z0 in z_levels:
        mask = (cluster[:, 2] >= z0) & (cluster[:, 2] < z0 + resolution)
        pts = cluster[mask]
        if len(pts) == 0:
            continue
        min_xy = pts[:, :2].min(axis=0)
        max_xy = pts[:, :2].max(axis=0)
        center_xy = (min_xy + max_xy) * 0.5
        size_xy = max_xy - min_xy
        layers.append({
            'z': z0,
            'points': pts,
            'center_xy': center_xy,
            'size_xy': size_xy,
        })

    if len(layers) < 2:
        return [cluster]

    split_zs: List[float] = []
    for i in range(1, len(layers)):
        prev_layer = layers[i - 1]
        curr_layer = layers[i]
        center_shift = np.linalg.norm(curr_layer['center_xy'] - prev_layer['center_xy'])
        size_delta = np.max(np.abs(curr_layer['size_xy'] - prev_layer['size_xy']))
        prev_span = np.max(prev_layer['size_xy'])
        curr_span = np.max(curr_layer['size_xy'])

        if center_shift > resolution * 2.0 or size_delta > 0.08:
            split_zs.append(float(curr_layer['z']))
            continue

        if (0.20 <= prev_span <= 0.35 and 0.06 <= curr_span <= 0.15) or (0.20 <= curr_span <= 0.35 and 0.06 <= prev_span <= 0.15):
            split_zs.append(float(curr_layer['z']))

    if split_zs:
        sub_clusters = []
        boundaries = [z_min] + split_zs + [z_max + resolution]
        for start, end in zip(boundaries[:-1], boundaries[1:]):
            part = cluster[(cluster[:, 2] >= start) & (cluster[:, 2] < end)]
            if len(part) >= 5:
                sub_clusters.append(part)
        if sub_clusters:
            return sub_clusters

    min_xy = cluster[:, :2].min(axis=0)
    max_xy = cluster[:, :2].max(axis=0)
    xy_span = max_xy - min_xy
    z_span = z_max - z_min
    max_xy_span = float(np.max(xy_span))
    if 0.06 <= max_xy_span <= 0.15 and z_span > 0.15:
        segment_height = 0.10
        segments = []
        curr_z = z_min
        while curr_z < z_max + resolution:
            part = cluster[(cluster[:, 2] >= curr_z) & (cluster[:, 2] < curr_z + segment_height)]
            if len(part) >= 5:
                segments.append(part)
            curr_z += segment_height
        if segments:
            return segments

    return [cluster]


def refine_with_priors(cluster: np.ndarray, camera_quadrant: str = 'neg_xy', target_size: Optional[float] = None) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
    min_bound = cluster.min(axis=0)
    max_bound = cluster.max(axis=0)
    observed_xy_span = float(np.max(max_bound[:2] - min_bound[:2]))

    z_max = float(max_bound[2])

    standard_size = target_size
    if standard_size is None:
        if 0.20 <= observed_xy_span <= 0.35:
            standard_size = 0.30
        elif 0.06 <= observed_xy_span <= 0.15:
            standard_size = 0.10
        else:
            return None

    size = np.array([standard_size, standard_size, standard_size], dtype=float)
    if camera_quadrant == 'neg_xy':
        center_x = float(min_bound[0] + standard_size * 0.5)
        center_y = float(min_bound[1] + standard_size * 0.5)
    else:
        center_x = float((min_bound[0] + max_bound[0]) * 0.5)
        center_y = float((min_bound[1] + max_bound[1]) * 0.5)

    center = np.array([center_x, center_y, z_max - standard_size * 0.5], dtype=float)
    refined_min = center - size * 0.5
    refined_max = center + size * 0.5
    return center, size, refined_min, refined_max


def filter_points_in_box(points: np.ndarray, min_bound: np.ndarray, max_bound: np.ndarray) -> np.ndarray:
    return np.all((points >= min_bound) & (points <= max_bound), axis=1)


def decompose_by_height_profile(points: np.ndarray, resolution: float, min_voxels: int = 10) -> List[np.ndarray]:
    """
    通过扫描高度轮廓线寻找“断崖”。
    修复版：过滤空隙产生的假断崖，精准捕捉小方块叠加。
    """
    if len(points) < min_voxels:
        return []

    min_b = points.min(axis=0)
    max_b = points.max(axis=0)
    size = max_b - min_b

    # 放宽最小尺寸限制：10cm的小方块，对角线约14cm，这里设为 0.12 保证能切
    if size[0] < 0.12 and size[1] < 0.12:
        return [points]

    # 优先扫描较长的那条边
    axes_to_try = [0, 1] if size[0] > size[1] else [1, 0]

    for axis in axes_to_try:
        if size[axis] < 0.12:
            continue

        # 适当放大扫描带，吸纳微小的噪点缝隙
        bin_size = max(resolution * 2.0, 0.03)
        bins = np.arange(min_b[axis], max_b[axis] + bin_size, bin_size)
        
        profile = []
        valid_bins_edges = []

        # 1. 提取有效的高度轮廓（直接跳过没有点云的空气带，防止假断崖）
        for i in range(len(bins) - 1):
            mask = (points[:, axis] >= bins[i]) & (points[:, axis] < bins[i + 1])
            if np.any(mask):
                profile.append(np.max(points[mask, 2]))
                # 记录这个有效区域的右边界，作为潜在的“切割线”
                valid_bins_edges.append(bins[i + 1])

        if len(profile) < 2:
            continue

        profile = np.array(profile)
        # 2. 计算相邻有效实体的高度差
        diffs = np.diff(profile)

        # 3. 核心修复：将断崖阈值降到 0.06m (6cm)。
        cliff_indices = np.where(np.abs(diffs) > 0.06)[0]

        if len(cliff_indices) > 0:
            # 找到最猛烈的那个断崖
            best_idx = cliff_indices[np.argmax(np.abs(diffs[cliff_indices]))]
            
            # 在断崖处精准下刀
            split_val = valid_bins_edges[best_idx]

            # 沿着这条线把点云一分为二
            mask_left = points[:, axis] < split_val
            points_left = points[mask_left]
            points_right = points[~mask_left]

            # 递归切分两半
            if len(points_left) >= min_voxels and len(points_right) >= min_voxels:
                res: List[np.ndarray] = []
                res.extend(decompose_by_height_profile(points_left, resolution, min_voxels))
                res.extend(decompose_by_height_profile(points_right, resolution, min_voxels))
                return res

    # 没有任何超过 6cm 的高度突变，说明它是一个平整的实体块，返回原样
    return [points]


def detect_workspace_boxes(
    points: np.ndarray,
    voxel_sizes: np.ndarray,
    plane_indices: np.ndarray,
    table_plane: Optional[PlaneResult],
    args,
):
    if table_plane is None:
        return [], np.array([], dtype=int)

    keep_mask = np.ones(len(points), dtype=bool)
    if plane_indices is not None and len(plane_indices) > 0:
        keep_mask[plane_indices] = False

    candidate_points = points[keep_mask]
    if len(candidate_points) < 5:
        return [], np.array([], dtype=int)

    # 提取当前候选点对应的体素大小
    candidate_sizes = voxel_sizes[keep_mask]
    resolution = float(np.median(voxel_sizes)) if len(voxel_sizes) else 0.01

    # ==================== 改进版：带“体积豁免”的浮点剔除 ====================
    tree = cKDTree(candidate_points)
    search_radius = resolution * 2.5
    counts = tree.query_ball_point(candidate_points, r=search_radius, return_length=True)

    # 条件1：如果它是最小分辨率的体素，必须有足够的邻居陪伴 (>= 4)
    has_enough_neighbors = counts >= 4

    # 条件2：如果它是被 OctoMap 合并过的大体素 (尺寸大于基础分辨率的1.5倍)，无条件保留！
    is_large_voxel = candidate_sizes > (resolution * 1.5)

    # 只要满足上述任意一个条件，就是合法的有效点
    valid_points_mask = has_enough_neighbors | is_large_voxel

    # 找到被拉黑的纯噪点在全局数组中的索引
    noise_global_indices = np.flatnonzero(keep_mask)[~valid_points_mask]
    clean_global_mask = np.ones(len(points), dtype=bool)
    if len(noise_global_indices) > 0:
        clean_global_mask[noise_global_indices] = False

    # 更新候选点
    candidate_points = candidate_points[valid_points_mask]
    if len(candidate_points) < 5:
        return [], np.array([], dtype=int)
    # ===============================================================

    table_min = np.array(table_plane.min_bound, dtype=float)
    table_max = np.array(table_plane.max_bound, dtype=float)

    xy_margin = float(args.workspace_xy_margin)
    z_min = float(table_plane.top_z + args.workspace_z_min_offset)
    z_max = float(table_plane.top_z + args.workspace_z_max_height)

    workspace_min = np.array([table_min[0] - xy_margin, table_min[1] - xy_margin, z_min], dtype=float)
    workspace_max = np.array([table_max[0] + xy_margin, table_max[1] + xy_margin, z_max], dtype=float)

    in_workspace_mask = filter_points_in_box(candidate_points, workspace_min, workspace_max)
    workspace_points = candidate_points[in_workspace_mask]
    if len(workspace_points) < 5:
        return [], np.array([], dtype=int)

    coarse_clusters = ckdtree_clustering(workspace_points, resolution, radius_scale=max(float(args.workspace_cluster_radius_scale), 1.0))

    obstacles: List[ObstacleResult] = []
    kept_masks = []
    min_voxels = max(int(args.workspace_min_cluster_voxels), 1)

    for cluster in coarse_clusters:
        if len(cluster) < min_voxels:
            continue

        for sub_cluster in decompose_by_height_profile(cluster, resolution, min_voxels=min_voxels):
            min_bound = sub_cluster.min(axis=0).astype(float)
            max_bound = sub_cluster.max(axis=0).astype(float)

            # 🌟 新增：重力支撑检测 (过滤悬空飞线)
            # 如果这个体素簇的最低点，距离桌面超过了 10cm (0.1m)
            # 说明它悬浮在空中，没有物理支撑，直接判定为相机噪点/飞线并剔除！
            try:
                if min_bound[2] > float(table_plane.top_z) + 0.10:
                    continue
            except Exception:
                # 保守起见，若 table_plane 数据不可用则不过滤
                pass

            min_bound[2] = min(min_bound[2], float(table_plane.top_z))
            if not args.no_snap_obstacles_to_table:
                min_bound[2] = float(table_plane.top_z)
            max_bound[2] = max(max_bound[2], min_bound[2] + resolution)

            center = (min_bound + max_bound) / 2.0
            size = np.maximum(max_bound - min_bound, resolution)
            box_mask = filter_points_in_box(points, min_bound, max_bound)
            kept_masks.append(box_mask)

            obstacles.append(ObstacleResult(
                id=len(obstacles),
                center=[float(x) for x in center],
                size=[float(x) for x in size],
                min_bound=[float(x) for x in min_bound],
                max_bound=[float(x) for x in max_bound],
                voxel_count=int(len(sub_cluster)),
                support_type='table',
                support_id='table',
                support_top_z=float(table_plane.top_z),
                stack_level=1,
            ))

    if not kept_masks:
        return [], np.array([], dtype=int)

    combined_mask = np.any(np.stack(kept_masks, axis=0), axis=0)
    kept_indices = np.flatnonzero(combined_mask)
    return obstacles, kept_indices

def color(r, g, b, a):
    return ColorRGBA(r=r, g=g, b=b, a=a)


def make_cube_marker(marker_id: int, ns: str, frame_id: str, center: Sequence[float], size: Sequence[float], rgba: ColorRGBA) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.ns = ns
    m.id = marker_id
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.pose.position.x = float(center[0])
    m.pose.position.y = float(center[1])
    m.pose.position.z = float(center[2])
    m.scale.x = max(float(size[0]), 1e-3)
    m.scale.y = max(float(size[1]), 1e-3)
    m.scale.z = max(float(size[2]), 1e-3)
    m.color = rgba
    m.lifetime = rospy.Duration(0)
    return m


def make_voxel_cube_list_marker(
    marker_id: int,
    ns: str,
    frame_id: str,
    points: np.ndarray,
    voxel_size: float,
    rgba: ColorRGBA,
) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.ns = ns
    m.id = marker_id
    m.type = Marker.CUBE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = max(float(voxel_size), 1e-3)
    m.scale.y = max(float(voxel_size), 1e-3)
    m.scale.z = max(float(voxel_size), 1e-3)
    m.color = rgba
    m.lifetime = rospy.Duration(0)
    for p in points:
        pt = Point()
        pt.x = float(p[0])
        pt.y = float(p[1])
        pt.z = float(p[2])
        m.points.append(pt)
    return m


def make_delete_all_marker(frame_id: str) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.action = Marker.DELETEALL
    return m


def export_result(result: ProcessingResult, out_json: Path):
    out_json.write_text(json.dumps({
        'resolution_hint': result.resolution_hint,
        'table_plane': asdict(result.table_plane) if result.table_plane else None,
        'obstacles': [asdict(o) for o in result.obstacles],
        'total_occupied_voxels': result.total_occupied_voxels,
        'tabletop_candidate_voxels': result.tabletop_candidate_voxels,
        'non_table_voxels': result.non_table_voxels,
        'kept_voxel_indices': result.kept_voxel_indices,
    }, indent=2, ensure_ascii=False))


def run_rebuilder(input_bt: Path, output_bt: Path, min_z: float, max_z: float):
    # Use standard shell command to run rosrun
    cmd = [
        'rosrun', 'map_process', 'map_rebuild',
        str(input_bt),
        str(output_bt),
        str(min_z),
        str(max_z)
    ]
    subprocess.run(cmd, check=True)


def run_converter(input_bt: Path, out_xyz: Path):
    cmd = [
        'rosrun', 'map_process', 'octomap_bt_to_points',
        str(input_bt),
        str(out_xyz),
        'occupied'
    ]
    subprocess.run(cmd, check=True)


def run_generator(input_xyz: Path, output_bt: Path, resolution: float):
    cmd = [
        'rosrun', 'map_process', 'points_to_octomap_bt',
        str(input_xyz),
        str(output_bt),
        str(resolution)
    ]
    subprocess.run(cmd, check=True)


def export_filtered_xyz(points: np.ndarray, kept_indices: List[int], out_xyz: Path):
    if not kept_indices:
        out_xyz.write_text('')
        return

    # Use existing points array and filter it
    filtered_points = points[kept_indices]
    
    # Save to file
    with open(out_xyz, 'w') as f:
        for p in filtered_points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def build_marker_array(result: ProcessingResult, frame_id: str, raw_points: Optional[np.ndarray] = None, show_raw_voxels: bool = True) -> MarkerArray:
    arr = MarkerArray()
    arr.markers.append(make_delete_all_marker(frame_id))
    next_id = 0

    if show_raw_voxels and raw_points is not None and len(raw_points) > 0:
        arr.markers.append(
            make_voxel_cube_list_marker(
                next_id,
                'raw_occupied_voxels',
                frame_id,
                raw_points,
                result.resolution_hint,
                color(0.2, 0.6, 1.0, 0.5),
            )
        )
        next_id += 1

    if result.table_plane:
        plane = result.table_plane
        size = [max(plane.extents_xy[0], 1e-3), max(plane.extents_xy[1], 1e-3), max(plane.thickness, result.resolution_hint)]
        arr.markers.append(make_cube_marker(next_id, 'table_plane', frame_id, plane.center, size, color(0.1, 0.7, 0.2, 0.4)))
        next_id += 1

    for obstacle in result.obstacles:
        arr.markers.append(make_cube_marker(next_id, 'obstacle_aabb', frame_id, obstacle.center, obstacle.size, color(0.85, 0.2, 0.2, 0.5)))
        next_id += 1

    return arr


def build_environment_info_msg(result: ProcessingResult) -> EnvironmentInfo:
    env_info = EnvironmentInfo()
    env_info.table_z = float(result.table_plane.top_z) if result.table_plane else 0.0

    for obstacle in result.obstacles:
        box = BoundingBox()
        box.center.x = float(obstacle.center[0])
        box.center.y = float(obstacle.center[1])
        box.center.z = float(obstacle.center[2])
        box.size.x = float(obstacle.size[0])
        box.size.y = float(obstacle.size[1])
        box.size.z = float(obstacle.size[2])
        env_info.obstacles.append(box)

    return env_info


def process(points: np.ndarray, voxel_sizes: np.ndarray, args) -> ProcessingResult:
    resolution_hint = float(np.median(voxel_sizes)) if len(voxel_sizes) else 0.05

    # 1. 识别并提取桌面
    plane, plane_indices, candidate_count = detect_table_plane(
        points,
        voxel_sizes,
        min_z=args.table_min_z,
        max_z=args.table_max_z,
        distance_threshold=max(args.plane_distance, resolution_hint * 0.75),
    )

    # 2. 直接识别桌面上方的几个总包围盒，并据此裁剪原始点云
    obstacles, kept_indices = detect_workspace_boxes(
        points,
        voxel_sizes,
        plane_indices,
        plane,
        args,
    )

    non_table_voxels = int(len(points) - len(plane_indices))
    if len(kept_indices) > 0:
        non_table_voxels = int(len(kept_indices))

    kept_with_table_indices = np.array(kept_indices, dtype=int)
    if plane_indices is not None and len(plane_indices) > 0:
        kept_with_table_indices = np.unique(np.concatenate([np.array(plane_indices, dtype=int), kept_with_table_indices]))

    # 3. 返回结果
    return ProcessingResult(
        resolution_hint=resolution_hint,
        table_plane=plane,
        obstacles=obstacles,
        total_occupied_voxels=int(len(points)),
        tabletop_candidate_voxels=int(candidate_count),
        non_table_voxels=non_table_voxels,
        kept_voxel_indices=[int(i) for i in kept_with_table_indices],
    )


def parse_args():
    p = argparse.ArgumentParser(description='Process OctoMap .bt and publish segmented results as MarkerArray for RViz.')
    p.add_argument('--input-bt', required=False)
    p.add_argument('--workdir', default='.')
    p.add_argument('--frame-id', default='map')
    p.add_argument('--marker-topic', default='/octomap_processed_markers')
    p.add_argument('--environment-topic', default='/map_process/environment_info')
    # Use store_true/false to read boolean from command line switch easily? 
    # But since launch passes args differently, it's safer to keep using string or just check param server
    p.add_argument('--obstacle-avoidance-mode', type=str, default='aabb_apf', 
                   choices=['voxel_rrt', 'aabb_apf'], help='Obstacle avoidance mode: voxel_rrt or aabb_apf')
    p.add_argument('--table-min-z', type=float, default=0.4)
    p.add_argument('--table-max-z', type=float, default=1.1)
    p.add_argument('--plane-distance', type=float, default=0.03)
    p.add_argument('--cube-min', type=float, default=0.07)
    p.add_argument('--cube-max', type=float, default=0.45)
    p.add_argument('--cleaned-bt', default='cleaned_map.bt', help='Final output OctoMap name.')
    p.add_argument('--output-json', default='segmentation_result.json')
    p.add_argument('--output-xyz', default='occupied_voxels.xyz')
    p.add_argument('--hide-raw-voxels', type=str2bool, nargs='?', const=True, default=False, help='Do not publish raw occupied voxels from the original bt map.')
    p.add_argument('--support-gap-max', type=float, default=0.5, help='Maximum allowed gap between obstacle bottom and table top.')
    p.add_argument('--camera-quadrant', default='neg_xy', help='Camera viewing direction hint for visible-face compensation.')
    p.add_argument('--no-snap-obstacles-to-table', type=str2bool, nargs='?', const=True, default=False, help='Keep detected obstacle AABBs at raw voxel height without snapping their bottom to the tabletop.')
    p.add_argument('--workspace-xy-margin', type=float, default=0.03, help='Extra XY margin around the detected tabletop workspace.')
    p.add_argument('--workspace-z-min-offset', type=float, default=-0.02, help='Lower bound offset from table top for workspace box filtering.')
    p.add_argument('--workspace-z-max-height', type=float, default=0.60, help='Upper height above table top kept in workspace box filtering.')
    p.add_argument('--workspace-cluster-radius-scale', type=float, default=3.0, help='Radius scale used to cluster tabletop workspace points into coarse boxes.')
    p.add_argument('--workspace-min-cluster-voxels', type=int, default=20, help='Minimum voxel count for a workspace cluster to become a coarse bounding box.')
    p.add_argument('--flyer-min-neighbors', type=int, default=1, help='Light flyer removal: minimum neighboring voxels required to keep a point. Set 0 to disable.')
    p.add_argument('--flyer-radius-scale', type=float, default=1.05, help='Neighborhood radius scale for light flyer removal, relative to voxel resolution.')
    p.add_argument('--once', type=str2bool, nargs='?', const=True, default=False, help='Publish markers once and exit after a short delay.')
    p.add_argument('--rate', type=float, default=1.0)
    p.add_argument('--disable-markers', type=str2bool, nargs='?', const=True, default=False, help='Do not publish MarkerArray; still generate cleaned bt/json outputs.')
    args, _unknown = p.parse_known_args()
    return args


def main():
    # -------------------------------------------------------------
    # 0. Load params from ROS Parameter Server (preferred for launch files)
    # -------------------------------------------------------------
    # If launched from a launch file, params are usually set on the parameter server
    # We should merge command line args with params, prioritizing params if they exist.
    
    # We use a trick: parse known args first to get defaults
    args_cli, _ = argparse.ArgumentParser().parse_known_args() # Just to initialize
    
    # Re-parse properly
    args = parse_args()

    # Check if 'obstacle_avoidance_mode' is set on the private parameter server (~obstacle_avoidance_mode)
    if rospy.has_param('~obstacle_avoidance_mode'):
        args.obstacle_avoidance_mode = rospy.get_param('~obstacle_avoidance_mode')
        
    # Also check other params if needed, but let's stick to cli-args mapping for now 
    # because the launch file passes them as args in previous version?
    # Wait, the new launch file uses <param>. The old one used cli args in launch-prefix.
    # The 'process_octomap_markers.py' is now a standard node.
    # Standard nodes should read from rospy.get_param not argparse usually.
    # BUT to keep backward compatibility and minimize changes, we can just read from param if available.
    
    # The launch file I just edited puts <param name="obstacle_avoidance_mode" ... /> inside the node.
    # So `rospy.get_param('~obstacle_avoidance_mode')` will work.
    
    # However, the rest of the script relies heavily on `args`.
    # Let's map ALL relevant ROS params to `args` if they exist.
    
    rospy.init_node('map_process', anonymous=False) # Init node early to read params

    # Helper to load param
    def load_p(name, default):
        return rospy.get_param(f'~{name}', default)

    args.input_bt = load_p('input_bt', args.input_bt)
    args.workdir = load_p('workdir', args.workdir)
    args.table_min_z = float(load_p('table_min_z', args.table_min_z))
    args.table_max_z = float(load_p('table_max_z', args.table_max_z))
    args.plane_distance = float(load_p('plane_distance', args.plane_distance))
    args.cleaned_bt = load_p('cleaned_bt', args.cleaned_bt)
    args.output_json = load_p('output_json', args.output_json)
    args.frame_id = load_p('frame_id', args.frame_id)
    args.marker_topic = load_p('marker_topic', args.marker_topic)
    args.environment_topic = load_p('environment_topic', args.environment_topic)
    args.hide_raw_voxels = bool(load_p('hide_raw_voxels', args.hide_raw_voxels))
    args.disable_markers = bool(load_p('disable_markers', args.disable_markers))
    args.once = bool(load_p('once', args.once))
    args.rate = float(load_p('rate', args.rate))
    
    # Important switch
    args.obstacle_avoidance_mode = load_p('obstacle_avoidance_mode', args.obstacle_avoidance_mode)
    
    # ... map other params as needed ...
    args.workspace_xy_margin = float(load_p('workspace_xy_margin', args.workspace_xy_margin))
    args.workspace_z_min_offset = float(load_p('workspace_z_min_offset', args.workspace_z_min_offset))
    args.workspace_z_max_height = float(load_p('workspace_z_max_height', args.workspace_z_max_height))

    
    if not args.input_bt:
        import re
        from pathlib import Path
        workspace_dir = Path(__file__).resolve().parent.parent.parent.parent
        map_dir = workspace_dir / 'map'
        if map_dir.exists() and map_dir.is_dir():
            bt_files = list(map_dir.glob('*.bt'))
            if bt_files:
                def get_numeric_key(p):
                    match = re.search(r'\d+', p.stem)
                    return int(match.group()) if match else 0
                latest_bt = max(bt_files, key=get_numeric_key)
                args.input_bt = str(latest_bt)
                try:
                    rospy.loginfo(f"Autodetected latest map: {args.input_bt}")
                except:
                    pass

    if not args.input_bt:
        rospy.logerr("input_bt is required! Provide either via arg --input-bt or param ~input_bt.")
        return 1
    
    workdir = Path(args.workdir).resolve()
    input_bt = Path(args.input_bt).resolve()
    cleaned_bt = (workdir / args.cleaned_bt).resolve()
    out_json = (workdir / args.output_json).resolve()

    if not input_bt.exists():
        rospy.logerr(f'Input .bt not found: {input_bt}')
        return 1
        
    rospy.loginfo(f"Starting octomap processing in mode: {args.obstacle_avoidance_mode}")
    
    # -------------------------------------------------------------
    # I/O Optimization: Use /dev/shm (RAM Disk) for temp files
    # -------------------------------------------------------------
    import uuid
    session_id = str(uuid.uuid4())[:8]
    ram_base = Path('/dev/shm') / f'octomap_proc_{session_id}'
    ram_base.mkdir(parents=True, exist_ok=True)

    try:
        temp_pre_bt = ram_base / 'temp_pre.bt'
        temp_xyz = ram_base / 'temp_voxels.xyz'
        cleaned_xyz = ram_base / 'cleaned_voxels.xyz'
        temp_final_bt = ram_base / 'temp_final.bt'

        # 1. C++ Preprocessing (Height & Cluster Filter)
        # Passing table filtering args to prevent Blind Cut by C++ defaults
        # Always run basic cleaning regardless of mode
        run_rebuilder(
            input_bt, 
            temp_pre_bt,
            min_z=args.table_min_z - 0.2, # Give some margin for table legs or slightly uneven floor
            max_z=args.table_max_z + 1.0  # Allow some overhead
        )

        # Mode dependent processing
        if args.obstacle_avoidance_mode == 'voxel_rrt':
            # In voxel_rrt mode, we just want the cleaned map. 
            # The simple rebuilder output is enough, or we can optionally run 
            # points-to-map if we want to ensure consistency, but usually minimal processing is desired.
            # Assuming rebuilder output is sufficient for voxel_rrt
            
            # Since user asked to skip complex clustering/segmentation logic:
            rospy.loginfo("Mode is voxel_rrt: Skipping advanced clustering and segmentation.")
            
            # We just move the temp_pre_bt to the final location
            shutil.move(str(temp_pre_bt), str(cleaned_bt))
            rospy.loginfo(f"Generated clean voxel map: {cleaned_bt}")
            
            # Create a minimal result for JSON export (empty obstacles)
            result = ProcessingResult(
                resolution_hint=0.05, # Default guess
                table_plane=None,
                obstacles=[],
                total_occupied_voxels=0,
                tabletop_candidate_voxels=0,
                non_table_voxels=0,
                kept_voxel_indices=[]
            )
            export_result(result, out_json)
             
            # Return early - no markers to publish in this mode
            return 0


        # If we are here, mode is 'aabb_apf' - Proceed with full pipeline

        # 2. Convert to XYZ
        run_converter(temp_pre_bt, temp_xyz)
        points, voxel_sizes = load_xyzs(temp_xyz)
        
        # 3. Advanced Processing
        result = process(points, voxel_sizes, args)
        
        # Export JSON (to disk, persistent)
        export_result(result, out_json)

        # 4. Filter Points & Export Clean XYZ
        export_filtered_xyz(points, result.kept_voxel_indices, cleaned_xyz)

        # 5. Generate Final OctoMap containing ONLY the obstacles (no table, no noise)
        # Use resolution from result, default to 0.05
        res = result.resolution_hint if result.resolution_hint > 0 else 0.05
        run_generator(cleaned_xyz, temp_final_bt, res)

        # 6. Atomic Move to Result Path
        shutil.move(str(temp_final_bt), str(cleaned_bt))
        rospy.loginfo(f"Generated clean map for AABB APF: {cleaned_bt}")

        display_points = points
        if result.kept_voxel_indices:
            kept_idx = np.array(result.kept_voxel_indices, dtype=int)
            display_points = points[kept_idx]

        # ---------------------------------------------------------
        # ROS Publishing
        # ---------------------------------------------------------
        # rospy.init_node('octomap_processed_markers_publisher', anonymous=False) # Already initialized
        pub = None
        markers = None
        env_pub = rospy.Publisher(args.environment_topic, EnvironmentInfo, queue_size=1, latch=True)
        env_info_msg = build_environment_info_msg(result)
        if not args.disable_markers:
            pub = rospy.Publisher(args.marker_topic, MarkerArray, queue_size=1, latch=True)
            markers = build_marker_array(
                result,
                args.frame_id,
                raw_points=display_points,
                show_raw_voxels=not args.hide_raw_voxels,
            )

        rospy.loginfo('Processed %d occupied voxels, detected %d obstacles.', result.total_occupied_voxels, len(result.obstacles))
        if result.table_plane:
            rospy.loginfo('Detected table plane center=%s extents_xy=%s', result.table_plane.center, result.table_plane.extents_xy)
        else:
            rospy.logwarn('No table plane detected in z range [%.2f, %.2f].', args.table_min_z, args.table_max_z)

        if args.once:
            for _ in range(3):
                if rospy.is_shutdown(): break
                env_pub.publish(env_info_msg)
                if pub is not None and markers is not None:
                    for m in markers.markers:
                        m.header.stamp = rospy.Time.now()
                    pub.publish(markers)
                rospy.sleep(0.5)
            # Cleanup RAM
            shutil.rmtree(ram_base)
            return 0

        rate = rospy.Rate(max(args.rate, 0.2))
        while not rospy.is_shutdown():
            env_pub.publish(env_info_msg)
            if pub is not None and markers is not None:
                for m in markers.markers:
                    m.header.stamp = rospy.Time.now()
                pub.publish(markers)
            rate.sleep()

    finally:
        # Ensure RAM Cleanup
        if ram_base.exists():
            shutil.rmtree(ram_base)
            
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except subprocess.CalledProcessError as exc:
        print(f'Command failed: {exc}', file=sys.stderr)
        sys.exit(exc.returncode)
    except Exception as exc:
        print(f'Error: {exc}', file=sys.stderr)
        sys.exit(1)
