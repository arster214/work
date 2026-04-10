#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import struct
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from heatmap_common import (
    build_clearance_heatmap,
    compute_workspace_bounds_from_env_info,
    compute_workspace_bounds_from_segmentation,
)

try:
    import rospy
    from clearance_heatmap_msgs.msg import EnvironmentInfo
    from clearance_heatmap_msgs.srv import GenerateClearanceHeatmap, GenerateClearanceHeatmapRequest
    from geometry_msgs.msg import Point
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


def str2bool(value):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "y", "on"}:
        return True
    if text in {"false", "0", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {value}")


def resolve_workspace() -> Path:
    current = Path(__file__).resolve()
    for parent in [current.parent] + list(current.parents):
        if (parent / "src").is_dir() and (parent / "map").exists():
            return parent
    raise FileNotFoundError("Could not locate workspace root containing both 'src' and 'map'.")


def find_latest_heatmap_npz(workspace: Path) -> Path:
    candidates = sorted(workspace.glob("map/heatmap_analysis_*/heatmap_predicted_vs_true.npz"))
    if candidates:
        return max(candidates, key=lambda p: p.stat().st_mtime)
    fallback = workspace / "map" / "heatmap_predicted_vs_true.npz"
    if fallback.exists():
        return fallback
    raise FileNotFoundError("No heatmap npz file found under map/heatmap_analysis_*/")


def find_latest_segmentation_json(workspace: Path) -> Path:
    fallback = workspace / "map" / "segmentation_result.json"
    candidates = sorted(workspace.glob("map/heatmap_analysis_*/segmentation_latest.json"))
    if fallback.exists():
        candidates.append(fallback)
    if candidates:
        return max(candidates, key=lambda p: p.stat().st_mtime)
    raise FileNotFoundError("No segmentation json found under map/heatmap_analysis_*/ or map/segmentation_result.json")


def resolve_heatmap_key(name: str) -> str:
    aliases = {
        "pred": "pred_heatmap_zyx",
        "predicted": "pred_heatmap_zyx",
        "true": "true_heatmap_zyx",
        "analytic": "true_heatmap_zyx",
    }
    return aliases.get(name, name)


def scalar_to_rgb(value: float, vmin: float, vmax: float):
    if vmax <= vmin:
        t = 0.5
    else:
        t = float(np.clip((value - vmin) / (vmax - vmin), 0.0, 1.0))

    anchors = [
        (0.0, np.array([68.0, 1.0, 84.0])),
        (0.33, np.array([59.0, 82.0, 139.0])),
        (0.66, np.array([33.0, 145.0, 140.0])),
        (1.0, np.array([253.0, 231.0, 37.0])),
    ]

    for i in range(len(anchors) - 1):
        left_t, left_rgb = anchors[i]
        right_t, right_rgb = anchors[i + 1]
        if t <= right_t:
            local = 0.0 if right_t <= left_t else (t - left_t) / (right_t - left_t)
            rgb = left_rgb * (1.0 - local) + right_rgb * local
            rgb = np.clip(np.round(rgb), 0, 255).astype(np.uint8)
            return int(rgb[0]), int(rgb[1]), int(rgb[2])

    rgb = np.clip(np.round(anchors[-1][1]), 0, 255).astype(np.uint8)
    return int(rgb[0]), int(rgb[1]), int(rgb[2])


def rgb_to_uint32(r: int, g: int, b: int) -> int:
    return (r << 16) | (g << 8) | b


def rgb_to_float32(r: int, g: int, b: int) -> float:
    packed = rgb_to_uint32(r, g, b)
    return struct.unpack("f", struct.pack("I", packed))[0]


def build_points(
    xs,
    ys,
    zs,
    heatmap_zyx,
    value_threshold=None,
    percentile=0.0,
    selection_mode="above",
    max_points=0,
):
    heatmap = np.asarray(heatmap_zyx, dtype=np.float32)
    if heatmap.shape != (len(zs), len(ys), len(xs)):
        raise ValueError(
            f"Heatmap shape {heatmap.shape} does not match grid "
            f"({len(zs)}, {len(ys)}, {len(xs)})"
        )

    flat_values = heatmap.reshape(-1)
    effective_threshold = value_threshold
    if percentile > 0.0:
        pct_value = float(np.percentile(flat_values, percentile))
        if effective_threshold is None:
            effective_threshold = pct_value
        elif selection_mode == "below":
            effective_threshold = min(effective_threshold, pct_value)
        else:
            effective_threshold = max(effective_threshold, pct_value)

    vmin = float(np.min(flat_values))
    vmax = float(np.max(flat_values))
    vertices = []

    for iz, z in enumerate(zs):
        for iy, y in enumerate(ys):
            for ix, x in enumerate(xs):
                val = float(heatmap[iz, iy, ix])
                if effective_threshold is not None:
                    if selection_mode == "below" and val > effective_threshold:
                        continue
                    if selection_mode != "below" and val < effective_threshold:
                        continue
                r, g, b = scalar_to_rgb(val, vmin, vmax)
                vertices.append({
                    "x": float(x),
                    "y": float(y),
                    "z": float(z),
                    "intensity": val,
                    "red": r,
                    "green": g,
                    "blue": b,
                })

    if max_points and max_points > 0 and len(vertices) > max_points:
        step = int(np.ceil(len(vertices) / float(max_points)))
        vertices = vertices[::step]

    return vertices, vmin, vmax, effective_threshold


def load_heatmap_from_npz(workspace: Path, input_npz: str, heatmap_key_name: str):
    input_path = find_latest_heatmap_npz(workspace) if str(input_npz).upper() == "AUTO" else Path(input_npz).expanduser().resolve()
    if not input_path.exists():
        raise FileNotFoundError(f"Heatmap npz not found: {input_path}")

    heatmap_key = resolve_heatmap_key(heatmap_key_name)
    data = np.load(input_path)
    required_keys = {"xs", "ys", "zs", heatmap_key}
    missing = [key for key in required_keys if key not in data]
    if missing:
        raise KeyError(f"Missing keys in {input_path}: {missing}")

    metadata = {
        "source_type": "npz",
        "input_npz": str(input_path),
        "heatmap_key": heatmap_key,
    }
    return input_path, data["xs"], data["ys"], data["zs"], data[heatmap_key], metadata


def load_heatmap_from_fiesta_bbox(
    workspace: Path,
    segmentation_json: str,
    resolution: float,
    xy_margin: float,
    z_min_offset: float,
    z_max_height: float,
    robot_radius: float,
    safety_margin: float,
    clearance_clip: float,
):
    seg_path = find_latest_segmentation_json(workspace) if str(segmentation_json).upper() == "AUTO" else Path(segmentation_json).expanduser().resolve()
    if not seg_path.exists():
        raise FileNotFoundError(f"Segmentation json not found: {seg_path}")

    segmentation = json.loads(seg_path.read_text())
    obstacles = segmentation.get("obstacles", [])
    xmin, xmax, ymin, ymax, zmin, zmax = compute_workspace_bounds_from_segmentation(
        segmentation=segmentation,
        xy_margin=xy_margin,
        z_min_offset=z_min_offset,
        z_max_height=z_max_height,
    )
    centers = np.array([obs["center"] for obs in obstacles], dtype=np.float32).reshape(-1, 3) if obstacles else np.zeros((0, 3), dtype=np.float32)
    sizes = np.array([obs["size"] for obs in obstacles], dtype=np.float32).reshape(-1, 3) if obstacles else np.zeros((0, 3), dtype=np.float32)
    xs, ys, zs, heatmap_zyx = build_clearance_heatmap(
        centers=centers,
        sizes=sizes,
        resolution=resolution,
        min_bound=[xmin, ymin, zmin],
        max_bound=[xmax, ymax, zmax],
        robot_radius=robot_radius,
        safety_margin=safety_margin,
        clearance_clip=clearance_clip,
    )
    npz_path = seg_path.with_name("fiesta_bbox_heatmap.npz")
    np.savez_compressed(
        npz_path,
        xs=xs,
        ys=ys,
        zs=zs,
        fiesta_clearance_zyx=heatmap_zyx,
    )

    metadata = {
        "source_type": "fiesta_bbox",
        "segmentation_json": str(seg_path),
        "generated_npz": str(npz_path),
        "heatmap_key": "fiesta_clearance_zyx",
        "grid_resolution": float(resolution),
        "robot_radius": float(robot_radius),
        "safety_margin": float(safety_margin),
        "clearance_clip": float(clearance_clip),
    }
    return npz_path, xs, ys, zs, heatmap_zyx, metadata


def load_heatmap_from_live_env(
    workspace: Path,
    environment_topic: str,
    service_name: str,
    timeout_seconds: float,
    resolution: float,
    xy_margin: float,
    z_min_offset: float,
    z_max_height: float,
):
    if not ROS_AVAILABLE:
        raise RuntimeError("ROS Python dependencies are not available. Source ROS before using live_env.")

    wait_step = max(float(timeout_seconds), 0.5)
    env_info = None
    while not rospy.is_shutdown() and env_info is None:
        try:
            env_info = rospy.wait_for_message(environment_topic, EnvironmentInfo, timeout=wait_step)
        except rospy.ROSException:
            rospy.loginfo_throttle(
                2.0,
                "Waiting for environment info on %s before visualizing live heatmap...",
                environment_topic,
            )

    if env_info is None:
        raise RuntimeError(f"ROS shutdown while waiting for environment info on {environment_topic}.")

    min_bound, max_bound = compute_workspace_bounds_from_env_info(
        env_info=env_info,
        xy_margin=xy_margin,
        z_min_offset=z_min_offset,
        z_max_height=z_max_height,
    )

    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service(service_name, timeout=wait_step)
            break
        except rospy.ROSException:
            rospy.loginfo_throttle(
                2.0,
                "Waiting for heatmap service %s before visualizing live heatmap...",
                service_name,
            )

    if rospy.is_shutdown():
        raise RuntimeError(f"ROS shutdown while waiting for heatmap service {service_name}.")

    service = rospy.ServiceProxy(service_name, GenerateClearanceHeatmap)
    request = GenerateClearanceHeatmapRequest()
    request.env_info = env_info
    request.resolution = float(resolution)
    request.min_bound = Point(float(min_bound[0]), float(min_bound[1]), float(min_bound[2]))
    request.max_bound = Point(float(max_bound[0]), float(max_bound[1]), float(max_bound[2]))
    response = service(request)

    if not response.success:
        raise RuntimeError(f"Heatmap service failed: {response.message}")
    if response.dim_x <= 0 or response.dim_y <= 0 or response.dim_z <= 0:
        raise RuntimeError("Heatmap service returned invalid grid dimensions.")

    heatmap_zyx = np.asarray(response.heatmap_data, dtype=np.float32).reshape(
        int(response.dim_z), int(response.dim_y), int(response.dim_x)
    )
    xs = np.arange(min_bound[0], min_bound[0] + response.dim_x * resolution, resolution, dtype=np.float32)
    ys = np.arange(min_bound[1], min_bound[1] + response.dim_y * resolution, resolution, dtype=np.float32)
    zs = np.arange(min_bound[2], min_bound[2] + response.dim_z * resolution, resolution, dtype=np.float32)

    npz_path = workspace / "map" / "live_environment_heatmap.npz"
    metadata = {
        "source_type": "live_env",
        "environment_topic": environment_topic,
        "service_name": service_name,
        "heatmap_key": "live_heatmap_zyx",
        "resolution": resolution,
        "grid_origin_xyz": [float(min_bound[0]), float(min_bound[1]), float(min_bound[2])],
        "service_message": response.message,
    }
    return npz_path, xs, ys, zs, heatmap_zyx, metadata


def write_ply(vertices, output_path: Path):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(vertices)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("property float intensity\n")
        f.write("end_header\n")
        for item in vertices:
            f.write(
                f"{item['x']:.6f} {item['y']:.6f} {item['z']:.6f} "
                f"{item['red']} {item['green']} {item['blue']} {item['intensity']:.6f}\n"
            )


def make_pointcloud2(vertices, frame_id: str):
    header = Header()
    header.frame_id = frame_id
    header.stamp = rospy.Time.now()
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.FLOAT32, 1),
        PointField("rgb", 16, PointField.FLOAT32, 1),
    ]
    cloud_points = [
        (item["x"], item["y"], item["z"], item["intensity"], rgb_to_float32(item["red"], item["green"], item["blue"]))
        for item in vertices
    ]
    return point_cloud2.create_cloud(header, fields, cloud_points)


def parse_args():
    p = argparse.ArgumentParser(description="Export or publish a 3D point-cloud view of a saved heatmap npz.")
    p.add_argument(
        "--source-type",
        default="npz",
        choices=["npz", "fiesta_bbox", "live_env"],
        help="Load heatmap from an existing npz, generate a FIESTA-style field from bbox segmentation json, or request the live field from ROS.",
    )
    p.add_argument("--input-npz", default="AUTO", help="Path to heatmap_predicted_vs_true.npz. Use AUTO to pick the latest analysis file.")
    p.add_argument("--segmentation-json", default="AUTO", help="Path to segmentation json from map_process. Used when source-type=fiesta_bbox.")
    p.add_argument("--heatmap-key", default="pred", help="Heatmap key or alias: pred/predicted/true/analytic.")
    p.add_argument("--environment-topic", default="/map_process/environment_info", help="ROS topic used when source-type=live_env.")
    p.add_argument("--service-name", default="/clearance_heatmap_server/generate_clearance_heatmap", help="ROS service used when source-type=live_env.")
    p.add_argument("--service-timeout", type=float, default=3.0, help="Timeout in seconds for live_env topic/service waits.")
    p.add_argument("--output-ply", default="AUTO", help="Output .ply path. Use AUTO to save beside the input npz.")
    p.add_argument("--metadata-json", default="AUTO", help="Output .json summary path. Use AUTO to save beside the input npz.")
    p.add_argument(
        "--selection-mode",
        default="above",
        choices=["above", "below"],
        help="Point selection mode. 'above' keeps high-value points, 'below' keeps low-value points.",
    )
    p.add_argument("--value-threshold", type=float, default=None, help="Threshold used with selection_mode.")
    p.add_argument("--percentile", type=float, default=0.0, help="Percentile used with selection_mode. Example: 30 with 'below' keeps the lowest 30%%.")
    p.add_argument("--max-points", type=int, default=0, help="Optional cap on exported/published points.")
    p.add_argument("--fiesta-resolution", type=float, default=0.05, help="Grid resolution used for fiesta_bbox source.")
    p.add_argument("--fiesta-xy-margin", type=float, default=0.03, help="XY workspace padding around the table/obstacle bounds.")
    p.add_argument("--fiesta-z-min-offset", type=float, default=-0.02, help="Lower Z bound offset from table top or obstacle min bound.")
    p.add_argument("--fiesta-z-max-height", type=float, default=0.60, help="Upper Z height above table top for the fiesta grid.")
    p.add_argument("--robot-radius", type=float, default=0.05, help="Robot body/arm clearance radius deducted from signed distance.")
    p.add_argument("--safety-margin", type=float, default=0.02, help="Extra safety margin deducted from signed distance.")
    p.add_argument("--clearance-clip", type=float, default=0.40, help="Clip clearance field to [-clip, clip] for visualization.")
    p.add_argument("--frame-id", default="map")
    p.add_argument("--topic", default="/clearance_heatmap/point_cloud")
    p.add_argument("--publish", type=str2bool, nargs="?", const=True, default=False)
    p.add_argument("--once", type=str2bool, nargs="?", const=True, default=True)
    p.add_argument("--rate", type=float, default=1.0)
    argv = rospy.myargv(argv=sys.argv) if ROS_AVAILABLE else sys.argv
    return p.parse_args(argv[1:])


def main():
    args = parse_args()
    if ROS_AVAILABLE and (args.publish or args.source_type == "live_env"):
        rospy.init_node("clearance_heatmap_visualizer", anonymous=False)

    workspace = resolve_workspace()
    if args.source_type == "fiesta_bbox":
        input_path, xs, ys, zs, heatmap_zyx, source_metadata = load_heatmap_from_fiesta_bbox(
            workspace=workspace,
            segmentation_json=args.segmentation_json,
            resolution=args.fiesta_resolution,
            xy_margin=args.fiesta_xy_margin,
            z_min_offset=args.fiesta_z_min_offset,
            z_max_height=args.fiesta_z_max_height,
            robot_radius=args.robot_radius,
            safety_margin=args.safety_margin,
            clearance_clip=args.clearance_clip,
        )
    elif args.source_type == "live_env":
        input_path, xs, ys, zs, heatmap_zyx, source_metadata = load_heatmap_from_live_env(
            workspace=workspace,
            environment_topic=args.environment_topic,
            service_name=args.service_name,
            timeout_seconds=args.service_timeout,
            resolution=args.fiesta_resolution,
            xy_margin=args.fiesta_xy_margin,
            z_min_offset=args.fiesta_z_min_offset,
            z_max_height=args.fiesta_z_max_height,
        )
    else:
        input_path, xs, ys, zs, heatmap_zyx, source_metadata = load_heatmap_from_npz(
            workspace=workspace,
            input_npz=args.input_npz,
            heatmap_key_name=args.heatmap_key,
        )

    if args.value_threshold is not None and np.isnan(args.value_threshold):
        args.value_threshold = None

    vertices, vmin, vmax, effective_threshold = build_points(
        xs,
        ys,
        zs,
        heatmap_zyx,
        value_threshold=args.value_threshold,
        percentile=args.percentile,
        selection_mode=args.selection_mode,
        max_points=args.max_points,
    )

    if not vertices:
        raise RuntimeError("No heatmap points left after filtering. Lower the threshold or percentile.")

    output_ply = None
    if str(args.output_ply).upper() == "AUTO":
        output_ply = input_path.with_name(f"{source_metadata['heatmap_key']}_pointcloud.ply")
    elif args.output_ply:
        output_ply = Path(args.output_ply).expanduser().resolve()

    if output_ply is not None:
        write_ply(vertices, output_ply)
        print(f"Saved PLY: {output_ply}")

    metadata = {
        **source_metadata,
        "point_count": len(vertices),
        "value_min": vmin,
        "value_max": vmax,
        "effective_threshold": effective_threshold,
        "selection_mode": args.selection_mode,
        "grid_shape_zyx": list(map(int, heatmap_zyx.shape)),
        "frame_id": args.frame_id,
        "topic": args.topic,
    }

    if str(args.metadata_json).upper() == "AUTO":
        metadata_path = input_path.with_name(f"{source_metadata['heatmap_key']}_pointcloud.json")
        metadata_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False))
        print(f"Saved metadata: {metadata_path}")
    elif args.metadata_json:
        metadata_path = Path(args.metadata_json).expanduser().resolve()
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False))
        print(f"Saved metadata: {metadata_path}")

    if not args.publish:
        print(json.dumps(metadata, indent=2, ensure_ascii=False))
        return 0

    if not ROS_AVAILABLE:
        raise RuntimeError("ROS Python dependencies are not available. Source ROS before using --publish.")
    pub = rospy.Publisher(args.topic, PointCloud2, queue_size=1, latch=True)
    cloud_msg = make_pointcloud2(vertices, args.frame_id)

    if args.once:
        for _ in range(3):
            if rospy.is_shutdown():
                break
            cloud_msg.header.stamp = rospy.Time.now()
            pub.publish(cloud_msg)
            rospy.sleep(0.3)
        print(f"Published {len(vertices)} heatmap points once on {args.topic}")
        return 0

    rate = rospy.Rate(max(args.rate, 0.2))
    while not rospy.is_shutdown():
        cloud_msg.header.stamp = rospy.Time.now()
        pub.publish(cloud_msg)
        rate.sleep()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
