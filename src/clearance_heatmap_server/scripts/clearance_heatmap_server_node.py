#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
from pathlib import Path

import numpy as np
import rospy

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from heatmap_common import build_clearance_heatmap, compute_workspace_bounds_from_env_info
from clearance_heatmap_msgs.srv import GenerateClearanceHeatmap, GenerateClearanceHeatmapResponse


def point_msg_to_array(point_msg) -> np.ndarray:
    return np.asarray([point_msg.x, point_msg.y, point_msg.z], dtype=np.float32)


class ClearanceHeatmapServiceNode:
    def __init__(self):
        self.service_name = rospy.get_param("~service_name", "~generate_clearance_heatmap")
        self.robot_radius = float(rospy.get_param("~robot_radius", 0.05))
        self.safety_margin = float(rospy.get_param("~safety_margin", 0.02))
        self.clearance_clip = float(rospy.get_param("~clearance_clip", 0.40))
        self.default_resolution = float(rospy.get_param("~default_resolution", 0.05))
        self.default_xy_margin = float(rospy.get_param("~default_xy_margin", 0.03))
        self.default_z_min_offset = float(rospy.get_param("~default_z_min_offset", -0.02))
        self.default_z_max_height = float(rospy.get_param("~default_z_max_height", 0.60))

        self.service = rospy.Service(self.service_name, GenerateClearanceHeatmap, self.handle_generate_clearance_heatmap)

        backend_name = "FIESTA-style AABB clearance"
        rospy.loginfo("GenerateClearanceHeatmap is using the %s backend.", backend_name)

        resolved_service_name = rospy.resolve_name(self.service_name)
        rospy.loginfo(
            "GenerateClearanceHeatmap service ready at %s (robot_radius=%.3f, safety_margin=%.3f, clip=%.3f).",
            resolved_service_name,
            self.robot_radius,
            self.safety_margin,
            self.clearance_clip,
        )

    def _resolve_request_bounds(self, request):
        requested_min = point_msg_to_array(request.min_bound)
        requested_max = point_msg_to_array(request.max_bound)
        if np.all(requested_max > requested_min):
            return requested_min, requested_max, False

        inferred_min, inferred_max = compute_workspace_bounds_from_env_info(
            env_info=request.env_info,
            xy_margin=self.default_xy_margin,
            z_min_offset=self.default_z_min_offset,
            z_max_height=self.default_z_max_height,
        )
        return inferred_min, inferred_max, True

    def handle_generate_clearance_heatmap(self, request):
        response = GenerateClearanceHeatmapResponse()
        started_at = time.time()

        try:
            resolution = float(request.resolution) if request.resolution > 1e-6 else self.default_resolution
            min_bound, max_bound, used_auto_bounds = self._resolve_request_bounds(request)

            centers = np.asarray(
                [[box.center.x, box.center.y, box.center.z] for box in request.env_info.obstacles],
                dtype=np.float32,
            ).reshape(-1, 3) if request.env_info.obstacles else np.zeros((0, 3), dtype=np.float32)
            sizes = np.asarray(
                [[box.size.x, box.size.y, box.size.z] for box in request.env_info.obstacles],
                dtype=np.float32,
            ).reshape(-1, 3) if request.env_info.obstacles else np.zeros((0, 3), dtype=np.float32)

            xs, ys, zs, heatmap_zyx = build_clearance_heatmap(
                centers=centers,
                sizes=sizes,
                resolution=resolution,
                min_bound=min_bound,
                max_bound=max_bound,
                robot_radius=self.robot_radius,
                safety_margin=self.safety_margin,
                clearance_clip=self.clearance_clip,
            )

            response.heatmap_data = heatmap_zyx.reshape(-1).tolist()
            response.dim_x = int(len(xs))
            response.dim_y = int(len(ys))
            response.dim_z = int(len(zs))
            response.success = True

            elapsed_ms = (time.time() - started_at) * 1000.0
            bounds_mode = "auto-bounds" if used_auto_bounds else "request-bounds"
            response.message = (
                f"Generated {response.dim_x}x{response.dim_y}x{response.dim_z} clearance grid "
                f"from {len(request.env_info.obstacles)} AABBs in {elapsed_ms:.1f} ms ({bounds_mode})."
            )
            rospy.loginfo("%s", response.message)
            return response
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.dim_x = 0
            response.dim_y = 0
            response.dim_z = 0
            response.heatmap_data = []
            rospy.logerr("GenerateClearanceHeatmap failed: %s", exc)
            return response


def main():
    rospy.init_node("clearance_heatmap_server", anonymous=False)
    ClearanceHeatmapServiceNode()
    rospy.spin()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
