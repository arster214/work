#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import numpy as np
import rospy
from clearance_heatmap_msgs.msg import BoundingBox, EnvironmentInfo
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from shape_msgs.msg import SolidPrimitive


def quaternion_to_rotation_matrix(x, y, z, w):
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.asarray([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def pose_to_transform(pose_msg):
    translation = np.asarray(
        [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z],
        dtype=np.float64,
    )
    rotation = quaternion_to_rotation_matrix(
        pose_msg.orientation.x,
        pose_msg.orientation.y,
        pose_msg.orientation.z,
        pose_msg.orientation.w,
    )
    return rotation, translation


def transform_points(points_xyz, pose_msg):
    if points_xyz.size == 0:
        return points_xyz

    rotation, translation = pose_to_transform(pose_msg)
    return points_xyz.dot(rotation.T) + translation


def build_box_corners(size_x, size_y, size_z):
    hx = 0.5 * float(size_x)
    hy = 0.5 * float(size_y)
    hz = 0.5 * float(size_z)
    return np.asarray([
        [-hx, -hy, -hz],
        [-hx, -hy, hz],
        [-hx, hy, -hz],
        [-hx, hy, hz],
        [hx, -hy, -hz],
        [hx, -hy, hz],
        [hx, hy, -hz],
        [hx, hy, hz],
    ], dtype=np.float64)


def primitive_to_local_points(primitive_msg):
    if primitive_msg.type == SolidPrimitive.BOX:
        if len(primitive_msg.dimensions) < 3:
            return np.zeros((0, 3), dtype=np.float64)
        return build_box_corners(
            primitive_msg.dimensions[SolidPrimitive.BOX_X],
            primitive_msg.dimensions[SolidPrimitive.BOX_Y],
            primitive_msg.dimensions[SolidPrimitive.BOX_Z],
        )

    if primitive_msg.type == SolidPrimitive.SPHERE:
        if len(primitive_msg.dimensions) < 1:
            return np.zeros((0, 3), dtype=np.float64)
        radius = float(primitive_msg.dimensions[SolidPrimitive.SPHERE_RADIUS])
        return build_box_corners(2.0 * radius, 2.0 * radius, 2.0 * radius)

    if primitive_msg.type == SolidPrimitive.CYLINDER:
        if len(primitive_msg.dimensions) < 2:
            return np.zeros((0, 3), dtype=np.float64)
        height = float(primitive_msg.dimensions[SolidPrimitive.CYLINDER_HEIGHT])
        radius = float(primitive_msg.dimensions[SolidPrimitive.CYLINDER_RADIUS])
        return build_box_corners(2.0 * radius, 2.0 * radius, height)

    if primitive_msg.type == SolidPrimitive.CONE:
        if len(primitive_msg.dimensions) < 2:
            return np.zeros((0, 3), dtype=np.float64)
        height = float(primitive_msg.dimensions[SolidPrimitive.CONE_HEIGHT])
        radius = float(primitive_msg.dimensions[SolidPrimitive.CONE_RADIUS])
        return build_box_corners(2.0 * radius, 2.0 * radius, height)

    return np.zeros((0, 3), dtype=np.float64)


def mesh_to_local_points(mesh_msg):
    if not mesh_msg.vertices:
        return np.zeros((0, 3), dtype=np.float64)

    return np.asarray(
        [[vertex.x, vertex.y, vertex.z] for vertex in mesh_msg.vertices],
        dtype=np.float64,
    ).reshape(-1, 3)


def points_to_aabb(points_xyz):
    if points_xyz.size == 0:
        return None

    min_xyz = points_xyz.min(axis=0)
    max_xyz = points_xyz.max(axis=0)
    center = 0.5 * (min_xyz + max_xyz)
    size = max_xyz - min_xyz
    return center, size, min_xyz, max_xyz


class PlanningSceneEnvironmentPublisher:
    def __init__(self):
        self.environment_topic = rospy.get_param("~environment_topic", "/map_process/environment_info")
        self.service_name = rospy.get_param("~planning_scene_service", "/get_planning_scene")
        self.publish_rate = float(rospy.get_param("~publish_rate", 1.0))
        self.fixed_table_z = float(rospy.get_param("~fixed_table_z", float("nan")))
        self.table_object_names = set(rospy.get_param("~table_object_names", ["table"]))
        self.exclude_object_names = set(rospy.get_param(
            "~exclude_object_names",
            ["table", "ground", "ground_plane", "floor"],
        ))

        self.publisher = rospy.Publisher(self.environment_topic, EnvironmentInfo, queue_size=1, latch=True)

        rospy.loginfo("Waiting for planning scene service: %s", self.service_name)
        rospy.wait_for_service(self.service_name)
        self.scene_client = rospy.ServiceProxy(self.service_name, GetPlanningScene)

        self.last_signature = None

    def fetch_planning_scene(self):
        request = GetPlanningSceneRequest()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES |
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        return self.scene_client(request).scene

    def object_geometry_to_aabbs(self, collision_object):
        aabbs = []

        for primitive_msg, pose_msg in zip(collision_object.primitives, collision_object.primitive_poses):
            local_points = primitive_to_local_points(primitive_msg)
            world_points = transform_points(local_points, pose_msg)
            aabb = points_to_aabb(world_points)
            if aabb is not None:
                aabbs.append(aabb)

        for mesh_msg, pose_msg in zip(collision_object.meshes, collision_object.mesh_poses):
            local_points = mesh_to_local_points(mesh_msg)
            world_points = transform_points(local_points, pose_msg)
            aabb = points_to_aabb(world_points)
            if aabb is not None:
                aabbs.append(aabb)

        return aabbs

    def publish_once(self):
        scene = self.fetch_planning_scene()
        env_msg = EnvironmentInfo()

        if math.isfinite(self.fixed_table_z):
            env_msg.table_z = self.fixed_table_z

        obstacle_count = 0
        signature_parts = []

        for collision_object in scene.world.collision_objects:
            object_aabbs = self.object_geometry_to_aabbs(collision_object)
            if not object_aabbs:
                continue

            if collision_object.id in self.table_object_names and not math.isfinite(env_msg.table_z):
                env_msg.table_z = max(aabb[3][2] for aabb in object_aabbs)

            if collision_object.id in self.exclude_object_names:
                continue

            for center, size, _, _ in object_aabbs:
                box_msg = BoundingBox()
                box_msg.center.x = float(center[0])
                box_msg.center.y = float(center[1])
                box_msg.center.z = float(center[2])
                box_msg.size.x = float(size[0])
                box_msg.size.y = float(size[1])
                box_msg.size.z = float(size[2])
                env_msg.obstacles.append(box_msg)
                obstacle_count += 1
                signature_parts.append(
                    (
                        collision_object.id,
                        round(float(center[0]), 4),
                        round(float(center[1]), 4),
                        round(float(center[2]), 4),
                        round(float(size[0]), 4),
                        round(float(size[1]), 4),
                        round(float(size[2]), 4),
                    )
                )

        signature = (round(float(env_msg.table_z), 4), tuple(signature_parts))
        self.publisher.publish(env_msg)

        if signature != self.last_signature:
            self.last_signature = signature
            rospy.loginfo(
                "Published EnvironmentInfo with %d AABBs to %s (table_z=%.4f).",
                obstacle_count,
                self.environment_topic,
                float(env_msg.table_z),
            )

    def spin(self):
        rate = rospy.Rate(max(self.publish_rate, 0.2))
        while not rospy.is_shutdown():
            try:
                self.publish_once()
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(2.0, "Failed to fetch planning scene from %s: %s", self.service_name, exc)
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "Failed to publish EnvironmentInfo: %s", exc)
            rate.sleep()


def main():
    rospy.init_node("publish_planning_scene_environment", anonymous=False)
    PlanningSceneEnvironmentPublisher().spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
