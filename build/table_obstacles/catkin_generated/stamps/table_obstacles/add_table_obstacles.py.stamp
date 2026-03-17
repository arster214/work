#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def add_table_with_frame():
    rospy.init_node("add_table_frame_obstacles")
    
    rospy.loginfo("等待 MoveIt 规划场景接口初始化...")
    rospy.sleep(3)  # 额外等待确保 MoveIt 完全启动

    scene = PlanningSceneInterface()
    rospy.sleep(3)  # 等待场景接口初始化
    
    rospy.loginfo("开始添加障碍物到规划场景...")

    # ----------------- 桌子 -----------------
    table_pose = PoseStamped()
    table_pose.header.frame_id = "base_link_underpan"
    table_pose.pose.position.x = 0
    table_pose.pose.position.y = -0.9
    table_pose.pose.position.z = 0.35 + 0.5  # 桌子高度提升 0.5 m -> 中心在 0.85
    table_pose.pose.orientation.w = 1.0
    scene.add_box("table", table_pose, size=(1.2, 0.8, 0.7))

    
    # ----------------- 中点连杆 -----------------
    rod_thickness = 0.05
    rod_height = pillar_height  # 连杆高度与柱子顶部平齐
    mid_height = rod_height

    rods = [
        {"name": "rod1", "pos": ( half_length, 0.0 -0.9, mid_height), "size": (rod_thickness, 0.8, rod_thickness)},
        {"name": "rod2", "pos": (-half_length, 0.0 -0.9, mid_height), "size": (rod_thickness, 0.8, rod_thickness)},
        {"name": "rod3", "pos": (0.0,  half_width -0.9, mid_height), "size": (1.2, rod_thickness, rod_thickness)},
        {"name": "rod4", "pos": (0.0, -half_width -0.9, mid_height), "size": (1.2, rod_thickness, rod_thickness)},
    ]

    for rod in rods:
        pose = PoseStamped()
        pose.header.frame_id = "base_link_underpan"
        pose.pose.position.x = rod["pos"][0]
        pose.pose.position.y = rod["pos"][1]
        pose.pose.position.z = rod["pos"][2]
        pose.pose.orientation.w = 1.0
        scene.add_box(rod["name"], pose, size=rod["size"])
        rospy.sleep(0.2)

    # ----------------- 框架内小球 -----------------
    # 小球数量 4 个，高度整体提升 0.3 m
    spheres = [
        {"name": "ball1", "pos": ( 0.4, -0.75, 1.3 + 0.3), "radius": 0.05},
        {"name": "ball2", "pos": (-0.35, -0.7, 1.4 + 0.3), "radius": 0.05},
        {"name": "ball3", "pos": ( 0.1, -0.72, 1.35 + 0.3), "radius": 0.05},
        {"name": "ball4", "pos": (-0.2, -0.78, 1.33 + 0.3), "radius": 0.05},
    ]

    for ball in spheres:
        pose = PoseStamped()
        pose.header.frame_id = "base_link_underpan"
        pose.pose.position.x = ball["pos"][0]
        pose.pose.position.y = ball["pos"][1]
        pose.pose.position.z = ball["pos"][2]
        pose.pose.orientation.w = 1.0
        scene.add_sphere(ball["name"], pose, radius=ball["radius"])
        rospy.sleep(0.3)  # 等 MoveIt 刷新

    rospy.loginfo("桌子、柱子、连杆和小球已添加到规划场景")
    rospy.spin()

if __name__ == "__main__":
    add_table_with_frame()
