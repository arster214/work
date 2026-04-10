#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def add_simple_obstacles():
    rospy.init_node("add_simple_obstacles")
    
    rospy.loginfo("等待 MoveIt 规划场景接口初始化...")
    rospy.sleep(3)  # 额外等待确保 MoveIt 完全启动

    scene = PlanningSceneInterface()
    rospy.sleep(3)  # 等待场景接口初始化
    
    rospy.loginfo("开始添加简单障碍物到规划场景...")

    # ----------------- 桌子（桌面） -----------------
    table_pose = PoseStamped()
    table_pose.header.frame_id = "base_link_underpan"
    table_pose.pose.position.x = 0
    table_pose.pose.position.y = -0.9
    table_pose.pose.position.z = 0.85
    table_pose.pose.orientation.w = 1.0
    scene.add_box("table", table_pose, size=(1.2, 0.8, 0.7))
    rospy.sleep(0.2)
    
    # ----------------- 桌角柱子（桌腿） -----------------
    half_length = 1.2 / 2
    half_width = 0.8 / 2
    pillar_height = 1.2
    pillar_size = (0.05, 0.05, pillar_height)
    
    pillars = [
        {"name": "pillar1", "pos": ( half_length,  half_width -0.9, pillar_height/2)},
        {"name": "pillar2", "pos": ( half_length, -half_width -0.9, pillar_height/2)},
        {"name": "pillar3", "pos": (-half_length,  half_width -0.9, pillar_height/2)},
        {"name": "pillar4", "pos": (-half_length, -half_width -0.9, pillar_height/2)},
    ]
    
    for pillar in pillars:
        pose = PoseStamped()
        pose.header.frame_id = "base_link_underpan"
        pose.pose.position.x = pillar["pos"][0]
        pose.pose.position.y = pillar["pos"][1]
        pose.pose.position.z = pillar["pos"][2]
        pose.pose.orientation.w = 1.0
        scene.add_box(pillar["name"], pose, size=pillar_size)
        rospy.sleep(0.2)

    # ----------------- 桌面上两个方块 -----------------
    box_size = (0.2, 0.2, 0.3) # 方块大小 10cm 立方体
    table_top_z = 0.85 + 0.7/2.0 + box_size[2]/2.0 # 桌面中心z + 桌面厚度的一半 + 方块高度的一半
    
    boxes = [
        {"name": "box1", "pos": ( 0.3, -0.8, table_top_z)},
        {"name": "box2", "pos": (-0.3, -0.8, table_top_z)},
    ]
    
    for box in boxes:
        pose = PoseStamped()
        pose.header.frame_id = "base_link_underpan"
        pose.pose.position.x = box["pos"][0]
        pose.pose.position.y = box["pos"][1]
        pose.pose.position.z = box["pos"][2]
        pose.pose.orientation.w = 1.0
        scene.add_box(box["name"], pose, size=box_size)
        rospy.sleep(0.2)
        
    rospy.loginfo("桌面、桌角柱子以及两个方块已添加到规划场景")
    rospy.spin()

if __name__ == "__main__":
    add_simple_obstacles()
