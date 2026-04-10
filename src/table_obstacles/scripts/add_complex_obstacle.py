#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from clearance_heatmap_msgs.msg import BoundingBox, EnvironmentInfo


def publish_environment_info(obstacle_boxes, table_z, topic):
    pub = rospy.Publisher(topic, EnvironmentInfo, queue_size=1, latch=True)
    rospy.sleep(0.5)

    env = EnvironmentInfo()
    env.table_z = float(table_z)

    for center, size in obstacle_boxes:
        box = BoundingBox()
        box.center.x = float(center[0])
        box.center.y = float(center[1])
        box.center.z = float(center[2])
        box.size.x = float(size[0])
        box.size.y = float(size[1])
        box.size.z = float(size[2])
        env.obstacles.append(box)

    pub.publish(env)
    rospy.loginfo("Published %d virtual AABBs to %s for heatmap generation.", len(env.obstacles), topic)


def add_complex_obstacles():
    rospy.init_node("add_complex_obstacles")
    environment_topic = rospy.get_param("~environment_topic", "/map_process/environment_info")
    publish_environment = rospy.get_param("~publish_environment_info", True)
    
    rospy.loginfo("等待 MoveIt 规划场景接口初始化...")
    rospy.sleep(3)  # 额外等待确保 MoveIt 完全启动

    scene = PlanningSceneInterface()
    rospy.sleep(3)  # 等待场景接口初始化
    
    rospy.loginfo("开始添加复杂障碍物到规划场景...")
    heatmap_boxes = []

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
    box_size = (0.15, 0.15, 0.3) # 方块大小 10cm 立方体
    table_top_z = 0.85 + 0.7/2.0 + box_size[2]/2.0 # 桌面中心z + 桌面厚度的一半 + 方块高度的一半
    table_surface_z = 0.85 + 0.7 / 2.0
    
    boxes = [
        {"name": "box1", "pos": ( 0.2, -0.85, table_top_z)},
        {"name": "box2", "pos": (-0.2, -0.85, table_top_z)},
    ]
    
    for box in boxes:
        pose = PoseStamped()
        pose.header.frame_id = "base_link_underpan"
        pose.pose.position.x = box["pos"][0]
        pose.pose.position.y = box["pos"][1]
        pose.pose.position.z = box["pos"][2]
        pose.pose.orientation.w = 1.0
        scene.add_box(box["name"], pose, size=box_size)
        heatmap_boxes.append((box["pos"], box_size))
        rospy.sleep(0.2)
        
    # ----------------- 方块上方再堆叠一个小方块 -----------------
    little_box_size = (0.1, 0.1, 0.1) # 小方块大小
    little_box_z = table_top_z + box_size[2]/2.0 + little_box_size[2]/2.0
    
    little_boxes = [
        {"name": "box3", "pos": ( 0.15, -0.85, little_box_z)},
        {"name": "box4", "pos": (-0.15, -0.85, little_box_z)},
    ]
    
    for box in little_boxes:
        pose = PoseStamped()
        pose.header.frame_id = "base_link_underpan"
        pose.pose.position.x = box["pos"][0]
        pose.pose.position.y = box["pos"][1]
        pose.pose.position.z = box["pos"][2]
        pose.pose.orientation.w = 1.0
        scene.add_box(box["name"], pose, size=little_box_size)
        heatmap_boxes.append((box["pos"], little_box_size))
        rospy.sleep(0.2)
        
    # ----------------- 两侧各加一个小方块 -----------------
    small_box_size = (0.1, 0.1, 0.2) # 小方块大小
    side_box_z = 0.85 + 0.7/2.0 + small_box_size[2]/2.0 # 在桌面上
    side_boxes = [
        {"name": "box5", "pos": ( 0.55, -0.85, side_box_z)},
        {"name": "box6", "pos": (-0.55, -0.85, side_box_z)},
    ]
    
    for box in side_boxes:
        pose = PoseStamped()
        pose.header.frame_id = "base_link_underpan"
        pose.pose.position.x = box["pos"][0]
        pose.pose.position.y = box["pos"][1]
        pose.pose.position.z = box["pos"][2]
        pose.pose.orientation.w = 1.0
        scene.add_box(box["name"], pose, size=small_box_size)
        heatmap_boxes.append((box["pos"], small_box_size))
        rospy.sleep(0.2)

    if publish_environment:
        publish_environment_info(heatmap_boxes, table_surface_z, environment_topic)
        
    rospy.loginfo("桌面、桌角柱子以及六个方块已添加到规划场景")
    rospy.spin()

if __name__ == "__main__":
    add_complex_obstacles()
