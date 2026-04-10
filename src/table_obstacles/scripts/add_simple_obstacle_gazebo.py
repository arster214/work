#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
同时在 Gazebo 和 MoveIt 规划场景中添加简单障碍物
"""
import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel
import os

class SimpleObstacleManager:
    def __init__(self):
        rospy.init_node("add_simple_obstacles_gazebo")
        
        rospy.loginfo("等待 Gazebo 和 MoveIt 初始化...")
        rospy.sleep(3)
        
        # MoveIt 规划场景接口
        self.scene = PlanningSceneInterface()
        rospy.sleep(2)
        
        # Gazebo 服务
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    
        self.base_height = 0.30
        
        rospy.loginfo("初始化完成")
    
    def generate_box_sdf(self, name, size, color="0.8 0.8 0.8 1"):
        """生成盒子的 SDF 模型"""
        sdf = f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <box>
            <size>{size[0]} {size[1]} {size[2]}</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>{size[0]} {size[1]} {size[2]}</size>
          </box>
        </geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        return sdf
    
    def spawn_box_in_gazebo(self, name, pose, size, color="0.8 0.8 0.8 1"):
        """在 Gazebo 中生成盒子"""
        try:
            sdf = self.generate_box_sdf(name, size, color)
            self.spawn_model(name, sdf, "", pose, "world")
            rospy.loginfo(f"在 Gazebo 中生成盒子: {name}")
        except rospy.ServiceException as e:
            rospy.logwarn(f"生成 {name} 失败: {e}")
    
    def add_box_to_moveit(self, name, pose_stamped, size):
        """在 MoveIt 规划场景中添加盒子"""
        self.scene.add_box(name, pose_stamped, size=size)
        rospy.sleep(0.1)
    
    def add_obstacles(self):
        """添加简单障碍物"""
        rospy.loginfo("开始添加简单障碍物...")
        
        # ----------------- 桌子 -----------------
        table_pose_stamped = PoseStamped()
        table_pose_stamped.header.frame_id = "base_link_underpan"
        table_pose_stamped.pose.position.x = 0
        table_pose_stamped.pose.position.y = -0.9
        table_pose_stamped.pose.position.z = 0.85
        table_pose_stamped.pose.orientation.w = 1.0
        
        table_pose_gazebo = Pose()
        table_pose_gazebo.position.x = 0
        table_pose_gazebo.position.y = -0.9
        table_pose_gazebo.position.z = 0.85 + self.base_height
        table_pose_gazebo.orientation.w = 1.0
        
        table_size = (1.2, 0.8, 0.7)
        self.spawn_box_in_gazebo("table", table_pose_gazebo, table_size, "0.6 0.4 0.2 1")
        self.add_box_to_moveit("table", table_pose_stamped, table_size)
        
        # ----------------- 桌角柱子 -----------------
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
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link_underpan"
            pose_stamped.pose.position.x = pillar["pos"][0]
            pose_stamped.pose.position.y = pillar["pos"][1]
            pose_stamped.pose.position.z = pillar["pos"][2]
            pose_stamped.pose.orientation.w = 1.0
            
            pose_gazebo = Pose()
            pose_gazebo.position.x = pillar["pos"][0]
            pose_gazebo.position.y = pillar["pos"][1]
            pose_gazebo.position.z = pillar["pos"][2] + self.base_height
            pose_gazebo.orientation.w = 1.0
            
            self.spawn_box_in_gazebo(pillar["name"], pose_gazebo, pillar_size, "0.5 0.5 0.5 1")
            self.add_box_to_moveit(pillar["name"], pose_stamped, pillar_size)
        
        # ----------------- 桌面上两个方块 -----------------
        box_size = (0.2, 0.2, 0.3) # 方块大小 10cm 立方体
        table_top_z = 0.85 + 0.7/2.0 + box_size[2]/2.0 # MoveIt 中的高度：桌面z + 桌面厚度的一半 + 方块高度一半
        
        boxes = [
            {"name": "box1", "pos": ( 0.3, -0.8, table_top_z)},
            {"name": "box2", "pos": (-0.3, -0.8, table_top_z)},
        ]
        
        for box in boxes:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link_underpan"
            pose_stamped.pose.position.x = box["pos"][0]
            pose_stamped.pose.position.y = box["pos"][1]
            pose_stamped.pose.position.z = box["pos"][2]
            pose_stamped.pose.orientation.w = 1.0
            
            pose_gazebo = Pose()
            pose_gazebo.position.x = box["pos"][0]
            pose_gazebo.position.y = box["pos"][1]
            pose_gazebo.position.z = box["pos"][2] + self.base_height
            pose_gazebo.orientation.w = 1.0
            
            self.spawn_box_in_gazebo(box["name"], pose_gazebo, box_size, "0 0 1 1") # 蓝色方块
            self.add_box_to_moveit(box["name"], pose_stamped, box_size)

        rospy.loginfo("简单障碍物（桌面、桌角以及方块）已添加到 Gazebo 和 MoveIt 规划场景")

def main():
    try:
        manager = SimpleObstacleManager()
        manager.add_obstacles()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
