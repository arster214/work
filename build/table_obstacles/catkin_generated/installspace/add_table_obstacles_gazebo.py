#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
同时在 Gazebo 和 MoveIt 规划场景中添加障碍物
"""
import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel
import os

class ObstacleManager:
    def __init__(self):
        rospy.init_node("add_table_obstacles_gazebo")
        
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
    
    def generate_sphere_sdf(self, name, radius, color="1 0 0 1"):
        """生成球体的 SDF 模型"""
        sdf = f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <sphere>
            <radius>{radius}</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <sphere>
            <radius>{radius}</radius>
          </sphere>
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
    
    def spawn_sphere_in_gazebo(self, name, pose, radius, color="1 0 0 1"):
        """在 Gazebo 中生成球体"""
        try:
            sdf = self.generate_sphere_sdf(name, radius, color)
            self.spawn_model(name, sdf, "", pose, "world")
            rospy.loginfo(f"在 Gazebo 中生成球体: {name}")
        except rospy.ServiceException as e:
            rospy.logwarn(f"生成 {name} 失败: {e}")
    
    def add_box_to_moveit(self, name, pose_stamped, size):
        """在 MoveIt 规划场景中添加盒子"""
        self.scene.add_box(name, pose_stamped, size=size)
        rospy.sleep(0.1)
    
    def add_sphere_to_moveit(self, name, pose_stamped, radius):
        """在 MoveIt 规划场景中添加球体"""
        self.scene.add_sphere(name, pose_stamped, radius=radius)
        rospy.sleep(0.1)
    
    def add_obstacles(self):
        """添加所有障碍物"""
        rospy.loginfo("开始添加障碍物...")
        
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
        pillar_height = 1.5
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
        
        # ----------------- 中点连杆 -----------------
        rod_thickness = 0.05
        rod_height = 1.42
        
        rods = [
            {"name": "rod1", "pos": ( half_length, 0.0 -0.9, rod_height), "size": (rod_thickness, 0.8, rod_thickness)},
            {"name": "rod2", "pos": (-half_length, 0.0 -0.9, rod_height), "size": (rod_thickness, 0.8, rod_thickness)},
            {"name": "rod3", "pos": (0.0,  half_width -0.9, rod_height), "size": (1.2, rod_thickness, rod_thickness)},
            {"name": "rod4", "pos": (0.0, -half_width -0.9, rod_height), "size": (1.2, rod_thickness, rod_thickness)},
        ]
        
        for rod in rods:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link_underpan"
            pose_stamped.pose.position.x = rod["pos"][0]
            pose_stamped.pose.position.y = rod["pos"][1]
            pose_stamped.pose.position.z = rod["pos"][2]
            pose_stamped.pose.orientation.w = 1.0
            
            pose_gazebo = Pose()
            pose_gazebo.position.x = rod["pos"][0]
            pose_gazebo.position.y = rod["pos"][1]
            pose_gazebo.position.z = rod["pos"][2] + self.base_height
            pose_gazebo.orientation.w = 1.0
            
            self.spawn_box_in_gazebo(rod["name"], pose_gazebo, rod["size"], "0.3 0.3 0.3 1")
            self.add_box_to_moveit(rod["name"], pose_stamped, rod["size"])
        
        # ----------------- 框架内小球 -----------------
        spheres = [
            {"name": "ball1", "pos": ( 0.1, -0.7, 1.46), "radius": 0.05},
            {"name": "ball3", "pos": ( 0, -0.89, 1.4), "radius": 0.05},
            {"name": "ball4", "pos": (-0.2, -0.78, 1.35), "radius": 0.05},
        ]
        
        for ball in spheres:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link_underpan"
            pose_stamped.pose.position.x = ball["pos"][0]
            pose_stamped.pose.position.y = ball["pos"][1]
            pose_stamped.pose.position.z = ball["pos"][2]
            pose_stamped.pose.orientation.w = 1.0
            
            pose_gazebo = Pose()
            pose_gazebo.position.x = ball["pos"][0]
            pose_gazebo.position.y = ball["pos"][1]
            pose_gazebo.position.z = ball["pos"][2] + self.base_height
            pose_gazebo.orientation.w = 1.0
            
            self.spawn_sphere_in_gazebo(ball["name"], pose_gazebo, ball["radius"], "1 0 0 1")
            self.add_sphere_to_moveit(ball["name"], pose_stamped, ball["radius"])
        
        rospy.loginfo("所有障碍物已添加到 Gazebo 和 MoveIt 规划场景")

def main():
    try:
        manager = ObstacleManager()
        manager.add_obstacles()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
