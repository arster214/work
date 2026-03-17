#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import json
import glob
import time
import rospkg
import subprocess
import datetime
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from dual_arm_msgs.msg import MoveJ

class DualArmPlayback:
    def __init__(self):
        rospy.init_node('dual_arm_playback', anonymous=True)
        
        # Using the same command topics as in simple_grasp_demo.py
        # Assuming left arm prefix is l_arm and right arm is r_arm
        # Topic names inferred as: /<ns>/rm_driver/MoveJ_Cmd
        self.pub_l_movej = rospy.Publisher('/l_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
        self.pub_r_movej = rospy.Publisher('/r_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
        
        # 控制建图使能
        self.pub_enable = rospy.Publisher('/enable_point_process', Bool, queue_size=10, latch=True)

    def get_latest_pose_file(self):
        # 尝试多个路径寻找 pose 文件夹
        search_paths = [
            os.path.join(os.getcwd(), "pose"),  # 当前运行目录 (rosrun)
            os.path.join(os.environ['HOME'], "catkin_wsfrj/pose"), # 明确的工作区路径
            os.path.join(rospkg.RosPack().get_path('map_build'), "pose") #如果在包目录下
        ]

        pose_dir = None
        for p in search_paths:
            if os.path.exists(p):
                pose_dir = p
                rospy.loginfo(f"Found pose directory at: {pose_dir}")
                break
        
        if not pose_dir:
            rospy.logerr(f"Pose directory not found. Searched: {search_paths}")
            return None

        
        # Get list of all jsonl files
        files = glob.glob(os.path.join(pose_dir, "*.jsonl"))
        if not files:
            rospy.logerr("No pose files found.")
            return None
            
        # Sort by modification time (or filename if YYYYMMDDxx format) to get the latest
        # Since filename is strictly YYYYMMDDxx, alphabetical sort is also chronological
        files.sort()
        return files[-1]

    def save_octomap(self):
        rospy.loginfo("Auto-saving Octomap after sequence finished...")
        
        # 强制指定保存路径到 workspace 下的 map 文件夹
        # 使用 os.getcwd() 在 roslaunch 下通常是 ~/.ros，这会导致找不到文件
        # 我们这里使用固定路径或者相对 HOME 的路径
        home_dir = os.environ['HOME']
        workspace_dir = os.path.join(home_dir, "catkin_wsfrj")
        map_dir = os.path.join(workspace_dir, "map")
        
        if not os.path.exists(map_dir):
            os.makedirs(map_dir)
            rospy.loginfo(f"Created map directory: {map_dir}")

        # Filename logic
        today_str = datetime.datetime.now().strftime("%Y%m%d")
        max_id = 0
        if os.path.exists(map_dir):
            files = [f for f in os.listdir(map_dir) if f.startswith(today_str) and f.endswith(".bt")]
            for f in files:
                try:
                    base_name = os.path.splitext(f)[0]
                    num_part = base_name[len(today_str):]
                    if num_part.isdigit():
                        max_id = max(max_id, int(num_part))
                except Exception:
                    continue
        
        next_id = max_id + 1
        filename = f"{today_str}{next_id:02d}.bt"
        full_path = os.path.join(map_dir, filename)
        
        rospy.loginfo(f"Saving map to: {full_path}")
        
        # Command: rosrun octomap_server octomap_saver <filename> <topic_remap>
        # octomap_saver 需要订阅 Octomap 格式的消息 (.bt/.ot)，而不是 MarkerArray (可视化)
        # "/occupied_cells_vis_array" 是 MarkerArray，仅供 RViz 显示，无法保存为地图文件
        # 真正的数据话题是 "/octomap_binary"，它内容与 RViz 显示的一致
        # octomap_saver 默认订阅 "octomap_binary" (或 full)，我们需要重映射到我们新暴露的服务
        # 在 octomap_filter_node.cpp 中，我们新增了 service "octomap_binary_service"
        # 因此，我们要让 octomap_saver 去调用这个服务。
        # octomap_saver 的源码逻辑是查找名为 "octomap_binary" (如果存.bt) 的 Service。
        # 所以我们把内部的 octomap_binary 重映射到 octomap_binary_service
        
        cmd = ["rosrun", "octomap_server", "octomap_saver", full_path, "octomap_binary:=/octomap_binary_service"]
        
        try:
            # Run octomap_saver synchronously
            subprocess.check_call(cmd)
            rospy.loginfo(f"Successfully saved map to {filename}")
        except subprocess.CalledProcessError as e:
             rospy.logerr(f"Failed to save map: {e}")

    def publish_pose(self, left_pos, right_pos, duration=5.0):
        # 构造 MoveJ 消息 (参考 simple_grasp_demo.py)
        # 注意: RM 驱动通常接受浮点数组作为关节角，速度 speed 为 0~100 (或其它单位，参考 demo 这里设一个默认值)
        
        # 左臂指令
        msg_l = MoveJ()
        msg_l.joint = left_pos
        msg_l.speed = 0.2 # 设置一个安全的默认速度 (0.0~1.0 或 根据驱动定义，demo里用了 config['SPEED']，这里给个保守值)
        
        # 右臂指令
        msg_r = MoveJ()
        msg_r.joint = right_pos
        msg_r.speed = 0.2

        rospy.loginfo(f"Publishing pose. Waiting {duration}s...")
        self.pub_l_movej.publish(msg_l)
        self.pub_r_movej.publish(msg_r)
        
        # 简单延时等待到位 (demo 中用了 subscriber 回调 Plan_State，这里为简化脚本直接延时)
        rospy.sleep(duration)

    def move_to_home(self):
        rospy.loginfo("Moving to HOME (all zeros)...")
        zeros = [0.0] * 7
        self.publish_pose(zeros, zeros, duration=5.0)

    def set_mapping(self, enabled):
        self.pub_enable.publish(Bool(data=enabled))
        state = "ENABLED" if enabled else "DISABLED"
        rospy.loginfo(f"Mapping {state}")

    def run(self):
        # 1. Read latest file
        latest_file = self.get_latest_pose_file()
        if not latest_file:
            return
        
        rospy.loginfo(f"Reading poses from: {latest_file}")
        
        poses = []
        try:
            with open(latest_file, 'r') as f:
                for line in f:
                    if line.strip():
                        poses.append(json.loads(line))
        except Exception as e:
            rospy.logerr(f"Failed to read file: {e}")
            return

        if not poses:
            rospy.logwarn("File is empty.")
            return

        # 2. Step 1: Zero (Mapping DISABLED)
        self.set_mapping(False)
        self.move_to_home()
        
        # Enable mapping AFTER home move is done and before playback starts
        rospy.loginfo("Reached Home. Enabling mapping for scan sequence...")
        self.set_mapping(True)
        # Give a moment for AE/AWB to settle or filters to clear if needed, though not strictly necessary
        rospy.sleep(1.0)

        # 3. Step 2: Playback (Mapping ENABLED)
        rospy.loginfo(f"Starting playback of {len(poses)} poses...")
        for i, pose in enumerate(poses):
            rospy.loginfo(f"--- Pose {i+1}/{len(poses)} ---")
            l_pos = pose['left']['position']
            r_pos = pose['right']['position']
            
            # Sanity check
            if len(l_pos) != 7 or len(r_pos) != 7:
                 rospy.logwarn(f"Pose {i+1} has incorrect joint count. Left: {len(l_pos)}, Right: {len(r_pos)}. Skipping.")
                 continue

            self.publish_pose(l_pos, r_pos, duration=8.0)

        # Disable mapping for final home move
        rospy.loginfo("Scan sequence finished. Disabling mapping for return home...")
        self.set_mapping(False)

        # 4. Step 3: Zero (Mapping DISABLED)
        self.move_to_home()
        rospy.loginfo("Playback finished. All conditions met.")
        
        # 5. Step 4: Auto-save Map
        # 此时机械臂已回到零位，且建图已停止，是保存的最佳时机
        rospy.sleep(1.0) # 等待静态
        self.save_octomap()

if __name__ == '__main__':
    try:
        player = DualArmPlayback()
        # Give some time for connections
        rospy.sleep(1.0)
        player.run()
    except rospy.ROSInterruptException:
        pass
