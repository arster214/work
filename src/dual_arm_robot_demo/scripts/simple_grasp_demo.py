#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
单个抓取流程验证脚本 (7自由度适配版)
流程：
1. 切换到Base坐标系
2. 移动到拍照观测位 (detect_catch)
3. 等待视觉识别水瓶位姿
4. 规划并执行抓取 (Approaching -> Grasping -> Lifting)
"""

import numpy as np
import rospy
import yaml
import os
import tf2_ros
from tf2_geometry_msgs import PointStamped

from dual_arm_msgs.msg import GetArmState_Command, Arm_Current_State, ArmState, MoveJ, MoveJ_P, Plan_State, ChangeWorkFrame_Name, Gripper_Pick, Gripper_Set
from dual_arm_robot_demo.msg import ObjectPose

# 全局变量
ARM_DOF = 7
run_state = False
curret_pose2 = None  # 含位置和四元数
curret_pose3 = None  # 含欧拉角
buffer = None        # TF Buffer

def load_config(config_path):
    with open(config_path, 'r') as stream:
        config = yaml.safe_load(stream)
    return config

def convert(x, y, z):
    """
    将相机坐标系下的点转换到机械臂基坐标系 (Base)
    依赖全局 buffer 对象
    """
    point_source = PointStamped()
    point_source.header.frame_id = "camera"
    point_source.header.stamp = rospy.Time()
    point_source.point.x = x
    point_source.point.y = y
    point_source.point.z = z

    try:
        # 等待变换关系可用，超时时间1秒
        point_target = buffer.transform(point_source, "base", rospy.Duration(1.0))
        return point_target.point.x, point_target.point.y, point_target.point.z
    except Exception as e:
        rospy.logerr(f"TF Transform Failed: {e}")
        return None, None, None

def plan_state_callback(msg):
    """机械臂运动执行状态回调"""
    global run_state
    if msg.state:
        run_state = True
        rospy.loginfo("******* Plan State OK (Motion Completed)")
    else:
        run_state = False
        rospy.logerr("******* Plan State Fail! (Planner rejected the command)")

def joint_position_callback(msg):
    """获取欧拉角 pose state: [x, y, z, rx, ry, rz]"""
    global curret_pose3
    curret_pose3 = msg

def get_arm_state_callback2(msg):
    """获取位置、四元数及关节角"""
    global curret_pose2
    curret_pose2 = msg
    # 增加错误码监测
    if msg.arm_err != 0 or msg.sys_err != 0:
        rospy.logwarn_throttle(2.0, f"Robot Error detected! Arm Err: {msg.arm_err}, Sys Err: {msg.sys_err}")

def is_arrive():
    """阻塞等待机械臂动作完成"""
    global run_state
    rospy.loginfo("Waiting for motion to complete...")
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        if run_state:
            run_state = False
            break
    rospy.loginfo("Motion completed.")

def change_work_frame():
    """切换到基坐标系控制"""
    name = ChangeWorkFrame_Name()
    name.WorkFrame_name = "Base"
    change_work_frame_pub.publish(name)
    rospy.loginfo("Switched to Base frame.")

def get_object_info():
    """阻塞等待视觉识别结果"""
    rospy.loginfo("Waiting for object detection (topic: /object_pose_bottle)...")
    object_pose = rospy.wait_for_message("object_pose_bottle", ObjectPose, timeout=None)
    rospy.loginfo(f"Detected object at Camera Frame: x={object_pose.x}, y={object_pose.y}, z={object_pose.z}")
    return object_pose.x, object_pose.y, object_pose.z

# --- 数学辅助函数 ---

def euler_angles_to_rotation_matrix(rx, ry, rz):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx

def normalize_quaternion(q_list):
    """归一化四元数 [x, y, z, w]"""
    q = np.array(q_list)
    norm = np.linalg.norm(q)
    if norm == 0:
        return q_list
    return (q / norm).tolist()

def pose_to_homogeneous_matrix(pose):
    x, y, z, rx, ry, rz = pose
    R = euler_angles_to_rotation_matrix(rx, ry, rz)
    t = np.array([x, y, z]).reshape(3, 1)
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = t[:, 0]
    return H

def change_pose(pose, num, axis):
    """根据当前末端位姿，沿自身坐标轴偏移"""
    matrix = pose_to_homogeneous_matrix(pose)
    if axis == "z":
        offset = np.array([0, 0, num])
    elif axis == "y":
        offset = np.array([0, num, 0])
    else:
        offset = np.array([num, 0, 0])
    
    offset_homo = np.append(offset, [1])
    new_pos = matrix.dot(offset_homo)[:3]
    # 返回 [x, y, z, rx, ry, rz]
    return list(new_pos) + pose[3:]

# --- 核心动作逻辑 ---

def perform_joint_move(joint_values, speed):
    """执行关节空间移动"""
    move_cmd = MoveJ()
    move_cmd.joint = joint_values
    move_cmd.speed = speed
    pub_movej.publish(move_cmd)
    is_arrive()

def single_grasp_task(config):
    # 空间1 (观测位/预备位)
    joint_space_1 = [-2.312735, -1.244551, 1.420360, -0.201320, 2.725690, 1.095005, 4.694783]
    # 空间2 (夹取位 - 已经是最终抓取位置)
    joint_space_2 = [-2.202225, -1.361746, 1.420046, -0.209993, 2.715744, 0.858086, 4.694713]
    # 空间3 (撤离位/放置位)
    joint_space_3 = [-2.381279, -0.837164, 1.486862, -0.020416, 2.760450, 1.571564, 4.826007]
    
    speed = config['SPEED']

    # 1. 初始化：切坐标系 & 打开夹爪
    change_work_frame()
    
    rospy.loginfo("Initializing gripper (Open)...")
    gripper_set_msg = Gripper_Set()
    gripper_set_msg.position = 1000  # 1000 = Full Open
    pub_grippers_posi.publish(gripper_set_msg)
    rospy.sleep(1.0)

    # 2. 移动到空间1 (Pre-Position)
    rospy.loginfo("Moving to Joint Space 1 (Ready Position)...")
    perform_joint_move(joint_space_1, speed)
    rospy.sleep(1.0) 

    # 3. 移动到空间2 (Grasp Position - 直接到位)
    rospy.loginfo("Moving to Joint Space 2 (Grasping Position)...")
    perform_joint_move(joint_space_2, speed)

    # 4. Grasp Action (直接夹取)
    rospy.loginfo("Closing Gripper...")
    pick_msg = Gripper_Pick()
    pick_msg.speed = 500
    pick_msg.force = 500
    pub_grippers.publish(pick_msg)
    rospy.sleep(2.0) # 等待夹紧

    # 5. Move to Space 3
    rospy.loginfo("Moving to Joint Space 3 (Lift/Drop)...")
    perform_joint_move(joint_space_3, speed)
    
    rospy.loginfo("Task Complete!")


if __name__ == "__main__":
    rospy.init_node("simple_grasp_demo", anonymous=True)
    
    # 载入配置
    current_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(current_dir, 'config.yaml')
    if not os.path.exists(config_path):
        rospy.logerr(f"Config file not found at {config_path}")
        exit(1)
    config = load_config(config_path)
    
    ARM_DOF = rospy.get_param("~arm_dof", 7)

    # 初始化TF
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    # 初始化 Publishers
    pub_get_pose = rospy.Publisher("/r_arm/rm_driver/GetArmState_Cmd", GetArmState_Command, queue_size=10)
    pub_movej = rospy.Publisher("/r_arm/rm_driver/MoveJ_Cmd", MoveJ, queue_size=10)
    pub_to_pose = rospy.Publisher("/r_arm/rm_driver/MoveJ_P_Cmd", MoveJ_P, queue_size=10)
    change_work_frame_pub = rospy.Publisher("/r_arm/rm_driver/ChangeWorkFrame_Cmd", ChangeWorkFrame_Name, queue_size=10)
    pub_grippers = rospy.Publisher('/r_arm/rm_driver/Gripper_Pick', Gripper_Pick, queue_size=10)
    pub_grippers_posi = rospy.Publisher('/r_arm/rm_driver/Gripper_Set', Gripper_Set, queue_size=10)

    # 初始化 Subscribers
    rospy.Subscriber("/r_arm/rm_driver/ArmCurrentState", ArmState, get_arm_state_callback2, queue_size=10)
    rospy.Subscriber("/r_arm/rm_driver/Plan_State", Plan_State, plan_state_callback, queue_size=10)
    rospy.Subscriber("/r_arm/rm_driver/Arm_Current_State", Arm_Current_State, joint_position_callback, queue_size=10)

    rospy.sleep(1.0) # 等待连接

    try:
        single_grasp_task(config)
    except rospy.ROSInterruptException:
        pass
