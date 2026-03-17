#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
底盘移动、机械臂运动抓取逻辑
"""

import numpy as np
import rospy
import socket
import time
import yaml
import os

import tf2_ros


from dual_arm_msgs.msg import GetArmState_Command, Arm_Current_State, ArmState, MoveJ_P, MoveL, Plan_State, ChangeWorkFrame_Name

from dual_arm_robot_demo.msg import ObjectPose

from tf2_geometry_msgs import PointStamped

import aoyi_hand



def load_config(config_path):
    
    with open(config_path, 'r') as stream:
        config = yaml.safe_load(stream)
    return config


def convert(x, y, z):
    """创建一依赖于 camera 的坐标点，调用 API 求出该点在 base 中的坐标


    Args:
        x (float): 某点在相机坐标系下的x轴位置
        y (float): 某点在相机坐标系下的y轴位置
        z (float): 某点在相机坐标系下的z轴位置

    Returns:
        tuple: 某点在机械臂基坐标系下的位置
    """
    
    point_source = PointStamped()
    point_source.header.frame_id = "camera"
    point_source.header.stamp = rospy.Time()
    point_source.point.x = x
    point_source.point.y = y
    point_source.point.z = z

    point_target = buffer.transform(point_source,"base",rospy.Duration(0.5))

    return point_target.point.x,point_target.point.y,point_target.point.z



def plan_state_callback(msg):
    
    """当订阅的机械臂执行状态消息到达时，会调用此回调函数

    """
    global run_state

    if msg.state:
        run_state = True
        rospy.loginfo("*******Plan State OK")
    else:
        run_state = False
        rospy.loginfo("*******Plan State Fail")

    # 接收到订阅的机械臂执行状态消息后，会进入消息回调函数

def joint_position_callback(msg):

    """订阅/r_arm/rm_driver/Arm_Current_State话题的回调函数

    Args:
        msg (Arm_Current_State): 
    """
    
    # 接收位姿信息
    global curret_pose3

    curret_pose3 = msg

    rospy.loginfo("pose state is: [{}, {}, {}, {}, {}, {}]".format(

    msg.Pose[0], msg.Pose[1], msg.Pose[2], msg.Pose[3], msg.Pose[4], msg.Pose[5]))



def get_arm_state_callback2(msg):

    # 接收位置和四元数|关节角度

    global curret_pose2
    curret_pose2 = msg

    rospy.loginfo("joint state is: [{}, {}, {}, {}, {}, {}]".format(

    msg.joint[0], msg.joint[1], msg.joint[2], msg.joint[3], msg.joint[4], msg.joint[5]))


    rospy.loginfo("position is: [{}, {}, {}]".format(

        msg.Pose.position.x, msg.Pose.position.y, msg.Pose.position.z))

    rospy.loginfo("四元数: [{}, {}, {}, {}]".format(

        msg.Pose.orientation.x, msg.Pose.orientation.y, msg.Pose.orientation.z, msg.Pose.orientation.w))

    rospy.loginfo("arm_err is: {}".format(msg.arm_err))

    rospy.loginfo("sys_err is: {}".format(msg.sys_err))


def get_pose_pub():
    """创建并发布获取机械臂状态的命令

    """
    
    command = GetArmState_Command()
    command.command = "get_current_arm_state"
    pub_get_pose.publish(command)
    rospy.loginfo("*******published command: {}".format(command.command))


def detect_catch():

    # 机械臂拍照识别位置
    moveJ_P_TargetPose = MoveJ_P()
    moveJ_P_TargetPose.Pose.position.x = camera_pose[0]
    moveJ_P_TargetPose.Pose.position.y = camera_pose[1]
    moveJ_P_TargetPose.Pose.position.z = camera_pose[2]
    moveJ_P_TargetPose.Pose.orientation.x = camera_pose[3]
    moveJ_P_TargetPose.Pose.orientation.y = camera_pose[4]
    moveJ_P_TargetPose.Pose.orientation.z = camera_pose[5]
    moveJ_P_TargetPose.Pose.orientation.w = camera_pose[6]
    moveJ_P_TargetPose.speed = speed
    


    pub_to_pose.publish(moveJ_P_TargetPose)

    # 动作是否规划成功
    is_arrive()


def is_arrive():
    """机械臂动作是否规划成功
    """

    global run_state

    # 等待run_state变成True
    while 1:

        rospy.sleep(0.1)
        if run_state:
            run_state = False
            break



def get_object_info():
    """获取模型检测的物体信息（相机坐标系下）将其转化成机械臂基坐标系下的坐标
    
    Returns:
        tuple: 返回的在相机坐标系下物体的位置坐标
    """
 
    object_pose = rospy.wait_for_message("object_pose_bottle", ObjectPose, timeout=None)

    rospy.loginfo(f"object_pose:{object_pose}")
    rospy.loginfo(
        f"object_pose.x:{object_pose.x} -- object_info.y:{object_pose.y} -- object_info.z:{object_pose.z} ")

    return object_pose.x, object_pose.y, object_pose.z




def go_to_position(camera_x, camera_y, camera_z):
    """
    将相机坐标系下的物体位置转化为基坐标系下并移动到做抓取动作

    Args:
        camera_x (float): 相机坐标系下物体 x
        camera_y (float): 相机坐标系下物体 y
        camera_z (float): 相机坐标系下物体 z

    """

    # 将相机坐标系物体坐标转化为基坐标系下物体坐标
    object_x, object_y, object_z = convert(camera_x, camera_y, camera_z)
    object_x -= 0.025

    rospy.loginfo(f'object_x:{object_x}\n object_y:{object_y} \n object_z:{object_z}')
    
    
    moveJ_P_TargetPose = MoveJ_P()
    moveJ_P_TargetPose.Pose.position.x = middle_pose[0]
    moveJ_P_TargetPose.Pose.position.y = middle_pose[1]
    moveJ_P_TargetPose.Pose.position.z = middle_pose[2]
    moveJ_P_TargetPose.Pose.orientation.x = middle_pose[3]
    moveJ_P_TargetPose.Pose.orientation.y = middle_pose[4]
    moveJ_P_TargetPose.Pose.orientation.z = middle_pose[5]
    moveJ_P_TargetPose.Pose.orientation.w = middle_pose[6]
    moveJ_P_TargetPose.speed = speed

    pub_to_pose.publish(moveJ_P_TargetPose)

    is_arrive()
    

    get_pose_pub()
    rospy.sleep(0.2)

    # 四元数
    x, y, z, w = curret_pose2.Pose.orientation.x, curret_pose2.Pose.orientation.y, curret_pose2.Pose.orientation.z, curret_pose2.Pose.orientation.w
    # 姿态
    rx,ry,rz = curret_pose3.Pose[3],curret_pose3.Pose[4],curret_pose3.Pose[5]

    #物体的位姿物体的z轴负方向移动10.5cm
    object_x,object_y,object_z = change_pose([object_x,object_y,object_z,rx,ry,rz],-0.12,"z")[:3]
    #物体的位姿物体的y轴正方向方向移动8cm
    object_x2,object_y2,object_z2 = change_pose([object_x,object_y,object_z,rx,ry,rz],-0.04,"y")[:3]
    #先移动到物体位姿的y轴正方向20cm处
    object_x,object_y,object_z = change_pose([object_x,object_y,object_z,rx,ry,rz],-0.085,"y")[:3]

    # movej_p移动到物体处
    moveJ_P_TargetPose = MoveJ_P()
    moveJ_P_TargetPose.Pose.position.x = object_x
    moveJ_P_TargetPose.Pose.position.y = object_y
    moveJ_P_TargetPose.Pose.position.z = object_z
    moveJ_P_TargetPose.Pose.orientation.x = x
    moveJ_P_TargetPose.Pose.orientation.y = y
    moveJ_P_TargetPose.Pose.orientation.z = z
    moveJ_P_TargetPose.Pose.orientation.w = w
    moveJ_P_TargetPose.speed = speed

    pub_to_pose.publish(moveJ_P_TargetPose)

    is_arrive()
    
    
    # movej_p移动到物体处
    moveJ_P_TargetPose = MoveJ_P()
    moveJ_P_TargetPose.Pose.position.x = object_x
    moveJ_P_TargetPose.Pose.position.y = object_y
    moveJ_P_TargetPose.Pose.position.z = object_z
    moveJ_P_TargetPose.Pose.orientation.x = craw_before[0]
    moveJ_P_TargetPose.Pose.orientation.y = craw_before[1]
    moveJ_P_TargetPose.Pose.orientation.z = craw_before[2]
    moveJ_P_TargetPose.Pose.orientation.w = craw_before[3]
    moveJ_P_TargetPose.speed = speed

    pub_to_pose.publish(moveJ_P_TargetPose)

    is_arrive()

    moveJ_P_TargetPose = MoveJ_P()
    moveJ_P_TargetPose.Pose.position.x = object_x2
    moveJ_P_TargetPose.Pose.position.y = object_y2
    moveJ_P_TargetPose.Pose.position.z = object_z2
    moveJ_P_TargetPose.Pose.orientation.x = craw_before[0]
    moveJ_P_TargetPose.Pose.orientation.y = craw_before[1]
    moveJ_P_TargetPose.Pose.orientation.z = craw_before[2]
    moveJ_P_TargetPose.Pose.orientation.w = craw_before[3]
    moveJ_P_TargetPose.speed = speed

    pub_to_pose.publish(moveJ_P_TargetPose)
    
    rospy.sleep(2)

    is_arrive()

    # 发布获取当前状态指令
    get_pose_pub()
    rospy.sleep(0.3)

    # 夹爪夹取
    aoyi.catch_dumb()

    #抓取物体后移动到物体上方5cm处
    object_x3,object_y3,object_z3 = change_pose([object_x2,object_y2,object_z2,rx,ry,rz],-0.1,"x")[:3]

    # movej_p移动到物体处
    moveJ_P_TargetPose = MoveJ_P()
    moveJ_P_TargetPose.Pose.position.x = object_x3
    moveJ_P_TargetPose.Pose.position.y = object_y3
    moveJ_P_TargetPose.Pose.position.z = object_z3
    moveJ_P_TargetPose.Pose.orientation.x = x
    moveJ_P_TargetPose.Pose.orientation.y = y
    moveJ_P_TargetPose.Pose.orientation.z = z
    moveJ_P_TargetPose.Pose.orientation.w = w
    moveJ_P_TargetPose.speed = speed

    pub_to_pose.publish(moveJ_P_TargetPose)

    is_arrive()

def work_change():

    """工作坐标系切换到base坐标系
    """

    # 创建并发布空间规划指令

    name = ChangeWorkFrame_Name()

    name.WorkFrame_name = "Base"

    change_work_frame_pub.publish(name)

    rospy.loginfo("*******published tool name:%s", name.WorkFrame_name)


def change_pose(pose, num,axis):
    """
    根据物体和基座的其次变换矩阵 求得 物体z轴 0 0 num 所在位置对应 基座标系的位姿
    
    Args:
        pose:位姿
        num:偏移距离
        axis:需要偏移的坐标轴
    Returns:
    """
    matrix = pose_to_homogeneous_matrix(pose)
    if axis == "z":
        obj_init = np.array([0,0,num])
    elif axis == "y":
        obj_init = np.array([0,num,0])
    else:
        obj_init = np.array([num,0,0])

    obj_init = np.append(obj_init, [1])  # 将物体坐标转换为齐次坐标
    obj_base_init = matrix.dot(obj_init)
    return [i for i in obj_base_init[:3]] + pose[3:]

def pose_to_homogeneous_matrix(pose):
    """将位姿转化为其次变换矩阵

    Args:
        pose (list): 机械臂位姿

    Returns:
        矩阵: _description_
    """
    x, y, z, rx, ry, rz = pose
    R = euler_angles_to_rotation_matrix(rx, ry, rz)
    t = np.array([x, y, z]).reshape(3, 1)
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = t[:, 0]
    return H

def euler_angles_to_rotation_matrix(rx, ry, rz):
    # 计算旋转矩阵
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])
    R = Rz @ Ry @ Rx  # 先绕z轴旋转 再绕y轴旋转  最后绕x轴旋转
    return R

def main():

    """
    主函数
    """

    # 机械臂工作坐标系切换到base
    work_change()
    rospy.sleep(0.1)
    # 灵巧手张开
    aoyi.open_hand()

    # 机械臂从初始位到拍照位
    detect_catch()

    # 休息3s 等待识别稳定
    rospy.sleep(3)

    # 获取模型识别到的物体位姿
    camera_x, camera_y, camera_z = get_object_info()

    # 相机坐标系的物体的坐标可能是0 0 0
    while camera_x == 0 and camera_y == 0 and camera_z == 0:

        rospy.sleep(1.5)

        camera_x, camera_y, camera_z = get_object_info()
        
    # 机械臂到达物体上方并夹取
    go_to_position(camera_x, camera_y, camera_z)

    detect_catch()

    # 灵巧手张开
    aoyi.open_hand()

if __name__ == "__main__":

    
    # 初始化抓取节点
    rospy.init_node("catch",anonymous=True)

    aoyi = aoyi_hand.AoYiHand()
    
    
    # 获取当前文件的绝对路径
    current_file_path = os.path.abspath(__file__)
    
    # 获取当前文件所在目录的路径
    current_dir = os.path.dirname(current_file_path)

    config = load_config(os.path.join(current_dir,'config.yaml'))

    camera_pose = config['CAMERA_POSE']
    craw_before = config['CRAW_BEFORE']
    speed = config['SPEED']
    middle_pose = config['MIDDLE_POSE']


    # 创建 TF 订阅对象
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    # 创建一个发布者（Publisher），用于向/rm_driver/GetArmState_Cmd话题发布GetArmState_Command类型的消息。
    # 这个话题用于请求机械臂的当前状态。
    pub_get_pose = rospy.Publisher("/r_arm/rm_driver/GetArmState_Cmd", GetArmState_Command, queue_size=10)

    # 创建一个发布者（Publisher），用于向/rm_driver/MoveJ_P_Cmd话题发布MoveJ_P类型的消息。
    # 这个话题用于发送机 
    pub_to_pose = rospy.Publisher("/r_arm/rm_driver/MoveJ_P_Cmd", MoveJ_P, queue_size=10)

    # 空间规划指令Publisher
    change_work_frame_pub = rospy.Publisher("/r_arm/rm_driver/ChangeWorkFrame_Cmd", ChangeWorkFrame_Name, queue_size=10)

    rospy.sleep(1)

    # 订阅当前状态（关节角度和位姿）| 当前状态（机械臂末端位置和四元数）|机械臂运动规划是否成功（movej、movej_p和moveL）
    armstate_sub = rospy.Subscriber("/r_arm/rm_driver/ArmCurrentState", ArmState, get_arm_state_callback2, queue_size=10)

    planState_sub = rospy.Subscriber("/r_arm/rm_driver/Plan_State", Plan_State, plan_state_callback)

    joint_positi_sub = rospy.Subscriber("/r_arm/rm_driver/Arm_Current_State",Arm_Current_State,joint_position_callback)

    rospy.sleep(1)

    # 储存位置、四元数信息
    curret_pose2 = None
    # 位姿
    curret_pose3 = None
    # 轨迹规划状态
    run_state = False

    main()

    rospy.spin()
