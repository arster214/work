#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
机械臂移动到拍照点位
"""

import rospy
import os
import sys
import yaml

from dual_arm_msgs.msg import MoveJ_P, Plan_State, ChangeWorkFrame_Name



# 获取当前文件的路径
script_dir = os.path.dirname(os.path.abspath(__file__))


def load_config(config_path):
    with open(config_path, 'r') as stream:
        config = yaml.safe_load(stream)
    return config


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


def movej_P(pose):
    """movej_p到达某个点位

    Args:
        pose (_type_): _description_
        speed (_type_, optional): _description_. Defaults to None.
    """
    moveJ_P_PutC_Pose = MoveJ_P()
    moveJ_P_PutC_Pose.Pose.position.x = pose[0]
    moveJ_P_PutC_Pose.Pose.position.y = pose[1]
    moveJ_P_PutC_Pose.Pose.position.z = pose[2]
    moveJ_P_PutC_Pose.Pose.orientation.x = pose[3]
    moveJ_P_PutC_Pose.Pose.orientation.y = pose[4]
    moveJ_P_PutC_Pose.Pose.orientation.z = pose[5]
    moveJ_P_PutC_Pose.Pose.orientation.w = pose[6]

    moveJ_P_PutC_Pose.speed = speed


    pub_to_pose.publish(moveJ_P_PutC_Pose)

    is_arrive()


def work_change():

    """工作坐标系切换到base坐标系
    """

    # 创建并发布空间规划指令

    name = ChangeWorkFrame_Name()

    name.WorkFrame_name = "Base"

    change_work_frame_pub.publish(name)

    rospy.loginfo("*******published tool name:%s", name.WorkFrame_name)


def main():

    """
    主函数
    """

    # 机械臂工作坐标系切换到base
    work_change()
    rospy.sleep(0.1)

    movej_P(camera_pose)


if __name__ == "__main__":

    
    # 初始化抓取节点
    rospy.init_node("camera_pose",anonymous=True)
    
        # 获取当前文件的绝对路径
    current_file_path = os.path.abspath(__file__)
    
    # 获取当前文件所在目录的路径
    current_dir = os.path.dirname(current_file_path)


    # 引入配置
    config = load_config(os.path.join(current_dir,'config.yaml'))
    camera_pose = config['CAMERA_POSE']
    speed = config['SPEED']


    # 创建一个发布者（Publisher），用于向/rm_driver/MoveJ_P_Cmd话题发布MoveJ_P类型的消息。
    pub_to_pose = rospy.Publisher("/r_arm/rm_driver/MoveJ_P_Cmd", MoveJ_P, queue_size=10)

    # 空间规划指令Publisher
    change_work_frame_pub = rospy.Publisher("/r_arm/rm_driver/ChangeWorkFrame_Cmd", ChangeWorkFrame_Name, queue_size=10)

    rospy.sleep(1)

    # 机械臂运动规划是否成功（movej_p）

    planState_sub = rospy.Subscriber("/r_arm/rm_driver/Plan_State", Plan_State, plan_state_callback)

    rospy.sleep(1)

    # 轨迹规划状态
    run_state = False

    main()

    rospy.spin()
