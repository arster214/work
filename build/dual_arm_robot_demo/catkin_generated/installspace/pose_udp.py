#!/usr/bin/env python3
# -*- coding=UTF-8 -*-
"""
    动态的坐标系相对姿态发布（机械臂末端相对于机械臂基座的位姿）

"""


import rospy
import tf2_ros

from dual_arm_msgs.msg import Set_Realtime_Push
from geometry_msgs.msg import Pose,TransformStamped


#4.回调函数处理
def doPose(pose):

    #创建 TF 广播器
    broadcaster = tf2_ros.TransformBroadcaster()
    
    #创建 广播的数据(通过 pose 设置)
    tfs = TransformStamped()
    tfs.header.frame_id = "base"
    tfs.header.stamp = rospy.Time.now()
    tfs.child_frame_id = "end"

    tfs.transform.translation.x = pose.position.x
    tfs.transform.translation.y = pose.position.y
    tfs.transform.translation.z = pose.position.z

    tfs.transform.rotation.x = pose.orientation.x
    tfs.transform.rotation.y = pose.orientation.y
    tfs.transform.rotation.z = pose.orientation.z
    tfs.transform.rotation.w = pose.orientation.w

    #广播器发布数据
    broadcaster.sendTransform(tfs)

def set_udp_config(cycle, port, force_coordinate, ip):
    
    pub = rospy.Publisher('/r_arm/rm_driver/Set_Realtime_Push', Set_Realtime_Push, queue_size=10)
    msg = Set_Realtime_Push()
    msg.cycle = cycle
    msg.port = port
    msg.force_coordinate = force_coordinate
    msg.ip = ip

    pub.publish(msg)

if __name__ == "__main__":
    
    
    # 2.初始化 ROS 节点
    rospy.init_node("dynamic_tf_pub_p")

    udp_pub =  rospy.Publisher('/r_arm/rm_driver/Set_Realtime_Push', Set_Realtime_Push, queue_size=10)

    set_udp_config(1, 8089, 0, '169.254.128.40')

    rospy.sleep(0.1)

    # 3.订阅 /r_arm/rm_driver/Pose_State 话题消息
    sub = rospy.Subscriber("/r_arm/rm_driver/Pose_State",Pose,doPose)

    #4.回调函数处理
    # 4-1.创建 TF 广播器
    # 4-2.创建 广播的数据(通过 pose 设置)
    # 4-3.广播器发布数据
    # 5.spin
    rospy.spin()
