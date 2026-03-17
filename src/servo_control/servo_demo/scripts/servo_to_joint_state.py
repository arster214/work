#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from servo_ros.msg import ServoAngle
import math

class ServoToJointState:
    def __init__(self):
        rospy.init_node('servo_to_joint_state_publisher', anonymous=True)

        # 配置参数 (根据实际情况调整)
        # LX-224HV 舵机参数:
        # 范围: 0-1000 对应 0-240 度
        # 分辨率: 0.24 度/tick
        # 1 tick = 0.24 * (pi/180) = 0.00418879 弧度
        self.ticks_to_rad = 0.00418879 

        # 舵机1 (上下点头) -> head_joint2
        self.servo1_id = 1
        self.servo1_center = 500  # 中位值
        self.servo1_joint_name = "head_joint2"
        self.servo1_direction = 1 # 1 或 -1，用于反转方向

        # 舵机2 (水平旋转) -> head_joint1
        self.servo2_id = 2
        self.servo2_center = 500  # 中位值
        self.servo2_joint_name = "head_joint1"
        self.servo2_direction = 1 # 1 或 -1，用于反转方向

        # 发布者
        self.joint_pub = rospy.Publisher('/head/joint_states', JointState, queue_size=10)
        
        # 订阅者
        self.servo_sub = rospy.Subscriber('/servo_state', ServoAngle, self.servo_cb)
        
        # 滤波参数
        self.last_angle_1 = 500
        self.last_angle_2 = 500
        self.deadband = 2 # 忽略 2 tick (约0.5度) 以内的微小抖动

        rospy.loginfo("Servo to JointState node started.")

    def servo_cb(self, msg):
        # 调试日志：确认回调函数是否被触发
        # rospy.loginfo_throttle(2.0, f"Received servo angles: {msg.angle_1}, {msg.angle_2}")

        # 简单的异常值过滤：如果读到 0 (通常是通信错误)，则忽略
        if abs(msg.angle_1) < 0.1 or abs(msg.angle_2) < 0.1:
            # rospy.logwarn_throttle(2.0, f"Received invalid servo angles: {msg.angle_1}, {msg.angle_2}")
            return

        # 死区滤波 (Deadband Filter)
        # 只有当角度变化超过 deadband 时才更新，消除静止时的微小抖动
        if abs(msg.angle_1 - self.last_angle_1) > self.deadband:
            self.last_angle_1 = msg.angle_1
            
        if abs(msg.angle_2 - self.last_angle_2) > self.deadband:
            self.last_angle_2 = msg.angle_2

        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        
        # 处理舵机 1 (Pitch)
        # 计算 head_joint2 (Pitch)
        diff1 = (self.last_angle_1 - self.servo1_center)
        rad1 = diff1 * self.ticks_to_rad * self.servo1_direction
        
        # 计算 head_joint1 (Yaw)
        diff2 = (self.last_angle_2 - self.servo2_center)
        rad2 = diff2 * self.ticks_to_rad * self.servo2_direction

        # 填充消息
        joint_msg.name = [self.servo1_joint_name, self.servo2_joint_name]
        joint_msg.position = [rad1, rad2]
        joint_msg.velocity = []
        joint_msg.effort = []

        self.joint_pub.publish(joint_msg)

if __name__ == '__main__':
    try:
        node = ServoToJointState()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
