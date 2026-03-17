#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from servo_ros.msg import ServoMove
import time

def head_shake():
    rospy.init_node('head_shake_demo', anonymous=True)
    pub = rospy.Publisher("/servo_control/move", ServoMove, queue_size=10)
    
    # 舵机2负责水平旋转 (Yaw)
    servo_id = 2
    center = 450
    amplitude = 200 # 摆动幅度
    duration = 1000 # 每次移动耗时(ms)

    rospy.sleep(1) # 等待连接

    while not rospy.is_shutdown():
        # 向左转
        msg = ServoMove()
        msg.servo_id = servo_id
        msg.angle = center + amplitude
        msg.time = duration
        pub.publish(msg)
        rospy.loginfo(f"Head Left: {msg.angle}")
        rospy.sleep(duration / 1000.0)

        # 向右转
        msg = ServoMove()
        msg.servo_id = servo_id
        msg.angle = center - amplitude
        msg.time = duration
        pub.publish(msg)
        rospy.loginfo(f"Head Right: {msg.angle}")
        rospy.sleep(duration / 1000.0)

if __name__ == '__main__':
    try:
        head_shake()
    except rospy.ROSInterruptException:
        pass
