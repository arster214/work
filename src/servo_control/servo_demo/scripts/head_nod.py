#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from servo_ros.msg import ServoMove
import time

def head_nod():
    rospy.init_node('head_nod_demo', anonymous=True)
    pub = rospy.Publisher("/servo_control/move", ServoMove, queue_size=10)
    
    # 舵机1负责垂直俯仰 (Pitch)
    servo_id = 1
    center = 500
    amplitude = 150 # 点头幅度
    duration = 800 # 每次移动耗时(ms)

    rospy.sleep(1) # 等待连接

    while not rospy.is_shutdown():
        # 抬头
        msg = ServoMove()
        msg.servo_id = servo_id
        msg.angle = center + amplitude
        msg.time = duration
        pub.publish(msg)
        rospy.loginfo(f"Head Up: {msg.angle}")
        rospy.sleep(duration / 1000.0)

        # 低头
        msg = ServoMove()
        msg.servo_id = servo_id
        msg.angle = center - amplitude
        msg.time = duration
        pub.publish(msg)
        rospy.loginfo(f"Head Down: {msg.angle}")
        rospy.sleep(duration / 1000.0)

if __name__ == '__main__':
    try:
        head_nod()
    except rospy.ROSInterruptException:
        pass
