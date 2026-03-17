#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import JointState

class JointRenamer:
    def __init__(self):
        rospy.init_node('joint_state_renamer', anonymous=True)
        
        # 发布重命名后的话题
        self.pub_r = rospy.Publisher('/r_arm/joint_states_renamed', JointState, queue_size=10)
        self.pub_l = rospy.Publisher('/l_arm/joint_states_renamed', JointState, queue_size=10)
        
        # 订阅原始话题
        rospy.Subscriber('/r_arm/joint_states', JointState, self.callback_r)
        rospy.Subscriber('/l_arm/joint_states', JointState, self.callback_l)

    def callback_r(self, msg):
        # 复制消息以避免修改原始数据（虽然 Python 中 msg 是引用，但这里我们直接修改属性）
        # 将 rightarm_jointX 重命名为 r_jointX 以匹配 URDF
        new_names = [n.replace('rightarm_joint', 'r_joint') for n in msg.name]
        msg.name = new_names
        self.pub_r.publish(msg)

    def callback_l(self, msg):
        # 将 leftarm_jointX 重命名为 l_jointX 以匹配 URDF
        new_names = [n.replace('leftarm_joint', 'l_joint') for n in msg.name]
        msg.name = new_names
        self.pub_l.publish(msg)

if __name__ == '__main__':
    try:
        JointRenamer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
