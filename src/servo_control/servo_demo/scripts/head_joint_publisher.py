#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from servo_ros.msg import ServoAngle
import math

# 转换参数 (假设 LX-224HV 0-1000 对应 0-240度)
# 中心位 500 对应 0 弧度
# 1 单位 = 0.24 度
SERVO_CENTER = 500
DEG_PER_UNIT = 0.24
RAD_PER_DEG = math.pi / 180.0

def convert_raw_to_rad(raw_value):
    # 偏差值
    diff = raw_value - SERVO_CENTER
    # 转换为角度再转弧度
    # 注意：如果方向反了，这里加个负号
    return diff * DEG_PER_UNIT * RAD_PER_DEG

class HeadJointPublisher:
    def __init__(self):
        rospy.init_node('head_joint_publisher')
        
        self.pub = rospy.Publisher('/head/joint_states', JointState, queue_size=10)
        self.sub = rospy.Subscriber('/servo_state', ServoAngle, self.callback)
        
        self.joint_state = JointState()
        self.joint_state.name = ['head_joint1', 'head_joint2']
        self.joint_state.position = [0.0, 0.0]
        self.joint_state.velocity = []
        self.joint_state.effort = []
        
        rospy.loginfo("Head Joint Publisher Started")

    def callback(self, msg):
        self.joint_state.header.stamp = rospy.Time.now()
        
        # 假设 ID 1 是 head_joint1 (Yaw/摇头)
        # 假设 ID 2 是 head_joint2 (Pitch/点头)
        # 如果发现 RViz 里动的方向不对，可以在这里修改符号或交换 ID
        
        # 保护一下范围，防止通信错误导致的异常值
        if 0 <= msg.angle_1 <= 1000 and 0 <= msg.angle_2 <= 1000:
            # 注意：这里可能需要根据实际安装方向加负号
            # 例如：yaw = -convert_raw_to_rad(msg.angle_1)
            yaw = convert_raw_to_rad(msg.angle_1)
            pitch = convert_raw_to_rad(msg.angle_2)
            
            self.joint_state.position = [yaw, pitch]
            self.pub.publish(self.joint_state)

if __name__ == '__main__':
    try:
        node = HeadJointPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
