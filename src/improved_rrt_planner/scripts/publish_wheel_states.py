#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

def publish_wheel_states():
    rospy.init_node('wheel_state_publisher')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    msg = JointState()
    msg.name = [
        'joint_left_wheel', 'joint_right_wheel',
        'joint_swivel_wheel_1_1', 'joint_swivel_wheel_1_2',
        'joint_swivel_wheel_2_1', 'joint_swivel_wheel_2_2',
        'joint_swivel_wheel_3_1', 'joint_swivel_wheel_3_2',
        'joint_swivel_wheel_4_1', 'joint_swivel_wheel_4_2'
    ]
    msg.position = [0.0] * len(msg.name)
    msg.velocity = []
    msg.effort = []

    rospy.loginfo("Publishing fake states for AGV wheels to satisfy MoveIt...")

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_wheel_states()
    except rospy.ROSInterruptException:
        pass
