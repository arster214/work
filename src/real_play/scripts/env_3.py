#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from clearance_heatmap_msgs.msg import BoundingBox, EnvironmentInfo

def publish_env(env_name, obstacles, table_z):
    rospy.init_node(env_name)
    topic = rospy.get_param("~environment_topic", "/map_process/environment_info")
    
    pub = rospy.Publisher(topic, EnvironmentInfo, queue_size=1, latch=True)
    marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1, latch=True)
    
    env = EnvironmentInfo()
    env.table_z = table_z

    ma = MarkerArray()
    
    for i, (pos, size, color) in enumerate(obstacles):
        bb = BoundingBox()
        bb.center.x, bb.center.y, bb.center.z = pos
        bb.size.x, bb.size.y, bb.size.z = size
        env.obstacles.append(bb)
        
        m = Marker()
        m.header.frame_id = "base_link_underpan"
        m.header.stamp = rospy.Time.now()
        m.ns = env_name
        m.id = i
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = size
        m.color.r, m.color.g, m.color.b, m.color.a = color
        ma.markers.append(m)

    rospy.sleep(1.0)
    pub.publish(env)
    marker_pub.publish(ma)
    rospy.loginfo(f"{env_name} published.")
    rospy.spin()

if __name__ == "__main__":
    table_pos = [0.0, -0.75, 0.375]
    table_size = [1.4, 0.5, 0.75]
    green = (0.0, 0.8, 0.0, 0.5)
    orange = (1.0, 0.5, 0.0, 0.8)

    # 大方块 (31cm = 0.31m)
    big_box_size = [0.31, 0.31, 0.31]
    # 用户提供的坐标 (cm) -> (m)，并加上桌面高度 0.75m
    big_box_centers = [
        (0.245, -0.655, 0.155 + 0.75),
        (-0.245, -0.655, 0.155 + 0.75),
        (0.245, -0.655, 0.465 + 0.75),
        (-0.245, -0.655, 0.465 + 0.75)
    ]

    obstacles = [(table_pos, table_size, green)]
    for center in big_box_centers:
        obstacles.append((list(center), big_box_size, orange))

    publish_env("env_3", obstacles, 0.75)
