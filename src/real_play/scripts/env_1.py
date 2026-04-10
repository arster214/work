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
        # 填充 EnvironmentInfo
        bb = BoundingBox()
        bb.center.x, bb.center.y, bb.center.z = pos
        bb.size.x, bb.size.y, bb.size.z = size
        env.obstacles.append(bb)
        
        # 填充 Marker 用于显示
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
    rospy.loginfo(f"{env_name} published with table and placeholder cubes.")
    rospy.spin()

if __name__ == "__main__":
    # 所有的桌面中心都在0 -0.75 0.375
    table_pos = [0.0, -0.75, 0.375]
    table_size = [1.4, 0.5, 0.75]
    
    # 颜色定义
    green = (0.0, 0.8, 0.0, 0.5)
    orange = (1.0, 0.5, 0.0, 0.8) # 大方块颜色
    blue = (0.0, 0.5, 1.0, 0.8)   # 小方块颜色

    # 单位转换：用户提供的中心坐标似乎是厘米(cm)，需要转换为米(m)
    # 大方块 (31cm = 0.31m)
    big_box_size = [0.31, 0.31, 0.31]
    big_box_pos = [-0.155, -0.665, 0.905]

    # 小方块 (10cm = 0.1m)
    small_box_size = [0.1, 0.1, 0.1]
    small_box_centers = [
        (-0.105, -0.615, 0.36 + 0.75),
        (-0.105, -0.715, 0.36 + 0.75),
        (-0.105, -0.615, 0.46 + 0.75),
        (-0.205, -0.615, 0.36 + 0.75),
        (-0.205, -0.715, 0.36 + 0.75),
        (-0.205, -0.615, 0.46 + 0.75)
    ]

    obstacles = [
        (table_pos, table_size, green),  # 桌面
        (big_box_pos, big_box_size, orange) # 大方块
    ]

    # 添加 6 个小方块
    for center in small_box_centers:
        obstacles.append((list(center), small_box_size, orange))
    
    # 调用发布函数
    publish_env("env_1", obstacles, 0.75)
