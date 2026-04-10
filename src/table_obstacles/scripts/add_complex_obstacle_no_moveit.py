#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from clearance_heatmap_msgs.msg import BoundingBox, EnvironmentInfo

def publish_environment_info():
    rospy.init_node("add_complex_obstacle_no_moveit")
    topic = rospy.get_param("~environment_topic", "/map_process/environment_info")
    
    pub = rospy.Publisher(topic, EnvironmentInfo, queue_size=1, latch=True)
    marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1, latch=True)
    
    env = EnvironmentInfo()
    env.table_z = 0.85 + 0.7 / 2.0 # table_surface_z

    # 1. 桌子 (Table)
    table_center = [0.0, -0.9, 0.85]
    table_size = [1.2, 0.8, 0.7]
    
    # 2. 桌腿 (Pillars)
    half_l, half_w = 1.2/2, 0.8/2
    p_h = 1.2
    p_size = [0.05, 0.05, p_h]
    pillars = [
        [ half_l,  half_w - 0.9, p_h/2],
        [ half_l, -half_w - 0.9, p_h/2],
        [-half_l,  half_w - 0.9, p_h/2],
        [-half_l, -half_w - 0.9, p_h/2],
    ]

    # 3. 桌面上的方块 (Boxes)
    box_size = [0.15, 0.15, 0.3]
    t_top_z = 0.85 + 0.7/2.0 + box_size[2]/2.0
    boxes = [
        ([ 0.2, -0.85, t_top_z], box_size),
        ([-0.2, -0.85, t_top_z], box_size),
    ]

    # 4. 堆叠的小方块 (Little Boxes)
    l_box_size = [0.3, 0.15, 0.07]
    l_box_z = t_top_z + box_size[2]/2.0 + l_box_size[2]/2.0
    boxes.append(([ 0.15, -0.85, l_box_z], l_box_size))
    boxes.append(([-0.15, -0.85, l_box_z], l_box_size))

    # 5. 两侧小方块 (Side Boxes)
    s_box_size = [0.1, 0.1, 0.2]
    s_box_z = 0.85 + 0.7/2.0 + s_box_size[2]/2.0
    boxes.append(([ 0.55, -0.85, s_box_z], s_box_size))
    boxes.append(([-0.55, -0.85, s_box_z], s_box_size))

    # 构建 EnvironmentInfo
    all_obstacles = [ (table_center, table_size) ] + \
                    [ (p, p_size) for p in pillars ] + \
                    boxes

    for pos, size in all_obstacles:
        bb = BoundingBox()
        bb.center.x, bb.center.y, bb.center.z = pos
        bb.size.x, bb.size.y, bb.size.z = size
        env.obstacles.append(bb)

    # 构建 MarkerArray 用于 RViz 显示 (对应的 Topic 为 /visualization_marker_array)
    ma = MarkerArray()
    for i, (pos, size) in enumerate(all_obstacles):
        m = Marker()
        m.header.frame_id = "base_link_underpan"
        m.header.stamp = rospy.Time.now()
        m.ns = "complex_env"
        m.id = i
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = size
        # 修改颜色为绿色，保持一定的透明度 (Green)
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.8, 0.0, 0.5
        ma.markers.append(m)

    rospy.sleep(1.0)
    pub.publish(env)
    marker_pub.publish(ma)
    rospy.loginfo("Full Complex environment (Table + Pillars + 6 Boxes) published.")
    rospy.spin()

if __name__ == "__main__":
    publish_environment_info()
