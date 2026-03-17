#!/usr/bin/env python3
"""
Subscribe to an octomap (published by octomap_server) and continuously publish
it inside a MoveIt PlanningScene so MoveIt always has the latest environment.

Usage (example):
rosrun table_obstacles publish_map_to_moveit.py _octomap_topic:=/octomap_full _publish_rate:=1.0
Or use the provided launch file which will start octomap_server and this node.
"""

import rospy
from octomap_msgs.msg import Octomap
from moveit_msgs.msg import PlanningScene


def main():
    rospy.init_node('publish_map_to_moveit')

    octomap_topic = rospy.get_param('~octomap_topic', '/octomap_full')
    publish_rate = rospy.get_param('~publish_rate', 1.0)

    latest_map = {'msg': None}

    def octomap_cb(msg):
        # store latest octomap message
        latest_map['msg'] = msg

    rospy.Subscriber(octomap_topic, Octomap, octomap_cb, queue_size=1)
    pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)

    rate = rospy.Rate(float(publish_rate))
    rospy.loginfo("publish_map_to_moveit: listening on '%s', publishing to /planning_scene @ %.2f Hz", octomap_topic, float(publish_rate))

    while not rospy.is_shutdown():
        if latest_map['msg'] is not None:
            scene = PlanningScene()
            scene.is_diff = True
            # place the octomap into the planning scene world
            scene.world.octomap.header = latest_map['msg'].header
            scene.world.octomap.octomap = latest_map['msg']
            # publish planning scene (MoveIt will merge diffs)
            pub.publish(scene)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
