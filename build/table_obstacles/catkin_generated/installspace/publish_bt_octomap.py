#!/usr/bin/env python3
"""
Publish a binary OctoMap (.bt) file as octomap_msgs/Octomap, typically after an
external preprocessing step has generated a cleaned map.
"""

import os
import time
import rospy
from octomap_msgs.msg import Octomap


def load_bt_bytes(path: str) -> bytes:
    with open(path, 'rb') as f:
        return f.read()


def main():
    rospy.init_node('publish_bt_octomap')

    bt_file = rospy.get_param('~bt_file')
    frame_id = rospy.get_param('~frame_id', 'map')
    topic = rospy.get_param('~octomap_topic', '/octomap_full')
    resolution = float(rospy.get_param('~resolution', 0.05))
    publish_rate = float(rospy.get_param('~publish_rate', 1.0))
    wait_timeout = float(rospy.get_param('~wait_timeout', 60.0))

    pub = rospy.Publisher(topic, Octomap, queue_size=1, latch=True)
    start = time.time()

    while not rospy.is_shutdown() and not os.path.exists(bt_file):
        if wait_timeout > 0.0 and (time.time() - start) > wait_timeout:
            rospy.logerr("publish_bt_octomap: bt file not found within timeout: %s", bt_file)
            return 1
        rospy.loginfo_throttle(5.0, "publish_bt_octomap: waiting for processed bt file: %s", bt_file)
        rospy.sleep(0.5)

    if rospy.is_shutdown():
        return 0

    msg = Octomap()
    msg.header.frame_id = frame_id
    msg.binary = True
    msg.id = 'OcTree'
    msg.resolution = resolution
    msg.data = load_bt_bytes(bt_file)

    rospy.loginfo("publish_bt_octomap: publishing processed bt '%s' to %s", bt_file, topic)
    rate = rospy.Rate(max(publish_rate, 0.2))
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except rospy.ROSInterruptException:
        pass
