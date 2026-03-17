#!/usr/bin/env python3
"""
Wait for a processed .bt file to appear, then launch octomap_server_node with it.
This avoids race conditions between map preprocessing and scene publication.
"""

import os
import sys
import time
import subprocess
import rospy


def main():
    rospy.init_node('wait_and_start_octomap_server')

    bt_file = rospy.get_param('~bt_file')
    frame_id = rospy.get_param('~frame_id', 'base_link_underpan')
    wait_timeout = float(rospy.get_param('~wait_timeout', 120.0))

    start = time.time()
    while not rospy.is_shutdown() and not os.path.exists(bt_file):
        if wait_timeout > 0.0 and (time.time() - start) > wait_timeout:
            rospy.logerr('wait_and_start_octomap_server: bt file not found within timeout: %s', bt_file)
            return 1
        rospy.loginfo_throttle(5.0, 'wait_and_start_octomap_server: waiting for %s', bt_file)
        rospy.sleep(0.5)

    if rospy.is_shutdown():
        return 0

    cmd = [
        'rosrun', 'octomap_server', 'octomap_server_node',
        '_frame_id:=' + frame_id,
        '_map_file:=' + bt_file,
    ]
    rospy.loginfo('wait_and_start_octomap_server: starting octomap_server with %s', bt_file)
    return subprocess.call(cmd)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except rospy.ROSInterruptException:
        pass
