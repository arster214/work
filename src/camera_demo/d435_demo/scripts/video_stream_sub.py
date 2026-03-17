#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class NodeSubscribe:

    def __init__(self, camera_index=0):
        
        self.bridge = CvBridge()
        rospy.loginfo(f"Starting visualization for camera {camera_index}. Press Ctrl+C to exit.")

        # Subscribe to specific camera topics
        color_topic = f'/camera_d435_{camera_index}/color/image_raw'
        depth_topic = f'/camera_d435_{camera_index}/depth/image_raw'

        self.color_sub = rospy.Subscriber(color_topic, Image, self.color_callback)
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback)
        self.color_image = None
        self.depth_image = None

    def color_callback(self, data):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    
    rospy.init_node(f'sub_image_node', anonymous=True)


    camera_index = rospy.get_param('~camera_index', 3)

    # 检查话题是否存在
    topics = rospy.get_published_topics()

    topic_list = [t[0] for t in topics]

    rospy.logwarn(f'topic_list:{topic_list}')

    camera_topic = f'/camera_d435_{camera_index}/color/image_raw'

    if camera_topic in topic_list:

        node = NodeSubscribe(camera_index)
        try:
            while not rospy.is_shutdown():
                if node.color_image is not None:
                    cv2.imshow(f"Camera {camera_index} - Color Frame", node.color_image)
                if node.depth_image is not None:
                    cv2.imshow(f"Camera {camera_index} - Depth Frame", node.depth_image)
                cv2.waitKey(1)
        except KeyboardInterrupt:
            pass
        finally:
            cv2.destroyAllWindows()
    else:
        rospy.logwarn(f"devel{camera_index}不存在，跳过订阅。")


if __name__ == "__main__":
    main()
