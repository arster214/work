#!/usr/bin/env python3

import signal
import threading

import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RealSenseCamera:

    def __init__(self):

        rospy.init_node('realsense_camera_node', anonymous=True)

        self.bridge = CvBridge()
        self.devices = self.list_realsense_devices()


        if not self.devices:
            rospy.logerr("No RealSense devices found.")
            return

        idx = int(rospy.get_param('~idx',3))

        rospy.loginfo(f'idx:{idx}')

        if idx == 3:

            choice_indices = range(len(self.devices))
            try:
                choice_indices = [int(index) for index in choice_indices]
            except ValueError:
                rospy.logerr("Invalid input. Please enter numbers separated by spaces.")
                return

            for index in choice_indices:
                if index not in range(len(self.devices)):
                    rospy.logerr(f"No device with number {index} found.")
            self.start_cameras(choice_indices)

        else:

            # Start selected devices
            self.start_cameras(idx)

    def start_cameras(self, choice_indices):

        if isinstance(choice_indices,int):

            serial_number = self.devices[choice_indices]["serial"]
            pipeline = self.select_realsense_device(serial_number)
            self.start_camera_pipeline(pipeline,choice_indices)

        else:
            thread_list = []

            for index in choice_indices:
                device = self.devices[index]
                serial_number = device["serial"]
                pipeline = self.select_realsense_device(serial_number)
                thread_ = threading.Thread(
                    target=self.start_camera_pipeline,
                    args=(pipeline, index)
                )
                thread_.start()
                thread_list.append(thread_)

            # Wait for all threads to finish
            for thread_ in thread_list:
                thread_.join()

    def list_realsense_devices(self):
        """List all connected RealSense devices."""
        ctx = rs.context()
        devices = ctx.query_devices()
        device_list = []
        for i, dev in enumerate(devices):
            device_info = {}
            device_info['number'] = i
            device_info['name'] = dev.get_info(rs.camera_info.name)
            device_info['serial'] = dev.get_info(rs.camera_info.serial_number)
            device_list.append(device_info)
            rospy.loginfo(f"{i}: {device_info['name']} (Serial: {device_info['serial']})")

        return device_list

    def select_realsense_device(self, serial_number):
        """Select and start the specified RealSense device."""
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial_number)

        # Enable streams
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        # Add depth stream if needed
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

        pipeline.start(config)
        return pipeline

    def start_camera_pipeline(self, pipeline, camera_index):
        """Start camera data stream and publish images."""
        color_pub = rospy.Publisher(f'/camera_d435_{camera_index}/color/image_raw', Image, queue_size=10)
        depth_pub = rospy.Publisher(f'/camera_d435_{camera_index}/depth/image_raw', Image, queue_size=10)

        try:
            while not rospy.is_shutdown():
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                # Process color image
                color_image = np.asanyarray(color_frame.get_data())
                color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                color_pub.publish(color_msg)

                # Process depth image
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
                depth_pub.publish(depth_msg)

        except rospy.ROSInterruptException:
            pass
        finally:
            pipeline.stop()
            rospy.loginfo(f"Camera {camera_index} stopped.")


if __name__ == "__main__":
    RealSenseCamera()
