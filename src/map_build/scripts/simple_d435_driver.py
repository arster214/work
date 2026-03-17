#!/usr/bin/env python3

# 修复 "cannot allocate memory in static TLS block" 错误
# 在导入 cv_bridge 之前先导入 cv2
import cv2
import rospy
import pyrealsense2 as rs
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class SimpleD435Driver:
    def __init__(self):
        # 允许通过 launch 文件定义的节点名，不需要在 init 内部写死
        if not rospy.core.is_initialized():
            rospy.init_node('d435_driver', anonymous=True)
        
        # 强制转换为字符串
        self.serial_no = str(rospy.get_param('~serial_no', ''))
        self.camera_name = rospy.get_param('~camera_name', 'camera')
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.fps = rospy.get_param('~fps', 15)
        
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        if self.serial_no:
            self.config.enable_device(self.serial_no)
        else:
            rospy.logwarn(f"[{self.camera_name}] No serial_no provided, using default device.")
        
        # 配置流
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        
        # 发布者
        self.depth_pub = rospy.Publisher(f'/{self.camera_name}/depth/image_raw', Image, queue_size=10)
        self.color_pub = rospy.Publisher(f'/{self.camera_name}/color/image_raw', Image, queue_size=10)
        self.depth_info_pub = rospy.Publisher(f'/{self.camera_name}/depth/camera_info', CameraInfo, queue_size=10)
        self.color_info_pub = rospy.Publisher(f'/{self.camera_name}/color/camera_info', CameraInfo, queue_size=10)
        
        self.start()

    def get_camera_info(self, intrinsics, header):
        info = CameraInfo()
        info.header = header
        info.height = intrinsics.height
        info.width = intrinsics.width
        info.distortion_model = "plumb_bob"
        info.D = intrinsics.coeffs
        info.K = [intrinsics.fx, 0, intrinsics.ppx,
                  0, intrinsics.fy, intrinsics.ppy,
                  0, 0, 1]
        info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        info.P = [intrinsics.fx, 0, intrinsics.ppx, 0,
                  0, intrinsics.fy, intrinsics.ppy, 0,
                  0, 0, 1, 0]
        return info

    def start(self):
        started = False
        max_retries = 5
        retry_count = 0
        
        while not started and retry_count < max_retries and not rospy.is_shutdown():
            try:
                profile = self.pipeline.start(self.config)
                started = True
                rospy.loginfo(f"[{self.camera_name}] Camera started successfully (Serial: {self.serial_no})")
            except Exception as e:
                retry_count += 1
                rospy.logerr(f"[{self.camera_name}] Failed to start (Serial: {self.serial_no}), retry {retry_count}/{max_retries}. Error: {e}")
                rospy.sleep(2.0) # 等待 2 秒重试

        if not started:
            rospy.logfatal(f"[{self.camera_name}] Could not connect to camera after {max_retries} attempts.")
            return

        try:
            # 获取内参
            profile_active = self.pipeline.get_active_profile()
            depth_stream = profile_active.get_stream(rs.stream.depth).as_video_stream_profile()
            color_stream = profile_active.get_stream(rs.stream.color).as_video_stream_profile()
            depth_intrinsics = depth_stream.get_intrinsics()
            color_intrinsics = color_stream.get_intrinsics()
            
            while not rospy.is_shutdown():
                try:
                    # 增加超时保护与异常捕获
                    frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                except RuntimeError as e:
                    continue
                # ...existing code...

                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                now = rospy.Time.now()
                
                # 发布深度图
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
                depth_msg.header.stamp = now
                depth_msg.header.frame_id = f"{self.camera_name}_depth_optical_frame"
                self.depth_pub.publish(depth_msg)
                
                # 发布深度内参
                depth_info = self.get_camera_info(depth_intrinsics, depth_msg.header)
                self.depth_info_pub.publish(depth_info)
                
                # 发布彩色图
                color_image = np.asanyarray(color_frame.get_data())
                color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                color_msg.header.stamp = now
                color_msg.header.frame_id = f"{self.camera_name}_color_optical_frame"
                self.color_pub.publish(color_msg)

                # 发布彩色内参
                color_info = self.get_camera_info(color_intrinsics, color_msg.header)
                self.color_info_pub.publish(color_info)
                
        except Exception as e:
            rospy.logerr(f"Camera {self.camera_name} (Serial: {self.serial_no}) error: {e}")
        finally:
            if started:
                self.pipeline.stop()

if __name__ == "__main__":
    SimpleD435Driver()