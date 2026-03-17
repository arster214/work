#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
yolov8读取模型 显示识别出来物体的位置和图像
"""


import time
import os
import math
import yaml

import cv2
import rospy
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO


from dual_arm_robot_demo.msg import ObjectPose


#初始化节点
rospy.init_node("model_detect")
#创建发布者对象,发布 bottle类型数据
bottle_pub = rospy.Publisher("object_pose_bottle",ObjectPose)
#物体信息
object_info_msg = ObjectPose()


# 获取当前文件的绝对路径
current_file_path = os.path.abspath(__file__)

# 获取当前文件所在目录的路径
current_dir = os.path.dirname(current_file_path)

# Load the YAML configuration file
def load_config(config_path):
    with open(config_path, 'r') as stream:
        config = yaml.safe_load(stream)
    return config

config_ = load_config(os.path.join(current_dir,'config.yaml'))

camera_id = config_['CAMERA_ID']

# 构建 weight 文件夹内 best.pt 文件的绝对路径
best_pt_path = os.path.join(current_dir, 'weight', 'best.pt')

rospy.loginfo(f"best_pt_path:{best_pt_path}")

# 加载 YOLOv8 模型
model = YOLO(best_pt_path)
 

# 配置 RealSense
pipeline = rs.pipeline()
config = rs.config()

# 获取所有连接的设备列表
#devices = [dev for dev in rs.context().devices]

# 获取所有连接的设备列表
devices = [dev for dev in rs.context().devices if dev.supports(rs.camera_info.serial_number)]

# 指定要使用的设备的序列号（ID）
target_serial_number = camera_id # 您的D435相机的实际序列号

target_device = next((dev for dev in devices if dev.get_info(rs.camera_info.serial_number) == target_serial_number), None)

if target_device:
    config.enable_device(target_device.get_info(rs.camera_info.serial_number))
    rospy.loginfo(f"Enabled device with serial number: {target_serial_number}")
else:
    rospy.logerr("Target device not found, exiting...")
    exit(1)  # 如果未找到目标设备，则退出程序


#config.enable_device(devices[0].get_info(rs.camera_info.serial_number))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
 
# 启动相机流
pipeline.start(config)
align_to = rs.stream.color  # 与color流对齐
align = rs.align(align_to)
 
 
 
def get_aligned_images():

    frames = pipeline.wait_for_frames()  # 等待获取图像帧
    aligned_frames = align.process(frames)  # 获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧
 
    # 相机参数的获取
    intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile(
    ).intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    '''camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }'''
 
    # 保存内参到本地
    # with open('./intrinsics.json', 'w') as fp:
    # json.dump(camera_parameters, fp)
    #######################################################
 
    depth_image = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 深度图（8位）
    depth_image_3d = np.dstack(
        (depth_image_8bit, depth_image_8bit, depth_image_8bit))  # 3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图
 
    # 返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame
 
 
def get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin):
    x = depth_pixel[0]
    y = depth_pixel[1]
    dis = aligned_depth_frame.get_distance(x, y)  # 获取该像素点对应的深度
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    return dis, camera_coordinate
 
# 初始化 FPS 计算
fps = 0
frame_count = 0
start_time = time.time()

rate = rospy.Rate(10)

try:
    
    
    while not rospy.is_shutdown():
        # 等待获取一对连续的帧：深度和颜色
        intr, depth_intrin, color_image, depth_image, aligned_depth_frame = get_aligned_images()
 
        if not depth_image.any() or not color_image.any():
            continue
 
        # 获取当前时间
        time1 = time.time()
 
        # 将图像转为numpy数组
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
            depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((color_image, depth_colormap))
 
        # 使用 YOLOv8 进行目标检测
        results = model.predict(color_image, conf=0.45)

        annotated_frame = results[0].plot()

        detected_boxes = results[0].boxes.xyxy  # 获取边界框坐标

        detected_classes = results[0].names  # 获取检测到的类别名称

        for i, box in enumerate(detected_boxes):

            x1, y1, x2, y2 = map(int, box)  # 获取边界框坐标
  
            class_name = detected_classes.get(results[0].boxes.cls[i])

            # 计算步长
            xrange = max(1, math.ceil(abs((x1 - x2) / 30)))
            yrange = max(1, math.ceil(abs((y1 - y2) / 30)))
            print(f"class_name:{class_name}--detected_boxes:{detected_boxes}")

            # 显示中心点坐标
            ux = int((x1 + x2) / 2)
            uy = int((y1 + y2) / 2)
            dis, camera_coordinate = get_3d_camera_coordinate([ux, uy], aligned_depth_frame,
                                                              depth_intrin)  # 获取对应像素点的三维坐标
 
            #组织信息
            
            object_info_msg.x = camera_coordinate[0]
            object_info_msg.y = camera_coordinate[1]
            object_info_msg.z = camera_coordinate[2]
            bottle_pub.publish(object_info_msg)

            formatted_camera_coordinate = f"({camera_coordinate[0]:.2f}, {camera_coordinate[1]:.2f}, {camera_coordinate[2]:.2f})"
            print(f"formatted_camera_coordinate:{formatted_camera_coordinate}")
            cv2.circle(annotated_frame, (ux, uy), 4, (255, 255, 255), 5)  # 标出中心点
            cv2.putText(annotated_frame, formatted_camera_coordinate, (ux + 20, uy + 10), 0, 1,
                        [225, 255, 255], thickness=1, lineType=cv2.LINE_AA)  # 标出坐标

        cv2.imshow('YOLOv8 RealSense', annotated_frame)
        key = cv2.waitKey(1)
        
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q')  or rospy.is_shutdown():
            cv2.destroyAllWindows()
            break 
        
        rate.sleep()  #休眠
 
finally:
    # 停止流
    pipeline.stop()
