#!/usr/bin/env python3
# -*- coding=UTF-8 -*-
"""
版权所有 (c) 2024 [睿尔曼智能科技有限公司]。保留所有权利。

在满足以下条件的情况下，允许重新分发和使用源代码和二进制形式的代码，无论是否修改：
1. 重新分发的源代码必须保留上述版权声明、此条件列表和以下免责声明。
2. 以二进制形式重新分发的代码必须在随分发提供的文档和/或其他材料中复制上述版权声明、此条件列表和以下免责声明。

本软件由版权持有者和贡献者“按原样”提供，不提供任何明示或暗示的保证，
包括但不限于对适销性和特定用途适用性的暗示保证。
在任何情况下，即使被告知可能发生此类损害的情况下，
版权持有者或贡献者也不对任何直接的、间接的、偶然的、特殊的、惩罚性的或后果性的损害
（包括但不限于替代商品或服务的采购；使用、数据或利润的损失；或业务中断）负责，
无论是基于合同责任、严格责任还是侵权行为（包括疏忽或其他原因）。

此模块通过话题的发布者，将双臂复合机器人模块的基本功能都进行了测试。
"""

import rospy
import json
import math

from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose 
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2

from servo_ros.msg import  ServoMove
from dual_arm_msgs.msg import MoveJ_P , MoveJ,Lift_Height



class D435CameraSubscriber:

    def __init__(self, camera_index=0):
        
        self.bridge = CvBridge()
        rospy.loginfo(f"Starting visualization for camera {camera_index}. Press Ctrl+C to exit.")

        # Subscribe to specific camera topics
        color_topic = f'/camera_d435_0/color/image_raw'
        depth_topic = f'/camera_d435_0/depth/image_raw'

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

class UsbCameraSubscriber:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None

        topic_name = f"/camera0/image_raw"
        rospy.loginfo(f"订阅主题：{topic_name}")
        self.sub = rospy.Subscriber(topic_name, Image, self.image_callback)

    def image_callback(self, msg):

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"usb图像转换失败：{e}")



def servo_example():

    """舵机样例
    """

    #测试头部舵机
    move_msg1 = ServoMove()
    move_msg1.servo_id = 1
    move_msg1.angle = 500      
    move_msg2 = ServoMove()
    move_msg2.servo_id = 2
    move_msg2.angle = 450
    
    rospy.loginfo("发布舵机控制信息: %s", move_msg1)
    pub_servo_control.publish(move_msg1)
    rospy.loginfo("发布舵机控制信息: %s", move_msg2)
    pub_servo_control.publish(move_msg2)
    rospy.sleep(3)

def degrees_to_radians(degrees):  
    """  
    将度转换为弧度  
      
    参数:  
    degrees -- 度值  
      
    返回:  
    转换后的弧度值  
    """  
    radians = degrees * (math.pi / 180)  
    return radians  
  

def dual_arm_example():

    """双臂样例
    """
    
    radians_0 = degrees_to_radians(0) #0°对应的弧度值
    radians_57 = degrees_to_radians(57) #57°对应的弧度值

    #测试moveJ 左臂回到零点，单位是弧度。
    movej_cmd_l = MoveJ()
    movej_cmd_l.speed = 0.2
    movej_cmd_l.joint = [radians_0, radians_0,  radians_0, radians_0, radians_0, radians_57]
    pub_MoveJ_l.publish(movej_cmd_l)
    rospy.loginfo("Published MoveJ Command to /l_arm/MoveJ_Cmd")
    rospy.sleep(1)
   
    #测试moveJ 右臂回到零点，单位是弧度。
    movej_cmd_r = MoveJ()
    movej_cmd_r.speed = 0.2
    movej_cmd_r.joint = [radians_0, radians_0,  radians_0, radians_0, radians_0, radians_57]
    pub_MoveJ_r.publish(movej_cmd_r)
    rospy.loginfo("Published MoveJ Command to /r_arm/MoveJ_Cmd")
    rospy.sleep(5)
    
    #测试机械臂moveJ-P空间运动
    message = MoveJ_P()
    message.Pose.position.x = 0.1
    message.Pose.position.y = 0.37
    message.Pose.position.z = 0.61
    message.Pose.orientation.x = 0.0
    message.Pose.orientation.y = 0.0
    message.Pose.orientation.z = 0.0
    message.Pose.orientation.w = 1.0
    message.speed = 0.1

    #左臂运动到该点
    pub_MoveJ_P_l.publish(message)
    rospy.sleep(10)

    #右臂运动到该点
    pub_MoveJ_P_r.publish(message)
    rospy.sleep(10)

def camera_show():
    """d435相机和usb相机显示"""

    try:
        while not rospy.is_shutdown():
            if d435_node.color_image is not None:
                cv2.imshow(f"Camera 0 d435 - Color Frame", d435_node.color_image)
            if d435_node.depth_image is not None:
                cv2.imshow(f"Camera 0 d435 - Depth Frame", d435_node.depth_image)
            if usb_node.cv_image is not None:
                cv2.imshow(f"Camera 0 usb - Color Frame", usb_node.cv_image)            
            cv2.waitKey(1)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


def lift_example():
    """升降示例
    """

    #测试升降
    height_msg = Lift_Height()
    height_msg.height = 380         #范围： 10 - 390   
    height_msg.speed = 30
    pub_height.publish(height_msg)
    rospy.loginfo(f"发布目标高度: {height_msg.height}")
    rospy.sleep(5)
    
    #测试升降
    height_msg = Lift_Height()
    height_msg.height = 200     #范围： 10 - 390   
    height_msg.speed = 30
    pub_height.publish(height_msg)
    rospy.loginfo(f"发布目标高度: {height_msg.height}")
    rospy.sleep(5)


def main():

    #双臂样例
    dual_arm_example()

    #升降示例
    lift_example()

    #舵机样例
    servo_example()

    #订阅头部d435相机的颜色帧和深度帧
    camera_show()

if __name__ == '__main__':
    
    rospy.init_node('navigation_publisher', anonymous=True)  # 初始化ROS节点


    #舵机相关话题
    pub_servo_control = rospy.Publisher("/servo_control/move", ServoMove, queue_size=10)
    
    #双臂相关话题
    pub_MoveJ_l = rospy.Publisher('/l_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
    pub_MoveJ_P_l = rospy.Publisher('/l_arm/rm_driver/MoveJ_P_Cmd', MoveJ_P, queue_size=10)
    pub_MoveJ_r = rospy.Publisher('/r_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
    pub_MoveJ_P_r = rospy.Publisher('/r_arm/rm_driver/MoveJ_P_Cmd', MoveJ_P, queue_size=10)

    #升降
    pub_height = rospy.Publisher("/l_arm/rm_driver/Lift_SetHeight", Lift_Height, queue_size=10)  



    #d435相机订阅话题
    d435_node = D435CameraSubscriber()

    #usb相机订阅话题
    usb_node = UsbCameraSubscriber()

    rospy.sleep(1)  # 等待发布器初始化

    main()
    

