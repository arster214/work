#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from dual_arm_msgs.msg import LiftState
from std_msgs.msg import Empty

# 全局变量存储当前高度 (单位: 米)
# 默认设为最高点 (0.79m)，对应 joint=0
current_height_m = 0.79 

def lift_callback(msg):
    global current_height_m
    # msg.height 单位是 mm (参考 lift_control.py)
    current_height_m = msg.height / 1000.0

def main():
    global current_height_m # 关键修正：声明使用全局变量，否则会被当做局部变量
    rospy.init_node('lift_to_joint_state')
    
    # 发布到 /lift_joint_states，供 joint_state_publisher 聚合
    pub_joint = rospy.Publisher('/lift_joint_states', JointState, queue_size=10)
    
    # 关键修正：必须主动请求升降柱状态，驱动才会发布
    # 注意：驱动可能需要一点时间初始化，或者话题名有细微差别
    pub_req = rospy.Publisher('/l_arm/rm_driver/Lift_GetState', Empty, queue_size=10)
    
    # 订阅真实的升降状态
    rospy.Subscriber('/l_arm/rm_driver/LiftState', LiftState, lift_callback)
    
    # 关键：等待发布者建立连接，否则第一条请求可能会丢失
    rospy.loginfo("Waiting for publisher connection...")
    rospy.sleep(1.5)
    
    rate = rospy.Rate(10) # 10Hz
    
    # 物理参数 (根据 lift_control.py 和 URDF 推断)
    # 真实高度: 0 (底) ~ 0.79 (顶)
    # URDF关节: 0 (顶) ~ 1.0 (底，Z轴向下)
    # 转换公式: joint_value = MAX_HEIGHT - real_height
    MAX_HEIGHT_M = 0.79 
    
    # 恢复默认值为最高点，等待真实数据覆盖
    current_height_m = 0.79

    # 计数器，用于降低请求频率
    count = 0

    while not rospy.is_shutdown():
        # 1. 主动请求状态 (降低频率到 1Hz，防止阻塞驱动)
        if count % 10 == 0:
            pub_req.publish(Empty())
        count += 1
        
        # 2. 发布关节状态
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = ['platform_joint']
        
        # 计算关节值
        joint_val = MAX_HEIGHT_M - current_height_m
        
        # 安全限位
        if joint_val < 0: joint_val = 0
        if joint_val > 1.0: joint_val = 1.0
        
        js.position = [joint_val]
        js.velocity = []
        js.effort = []
        
        pub_joint.publish(js)
        
        
        rate.sleep()

if __name__ == '__main__':
    main()
