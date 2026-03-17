#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

class DualArmTaskDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('dual_arm_pick_place_demo', anonymous=True)

        # 1. 初始化机器人和场景
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # 规划组 (确保和你的 SRDF 一致)
        # 假设 dual_arms 用于规划，l_gripper/r_gripper 用于逻辑抓取
        self.group = moveit_commander.MoveGroupCommander("dual_arms")
        self.l_gripper = moveit_commander.MoveGroupCommander("l_gripper")
        self.r_gripper = moveit_commander.MoveGroupCommander("r_gripper")
        
        # 设置规划时间
        self.group.set_planning_time(10.0)
        self.group.set_num_planning_attempts(5) # 允许失败重试

        rospy.sleep(2) # 等待连接

    def pick_ball(self, arm_name, ball_name, ball_pose):
        """
        执行抓取任务：移动 -> 逻辑吸附
        arm_name: 'left' or 'right'
        """
        rospy.loginfo(f"=== 开始任务: {arm_name} 臂去抓 {ball_name} ===")

        # 1. 发送 Pose 目标给 RRT 规划器
        # 注意：这里我们只发球心的位置，姿态由 C++ 的 Stream 自动生成
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = ball_pose[0]
        target_pose.position.y = ball_pose[1]
        target_pose.position.z = ball_pose[2]
        # Orientation 设为默认，C++ 会覆盖它或者以它为基准
        target_pose.orientation.w = 1.0

        # 设置 MoveIt 约束
        self.group.clear_pose_targets()
        
        # 关键：指定是哪个 link 去那个位置
        # 根据你的 SRDF，左臂末端可能是 l_link7，右臂是 r_link7
        ee_link = "l_link7" if arm_name == 'left' else "r_link7"
        
        self.group.set_pose_target(target_pose, ee_link)
        
        # 2. 规划并执行 (调用你的 DualArmRRT)
        rospy.loginfo("正在规划路径 (DualArmRRT)...")
        plan = self.group.plan()
        success = plan[0]
        
        if success:
            rospy.loginfo("规划成功！开始移动...")
            self.group.execute(plan[1], wait=True)
        else:
            rospy.logerr("规划失败！请检查目标是否可达或是否碰撞。")
            return False

        # 3. 逻辑抓取 (Logical Grasping)
        # 告诉 MoveIt: 这个球现在粘在手上了，碰撞检测要把它当做手的一部分
        rospy.loginfo(f"逻辑抓取: 将 {ball_name} 附着到 {ee_link}")
        touch_links = self.robot.get_link_names(group=f"{arm_name[0]}_gripper")
        self.scene.attach_box(ee_link, ball_name, touch_links=touch_links)
        
        rospy.sleep(1.0)
        return True

    def place_ball(self, arm_name, ball_name):
        """
        放置任务：抬起/移动 -> 逻辑分离
        """
        rospy.loginfo(f"=== 放置物体: {ball_name} ===")
        
        # 这里简单做一个“抬起”动作作为放置演示
        # 获取当前关节角
        joint_vals = self.group.get_current_joint_values()
        
        # 假设前7个是左臂，后7个是右臂 (取决于你的 joint_model_group 顺序)
        # 简单起见，我们只让规划器回到一个稍微高一点的 Home 点，或者手动修改关节角
        # 这里为了演示，我们让它原地保持，只做逻辑分离
        
        rospy.loginfo("逻辑分离: 释放物体")
        ee_link = "l_link7" if arm_name == 'left' else "r_link7"
        self.scene.remove_attached_object(ee_link, name=ball_name)
        rospy.sleep(1.0)

if __name__ == '__main__':
    demo = DualArmTaskDemo()
    
    # 定义小球坐标 (来自你的 add_table_obstacles_gazebo.py)
    # 注意：这些坐标是相对于 base_link_underpan 的
    balls = {
        "ball1": [0.1, -0.7, 1.46],
        "ball3": [0.0, -0.89, 1.4],
        "ball4": [-0.2, -0.78, 1.35]
    }

    # === 实验流程 ===
    
    # 1. 左手抓 Ball4 (因为它在左边 -0.2)
    demo.pick_ball("left", "ball4", balls["ball4"])
    
    # 2. 右手抓 Ball1 (因为它在右边 0.1)
    demo.pick_ball("right", "ball1", balls["ball1"])
    
    # 3. (可选) 同时归位或移动到篮子
    # demo.move_to_home()
