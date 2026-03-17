#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
检查 MoveIt/OMPL 能提供哪些性能指标
"""

import rospy
import moveit_commander
import sys

def check_metrics():
    """检查可用的性能指标"""
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('check_metrics', anonymous=True)
    
    group = moveit_commander.MoveGroupCommander("r_arm")
    
    print("=" * 80)
    print("检查 MoveIt/OMPL 可用的性能指标")
    print("=" * 80)
    
    # 设置一个简单的目标
    home = [0.0] * 7
    goal = [1.375, 0.318, 0.339, 1.215, -1.107, 0.905, -0.002]
    
    group.set_joint_value_target(home)
    group.go(wait=True)
    group.stop()
    rospy.sleep(1.0)
    
    # 测试 RRTstar
    print("\n测试 OMPL RRTstar...")
    group.set_planner_id("RRTstar")
    group.set_joint_value_target(goal)
    group.set_planning_time(5.0)
    
    plan = group.plan()
    
    if isinstance(plan, tuple):
        success = plan[0]
        trajectory = plan[1]
        print(f"  plan() 返回元组，长度: {len(plan)}")
        print(f"  元组内容: success={plan[0]}, trajectory类型={type(plan[1])}")
        if len(plan) > 2:
            print(f"  planning_time={plan[2]}")
        if len(plan) > 3:
            print(f"  error_code={plan[3]}")
    else:
        success = len(plan.joint_trajectory.points) > 0
        trajectory = plan
        print(f"  plan() 返回轨迹对象")
    
    if success:
        print(f"\n  规划成功!")
        print(f"\n  trajectory 对象属性:")
        print(f"    - dir(trajectory): {[x for x in dir(trajectory) if not x.startswith('_')]}")
        
        if hasattr(trajectory, 'joint_trajectory'):
            jt = trajectory.joint_trajectory
            print(f"\n  joint_trajectory 属性:")
            print(f"    - dir(joint_trajectory): {[x for x in dir(jt) if not x.startswith('_')]}")
            print(f"    - joint_names: {jt.joint_names}")
            print(f"    - points 数量: {len(jt.points)}")
            
            if len(jt.points) > 0:
                point = jt.points[0]
                print(f"\n  trajectory_point 属性:")
                print(f"    - dir(point): {[x for x in dir(point) if not x.startswith('_')]}")
                print(f"    - positions: {point.positions}")
                print(f"    - velocities: {point.velocities if hasattr(point, 'velocities') else 'N/A'}")
                print(f"    - accelerations: {point.accelerations if hasattr(point, 'accelerations') else 'N/A'}")
                print(f"    - time_from_start: {point.time_from_start if hasattr(point, 'time_from_start') else 'N/A'}")
        
        # 检查是否有其他信息
        if hasattr(trajectory, 'multi_dof_joint_trajectory'):
            print(f"\n  multi_dof_joint_trajectory: {trajectory.multi_dof_joint_trajectory}")
    
    print("\n" + "=" * 80)
    print("从 MoveIt/OMPL 可以直接获取的指标:")
    print("=" * 80)
    print("✓ 规划时间 (planning_time) - 通过 Python 计时")
    print("✓ 路径点数量 (num_waypoints) - len(trajectory.joint_trajectory.points)")
    print("✓ 路径长度 (path_length) - 计算相邻点的欧氏距离之和")
    print("✓ 轨迹执行时间 (execution_time) - points[-1].time_from_start")
    print("✓ 平滑度 (smoothness) - 通过二阶差分计算加速度变化")
    print("\n✗ 树节点数量 (num_nodes) - MoveIt Python API 不直接提供")
    print("  注意: 需要修改 C++ 规划器代码来输出节点数")
    print("\n" + "=" * 80)
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        check_metrics()
    except rospy.ROSInterruptException:
        pass
