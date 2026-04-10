#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手动规划数据采集脚本
"""

import rospy
import moveit_commander
import sys
import time
import json
import math
import os
from datetime import datetime

def calculate_path_length(points):
    """计算路径总长度（关节空间角度变化的 L2 范数）"""
    length = 0.0
    for i in range(1, len(points)):
        prev_positions = points[i-1].positions
        curr_positions = points[i].positions
        
        # 计算两点之间的 L2 距离
        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(prev_positions, curr_positions)))
        length += dist
    return length

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('manual_data_collection', anonymous=True)
    
    # 默认使用 dual_arms 操作组
    group_name = rospy.get_param('~group_name', 'dual_arms')
    
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(group_name)
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_file = f"manual_planning_data_{timestamp}.json"
    
    print(f"=== 手动数据收集脚本 已启动 ===")
    print(f"使用的规划组: {group_name}")
    print(f"数据将保存到: {os.path.abspath(output_file)}")
    
    all_data = []
    
    try:
        while not rospy.is_shutdown():
            user_input = input("\n[操作] 按 [Enter] 记录一次规划数据，或输入 'q' 退出并保存: ")
            
            if user_input.strip().lower() == 'q':
                break
                
            # 将当前状态设为起点
            group.set_start_state_to_current_state()
            
            # 随机目标（如果你在 RViz 手动拖动了目标，把下一行注释掉或者修改为你需要的目标获取方式即可）
            # 当前逻辑为：触发后，规划到随机目标并收集数据
            # group.set_random_target()  # 如果你想使用随机目标请取消注释本行，否则默认使用RViz中设置的目标
            
            print("正在规划中...")
            
            start_time = time.time()
            
            # MoveIt 1 接口 (返回值为 tuple 格式: (success, plan, time, error_code))
            plan_res = group.plan()
            
            if len(plan_res) == 4:
                success, plan, planning_time, error_code = plan_res
            else:
                # 兼容旧版本 MoveIt
                success = True if len(plan_res.joint_trajectory.points) > 0 else False
                plan = plan_res
                planning_time = time.time() - start_time
            
            # 使用实际经过的时间作为耗时，或者直接用 MoveIt 的 planning_time
            actual_planning_time = time.time() - start_time
            
            if success and plan and hasattr(plan, 'joint_trajectory') and len(plan.joint_trajectory.points) > 0:
                points = plan.joint_trajectory.points
                path_length = calculate_path_length(points)
                
                waypoints = [list(p.positions) for p in points]
                
                print(f"✅ 规划成功!")
                print(f" - 规划时间: {actual_planning_time:.4f}s")
                print(f" - 路径长度 (L2): {path_length:.4f} rad")
                print(f" - 航点数量: {len(waypoints)}")
                
                record = {
                    "time": actual_planning_time,
                    "path_length": path_length,
                    "trajectory_points": waypoints
                }
                all_data.append(record)
            else:
                print("❌ 规划失败或未生成有效路径。")
                
    except KeyboardInterrupt:
        print("\n捕获到中断退出信号...")
    except Exception as e:
        print(f"\n发生异常: {e}")
        
    finally:
        if all_data:
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(all_data, f, indent=4, ensure_ascii=False)
            print(f"\n[保存] 成功保存 {len(all_data)} 条记录至 {os.path.abspath(output_file)}")
        else:
            print("\n[退出] 未收集到任何有效数据，未生成文件。")

if __name__ == '__main__':
    main()
