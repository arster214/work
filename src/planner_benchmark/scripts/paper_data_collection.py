#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
论文数据采集脚本
用于系统化测试路径规划算法并采集性能数据
"""

import rospy
import moveit_commander
import sys
import time
import csv
import os
import yaml
import argparse
from datetime import datetime


class PaperDataCollector:
    """论文数据采集器"""
    
    def __init__(self, config_file=None):
        """初始化数据采集器"""
        
        # 初始化 ROS 和 MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('paper_data_collector', anonymous=True)
        
        # 加载配置
        self.config = self.load_config(config_file)
        
        # 初始化 MoveIt 接口
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(self.config['group_name'])
        
        # 存储结果
        self.results = []
        
        # 打印初始化信息
        self.print_initialization_info()
    
    def load_config(self, config_file):
        """加载配置文件"""
        # 如果未提供 config_file，尝试从 ROS 参数获取
        if not config_file and rospy.has_param('~config_file'):
            config_file = rospy.get_param('~config_file')
            rospy.loginfo("从 ROS 参数加载配置文件: %s" % config_file)

        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
        else:
            # 尝试自动查找配置文件
            import rospkg
            try:
                rospack = rospkg.RosPack()
                pkg_path = rospack.get_path('planner_benchmark')
                default_config_path = os.path.join(pkg_path, 'config', 'test_poses.yaml')
                if os.path.exists(default_config_path):
                    rospy.loginfo("使用配置文件: %s" % default_config_path)
                    with open(default_config_path, 'r') as f:
                        config = yaml.safe_load(f)
                else:
                    rospy.logwarn("找不到配置文件，使用硬编码默认配置")
                    config = self.get_default_config()
            except:
                rospy.logwarn("无法查找配置文件，使用硬编码默认配置")
                config = self.get_default_config()
        
        return config
    
    def get_default_config(self):
        """获取默认配置"""
        return {
            'group_name': 'r_arm',
            'home_joints': [0.0] * 7,
            'goal_poses': [
                {
                    'id': 'pose_1',
                    'description': '目标姿态 1',
                    'joints': [1.862, 0.664, 0.345, 1.449, -1.457, 1.128, 0.849]
                },
                {
                    'id': 'pose_2',
                    'description': '目标姿态 2',
                    'joints': [-1.012, -1.372, 2.331, 0.195, -2.949, -0.966, -3.475]
                },
                {
                    'id': 'pose_3',
                    'description': '目标姿态 3',
                    'joints': [2.630, 0.850, -0.761, 1.432, -2.464, -0.535, 1.681]
                }
            ],
            'planners': [
                {
                    'name': 'BasicRRT',
                    'display_name': 'RRT (Baseline)',
                    'params': {
                        'range': 0.5
                    }
                },
                {
                    'name': 'BasicRRTStar',
                    'display_name': 'RRT* (Baseline)',
                    'params': {
                        'range': 0.5,
                        'rewire_radius': 0.75
                    }
                },
                {
                    'name': 'ImprovedRRT',
                    'display_name': 'T-RRT+RRT* (Ours)',
                    'params': {
                        'enable_trrt': True,
                        'enable_rrt_star': True,
                        'range': 0.5,
                        'goal_bias': 0.05,
                        'init_temperature': 100.0,
                        'temp_change_factor': 0.1,
                        'rewire_radius': 0.75
                    }
                }
            ],
            'test_params': {
                'num_runs': 20,
                'timeout': 10.0,
                'reset_delay': 1.0,  # 增加到 1 秒，让系统更稳定
                'output_dir': os.path.expanduser('~/benchmark_results')
            }
        }
    
    def print_initialization_info(self):
        """打印初始化信息"""
        rospy.loginfo("=" * 80)
        rospy.loginfo("论文数据采集系统")
        rospy.loginfo("=" * 80)
        rospy.loginfo("规划组: %s" % self.config['group_name'])
        rospy.loginfo("参考坐标系: %s" % self.group.get_planning_frame())
        rospy.loginfo("末端执行器: %s" % self.group.get_end_effector_link())
        rospy.loginfo("关节名称: %s" % self.group.get_active_joints())
        rospy.loginfo("目标姿态数量: %d" % len(self.config['goal_poses']))
        rospy.loginfo("规划器数量: %d" % len(self.config['planners']))
        rospy.loginfo("每个姿态重复: %d 次" % self.config['test_params']['num_runs'])
        rospy.loginfo("=" * 80)
    
    def set_planner(self, planner_config):
        """设置规划器和参数"""
        planner_name = planner_config['name']
        
        # 根据规划器名称确定使用哪个 planning pipeline
        if planner_name in ['BasicRRT', 'BasicRRTStar']:
            # 使用 baseline_rrt pipeline
            self.group.set_planning_pipeline_id("baseline_rrt")
            rospy.loginfo("使用 planning pipeline: baseline_rrt")
        elif planner_name == 'ImprovedRRT':
            # 使用 improved_rrt pipeline
            self.group.set_planning_pipeline_id("improved_rrt")
            rospy.loginfo("使用 planning pipeline: improved_rrt")
        else:
            # 使用默认的 ompl pipeline
            self.group.set_planning_pipeline_id("ompl")
            rospy.loginfo("使用 planning pipeline: ompl")
        
        self.group.set_planner_id(planner_name)
        
        # 设置规划器参数到 ROS 参数服务器
        if 'params' in planner_config:
            namespace = "/move_group/planner_configs/%s/" % planner_name
            for key, value in planner_config['params'].items():
                param_name = namespace + key
                rospy.set_param(param_name, value)
        
        rospy.loginfo("切换到规划器: %s" % planner_config['display_name'])
        
        # 等待规划器切换完成，避免崩溃
        rospy.sleep(2.0)
    
    def reset_to_home(self):
        """重置到 home 姿态"""
        home_joints = self.config['home_joints']
        
        # 对于数据收集，我们只需要设置起始状态，不需要实际执行
        # 使用 set_start_state_to_current_state() 会从当前机器人状态读取
        # 但我们想要从固定的 home 位置开始规划
        
        # 方法：直接设置当前状态为 home（仅用于规划，不执行）
        # 这样每次规划都从相同的起点开始，保证测试一致性
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        
        robot_state = RobotState()
        robot_state.joint_state.name = self.group.get_active_joints()
        robot_state.joint_state.position = home_joints
        
        self.group.set_start_state(robot_state)
        self.group.clear_pose_targets()
        
        # 短暂等待确保状态设置生效
        rospy.sleep(self.config['test_params']['reset_delay'])
    
    def plan_to_goal(self, goal_joints, timeout):
        """
        规划到目标姿态
        
        返回: (success, planning_time, trajectory)
        """
        # 验证目标状态是否有效
        try:
            self.group.set_joint_value_target(goal_joints)
        except Exception as e:
            rospy.logerr("设置目标失败: %s" % str(e))
            return False, 0.0, None
        
        self.group.clear_pose_targets()
        self.group.set_planning_time(timeout)
        
        # 短暂等待确保设置生效
        rospy.sleep(0.1)
        
        start_time = time.time()
        plan = self.group.plan()
        planning_time = time.time() - start_time
        
        # 兼容不同版本的 MoveIt
        if isinstance(plan, tuple):
            success = plan[0]
            trajectory = plan[1]
        else:
            success = len(plan.joint_trajectory.points) > 0
            trajectory = plan
        
        # 规划完成后清理状态
        self.group.clear_pose_targets()
        self.group.stop()
        
        # 等待确保状态稳定
        rospy.sleep(0.3)
        
        return success, planning_time, trajectory
    
    def extract_metrics(self, trajectory):
        """
        从轨迹中提取性能指标
        
        返回指标字典
        """
        if not trajectory or not hasattr(trajectory, 'joint_trajectory'):
            return None
        
        points = trajectory.joint_trajectory.points
        if len(points) == 0:
            return None
        
        # 1. 路径点数量
        num_waypoints = len(points)
        
        # 2. 路径长度（关节空间欧氏距离）
        path_length = 0.0
        for i in range(1, len(points)):
            dist = 0.0
            for j in range(len(points[i].positions)):
                diff = points[i].positions[j] - points[i-1].positions[j]
                dist += diff * diff
            path_length += (dist ** 0.5)
        
        # 3. 平滑度（加速度变化）
        smoothness = 0.0
        if len(points) > 2:
            for i in range(1, len(points) - 1):
                for j in range(len(points[i].positions)):
                    # 二阶差分近似加速度
                    acc = points[i+1].positions[j] - 2*points[i].positions[j] + points[i-1].positions[j]
                    smoothness += abs(acc)
            smoothness /= (len(points) - 2)
        
        # 4. 轨迹执行时间
        execution_time = 0.0
        if len(points) > 0 and hasattr(points[-1], 'time_from_start'):
            execution_time = points[-1].time_from_start.to_sec()
        
        return {
            'num_waypoints': num_waypoints,
            'path_length': path_length,
            'smoothness': smoothness,
            'execution_time': execution_time
        }
    
    def run_single_test(self, planner_config, pose_config, run_number):
        """
        运行单次测试
        
        返回: 测试结果字典
        """
        # 重置到 home
        self.reset_to_home()
        
        # 执行规划
        goal_joints = pose_config['joints']
        timeout = self.config['test_params']['timeout']
        
        success, planning_time, trajectory = self.plan_to_goal(goal_joints, timeout)
        
        # 构建结果
        result = {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'planner_name': planner_config['name'],
            'planner_display_name': planner_config['display_name'],
            'pose_id': pose_config['id'],
            'pose_description': pose_config['description'],
            'run_number': run_number,
            'success': success,
            'planning_time': planning_time
        }
        
        # 提取轨迹指标
        if success:
            metrics = self.extract_metrics(trajectory)
            if metrics:
                result.update(metrics)
            else:
                # 如果无法提取指标，设为 None
                result.update({
                    'num_waypoints': None,
                    'path_length': None,
                    'smoothness': None,
                    'execution_time': None
                })
        else:
            # 失败时指标为 None
            result.update({
                'num_waypoints': None,
                'path_length': None,
                'smoothness': None,
                'execution_time': None
            })
        
        return result
    
    def run_full_benchmark(self):
        """运行完整的基准测试"""
        num_runs = self.config['test_params']['num_runs']
        total_tests = len(self.config['planners']) * len(self.config['goal_poses']) * num_runs
        current_test = 0
        
        rospy.loginfo("\n" + "=" * 80)
        rospy.loginfo("开始数据采集")
        rospy.loginfo("总测试数: %d" % total_tests)
        rospy.loginfo("=" * 80 + "\n")
        
        start_time = time.time()
        
        # 遍历每个规划器
        for planner_config in self.config['planners']:
            rospy.loginfo("\n" + "-" * 80)
            rospy.loginfo("测试规划器: %s" % planner_config['display_name'])
            rospy.loginfo("-" * 80)
            
            # 设置规划器
            self.set_planner(planner_config)
            
            planner_success_count = 0
            planner_total_count = 0
            
            # 重复测试轮次
            for run in range(1, num_runs + 1):
                rospy.loginfo("\n  第 %d/%d 轮测试" % (run, num_runs))
                
                # 遍历每个目标姿态（按顺序：pose1 → pose2 → pose3 → pose4）
                for pose_config in self.config['goal_poses']:
                    current_test += 1
                    
                    rospy.loginfo("    目标: %s (总进度: %d/%d)..." % 
                                (pose_config['description'], current_test, total_tests))
                    
                    try:
                        result = self.run_single_test(planner_config, pose_config, run)
                        self.results.append(result)
                        
                        if result['success']:
                            planner_success_count += 1
                            rospy.loginfo("      ✓ 成功 | 时间: %.3fs | 路径长度: %.3f | 路径点: %d" %
                                        (result['planning_time'],
                                         result['path_length'] if result['path_length'] else 0,
                                         result['num_waypoints'] if result['num_waypoints'] else 0))
                        else:
                            rospy.logwarn("      ✗ 失败 | 时间: %.3fs" % result['planning_time'])
                        
                        planner_total_count += 1
                        
                    except Exception as e:
                        rospy.logerr("      ✗ 异常: %s" % str(e))
                        # 记录失败
                        self.results.append({
                            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                            'planner_name': planner_config['name'],
                            'planner_display_name': planner_config['display_name'],
                            'pose_id': pose_config['id'],
                            'pose_description': pose_config['description'],
                            'run_number': run,
                            'success': False,
                            'planning_time': None,
                            'num_waypoints': None,
                            'path_length': None,
                            'smoothness': None,
                            'execution_time': None
                        })
                        planner_total_count += 1
                
                # 显示该轮的进度
                rospy.loginfo("    第 %d 轮完成" % run)
                
                # 轮次之间增加等待时间，让系统稳定
                if run < num_runs:
                    rospy.loginfo("    等待 5 秒后继续下一轮...")
                    rospy.sleep(5.0)
            
            # 显示该规划器的总体统计
            overall_success_rate = 100.0 * planner_success_count / planner_total_count if planner_total_count > 0 else 0
            rospy.loginfo("\n  %s 总体成功率: %d/%d (%.1f%%)" % 
                        (planner_config['display_name'],
                         planner_success_count, planner_total_count, overall_success_rate))
        
        total_time = time.time() - start_time
        
        rospy.loginfo("\n" + "=" * 80)
        rospy.loginfo("数据采集完成!")
        rospy.loginfo("总耗时: %.1f 分钟" % (total_time / 60.0))
        rospy.loginfo("=" * 80 + "\n")
    
    def save_results(self):
        """保存结果到 CSV 文件"""
        output_dir = os.path.expanduser(self.config['test_params']['output_dir'])
        
        # 创建输出目录
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            rospy.loginfo("创建输出目录: %s" % output_dir)
        
        # 生成文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(output_dir, "benchmark_raw_%s.csv" % timestamp)
        
        # 写入 CSV
        if self.results:
            fieldnames = [
                'timestamp', 'planner_name', 'planner_display_name',
                'pose_id', 'pose_description', 'run_number',
                'success', 'planning_time', 'num_waypoints',
                'path_length', 'smoothness', 'execution_time'
            ]
            
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                for result in self.results:
                    writer.writerow(result)
            
            rospy.loginfo("结果已保存到: %s" % filename)
            rospy.loginfo("总记录数: %d" % len(self.results))
        else:
            rospy.logwarn("没有结果可保存")
    
    def shutdown(self):
        """关闭"""
        moveit_commander.roscpp_shutdown()


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='论文数据采集脚本')
    parser.add_argument('--config', type=str, help='配置文件路径')
    parser.add_argument('--runs', type=int, help='每个姿态的重复次数')
    parser.add_argument('--timeout', type=float, help='规划超时时间（秒）')
    parser.add_argument('--output', type=str, help='输出目录')
    parser.add_argument('--planner', type=str, help='只测试指定的规划器（BasicRRT, BasicRRTStar, ImprovedRRT）')
    
    # 使用 parse_known_args 避免 ROS 参数导致报错
    args, unknown = parser.parse_known_args()
    
    try:
        # 创建数据采集器
        collector = PaperDataCollector(config_file=args.config)
        
        # 优先使用命令行参数，其次尝试读取 ROS 参数
        
        # 运行次数
        if args.runs:
            collector.config['test_params']['num_runs'] = args.runs
        elif rospy.has_param('~num_runs'):
            collector.config['test_params']['num_runs'] = rospy.get_param('~num_runs')
            
        # 超时时间
        if args.timeout:
            collector.config['test_params']['timeout'] = args.timeout
        elif rospy.has_param('~timeout'):
            collector.config['test_params']['timeout'] = rospy.get_param('~timeout')
            
        # 输出目录
        if args.output:
            collector.config['test_params']['output_dir'] = args.output
        elif rospy.has_param('~output_dir'):
            collector.config['test_params']['output_dir'] = rospy.get_param('~output_dir')
        
        # 如果指定了单个规划器，只测试该规划器
        if args.planner:
            original_planners = collector.config['planners']
            collector.config['planners'] = [p for p in original_planners if p['name'] == args.planner]
            if not collector.config['planners']:
                rospy.logerr("找不到规划器: %s" % args.planner)
                rospy.loginfo("可用的规划器: %s" % ', '.join([p['name'] for p in original_planners]))
                return
            rospy.loginfo("只测试规划器: %s" % args.planner)
        
        # 运行基准测试
        collector.run_full_benchmark()
        
        # 保存结果
        collector.save_results()
        
        # 关闭
        collector.shutdown()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("数据采集被中断")
    except KeyboardInterrupt:
        rospy.loginfo("用户中断数据采集")
    except Exception as e:
        rospy.logerr("发生错误: %s" % str(e))
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
