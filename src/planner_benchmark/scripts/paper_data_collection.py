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
import actionlib
import json
import shlex
import signal
import subprocess
import tempfile
import threading
import traceback
import rosnode
from datetime import datetime
from std_msgs.msg import String as StringMsg


class PlannerStatsListener:
    """监听规划器发布的树统计信息。"""

    def __init__(self, topic_name):
        self.topic_name = topic_name
        self._messages = []
        self._lock = threading.Lock()
        self._subscriber = rospy.Subscriber(topic_name, StringMsg, self._callback, queue_size=10)

    def _callback(self, message):
        try:
            payload = json.loads(message.data)
        except Exception:
            rospy.logwarn_throttle(5.0, "无法解析规划器统计消息: %s", self.topic_name)
            return

        if not isinstance(payload, dict):
            return

        with self._lock:
            self._messages.append(payload)
            if len(self._messages) > 50:
                self._messages = self._messages[-50:]

    def get_latest(self, min_stamp=None, planner_name=None):
        with self._lock:
            messages = list(self._messages)

        candidates = []
        for payload in messages:
            stamp = payload.get('stamp')
            try:
                stamp = float(stamp) if stamp is not None else None
            except (TypeError, ValueError):
                stamp = None

            if min_stamp is not None and stamp is not None and stamp + 1e-6 < min_stamp:
                continue

            if planner_name:
                candidate_names = {
                    payload.get('planner_name'),
                    payload.get('planner_id'),
                    payload.get('requested_planner_id')
                }
                candidate_names = {name for name in candidate_names if isinstance(name, str) and name}
                if candidate_names and planner_name not in candidate_names:
                    continue

            candidates.append(payload)

        if not candidates:
            return None

        def safe_float(value):
            try:
                return float(value) if value is not None else None
            except (TypeError, ValueError):
                return None

        def safe_bool(value):
            if isinstance(value, bool):
                return value
            if isinstance(value, str):
                lowered = value.strip().lower()
                if lowered in ('true', '1', 'yes', 'on'):
                    return True
                if lowered in ('false', '0', 'no', 'off'):
                    return False
            if isinstance(value, (int, float)):
                return bool(value)
            return None

        def tree_signal_score(item):
            vertices = safe_float(item.get('tree_vertices'))
            edges = safe_float(item.get('tree_edges'))
            solution_states = safe_float(item.get('solution_states'))
            planning_succeeded = safe_bool(item.get('planning_succeeded'))

            # 同一轮规划若出现多条 stats，优先使用真正带树信息的那条。
            # 实测某些 MoveIt/adapter 路径下会冒出成功但 0/0 的“瘦消息”，
            # 直接取最新一条会把有效树规模覆盖掉。
            has_nonzero_tree = (
                (vertices is not None and vertices > 0.0) or
                (edges is not None and edges > 0.0)
            )
            has_tree_fields = vertices is not None or edges is not None

            return (
                1 if has_nonzero_tree else 0,
                1 if planning_succeeded is True else 0,
                1 if has_tree_fields else 0,
                int(solution_states or 0),
                int(vertices or 0),
                int(edges or 0)
            )

        candidates.sort(
            key=lambda item: (
                tree_signal_score(item),
                float(item.get('stamp', 0.0) or 0.0),
                int(item.get('attempt_index', 0) or 0)
            )
        )
        return dict(candidates[-1])

    def wait_for_stats(self, min_stamp=None, planner_name=None, timeout=1.0, poll_interval=0.05):
        deadline = time.time() + max(0.0, timeout)

        while time.time() < deadline and not rospy.is_shutdown():
            stats = self.get_latest(min_stamp=min_stamp, planner_name=planner_name)
            if stats is not None:
                return stats
            rospy.sleep(max(0.01, poll_interval))

        return self.get_latest(min_stamp=min_stamp, planner_name=planner_name)


class PaperDataCollector:
    """论文数据采集器"""

    AUTO_GROUP_NAMES = {'auto', 'automatic'}
    DEFAULT_SUPPORTED_GROUPS = {
        'RRT': {'l_arm', 'r_arm', 'dual_arms'},
        'RRTConnect': {'l_arm', 'r_arm', 'dual_arms'},
        'TRRT': {'l_arm', 'r_arm', 'dual_arms'},
        'BiTRRT': {'l_arm', 'r_arm', 'dual_arms'},
        'RRTstar': {'l_arm', 'r_arm', 'dual_arms'},
        'DualArmTRRT': {'dual_arms'}
    }
    
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

        # 在真正开始采集前做一次配置一致性检查，避免跑到一半才发现维度或 pipeline 不匹配
        self.validate_configuration()
        
        # 存储结果
        self.results = []
        
        # 打印初始化信息
        self.print_initialization_info()

    def validate_configuration(self):
        """校验规划组、关节维度和规划器选择是否兼容"""
        active_joint_names = self.group.get_active_joints()
        expected_dof = len(active_joint_names)
        start_joints = self.config['test_case']['start_joints']
        goal_joints = self.config['test_case']['goal_pose']['joints']
        group_name = self.config['group_name']

        if len(start_joints) != expected_dof:
            raise ValueError(
                "start_joints 维度与规划组 %s 不匹配: 期望 %d, 实际 %d。" %
                (group_name, expected_dof, len(start_joints))
            )

        if len(goal_joints) != expected_dof:
            raise ValueError(
                "goal_pose.joints 维度与规划组 %s 不匹配: 期望 %d, 实际 %d。" %
                (group_name, expected_dof, len(goal_joints))
            )

        compatible_planners = []
        skipped_planners = []
        for planner_config in self.config['planners']:
            if self.is_planner_supported_for_group(planner_config, group_name):
                compatible_planners.append(planner_config)
            else:
                skipped_planners.append(planner_config['name'])

        if skipped_planners:
            rospy.logwarn("规划组 %s 不支持以下规划器，本次采集将自动跳过: %s",
                          group_name, ', '.join(skipped_planners))

        if not compatible_planners:
            raise ValueError(
                "当前规划组 %s 没有可用的规划器，请检查 planners 配置和 supported_groups 设置。" %
                group_name
            )

        self.config['planners'] = compatible_planners

    @classmethod
    def get_supported_groups_for_planner(cls, planner_config):
        """解析规划器支持的 planning group 范围"""
        supported_groups = planner_config.get('supported_groups')
        if supported_groups:
            return set(supported_groups)
        return cls.DEFAULT_SUPPORTED_GROUPS.get(planner_config['name'])

    @classmethod
    def is_planner_supported_for_group(cls, planner_config, group_name):
        """判断当前规划器是否支持给定 planning group"""
        supported_groups = cls.get_supported_groups_for_planner(planner_config)
        if not supported_groups:
            return True
        return group_name in supported_groups

    @staticmethod
    def is_zero_joint_vector(joint_values, tolerance=1e-9):
        """判断一组关节是否可视为全零，用于自动识别单臂/双臂模式"""
        if not isinstance(joint_values, (list, tuple)) or len(joint_values) == 0:
            return False
        try:
            return all(abs(float(value)) <= tolerance for value in joint_values)
        except (TypeError, ValueError):
            return False

    @staticmethod
    def get_dual_arm_joint_map(joint_values):
        """从配置中提取左右臂关节；若不是双臂字典格式则返回 None"""
        if not isinstance(joint_values, dict):
            return None

        left_joints = joint_values.get('l_arm', joint_values.get('left_arm'))
        right_joints = joint_values.get('r_arm', joint_values.get('right_arm'))
        if left_joints is None or right_joints is None:
            return None

        return {
            'l_arm': list(left_joints),
            'r_arm': list(right_joints)
        }

    @classmethod
    def infer_group_name_from_joint_config(cls, start_joints_raw, goal_joints_raw):
        """按左右臂起终点自动识别规划组"""
        start_arm_map = cls.get_dual_arm_joint_map(start_joints_raw)
        goal_arm_map = cls.get_dual_arm_joint_map(goal_joints_raw)
        if not start_arm_map or not goal_arm_map:
            return None, None

        left_inactive = (
            cls.is_zero_joint_vector(start_arm_map['l_arm']) and
            cls.is_zero_joint_vector(goal_arm_map['l_arm'])
        )
        right_inactive = (
            cls.is_zero_joint_vector(start_arm_map['r_arm']) and
            cls.is_zero_joint_vector(goal_arm_map['r_arm'])
        )

        if left_inactive and right_inactive:
            raise ValueError("左右臂的起点和终点都为全零，无法判断当前是单臂还是双臂测试。")

        if left_inactive:
            return 'r_arm', ['r_arm']
        if right_inactive:
            return 'l_arm', ['l_arm']
        return 'dual_arms', ['l_arm', 'r_arm']

    @staticmethod
    def normalize_joint_values(joint_values, group_name, field_name):
        """支持单臂平铺列表和双臂按组填写两种关节配置格式"""
        def coerce_sequence(raw_values, raw_field_name):
            if isinstance(raw_values, (list, tuple)):
                values = list(raw_values)
                # 兼容误写成 ["1 2 3"] 这种 YAML 流式列表
                if len(values) == 1 and isinstance(values[0], str):
                    parts = values[0].replace(',', ' ').split()
                    if len(parts) > 1:
                        try:
                            return [float(part) for part in parts]
                        except ValueError:
                            pass
                return values

            if isinstance(raw_values, str):
                parts = raw_values.replace(',', ' ').split()
                if parts:
                    try:
                        return [float(part) for part in parts]
                    except ValueError:
                        raise ValueError("%s 包含无法解析的关节值: %s" % (raw_field_name, raw_values))

            raise ValueError("%s 必须是关节列表。" % raw_field_name)

        if joint_values is None:
            raise ValueError("%s 为空。" % field_name)

        if isinstance(joint_values, (list, tuple)):
            return coerce_sequence(joint_values, field_name)

        if not isinstance(joint_values, dict):
            raise ValueError("%s 必须是关节列表，或按规划组组织的字典。" % field_name)

        if 'joints' in joint_values and isinstance(joint_values['joints'], (list, tuple)):
            return coerce_sequence(joint_values['joints'], "%s.joints" % field_name)

        if group_name == 'dual_arms':
            if 'l_arm' in joint_values and 'r_arm' in joint_values:
                return (
                    coerce_sequence(joint_values['l_arm'], "%s.l_arm" % field_name) +
                    coerce_sequence(joint_values['r_arm'], "%s.r_arm" % field_name)
                )
            if 'left_arm' in joint_values and 'right_arm' in joint_values:
                return (
                    coerce_sequence(joint_values['left_arm'], "%s.left_arm" % field_name) +
                    coerce_sequence(joint_values['right_arm'], "%s.right_arm" % field_name)
                )
            raise ValueError(
                "%s 在 dual_arms 模式下必须提供 l_arm/r_arm 两组关节。" % field_name
            )

        if group_name in joint_values and isinstance(joint_values[group_name], (list, tuple, str)):
            return coerce_sequence(joint_values[group_name], "%s.%s" % (field_name, group_name))
        if group_name == 'l_arm' and 'left_arm' in joint_values and isinstance(joint_values['left_arm'], (list, tuple, str)):
            return coerce_sequence(joint_values['left_arm'], "%s.left_arm" % field_name)
        if group_name == 'r_arm' and 'right_arm' in joint_values and isinstance(joint_values['right_arm'], (list, tuple, str)):
            return coerce_sequence(joint_values['right_arm'], "%s.right_arm" % field_name)

        raise ValueError(
            "%s 无法匹配当前规划组 %s。单臂模式请直接给列表，或使用 {\"%s\": [...]}。"
            % (field_name, group_name, group_name)
        )
    
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
        
        return self.normalize_config(config)

    def normalize_config(self, config):
        """将配置统一为单测试用例格式，同时兼容旧版多目标配置"""
        if not config:
            raise ValueError("配置文件为空，无法开始数据采集。")

        normalized = dict(config)
        start_joints_raw = normalized.get('start_joints', normalized.get('home_joints'))
        if start_joints_raw is None:
            raise ValueError("配置缺少 start_joints/home_joints。")

        goal_pose = normalized.get('goal_pose')
        if goal_pose is None:
            goal_poses = normalized.get('goal_poses', [])
            if not goal_poses:
                raise ValueError("配置缺少 goal_pose，且 goal_poses 为空。")
            if len(goal_poses) > 1:
                rospy.logwarn("检测到旧版多目标配置，当前单轮测试模式只会使用第一组目标: %s",
                              goal_poses[0].get('id', 'unknown'))
            goal_pose = goal_poses[0]
        goal_pose = dict(goal_pose)

        requested_group_name = str(normalized.get('group_name', 'auto')).strip()
        goal_joints_raw = goal_pose.get('joints')
        inferred_group_name, active_arms = self.infer_group_name_from_joint_config(
            start_joints_raw,
            goal_joints_raw
        )

        if requested_group_name.lower() in self.AUTO_GROUP_NAMES:
            if inferred_group_name is None:
                raise ValueError(
                    "group_name=auto 时，start_joints 和 goal_pose.joints 都必须提供 l_arm/r_arm 两组关节，"
                    "并通过将某一侧起点和终点设为全零来表示单臂测试。"
                )
            group_name = inferred_group_name
            group_name_source = 'auto'
        else:
            group_name = requested_group_name
            group_name_source = 'explicit'
            if inferred_group_name and inferred_group_name != group_name:
                rospy.logwarn(
                    "配置中显式指定 group_name=%s，但按左右臂零向量规则可推断为 %s。"
                    "当前仍按显式 group_name 执行；如需自动判定，请把 group_name 设为 auto。",
                    group_name,
                    inferred_group_name
                )

        normalized['group_name'] = group_name
        normalized['group_name_source'] = group_name_source
        normalized['requested_group_name'] = requested_group_name
        normalized['inferred_group_name'] = inferred_group_name
        normalized['active_arms'] = active_arms if active_arms else (
            ['l_arm', 'r_arm'] if group_name == 'dual_arms' else [group_name]
        )

        start_joints = self.normalize_joint_values(start_joints_raw, group_name, 'start_joints')
        goal_pose['joints'] = self.normalize_joint_values(
            goal_joints_raw,
            group_name,
            'goal_pose.joints'
        )

        test_case = normalized.get('test_case', {})
        normalized['test_case'] = {
            'id': test_case.get('id', goal_pose.get('id', 'case_1')),
            'description': test_case.get('description', goal_pose.get('description', '单轮测试')),
            'group_name': group_name,
            'active_arms': list(normalized['active_arms']),
            'joint_dof': len(start_joints),
            'start_joints': list(start_joints),
            'goal_pose': dict(goal_pose)
        }

        # 保留兼容字段，避免其余代码或旧分析脚本立刻失效
        normalized['start_joints'] = list(start_joints)
        normalized['home_joints'] = list(start_joints)
        normalized['goal_pose'] = dict(goal_pose)

        test_params = dict(normalized.get('test_params', {}))
        test_params.setdefault('num_runs', 20)
        test_params.setdefault('timeout', 30.0)
        test_params.setdefault('reset_delay', 1.0)
        test_params.setdefault('output_dir', os.path.expanduser('~/benchmark_results'))
        test_params.setdefault('planning_watchdog_padding', 15.0)
        test_params.setdefault('planning_attempts', 3)
        test_params.setdefault('worker_poll_interval', 0.2)
        test_params.setdefault('kill_move_group_on_timeout', True)
        test_params.setdefault('move_group_node_name', '/move_group')
        test_params.setdefault('move_group_action_name', 'move_group')
        test_params.setdefault('move_group_recovery_wait', 15.0)
        test_params.setdefault('move_group_shutdown_wait', 5.0)
        test_params.setdefault('move_group_ready_wait', 15.0)
        test_params.setdefault('reconnect_retry_delay', 2.0)
        test_params.setdefault('max_recovery_attempts', 1)
        test_params.setdefault('restart_move_group_command', '')
        test_params.setdefault('stop_on_unrecoverable_move_group', True)
        normalized['test_params'] = test_params

        return normalized
    
    def get_default_config(self):
        """获取默认配置"""
        return {
            'group_name': 'auto',
            'start_joints': {
                'l_arm': [0.0] * 7,
                'r_arm': [0.0] * 7
            },
            'goal_pose': {
                'id': 'pose_1',
                'description': '目标姿态 1',
                'joints': {
                    'l_arm': [0.0] * 7,
                    'r_arm': [1.862, 0.664, 0.345, 1.449, -1.457, 1.128, 0.849]
                }
            },
            'planners': [
                {
                    'name': 'RRTConnect',
                    'display_name': 'OMPL RRTConnect',
                    'supported_groups': ['l_arm', 'r_arm', 'dual_arms']
                },
                {
                    'name': 'TRRT',
                    'display_name': 'OMPL TRRT',
                    'supported_groups': ['l_arm', 'r_arm', 'dual_arms']
                },
                {
                    'name': 'BiTRRT',
                    'display_name': 'OMPL BiTRRT',
                    'supported_groups': ['l_arm', 'r_arm', 'dual_arms'],
                    'params': {
                        'range': 0.30,
                        'init_temperature': 100.0,
                        'temp_change_factor': 0.1,
                        'frontier_node_ratio': 0.1
                    }
                },
                {
                    'name': 'DualArmTRRT',
                    'display_name': 'DualArmTRRT (dual_arm_rrt)',
                    'supported_groups': ['dual_arms'],
                    'params': {
                        'range': 0.1,
                        'goal_bias': 0.05,
                        'init_temperature': 100.0,
                        'temp_change_factor': 0.1
                    }
                }
            ],
            'test_params': {
                'num_runs': 50,
                'timeout': 30.0,
                'reset_delay': 1.0,  # 增加到 1 秒，让系统更稳定
                'output_dir': os.path.expanduser('~/benchmark_results'),
                'planning_watchdog_padding': 15.0,
                'planning_attempts': 3,
                'worker_poll_interval': 0.2,
                'kill_move_group_on_timeout': True,
                'move_group_node_name': '/move_group',
                'move_group_action_name': 'move_group',
                'move_group_recovery_wait': 15.0,
                'move_group_shutdown_wait': 5.0,
                'move_group_ready_wait': 15.0,
                'reconnect_retry_delay': 2.0,
                'max_recovery_attempts': 1,
                'restart_move_group_command': '',
                'stop_on_unrecoverable_move_group': True
            }
        }
    
    def print_initialization_info(self):
        """打印初始化信息"""
        rospy.loginfo("=" * 80)
        rospy.loginfo("论文数据采集系统")
        rospy.loginfo("=" * 80)
        rospy.loginfo("规划组: %s" % self.config['group_name'])
        if self.config.get('group_name_source') == 'auto':
            rospy.loginfo("规划组判定: 自动识别 (%s)" % ', '.join(self.config.get('active_arms', [])))
        rospy.loginfo("参考坐标系: %s" % self.group.get_planning_frame())
        rospy.loginfo("末端执行器: %s" % self.group.get_end_effector_link())
        rospy.loginfo("关节名称: %s" % self.group.get_active_joints())
        rospy.loginfo("测试模式: 单轮单测试用例")
        rospy.loginfo("起始关节: %s" % self.config['test_case']['start_joints'])
        rospy.loginfo("目标姿态: %s" % self.config['test_case']['goal_pose']['description'])
        rospy.loginfo("规划器数量: %d" % len(self.config['planners']))
        rospy.loginfo("重复次数: %d 次" % self.config['test_params']['num_runs'])
        rospy.loginfo("=" * 80)

    @staticmethod
    def resolve_pipeline_id(planner_name):
        """根据规划器名称选择对应的 planning pipeline"""
        if planner_name == 'DualArmTRRT':
            return "dual_rrt"
        return "ompl"

    @staticmethod
    def apply_planner_settings(group, planner_config, switch_delay=0.0):
        """将规划器配置应用到 MoveGroupCommander"""
        planner_name = planner_config['name']
        pipeline_id = PaperDataCollector.resolve_pipeline_id(planner_name)

        group.set_planning_pipeline_id(pipeline_id)
        group.set_planner_id(planner_name)

        if 'params' in planner_config:
            # MoveIt 多 pipeline 配置的真实参数通常位于 pipeline scoped namespace。
            # 同时兼容旧的 /move_group/planner_configs/<planner> 写法，避免不同 launch 结构下失效。
            namespaces = [
                "/move_group/planning_pipelines/%s/planner_configs/%s/" % (pipeline_id, planner_name),
                "/move_group/planner_configs/%s/" % planner_name,
            ]
            for key, value in planner_config['params'].items():
                for namespace in namespaces:
                    rospy.set_param(namespace + key, value)

        if switch_delay > 0.0:
            rospy.sleep(switch_delay)

        return pipeline_id

    @staticmethod
    def resolve_planner_stats_topic(planner_name):
        """返回规划器对应的树统计 topic；若无则返回 None。"""
        if planner_name == 'DualArmTRRT':
            return '/dual_arm_trrt/planning_stats'
        if planner_name in {'RRT', 'RRTConnect', 'TRRT', 'BiTRRT', 'RRTstar'}:
            return '/ompl_planning_stats'
        return None

    @staticmethod
    def extract_tree_metrics(planner_stats):
        """从规划器统计消息中提取树规模指标。"""
        metrics = {
            'tree_vertices': None,
            'tree_edges': None,
            'tree_size': None,
            'planner_data_available': None,
            'requested_planning_attempts': None
        }

        if not isinstance(planner_stats, dict):
            return metrics

        for field_name in ('tree_vertices', 'tree_edges'):
            value = planner_stats.get(field_name)
            try:
                metrics[field_name] = int(value) if value is not None else None
            except (TypeError, ValueError):
                metrics[field_name] = None

        availability = planner_stats.get('planner_data_available')
        if isinstance(availability, bool):
            metrics['planner_data_available'] = availability

        attempts = planner_stats.get('requested_planning_attempts')
        try:
            metrics['requested_planning_attempts'] = int(attempts) if attempts is not None else None
        except (TypeError, ValueError):
            metrics['requested_planning_attempts'] = None

        solution_states = planner_stats.get('solution_states')
        try:
            solution_states = int(solution_states) if solution_states is not None else None
        except (TypeError, ValueError):
            solution_states = None

        planning_succeeded = planner_stats.get('planning_succeeded')
        if isinstance(planning_succeeded, str):
            lowered = planning_succeeded.strip().lower()
            if lowered in ('true', '1', 'yes', 'on'):
                planning_succeeded = True
            elif lowered in ('false', '0', 'no', 'off'):
                planning_succeeded = False
            else:
                planning_succeeded = None

        # 对 RRT/RRTConnect/TRRT 这类采样树规划器来说，成功且存在 solution_states
        # 却同时拿到 0 顶点 / 0 边，通常不是“真实树为空”，而是拿到了无效/瘦 stats。
        # 这种值宁可视为缺失，也不要污染 benchmark。
        if (
            planning_succeeded is True and
            solution_states is not None and solution_states > 0 and
            metrics['tree_vertices'] == 0 and
            metrics['tree_edges'] == 0
        ):
            metrics['tree_vertices'] = None
            metrics['tree_edges'] = None

        # 论文里“树大小”默认指搜索树节点数，即顶点数量。
        metrics['tree_size'] = metrics['tree_vertices']
        return metrics

    @staticmethod
    def build_robot_start_state(group, start_joints, robot=None):
        """构造固定起始关节状态，优先基于完整当前状态覆写规划组关节。"""
        from moveit_msgs.msg import RobotState

        active_joint_names = list(group.get_active_joints())
        start_joint_values = list(start_joints)

        if len(active_joint_names) != len(start_joint_values):
            raise ValueError(
                "起始关节维度与规划组不匹配: 期望 %d, 实际 %d" %
                (len(active_joint_names), len(start_joint_values))
            )

        current_state = None
        if robot is not None:
            try:
                current_state = robot.get_current_state()
            except Exception:
                current_state = None

        if current_state and hasattr(current_state, 'joint_state'):
            robot_state = current_state
            robot_state.is_diff = False

            existing_names = list(robot_state.joint_state.name)
            existing_positions = list(robot_state.joint_state.position)
            name_to_index = {name: index for index, name in enumerate(existing_names)}

            for joint_name, joint_value in zip(active_joint_names, start_joint_values):
                if joint_name in name_to_index:
                    existing_positions[name_to_index[joint_name]] = joint_value
                else:
                    existing_names.append(joint_name)
                    existing_positions.append(joint_value)

            robot_state.joint_state.name = existing_names
            robot_state.joint_state.position = existing_positions
            return robot_state

        robot_state = RobotState()
        robot_state.joint_state.name = active_joint_names
        robot_state.joint_state.position = start_joint_values
        return robot_state

    @staticmethod
    def unpack_plan_result(plan):
        """兼容不同 MoveIt Python API 的返回格式"""
        if isinstance(plan, tuple):
            success = bool(plan[0])
            trajectory = plan[1] if len(plan) > 1 else None
        else:
            trajectory = plan
            success = bool(
                trajectory and
                hasattr(trajectory, 'joint_trajectory') and
                len(trajectory.joint_trajectory.points) > 0
            )
        return success, trajectory

    @staticmethod
    def terminate_process_tree(process):
        """结束整个子进程组，防止挂死 worker 残留"""
        if process is None or process.poll() is not None:
            return

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=3.0)
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:
                pass

    def is_move_group_available(self):
        """检查 move_group 节点是否存在"""
        try:
            node_names = rosnode.get_node_names()
            return self.config['test_params']['move_group_node_name'] in node_names
        except Exception:
            return False

    @staticmethod
    def can_connect_move_group_action(action_name, timeout_seconds):
        """检查 move_group action server 是否真正可连接"""
        from moveit_msgs.msg import MoveGroupAction

        if timeout_seconds <= 0.0:
            timeout_seconds = 0.01

        client = actionlib.SimpleActionClient(action_name, MoveGroupAction)
        return client.wait_for_server(rospy.Duration(timeout_seconds))

    def wait_for_move_group_shutdown(self, timeout_seconds):
        """等待 move_group 节点彻底下线，避免把旧注册状态当成恢复成功"""
        deadline = time.time() + max(0.0, timeout_seconds)
        poll_interval = max(0.1, float(self.config['test_params']['worker_poll_interval']))

        while time.time() < deadline and not rospy.is_shutdown():
            if not self.is_move_group_available():
                return True
            rospy.sleep(poll_interval)

        return not self.is_move_group_available()

    def wait_for_move_group(self, timeout_seconds):
        """等待 move_group 节点和 action server 都恢复可用"""
        deadline = time.time() + max(0.0, timeout_seconds)
        poll_interval = max(0.1, float(self.config['test_params']['worker_poll_interval']))
        action_name = self.config['test_params']['move_group_action_name']

        while time.time() < deadline and not rospy.is_shutdown():
            remaining = max(0.0, deadline - time.time())
            action_wait = min(1.0, remaining) if remaining > 0.0 else 0.1
            if self.is_move_group_available() and self.can_connect_move_group_action(action_name, action_wait):
                return True
            rospy.sleep(poll_interval)

        return self.is_move_group_available() and self.can_connect_move_group_action(action_name, 0.2)

    def attempt_move_group_recovery(self, reason):
        """在规划 worker 超时后尝试恢复 move_group"""
        attempts = max(1, int(self.config['test_params']['max_recovery_attempts']))
        node_name = self.config['test_params']['move_group_node_name']
        restart_command = str(self.config['test_params'].get('restart_move_group_command', '')).strip()
        should_kill = bool(self.config['test_params']['kill_move_group_on_timeout'])
        retry_delay = float(self.config['test_params']['reconnect_retry_delay'])
        wait_timeout = float(self.config['test_params']['move_group_recovery_wait'])
        shutdown_wait = float(self.config['test_params']['move_group_shutdown_wait'])

        rospy.logwarn("开始 MoveIt 恢复流程，原因: %s", reason)

        for attempt in range(1, attempts + 1):
            rospy.logwarn("MoveIt 恢复尝试 %d/%d", attempt, attempts)

            if should_kill and self.is_move_group_available():
                try:
                    subprocess.run(['rosnode', 'kill', node_name], check=False,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                    rospy.logwarn("已请求关闭 %s，等待其重启。", node_name)
                    if not self.wait_for_move_group_shutdown(shutdown_wait):
                        rospy.logwarn("%s 在 %.1f 秒内没有完全下线，可能存在僵尸注册或外层 launch 正在重启。",
                                      node_name, shutdown_wait)
                except Exception as exc:
                    rospy.logwarn("关闭 %s 时出现异常: %s", node_name, str(exc))

            if restart_command:
                try:
                    subprocess.Popen(
                        shlex.split(restart_command),
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        preexec_fn=os.setsid
                    )
                    rospy.logwarn("已执行 move_group 重启命令。")
                except Exception as exc:
                    rospy.logwarn("执行 move_group 重启命令失败: %s", str(exc))

            if self.wait_for_move_group(wait_timeout):
                rospy.loginfo("move_group 已恢复在线，且 action server 可连接。")
                return True

            if attempt < attempts:
                rospy.sleep(max(0.0, retry_delay))

        rospy.logerr("move_group 恢复失败，请检查外部启动脚本是否支持自动重启。")
        return False

    def should_abort_after_result(self, result):
        """在 MoveIt 明显未恢复时尽早停止采集，避免后续无意义重复失败"""
        if not bool(self.config['test_params'].get('stop_on_unrecoverable_move_group', True)):
            return False

        fatal_statuses = {
            'timeout',
            'move_group_unavailable',
            'move_group_action_unavailable',
            'move_group_commander_init_failed',
            'worker_error',
            'worker_exception'
        }
        return result.get('result_status') in fatal_statuses and result.get('move_group_recovered') is False

    @staticmethod
    def build_failure_result(planner_config, test_case_config, run_number, status, failure_reason,
                             planning_time=None, move_group_recovered=None, worker_return_code=None,
                             tree_metrics=None):
        """统一生成失败记录"""
        tree_metrics = tree_metrics or {}
        return {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'planner_name': planner_config['name'],
            'planner_display_name': planner_config['display_name'],
            'group_name': test_case_config.get('group_name'),
            'joint_dof': test_case_config.get('joint_dof'),
            'pose_id': test_case_config['goal_pose']['id'],
            'pose_description': test_case_config['goal_pose']['description'],
            'run_number': run_number,
            'success': False,
            'planning_time': planning_time,
            'tree_vertices': tree_metrics.get('tree_vertices'),
            'tree_edges': tree_metrics.get('tree_edges'),
            'tree_size': tree_metrics.get('tree_size'),
            'path_length': None,
            'smoothness': None,
            'execution_time': None,
            'result_status': status,
            'failure_reason': failure_reason,
            'move_group_recovered': move_group_recovered,
            'worker_return_code': worker_return_code
        }

    @staticmethod
    def build_success_result(planner_config, test_case_config, run_number, planning_time, metrics,
                             worker_return_code=0):
        """统一生成成功记录"""
        metrics = metrics or {}
        return {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'planner_name': planner_config['name'],
            'planner_display_name': planner_config['display_name'],
            'group_name': test_case_config.get('group_name'),
            'joint_dof': test_case_config.get('joint_dof'),
            'pose_id': test_case_config['goal_pose']['id'],
            'pose_description': test_case_config['goal_pose']['description'],
            'run_number': run_number,
            'success': True,
            'planning_time': planning_time,
            'tree_vertices': metrics.get('tree_vertices'),
            'tree_edges': metrics.get('tree_edges'),
            'tree_size': metrics.get('tree_size'),
            'path_length': metrics.get('path_length'),
            'smoothness': metrics.get('smoothness'),
            'execution_time': metrics.get('execution_time'),
            'result_status': 'success',
            'failure_reason': '',
            'move_group_recovered': None,
            'worker_return_code': worker_return_code
        }

    def make_worker_payload(self, planner_config, test_case_config, run_number):
        """构造单次规划 worker 输入"""
        return {
            'group_name': self.config['group_name'],
            'move_group_node_name': self.config['test_params']['move_group_node_name'],
            'move_group_action_name': self.config['test_params']['move_group_action_name'],
            'planner_config': planner_config,
            'test_case': test_case_config,
            'run_number': run_number,
            'timeout': float(self.config['test_params']['timeout']),
            'reset_delay': float(self.config['test_params']['reset_delay']),
            'move_group_ready_wait': float(self.config['test_params']['move_group_ready_wait'])
        }

    def run_planning_worker(self, planner_config, test_case_config, run_number):
        """在独立子进程中执行一次规划，防止主进程被 plan() 卡死"""
        payload = self.make_worker_payload(planner_config, test_case_config, run_number)
        timeout = float(self.config['test_params']['timeout'])
        watchdog_timeout = timeout + float(self.config['test_params']['planning_watchdog_padding'])

        with tempfile.TemporaryDirectory(prefix='planner_benchmark_') as temp_dir:
            input_path = os.path.join(temp_dir, 'worker_input.json')
            output_path = os.path.join(temp_dir, 'worker_output.json')

            with open(input_path, 'w') as handle:
                json.dump(payload, handle)

            command = [
                sys.executable,
                os.path.abspath(__file__),
                '--worker',
                '--worker-input', input_path,
                '--worker-output', output_path
            ]

            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid
            )

            start_time = time.time()
            stdout = ''
            stderr = ''

            try:
                stdout, stderr = process.communicate(timeout=watchdog_timeout)
            except subprocess.TimeoutExpired:
                self.terminate_process_tree(process)
                planning_time = time.time() - start_time
                recovered = self.attempt_move_group_recovery('planning_worker_timeout')
                return self.build_failure_result(
                    planner_config,
                    test_case_config,
                    run_number,
                    status='timeout',
                    failure_reason='planning_worker_timeout',
                    planning_time=planning_time,
                    move_group_recovered=recovered,
                    worker_return_code=None
                )

            worker_return_code = process.returncode
            if os.path.exists(output_path):
                try:
                    with open(output_path, 'r') as handle:
                        worker_result = json.load(handle)
                    worker_result.setdefault('worker_return_code', worker_return_code)
                    worker_result.setdefault('move_group_recovered', None)
                    if worker_return_code not in (0, None) and worker_result.get('result_status') != 'success':
                        worker_result['failure_reason'] = worker_result.get('failure_reason') or stderr.strip() or (
                            'worker_exit_code_%d' % worker_return_code
                        )
                        recovery_statuses = {
                            'worker_setup_failed',
                            'move_group_unavailable',
                            'move_group_action_unavailable',
                            'move_group_commander_init_failed',
                            'worker_exception'
                        }
                        if worker_result.get('result_status') in recovery_statuses:
                            worker_result['move_group_recovered'] = self.attempt_move_group_recovery(
                                worker_result.get('result_status')
                            )
                    return worker_result
                except Exception as exc:
                    stderr = (stderr + '\n' + str(exc)).strip()

            recovered = None
            if worker_return_code not in (0, None):
                recovered = self.attempt_move_group_recovery('worker_process_error')

            failure_reason = stderr.strip() or stdout.strip() or (
                'worker_exit_code_%d' % worker_return_code if worker_return_code is not None else 'worker_failed'
            )
            return self.build_failure_result(
                planner_config,
                test_case_config,
                run_number,
                status='worker_error',
                failure_reason=failure_reason,
                planning_time=time.time() - start_time,
                move_group_recovered=recovered,
                worker_return_code=worker_return_code
            )
    
    def set_planner(self, planner_config):
        """设置规划器和参数"""
        pipeline_id = self.apply_planner_settings(self.group, planner_config, switch_delay=2.0)
        planner_name = planner_config['name']
        rospy.loginfo("切换到规划器: %s" % planner_config['display_name'])
        rospy.loginfo("使用 planning pipeline: %s" % pipeline_id)
    
    def reset_to_home(self):
        """重置到 home 姿态"""
        home_joints = self.config['test_case']['start_joints']
        
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
        self.group.set_num_planning_attempts(int(self.config['test_params']['planning_attempts']))
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
    
    @staticmethod
    def extract_metrics(trajectory):
        """
        从轨迹中提取性能指标
        
        返回指标字典
        """
        if not trajectory or not hasattr(trajectory, 'joint_trajectory'):
            return None
        
        points = trajectory.joint_trajectory.points
        if len(points) == 0:
            return None
        
        # 1. 路径长度（关节空间欧氏距离）
        path_length = 0.0
        for i in range(1, len(points)):
            dist = 0.0
            for j in range(len(points[i].positions)):
                diff = points[i].positions[j] - points[i-1].positions[j]
                dist += diff * diff
            path_length += (dist ** 0.5)
        
        # 2. 平滑度（加速度变化）
        smoothness = 0.0
        if len(points) > 2:
            for i in range(1, len(points) - 1):
                for j in range(len(points[i].positions)):
                    # 二阶差分近似加速度
                    acc = points[i+1].positions[j] - 2*points[i].positions[j] + points[i-1].positions[j]
                    smoothness += abs(acc)
            smoothness /= (len(points) - 2)
        
        # 3. 轨迹执行时间
        execution_time = 0.0
        if len(points) > 0 and hasattr(points[-1], 'time_from_start'):
            execution_time = points[-1].time_from_start.to_sec()
        
        return {
            'tree_vertices': None,
            'tree_edges': None,
            'tree_size': None,
            'path_length': path_length,
            'smoothness': smoothness,
            'execution_time': execution_time
        }
    
    def run_single_test(self, planner_config, test_case_config, run_number):
        """
        运行单次测试
        
        返回: 测试结果字典
        """
        return self.run_planning_worker(planner_config, test_case_config, run_number)
    
    def run_full_benchmark(self):
        """运行完整的基准测试"""
        num_runs = self.config['test_params']['num_runs']
        test_case = self.config['test_case']
        total_tests = len(self.config['planners']) * num_runs
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
            rospy.loginfo("将通过独立 worker 使用 planning pipeline: %s" %
                          self.resolve_pipeline_id(planner_config['name']))
            
            planner_success_count = 0
            planner_total_count = 0
            
            # 重复测试轮次
            for run in range(1, num_runs + 1):
                rospy.loginfo("\n  第 %d/%d 轮测试" % (run, num_runs))

                current_test += 1
                rospy.loginfo("    测试用例: %s (总进度: %d/%d)..." %
                              (test_case['description'], current_test, total_tests))

                try:
                    result = self.run_single_test(planner_config, test_case, run)
                    self.results.append(result)

                    if result['success']:
                        planner_success_count += 1
                        tree_size = result.get('tree_size')
                        tree_edges = result.get('tree_edges')
                        planner_data_available = result.get('planner_data_available')
                        requested_attempts = result.get('requested_planning_attempts')
                        if planner_data_available is False:
                            if requested_attempts and requested_attempts > 1:
                                tree_text = "ParallelPlan 模式不可用 (attempts=%d)" % requested_attempts
                            else:
                                tree_text = "当前不可用"
                        else:
                            tree_text = (
                                ("%d 节点 / %d 边" % (tree_size, tree_edges))
                                if tree_size is not None and tree_edges is not None
                                else ("tree_vertices=%d" % tree_size if tree_size is not None else "N/A")
                            )
                        rospy.loginfo("      ✓ 成功 | 时间: %.3fs | 路径长度: %.3f | 树规模: %s" %
                                      (result['planning_time'],
                                       result['path_length'] if result['path_length'] else 0,
                                       tree_text))
                    else:
                        tree_size = result.get('tree_size')
                        tree_suffix = ""
                        if tree_size is not None:
                            tree_suffix = " | 树规模: %d" % tree_size
                        rospy.logwarn("      ✗ 失败 | 状态: %s | 原因: %s | 时间: %s%s" %
                                      (result.get('result_status', 'failed'),
                                       result.get('failure_reason', 'unknown'),
                                       ('%.3fs' % result['planning_time']) if result.get('planning_time') is not None else 'N/A',
                                       tree_suffix))

                        if self.should_abort_after_result(result):
                            rospy.logerr("检测到 MoveIt 未恢复，提前结束本次 benchmark，避免后续重复失败。")
                            return

                    planner_total_count += 1

                except Exception as e:
                    rospy.logerr("      ✗ 异常: %s" % str(e))
                    self.results.append(self.build_failure_result(
                        planner_config,
                        test_case,
                        run,
                        status='collector_exception',
                        failure_reason=str(e),
                        planning_time=None,
                        move_group_recovered=None,
                        worker_return_code=None
                    ))
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
                'group_name', 'joint_dof',
                'pose_id', 'pose_description', 'run_number',
                'success', 'planning_time', 'tree_vertices', 'tree_edges', 'tree_size',
                'path_length', 'smoothness', 'execution_time',
                'result_status', 'failure_reason',
                'move_group_recovered', 'worker_return_code'
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
    parser.add_argument('--runs', type=int, help='当前单个测试用例的重复次数')
    parser.add_argument('--timeout', type=float, help='规划超时时间（秒）')
    parser.add_argument('--output', type=str, help='输出目录')
    parser.add_argument('--planner', type=str, help='只测试指定的规划器（RRT, RRTConnect, BiTRRT, TRRT, RRTstar, DualArmTRRT）')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--worker-input', type=str, help=argparse.SUPPRESS)
    parser.add_argument('--worker-output', type=str, help=argparse.SUPPRESS)
    
    # 使用 parse_known_args 避免 ROS 参数导致报错
    args, unknown = parser.parse_known_args()
    
    if args.worker:
        if not args.worker_input or not args.worker_output:
            parser.error("--worker 模式必须同时提供 --worker-input 和 --worker-output")
        return run_worker_mode(args.worker_input, args.worker_output)

    collector = None

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

        ros_test_param_overrides = [
            'planning_attempts',
            'planning_watchdog_padding',
            'worker_poll_interval',
            'kill_move_group_on_timeout',
            'move_group_node_name',
            'move_group_action_name',
            'move_group_recovery_wait',
            'move_group_shutdown_wait',
            'move_group_ready_wait',
            'reconnect_retry_delay',
            'max_recovery_attempts',
            'restart_move_group_command',
            'stop_on_unrecoverable_move_group'
        ]
        for param_name in ros_test_param_overrides:
            ros_param = '~' + param_name
            if rospy.has_param(ros_param):
                collector.config['test_params'][param_name] = rospy.get_param(ros_param)
        
        # 如果指定了单个规划器，只测试该规划器
        selected_planner_name = None
        if args.planner:
            original_planners = collector.config['planners']
            collector.config['planners'] = [p for p in original_planners if p['name'] == args.planner]
            if not collector.config['planners']:
                rospy.logerr("找不到规划器: %s" % args.planner)
                rospy.loginfo("可用的规划器: %s" % ', '.join([p['name'] for p in original_planners]))
                return 1
            selected_planner_name = args.planner
        elif rospy.has_param('~planner'):
            planner_name = rospy.get_param('~planner')
            if planner_name:
                original_planners = collector.config['planners']
                collector.config['planners'] = [p for p in original_planners if p['name'] == planner_name]
                if not collector.config['planners']:
                    rospy.logerr("找不到规划器: %s" % planner_name)
                    rospy.loginfo("可用的规划器: %s" % ', '.join([p['name'] for p in original_planners]))
                    return 1
                selected_planner_name = planner_name

        if selected_planner_name:
            rospy.loginfo("只测试规划器: %s" % selected_planner_name)
        
        # 运行基准测试
        collector.run_full_benchmark()
        
        # 保存结果
        collector.save_results()
        
        return 0
    except rospy.ROSInterruptException:
        rospy.loginfo("数据采集被中断")
        return 130
    except KeyboardInterrupt:
        rospy.loginfo("用户中断数据采集")
        return 130
    except Exception as e:
        rospy.logerr("发生错误: %s" % str(e))
        traceback.print_exc()
        return 1
    finally:
        if collector is not None:
            collector.shutdown()


def write_json_atomically(file_path, payload):
    """原子写入 JSON，避免主进程读到半截结果"""
    temp_path = file_path + '.tmp'
    with open(temp_path, 'w') as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
    os.replace(temp_path, file_path)


def run_worker_mode(input_path, output_path):
    """worker 模式：在独立子进程中执行单次规划"""
    moveit_initialized = False
    robot = None
    group = None
    result = None
    exit_code = 0

    with open(input_path, 'r') as handle:
        payload = json.load(handle)

    planner_config = payload['planner_config']
    test_case = payload['test_case']
    run_number = payload['run_number']
    planning_timeout = float(payload['timeout'])
    planning_attempts = int(payload.get('planning_attempts', 3))
    reset_delay = float(payload.get('reset_delay', 0.0))
    move_group_node_name = payload.get('move_group_node_name', '/move_group')
    move_group_action_name = payload.get('move_group_action_name', 'move_group')
    move_group_ready_wait = float(payload.get('move_group_ready_wait', 15.0))
    planner_stats_topic = PaperDataCollector.resolve_planner_stats_topic(planner_config['name'])
    planner_stats_listener = None

    try:
        moveit_commander.roscpp_initialize([sys.argv[0]])
        moveit_initialized = True
        rospy.init_node('paper_data_collection_worker', anonymous=True, disable_signals=True)

        if planner_stats_topic:
            planner_stats_listener = PlannerStatsListener(planner_stats_topic)

        try:
            node_names = rosnode.get_node_names()
        except Exception:
            node_names = []

        if move_group_node_name not in node_names:
            result = PaperDataCollector.build_failure_result(
                planner_config,
                test_case,
                run_number,
                status='move_group_unavailable',
                failure_reason='%s is not available' % move_group_node_name,
                planning_time=None,
                move_group_recovered=None,
                worker_return_code=2
            )
            exit_code = 2
        else:
            if not PaperDataCollector.can_connect_move_group_action(move_group_action_name, move_group_ready_wait):
                result = PaperDataCollector.build_failure_result(
                    planner_config,
                    test_case,
                    run_number,
                    status='move_group_action_unavailable',
                    failure_reason="Unable to connect to move_group action server '%s' within %.1fs" %
                                   (move_group_action_name, move_group_ready_wait),
                    planning_time=None,
                    move_group_recovered=None,
                    worker_return_code=2
                )
                exit_code = 2

        if result is None:
            try:
                robot = moveit_commander.RobotCommander()
                group = moveit_commander.MoveGroupCommander(payload['group_name'])
            except Exception as exc:
                result = PaperDataCollector.build_failure_result(
                    planner_config,
                    test_case,
                    run_number,
                    status='move_group_commander_init_failed',
                    failure_reason='%s: %s' % (type(exc).__name__, str(exc)),
                    planning_time=None,
                    move_group_recovered=None,
                    worker_return_code=2
                )
                exit_code = 2

        if result is None:
            PaperDataCollector.apply_planner_settings(group, planner_config, switch_delay=0.2)

            start_state = PaperDataCollector.build_robot_start_state(
                group,
                test_case['start_joints'],
                robot=robot
            )
            group.set_start_state(start_state)
            group.clear_pose_targets()

            if reset_delay > 0.0:
                rospy.sleep(reset_delay)

            try:
                goal_joints = test_case['goal_pose']['joints']
                group.set_joint_value_target(goal_joints)
            except Exception as exc:
                result = PaperDataCollector.build_failure_result(
                    planner_config,
                    test_case,
                    run_number,
                    status='invalid_goal',
                    failure_reason='%s: %s' % (type(exc).__name__, str(exc)),
                    planning_time=0.0,
                    move_group_recovered=None,
                    worker_return_code=0
                )

        if result is None:
            group.set_num_planning_attempts(planning_attempts)
            group.set_planning_time(planning_timeout)
            plan_start_ros_time = rospy.Time.now().to_sec()
            start_time = time.time()
            plan = group.plan()
            planning_time = time.time() - start_time
            planner_stats = None
            if planner_stats_listener is not None:
                planner_stats = planner_stats_listener.wait_for_stats(
                    min_stamp=plan_start_ros_time,
                    planner_name=planner_config['name'],
                    timeout=2.0
                )
            tree_metrics = PaperDataCollector.extract_tree_metrics(planner_stats)

            success, trajectory = PaperDataCollector.unpack_plan_result(plan)

            if success and (
                tree_metrics.get('tree_vertices') is None or
                tree_metrics.get('tree_edges') is None
            ):
                rospy.logwarn(
                    "规划成功但树统计缺失/可疑。planner=%s, raw_planner_stats=%s",
                    planner_config['name'],
                    json.dumps(planner_stats, ensure_ascii=False, sort_keys=True) if isinstance(planner_stats, dict) else str(planner_stats)
                )

            if success:
                metrics = PaperDataCollector.extract_metrics(trajectory)
                metrics.update(tree_metrics)
                result = PaperDataCollector.build_success_result(
                    planner_config,
                    test_case,
                    run_number,
                    planning_time=planning_time,
                    metrics=metrics,
                    worker_return_code=0
                )
            else:
                result = PaperDataCollector.build_failure_result(
                    planner_config,
                    test_case,
                    run_number,
                    status='planning_failed',
                    failure_reason='planner returned empty trajectory',
                    planning_time=planning_time,
                    move_group_recovered=None,
                    worker_return_code=0,
                    tree_metrics=tree_metrics
                )

    except rospy.ROSInterruptException:
        result = PaperDataCollector.build_failure_result(
            planner_config,
            test_case,
            run_number,
            status='worker_interrupted',
            failure_reason='worker interrupted by ROS shutdown',
            planning_time=None,
            move_group_recovered=None,
            worker_return_code=130
        )
        exit_code = 130
    except KeyboardInterrupt:
        result = PaperDataCollector.build_failure_result(
            planner_config,
            test_case,
            run_number,
            status='worker_interrupted',
            failure_reason='worker interrupted by keyboard signal',
            planning_time=None,
            move_group_recovered=None,
            worker_return_code=130
        )
        exit_code = 130
    except Exception as exc:
        print(traceback.format_exc(), file=sys.stderr)
        result = PaperDataCollector.build_failure_result(
            planner_config,
            test_case,
            run_number,
            status='worker_exception',
            failure_reason='%s: %s' % (type(exc).__name__, str(exc)),
            planning_time=None,
            move_group_recovered=None,
            worker_return_code=2
        )
        exit_code = 2
    finally:
        if group is not None:
            try:
                group.clear_pose_targets()
                group.stop()
            except Exception:
                pass

        if result is None:
            result = PaperDataCollector.build_failure_result(
                planner_config,
                test_case,
                run_number,
                status='worker_setup_failed',
                failure_reason='worker exited without producing a result',
                planning_time=None,
                move_group_recovered=None,
                worker_return_code=2
            )
            exit_code = 2

        try:
            write_json_atomically(output_path, result)
        except Exception:
            print(traceback.format_exc(), file=sys.stderr)
            exit_code = 2

        if moveit_initialized:
            try:
                moveit_commander.roscpp_shutdown()
            except Exception:
                pass

    return exit_code


if __name__ == '__main__':
    sys.exit(main() or 0)
