#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
被动数据收集脚本（监听 MoveIt 规划结果）
只要你在 RViz 中点击了 "Plan"，就会自动收集数据。
"""

import rospy
import csv
import json
import math
import os
import signal
import threading
import time
from datetime import datetime
from moveit_msgs.msg import MoveGroupActionResult, MoveItErrorCodes
from std_msgs.msg import String as StringMsg

class PassiveDataCollector:
    ERROR_CODE_NAME_MAP = {
        value: name
        for name, value in MoveItErrorCodes.__dict__.items()
        if name.isupper() and isinstance(value, int)
    }

    def __init__(self):
        rospy.init_node('passive_data_collection', anonymous=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_file = f"passive_planning_data_{timestamp}.csv"
        self.node_start_wall_time = time.time()
        self.node_start_ros_time = None
        try:
            current_ros_time = rospy.Time.now().to_sec()
            if current_ros_time > 0.0:
                self.node_start_ros_time = current_ros_time
        except Exception:
            self.node_start_ros_time = None
        
        self.all_data = []
        self.plan_count = 0
        self.recent_tree_stats = []
        self.stats_lock = threading.Lock()
        
        # 订阅 MoveIt 的 Action 返回结果
        rospy.Subscriber('/move_group/result', MoveGroupActionResult, self.result_callback)
        rospy.Subscriber('/ompl_planning_stats', StringMsg, self.tree_stats_callback,
                         callback_args='/ompl_planning_stats', queue_size=10)
        rospy.Subscriber('/dual_arm_trrt/planning_stats', StringMsg, self.tree_stats_callback,
                         callback_args='/dual_arm_trrt/planning_stats', queue_size=10)
        
        # 注册退出时的保存机制
        signal.signal(signal.SIGINT, self.save_data)
        rospy.on_shutdown(self.save_data_on_shutdown)
        
        print(f"=== 被动记录脚本已启动 ===")
        print(f"正在后台监听 RViz 里的每一次规划 (监听主题 /move_group/result)...")
        print(f"同时监听树规模主题: /ompl_planning_stats, /dual_arm_trrt/planning_stats")
        print(f"收集到的数据最终将保存到: {os.path.abspath(self.output_file)}")
        print(f"你可以随时按 Ctrl+C 退出并保存。")

    @staticmethod
    def safe_float(value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
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

    @staticmethod
    def normalize_planner_name(raw_name):
        if not isinstance(raw_name, str):
            return None

        name = raw_name.strip()
        if not name:
            return None

        if '[' in name and ']' in name:
            left = name.rfind('[')
            right = name.rfind(']')
            if 0 <= left < right:
                bracket_name = name[left + 1:right].strip()
                if bracket_name:
                    return bracket_name

        if '/' in name:
            suffix = name.split('/')[-1].strip()
            if suffix:
                return suffix

        return name

    def calculate_path_length(self, points):
        """计算路径总长度（关节空间角度变化的 L2 范数）"""
        length = 0.0
        for i in range(1, len(points)):
            prev_positions = points[i-1].positions
            curr_positions = points[i].positions
            
            dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(prev_positions, curr_positions)))
            length += dist
        return length

    @staticmethod
    def calculate_smoothness(points):
        smoothness = 0.0
        if len(points) > 2:
            for i in range(1, len(points) - 1):
                for j in range(len(points[i].positions)):
                    acc = points[i + 1].positions[j] - 2 * points[i].positions[j] + points[i - 1].positions[j]
                    smoothness += abs(acc)
            smoothness /= (len(points) - 2)
        return smoothness

    @staticmethod
    def extract_execution_time(points):
        if points and hasattr(points[-1], 'time_from_start'):
            try:
                return points[-1].time_from_start.to_sec()
            except Exception:
                return None
        return None

    @staticmethod
    def infer_group_name(joint_names):
        if not joint_names:
            return ""

        left_count = sum(1 for name in joint_names if isinstance(name, str) and name.startswith('l_'))
        right_count = sum(1 for name in joint_names if isinstance(name, str) and name.startswith('r_'))

        if left_count > 0 and right_count > 0:
            return "dual_arms"
        if right_count > 0:
            return "r_arm"
        if left_count > 0:
            return "l_arm"
        return ""

    @classmethod
    def describe_error_code(cls, value):
        try:
            int_value = int(value)
        except (TypeError, ValueError):
            return str(value)
        return cls.ERROR_CODE_NAME_MAP.get(int_value, str(int_value))

    def tree_stats_callback(self, msg, source_topic=None):
        """缓存最近的规划树统计信息。"""
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, f"无法解析规划树统计消息: {exc}")
            return

        if not isinstance(payload, dict):
            return

        record = dict(payload)
        record['received_at'] = time.time()
        record['source_topic'] = source_topic

        stats_stamp = self.safe_float(record.get('stamp'))
        if self.node_start_ros_time is not None and stats_stamp is not None:
            if stats_stamp + 1e-6 < self.node_start_ros_time:
                rospy.loginfo_throttle(
                    5.0,
                    "忽略节点启动前的锁存 planning_stats: source=%s stamp=%.6f < node_start=%.6f",
                    source_topic or 'unknown',
                    stats_stamp,
                    self.node_start_ros_time
                )
                return

        with self.stats_lock:
            self.recent_tree_stats.append(record)
            if len(self.recent_tree_stats) > 50:
                self.recent_tree_stats = self.recent_tree_stats[-50:]

    def find_best_tree_stats_index(self, result_stamp=None, callback_start_time=None, planning_time=None, max_age_sec=20.0,
                                   messages=None):
        """基于 stamp 和 planning_time 选出最可能属于本次规划的树统计。"""
        now = time.time()
        candidates = []
        if messages is None:
            with self.stats_lock:
                messages = list(self.recent_tree_stats)

        for index, item in enumerate(messages):
            received_at = self.safe_float(item.get('received_at'))
            if received_at is None:
                continue

            if now - received_at > max_age_sec:
                continue

            planning_succeeded = self.safe_bool(item.get('planning_succeeded'))
            if planning_succeeded is False:
                continue

            stats_stamp = self.safe_float(item.get('stamp'))
            stats_planning_time = self.safe_float(item.get('planning_time'))
            tree_vertices = self.safe_float(item.get('tree_vertices'))
            tree_edges = self.safe_float(item.get('tree_edges'))
            solution_states = self.safe_float(item.get('solution_states'))

            score = 0.0
            quality = "strong"

            if result_stamp is not None and stats_stamp is not None:
                stamp_diff = abs(stats_stamp - result_stamp)
                if stamp_diff > 10.0:
                    continue
                score += stamp_diff * 3.0
            elif callback_start_time is not None:
                recv_diff = abs(received_at - callback_start_time)
                if recv_diff > 10.0:
                    continue
                score += recv_diff * 2.0
                quality = "time_only"

            if planning_time is not None and stats_planning_time is not None:
                plan_diff = abs(stats_planning_time - planning_time)
                score += plan_diff
                if plan_diff > max(0.5, 3.0 * max(planning_time, 0.1)):
                    continue
                if plan_diff > max(0.2, 0.5 * max(planning_time, 0.1)):
                    quality = "weak"
                    score += 2.0
            else:
                quality = "time_only" if quality == "strong" else quality

            if callback_start_time is not None:
                if received_at < callback_start_time - 2.0:
                    continue
                if received_at > callback_start_time + 2.0:
                    continue
                score += abs(received_at - callback_start_time) * 0.5

            # 同一轮里若收到多条 planning_stats，优先选择真正带树信息的那条。
            # 这对 OMPL 的 RRT / BiTRRT / RRTConnect 都重要，因为有时会出现
            # 成功但 tree_vertices/tree_edges 为 0 的“瘦消息”。
            has_nonzero_tree = (
                (tree_vertices is not None and tree_vertices > 0.0) or
                (tree_edges is not None and tree_edges > 0.0)
            )
            has_tree_fields = tree_vertices is not None or tree_edges is not None
            if not has_nonzero_tree:
                score += 50.0
            if not has_tree_fields:
                score += 20.0
            if planning_succeeded is not True:
                score += 5.0
            score -= min(solution_states or 0.0, 100000.0) * 1e-6

            candidates.append((score, received_at, index, quality, item))

        if not candidates:
            return None, None, "missing"

        candidates.sort(key=lambda entry: (entry[0], -entry[1]))
        _, _, best_index, best_quality, best_item = candidates[0]
        return best_index, best_item, best_quality

    def pop_latest_recent_tree_stats(self, callback_start_time=None, planning_time=None, max_age_sec=20.0):
        """仅在候选足够可信时才兜底取最近的一条 stats。"""
        now = time.time()
        with self.stats_lock:
            candidates = []
            for index, item in enumerate(self.recent_tree_stats):
                received_at = self.safe_float(item.get('received_at'))
                if received_at is None or now - received_at > max_age_sec:
                    continue

                planning_succeeded = self.safe_bool(item.get('planning_succeeded'))
                if planning_succeeded is False:
                    continue

                if callback_start_time is not None:
                    if received_at < callback_start_time - 1.0 or received_at > callback_start_time + 2.0:
                        continue

                stats_planning_time = self.safe_float(item.get('planning_time'))
                if planning_time is not None and stats_planning_time is not None:
                    if abs(stats_planning_time - planning_time) > max(0.5, 3.0 * max(planning_time, 0.1)):
                        continue

                candidates.append((index, item))

            if not candidates:
                return None, "missing"

            best_index, best_item = max(candidates, key=lambda pair: self.safe_float(pair[1].get('received_at')) or 0.0)
            self.recent_tree_stats.pop(best_index)
            return best_item, "fallback"

    def pop_matching_tree_stats(self, result_stamp=None, callback_start_time=None, planning_time=None, wait_timeout=1.5):
        """等待并消费最匹配本次规划的一条树统计。"""
        deadline = time.time() + max(0.0, wait_timeout)

        while time.time() <= deadline and not rospy.is_shutdown():
            with self.stats_lock:
                messages = list(self.recent_tree_stats)
                best_index, best_item, quality = self.find_best_tree_stats_index(
                    result_stamp=result_stamp,
                    callback_start_time=callback_start_time,
                    planning_time=planning_time,
                    messages=messages
                )
                if best_index is not None:
                    popped_item = self.recent_tree_stats.pop(best_index)
                    return popped_item, quality

            rospy.sleep(0.05)

        return self.pop_latest_recent_tree_stats(callback_start_time=callback_start_time, planning_time=planning_time)

    def summarize_recent_tree_stats(self, callback_start_time=None, limit=3):
        """输出最近几条缓存 stats，便于诊断为什么没有匹配上。"""
        with self.stats_lock:
            messages = list(self.recent_tree_stats)

        if not messages:
            return "缓存为空"

        messages.sort(key=lambda item: self.safe_float(item.get('received_at')) or 0.0, reverse=True)
        summary_parts = []
        for item in messages[:max(1, limit)]:
            received_at = self.safe_float(item.get('received_at'))
            received_delta = None
            if callback_start_time is not None and received_at is not None:
                received_delta = received_at - callback_start_time

            summary_parts.append(
                "source={source}, planner={planner}, success={success}, plan_time={plan_time}, stamp={stamp}, dt={dt}".format(
                    source=item.get('source_topic') or 'unknown',
                    planner=item.get('planner_id') or item.get('planner_name') or 'unknown',
                    success=item.get('planning_succeeded'),
                    plan_time=item.get('planning_time'),
                    stamp=item.get('stamp'),
                    dt=("%.3fs" % received_delta) if received_delta is not None else "N/A"
                )
            )

        return " | ".join(summary_parts)

    @staticmethod
    def extract_tree_metrics(tree_stats):
        metrics = {
            "planner_name": None,
            "tree_vertices": None,
            "tree_edges": None,
            "tree_size": None,
            "planner_data_available": None,
            "requested_planning_attempts": None
        }

        if not isinstance(tree_stats, dict):
            return metrics

        planner_name = tree_stats.get('planner_id') or tree_stats.get('planner_name')
        normalized_planner_name = PassiveDataCollector.normalize_planner_name(planner_name)
        if normalized_planner_name:
            metrics["planner_name"] = normalized_planner_name

        for field_name in ("tree_vertices", "tree_edges"):
            value = tree_stats.get(field_name)
            try:
                metrics[field_name] = int(value) if value is not None else None
            except (TypeError, ValueError):
                metrics[field_name] = None

        planner_data_available = tree_stats.get('planner_data_available')
        if isinstance(planner_data_available, bool):
            metrics["planner_data_available"] = planner_data_available

        requested_attempts = tree_stats.get('requested_planning_attempts')
        try:
            metrics["requested_planning_attempts"] = int(requested_attempts) if requested_attempts is not None else None
        except (TypeError, ValueError):
            metrics["requested_planning_attempts"] = None

        solution_states = tree_stats.get('solution_states')
        try:
            solution_states = int(solution_states) if solution_states is not None else None
        except (TypeError, ValueError):
            solution_states = None

        planning_succeeded = PassiveDataCollector.safe_bool(tree_stats.get('planning_succeeded'))

        # 对 OMPL 采样树规划器来说，成功且 solution_states>0 却同时给出 0/0，
        # 通常说明这条 stats 不足以代表真实树规模，采集时宁可记为缺失。
        if (
            planning_succeeded is True and
            solution_states is not None and solution_states > 0 and
            metrics["tree_vertices"] == 0 and
            metrics["tree_edges"] == 0
        ):
            metrics["tree_vertices"] = None
            metrics["tree_edges"] = None

        metrics["tree_size"] = metrics["tree_vertices"]
        return metrics

    def result_callback(self, msg):
        trajectory = msg.result.planned_trajectory.joint_trajectory
        points = trajectory.points
        joint_names = list(trajectory.joint_names)
        group_name = self.infer_group_name(joint_names)
        joint_dof = len(joint_names)

        callback_start_time = time.time()
        planning_time = msg.result.planning_time
        result_stamp = None
        if hasattr(msg.header, 'stamp'):
            try:
                stamp_value = msg.header.stamp.to_sec()
                if stamp_value > 0.0:
                    result_stamp = stamp_value
            except Exception:
                result_stamp = None

        tree_stats, match_quality = self.pop_matching_tree_stats(
            result_stamp=result_stamp,
            callback_start_time=callback_start_time,
            planning_time=planning_time,
            wait_timeout=1.5
        )
        tree_metrics = self.extract_tree_metrics(tree_stats)

        success = (msg.result.error_code.val == MoveItErrorCodes.SUCCESS and len(points) > 0)
        path_length = self.calculate_path_length(points) if success else None
        smoothness = self.calculate_smoothness(points) if success else None
        execution_time = self.extract_execution_time(points) if success else None
        error_name = self.describe_error_code(msg.result.error_code.val)

        self.plan_count += 1
        print(f"\n[记录 {self.plan_count}] 检测到一次规划结果!")
        print(f" - 结果: {'成功' if success else '失败'}")
        print(f" - 错误码: {msg.result.error_code.val} ({error_name})")
        if planning_time is not None:
            print(f" - 规划时间: {planning_time:.4f} s")
        if success and path_length is not None:
            print(f" - 路径长度: {path_length:.4f} rad (L2)")
        if tree_metrics["tree_size"] is not None and tree_metrics["tree_edges"] is not None:
            print(f" - 树规模: {tree_metrics['tree_size']} 节点 / {tree_metrics['tree_edges']} 边")
        elif tree_metrics["tree_size"] is not None:
            print(f" - 树规模: {tree_metrics['tree_size']} 节点")
        else:
            print(f" - 树规模: N/A (本次未匹配到 planning_stats)")
            print(f" - 调试: 最近缓存 stats -> {self.summarize_recent_tree_stats(callback_start_time=callback_start_time)}")
        if tree_metrics["planner_name"]:
            print(f" - 规划器: {tree_metrics['planner_name']}")
        if tree_metrics["planner_data_available"] is False:
            print(" - 注意: 当前 planning_stats 明确标记 PlannerData 不可用")
        if tree_metrics["requested_planning_attempts"] not in (None, 1):
            print(f" - 注意: 当前请求 planning_attempts={tree_metrics['requested_planning_attempts']}")
        if isinstance(tree_stats, dict) and tree_stats.get('source_topic'):
            print(f" - 统计来源: {tree_stats['source_topic']}")
        if match_quality == "fallback":
            print(" - 注意: 本次使用的是时间窗口内最近一条可信 planning_stats 兜底匹配")
        elif match_quality == "time_only":
            print(" - 注意: 本次仅根据时间窗口匹配 planning_stats")
        elif match_quality == "weak":
            print(" - 注意: planning_time 与 planning_stats 略有偏差，已按最优候选匹配")

        record = {
            "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            "planner_name": tree_metrics["planner_name"],
            "planner_display_name": tree_metrics["planner_name"],
            "group_name": group_name,
            "joint_dof": joint_dof,
            "pose_id": "",
            "pose_description": "",
            "run_number": self.plan_count,
            "success": success,
            "planning_time": planning_time,
            "tree_vertices": tree_metrics["tree_vertices"],
            "tree_edges": tree_metrics["tree_edges"],
            "tree_size": tree_metrics["tree_size"],
            "path_length": path_length,
            "smoothness": smoothness,
            "execution_time": execution_time,
            "result_status": "success" if success else "planning_failed",
            "failure_reason": "" if success else f"{error_name} ({msg.result.error_code.val})",
            "move_group_recovered": "",
            "worker_return_code": "",
            "planner_data_available": tree_metrics["planner_data_available"],
            "requested_planning_attempts": tree_metrics["requested_planning_attempts"],
            "match_quality": match_quality,
            "stats_source_topic": tree_stats.get("source_topic") if isinstance(tree_stats, dict) else None
        }
        self.all_data.append(record)

    def save_data(self, sig=None, frame=None):
        if hasattr(self, 'saved') and self.saved:
            return
            
        self.saved = True
        print("\n\n[保存] 正在保存数据...")
        if self.all_data:
            fieldnames = [
                "timestamp",
                "planner_name",
                "planner_display_name",
                "group_name",
                "joint_dof",
                "pose_id",
                "pose_description",
                "run_number",
                "success",
                "planning_time",
                "tree_vertices",
                "tree_edges",
                "tree_size",
                "path_length",
                "smoothness",
                "execution_time",
                "result_status",
                "failure_reason",
                "move_group_recovered",
                "worker_return_code",
                "planner_data_available",
                "requested_planning_attempts",
                "match_quality",
                "stats_source_topic"
            ]
            with open(self.output_file, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.all_data)
            print(f"成功保存 {len(self.all_data)} 条记录至 {os.path.abspath(self.output_file)}")
        else:
            print("未收集到任何有效数据，未生成文件。")
            
        rospy.signal_shutdown("Data saved")

    def save_data_on_shutdown(self):
        self.save_data()

def main():
    collector = PassiveDataCollector()
    rospy.spin()

if __name__ == '__main__':
    main()
