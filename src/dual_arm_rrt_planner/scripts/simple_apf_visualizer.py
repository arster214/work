#!/usr/bin/env python3

import rospy
import math
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import Point
from clearance_heatmap_msgs.srv import GenerateClearanceHeatmap, GenerateClearanceHeatmapRequest
from clearance_heatmap_msgs.msg import EnvironmentInfo

def vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]

def vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]

def vec_scale(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]

def vec_norm(a):
    return math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)

def vec_normalize(a):
    n = vec_norm(a)
    if n < 1e-6: return [0,0,0]
    return [a[0]/n, a[1]/n, a[2]/n]

class SimpleApfVisualizer:
    def __init__(self):
        rospy.init_node("simple_apf_visualizer_node") # 给节点一个明确唯一的名称
        
        rospy.loginfo("[SimpleApfVisualizer] Initializing...")
        
        # Params
        self.base_frame = rospy.get_param("~base_frame", "base_link_underpan")
        self.marker_topic = rospy.get_param("~marker_topic", "/dual_arm_trrt/apf_force_markers")
        self.environment_topic = rospy.get_param("~environment_topic", "/map_process/environment_info")
        self.service_name = rospy.get_param("~service_name", "/clearance_heatmap_server/generate_clearance_heatmap")
        self.heatmap_resolution = float(rospy.get_param("~heatmap_resolution", 0.05))
        self.heatmap_retry_period = float(rospy.get_param("~heatmap_retry_period", 2.0))
        self.heatmap_min_bound = [
            rospy.get_param("~heatmap_min_x", -2.0),
            rospy.get_param("~heatmap_min_y", -2.0),
            rospy.get_param("~heatmap_min_z", -0.1),
        ]
        self.heatmap_max_bound = [
            rospy.get_param("~heatmap_max_x", 2.0),
            rospy.get_param("~heatmap_max_y", 2.0),
            rospy.get_param("~heatmap_max_z", 2.0),
        ]
        
        self.goal_pos = [
            rospy.get_param("~goal_x", 0.5),
            rospy.get_param("~goal_y", 0.0),
            rospy.get_param("~goal_z", 0.5)
        ]
        self.att_gain = rospy.get_param("~attractive_gain", 1.0)
        self.rep_gain = rospy.get_param("~repulsive_gain", 0.05)
        self.rep_dist = rospy.get_param("~repulsive_dist", 0.4)
        self.force_scale = rospy.get_param("~force_scale", 0.5)
        self.max_force_magnitude = float(rospy.get_param("~max_force_magnitude", 1.0))
        
        self.current_pos = [0.0, 0.0, 1.0]
        self.environment_info = None
        self.heatmap_grid = None
        self.last_heatmap_request = rospy.Time(0)

        # Services & Pubs
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=10, latch=True)
        self.server = InteractiveMarkerServer("apf_debug_marker")

        rospy.Subscriber(self.environment_topic, EnvironmentInfo, self.env_cb)
        self.heatmap_client = rospy.ServiceProxy(self.service_name, GenerateClearanceHeatmap)
        
        self.setup_interactive_marker()
        
        # Timer for visualization
        rospy.Timer(rospy.Duration(0.1), self.update_viz)
        
        rospy.loginfo("[SimpleApfVisualizer] Initialized. Ready to visualize APF.")

        rospy.spin()

    def env_cb(self, msg):
        self.environment_info = msg
        self.update_heatmap(force=True)

    def update_heatmap(self, force=False):
        if not self.environment_info:
            rospy.logwarn_throttle(5.0, "[SimpleApfVisualizer] No environment info yet, cannot update heatmap.")
            return

        now = rospy.Time.now()
        if not force and (now - self.last_heatmap_request).to_sec() < self.heatmap_retry_period:
            return

        self.last_heatmap_request = now

        try:
            rospy.wait_for_service(self.service_name, timeout=0.5)
        except rospy.ROSException:
            rospy.logwarn_throttle(
                2.0,
                "[SimpleApfVisualizer] Waiting for heatmap service %s...",
                self.service_name,
            )
            return

        req = GenerateClearanceHeatmapRequest()
        req.env_info = self.environment_info
        req.resolution = self.heatmap_resolution
        req.min_bound.x, req.min_bound.y, req.min_bound.z = self.heatmap_min_bound
        req.max_bound.x, req.max_bound.y, req.max_bound.z = self.heatmap_max_bound

        try:
            rospy.loginfo("[SimpleApfVisualizer] Requesting heatmap from server...")
            resp = self.heatmap_client(req)
            if resp.success:
                self.heatmap_grid = {
                    "res": self.heatmap_resolution,
                    "min": list(self.heatmap_min_bound),
                    "dims": [resp.dim_x, resp.dim_y, resp.dim_z],
                    "data": resp.heatmap_data
                }
                rospy.loginfo("[SimpleApfVisualizer] Heatmap updated successfully. Grid dims: %s", str(self.heatmap_grid["dims"]))
            else:
                rospy.logerr("[SimpleApfVisualizer] Heatmap service returned failure: %s", resp.message)
        except Exception as e:
            rospy.logerr("[SimpleApfVisualizer] Heatmap service call failed: %s", str(e))

    def get_clearance_and_grad(self, pos):
        if not self.heatmap_grid: return 1.0, [0,0,0]
        
        res = self.heatmap_grid["res"]
        m = self.heatmap_grid["min"]
        dims = self.heatmap_grid["dims"]
        d = self.heatmap_grid["data"]
        
        def get_val(ix, iy, iz):
            # 将索引限制在合法范围内，避免越界导致力消失
            ix = max(0, min(ix, dims[0]-1))
            iy = max(0, min(iy, dims[1]-1))
            iz = max(0, min(iz, dims[2]-1))
            return d[ix + iy*dims[0] + iz*dims[0]*dims[1]]

        # 计算当前点在 Grid 中的索引
        ix = int((pos[0]-m[0])/res)
        iy = int((pos[1]-m[1])/res)
        iz = int((pos[2]-m[2])/res)
        
        val = get_val(ix, iy, iz)
        
        # 增加有限差分的步长以提高对数值波动的敏感度
        dx = (get_val(ix+1, iy, iz) - get_val(ix-1, iy, iz)) / (2*res)
        dy = (get_val(ix, iy+1, iz) - get_val(ix, iy-1, iz)) / (2*res)
        dz = (get_val(ix, iy, iz+1) - get_val(ix, iy, iz-1)) / (2*res)
        
        return val, [dx, dy, dz]

    def setup_interactive_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.base_frame
        int_marker.name = "apf_probe"
        int_marker.description = ""
        int_marker.pose.position.x, int_marker.pose.position.y, int_marker.pose.position.z = self.current_pos
        
        # 1. 模拟 MoveIt 的蓝色透明球体
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = 0.1 # 与机械臂末端球体大小相仿
        sphere_marker.color.r, sphere_marker.color.g, sphere_marker.color.b, sphere_marker.color.a = 0.0, 0.5, 1.0, 0.5 # 类似 MoveIt 的蓝色
        
        # 2. 交互控制（平移）
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(sphere_marker)
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D # 允许自由拖拽
        int_marker.controls.append(control)
            
        self.server.insert(int_marker, self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.current_pos = [feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z]

    def make_arrow(self, id, start, vec, color):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = rospy.Time.now()
        
        # 将 NS 严格对应到之前配置的过滤条件
        if id == 0:
            marker.ns = "apf_right_attractive"
        elif id == 1:
            marker.ns = "apf_right_repulsive"
        else:
            marker.ns = "apf_right_resultant" # 合力
            
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # 力的起点
        marker.points.append(Point(*start))
        # 力的终点 (加了防止零向量崩溃的保护)
        v_norm = vec_norm(vec)
        if v_norm < 1e-4:
            # 如果力太小，把终点设为起点稍微偏移一点，避免 Marker 报错
            end = vec_add(start, [0, 0, 0.001])
        else:
            end = vec_add(start, vec_scale(vec, self.force_scale))
        marker.points.append(Point(*end))
        
        marker.scale.x = 0.02 # shaft diameter
        marker.scale.y = 0.04 # head diameter
        marker.scale.z = 0.06 # head length
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def limit_vec(self, vec, max_len=None):
        limit = self.max_force_magnitude if max_len is None else max_len
        n = vec_norm(vec)
        if n > limit and n > 1e-6:
            return vec_scale(vec, limit / n)
        return vec

    def update_viz(self, event):
        if self.environment_info and self.heatmap_grid is None:
            self.update_heatmap(force=False)

        # 记录心跳证明节点在工作
        rospy.logdebug_throttle(1.0, "[SimpleApfVisualizer] Timer tick")
        
        # Goal Marker (target) - 红色固定球体
        goal_marker = Marker()
        goal_marker.header.frame_id = self.base_frame
        goal_marker.id = 999
        goal_marker.type = Marker.SPHERE
        goal_marker.pose.position.x, goal_marker.pose.position.y, goal_marker.pose.position.z = self.goal_pos
        goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.08
        goal_marker.color.r, goal_marker.color.g, goal_marker.color.b, goal_marker.color.a = 1, 0, 0, 0.8
        self.marker_pub.publish(goal_marker)

        # Forces 力的计算使用 self.current_pos (Interactive Marker 的位置)
        # 1. 计算引力：F_att = att_gain * r
        # 为了不让引力在远处过大，可以限制最大值
        dist_to_goal = vec_norm(vec_sub(self.goal_pos, self.current_pos))
        f_att_raw = vec_sub(self.goal_pos, self.current_pos)
        f_att = vec_scale(f_att_raw, self.att_gain)
        
        # 2. 计算斥力
        clearance, grad = self.get_clearance_and_grad(self.current_pos)
        f_rep = [0,0,0]
        
        # 详细打印调试信息，确认 clearance 是否被正确读取
        rospy.logdebug_throttle(0.5, "[APF] Pos: %.2f, %.2f, %.2f | Clearance: %.3f", 
                                self.current_pos[0], self.current_pos[1], self.current_pos[2], clearance)

        if clearance < self.rep_dist:
            # 强化斥力计算
            # 即使 clearance 极小也保证分母不为 0
            safe_clearance = max(clearance, 0.02)
            magnitude = self.rep_gain * (1.0/safe_clearance - 1.0/self.rep_dist) / (safe_clearance**2)
            f_rep = self.limit_vec(vec_scale(vec_normalize(grad), magnitude))
            
        # 3. 计算合力
        f_att = self.limit_vec(f_att)
        f_total = self.limit_vec(vec_add(f_att, f_rep))

        # 发送引力、斥力、合力
        # 特别检查：合力 (f_total) 的颜色设为黄色 (1, 1, 0, 1)
        self.marker_pub.publish(self.make_arrow(0, self.current_pos, f_att, (0, 1, 0, 1))) # 绿色：引力
        self.marker_pub.publish(self.make_arrow(1, self.current_pos, f_rep, (1, 0, 1, 1))) # 品红：斥力
        self.marker_pub.publish(self.make_arrow(2, self.current_pos, f_total, (1, 1, 0, 1))) # 黄色：合力


if __name__ == "__main__":
    SimpleApfVisualizer()
