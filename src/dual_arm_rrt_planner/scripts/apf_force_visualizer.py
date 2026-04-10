#!/usr/bin/env python3

import math

import rospy
import tf2_geometry_msgs
import tf2_ros
from clearance_heatmap_msgs.msg import EnvironmentInfo
from clearance_heatmap_msgs.srv import GenerateClearanceHeatmap, GenerateClearanceHeatmapRequest
from geometry_msgs.msg import Point, PointStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarkerFeedback, Marker


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vec_scale(a, scale):
    return [a[0] * scale, a[1] * scale, a[2] * scale]


def vec_norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def vec_normalize(a):
    magnitude = vec_norm(a)
    if magnitude <= 1e-9:
      return [0.0, 0.0, 0.0], 0.0
    return [a[0] / magnitude, a[1] / magnitude, a[2] / magnitude], magnitude


def point_to_list(point):
    return [point.x, point.y, point.z]


def list_to_point(values):
    point = Point()
    point.x = values[0]
    point.y = values[1]
    point.z = values[2]
    return point


class ApfForceVisualizer(object):
    def __init__(self):
        self.base_frame = rospy.get_param("~base_frame", "base_link_underpan")
        self.left_tip_link = rospy.get_param("~left_tip_link", "l_link7")
        self.right_tip_link = rospy.get_param("~right_tip_link", "r_link7")
        self.active_arm = rospy.get_param("~active_arm", "both").strip().lower()
        self.environment_topic = rospy.get_param("~environment_topic", "/map_process/environment_info")
        self.heatmap_service_name = rospy.get_param("~heatmap_service_name", "/clearance_heatmap_server/generate_clearance_heatmap")
        self.feedback_topic = rospy.get_param("~feedback_topic", "/rviz/moveit/move_marker/feedback")
        self.marker_topic = rospy.get_param("~marker_topic", "/dual_arm_trrt/apf_force_markers")
        self.fk_service_name = rospy.get_param("~fk_service_name", "/compute_fk")
        self.publish_rate = rospy.get_param("~publish_rate", 20.0)
        self.goal_timeout = rospy.get_param("~goal_timeout", 5.0)
        self.force_scale = rospy.get_param("~force_scale", 0.10)
        self.attractive_gain = rospy.get_param("~attractive_gain", 1.0)
        self.repulsive_gain = rospy.get_param("~repulsive_gain", 0.010)
        self.heatmap_resolution = rospy.get_param("~heatmap_resolution", 0.05)
        self.workspace_padding = rospy.get_param("~workspace_padding", 0.10)
        self.repulsive_clearance_threshold = rospy.get_param("~repulsive_clearance_threshold", 0.15)
        self.max_force_magnitude = rospy.get_param("~max_force_magnitude", 1.0)
        self.min_visual_force_magnitude = rospy.get_param("~min_visual_force_magnitude", 0.03)
        self.debug_always_show_repulsive = rospy.get_param("~debug_always_show_repulsive", False)
        self.debug_min_repulsive_activation = rospy.get_param("~debug_min_repulsive_activation", 0.35)
        self.arrow_shaft_diameter = rospy.get_param("~arrow_shaft_diameter", 0.010)
        self.arrow_head_diameter = rospy.get_param("~arrow_head_diameter", 0.018)
        self.arrow_head_length = rospy.get_param("~arrow_head_length", 0.025)

        self.environment_info = None
        self.goal_positions = {}
        self.goal_timestamps = {}
        self.heatmap_grid = None
        self.heatmap_dirty = True
        self.latest_joint_state = JointState()
        self.has_joint_state = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.heatmap_client = rospy.ServiceProxy(self.heatmap_service_name, GenerateClearanceHeatmap)
        self.fk_client = rospy.ServiceProxy(self.fk_service_name, GetPositionFK)

        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=16, latch=True)
        self.environment_sub = rospy.Subscriber(self.environment_topic, EnvironmentInfo, self.environment_callback, queue_size=1)
        self.feedback_sub = rospy.Subscriber(self.feedback_topic, InteractiveMarkerFeedback, self.feedback_callback, queue_size=20)
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 1e-3)), self.timer_callback)

    def environment_callback(self, msg):
        self.environment_info = msg
        self.heatmap_dirty = True

    def feedback_callback(self, msg):
        arm_name = self.infer_arm_name(msg)
        if arm_name is None:
            return

        goal_position = self.transform_point_to_base(msg.header.frame_id, msg.pose.position)
        if goal_position is None:
            return

        self.goal_positions[arm_name] = goal_position
        self.goal_timestamps[arm_name] = rospy.Time.now()
        self.heatmap_dirty = True
        rospy.loginfo_throttle(1.0, "[APFVisualizer] Received goal feedback for %s arm from %s",
                               arm_name, self.feedback_topic)

    def joint_state_callback(self, msg):
        if not msg.name or len(msg.name) != len(msg.position):
            return
        self.latest_joint_state = msg
        self.has_joint_state = True

    def infer_arm_name(self, msg):
        if self.active_arm == "right":
            return "right"
        if self.active_arm == "left":
            return "left"

        marker_name = (msg.marker_name or "").lower()
        control_name = (msg.control_name or "").lower()
        combined_name = marker_name + " " + control_name

        if any(token in combined_name for token in ["dual_left", "left", "l_link7", "l_link", "left_gripper"]):
            return "left"
        if any(token in combined_name for token in ["dual_right", "right", "r_link7", "r_link", "right_gripper"]):
            return "right"

        goal_position = self.transform_point_to_base(msg.header.frame_id, msg.pose.position)
        if goal_position is None:
            return None

        distances = []
        for arm_name, tip_link in [("left", self.left_tip_link), ("right", self.right_tip_link)]:
            current_position = self.lookup_link_position(tip_link)
            if current_position is not None:
                distances.append((vec_norm(vec_sub(goal_position, current_position)), arm_name))

        if not distances:
            return None

        distances.sort(key=lambda item: item[0])
        return distances[0][1]

    def transform_point_to_base(self, source_frame, point):
        frame_id = source_frame if source_frame else self.base_frame
        stamped_point = PointStamped()
        stamped_point.header.stamp = rospy.Time(0)
        stamped_point.header.frame_id = frame_id
        stamped_point.point = point

        if frame_id != self.base_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, frame_id, rospy.Time(0), rospy.Duration(0.05)
                )
                stamped_point = tf2_geometry_msgs.do_transform_point(stamped_point, transform)
            except Exception as exc:
                rospy.logwarn_throttle(1.0, "[APFVisualizer] Failed to transform point from %s to %s: %s",
                                       frame_id, self.base_frame, str(exc))
                return None

        return point_to_list(stamped_point.point)

    def build_robot_state(self):
        if not self.has_joint_state:
            return None

        robot_state = RobotState()
        robot_state.joint_state = self.latest_joint_state
        robot_state.is_diff = False
        return robot_state

    def lookup_link_position(self, link_name):
        robot_state = self.build_robot_state()
        if robot_state is not None:
            fk_request = GetPositionFKRequest()
            fk_request.header.frame_id = self.base_frame
            fk_request.fk_link_names = [link_name]
            fk_request.robot_state = robot_state

            try:
                fk_response = self.fk_client(fk_request)
                if fk_response.error_code.val == 1 and fk_response.pose_stamped:
                    return point_to_list(fk_response.pose_stamped[0].pose.position)
            except Exception as exc:
                rospy.logwarn_throttle(1.0, "[APFVisualizer] FK lookup failed for %s via %s: %s",
                                       link_name, self.fk_service_name, str(exc))

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, link_name, rospy.Time(0), rospy.Duration(0.05)
            )
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "[APFVisualizer] Failed to resolve %s from FK/TF in frame %s: %s",
                                   link_name, self.base_frame, str(exc))
            return None

        translation = transform.transform.translation
        return [translation.x, translation.y, translation.z]

    def compute_attractive_force(self, current_position, goal_position):
        return vec_scale(vec_sub(goal_position, current_position), self.attractive_gain)

    def compute_workspace_bounds(self, query_points):
        if self.environment_info is None:
            return None

        min_bound = [float("inf"), float("inf"), float("inf")]
        max_bound = [float("-inf"), float("-inf"), float("-inf")]

        def include_point(point):
            for axis in range(3):
                min_bound[axis] = min(min_bound[axis], point[axis])
                max_bound[axis] = max(max_bound[axis], point[axis])

        for obstacle in self.environment_info.obstacles:
            center = point_to_list(obstacle.center)
            half_size = [0.5 * obstacle.size.x, 0.5 * obstacle.size.y, 0.5 * obstacle.size.z]
            include_point([center[0] - half_size[0], center[1] - half_size[1], center[2] - half_size[2]])
            include_point([center[0] + half_size[0], center[1] + half_size[1], center[2] + half_size[2]])

        for point in query_points:
            if point is not None:
                include_point(point)

        if not math.isfinite(min_bound[0]):
            return None

        for axis in range(3):
            min_bound[axis] -= self.workspace_padding
            max_bound[axis] += self.workspace_padding
            if max_bound[axis] - min_bound[axis] < self.heatmap_resolution:
                center = 0.5 * (min_bound[axis] + max_bound[axis])
                min_bound[axis] = center - 0.5 * self.heatmap_resolution
                max_bound[axis] = center + 0.5 * self.heatmap_resolution

        return min_bound, max_bound

    def refresh_heatmap_if_needed(self, query_points):
        if not self.heatmap_dirty and self.heatmap_grid is not None:
            return True

        bounds = self.compute_workspace_bounds(query_points)
        if bounds is None:
            return False

        min_bound, max_bound = bounds
        request = GenerateClearanceHeatmapRequest()
        request.env_info = self.environment_info
        request.resolution = self.heatmap_resolution
        request.min_bound.x = min_bound[0]
        request.min_bound.y = min_bound[1]
        request.min_bound.z = min_bound[2]
        request.max_bound.x = max_bound[0]
        request.max_bound.y = max_bound[1]
        request.max_bound.z = max_bound[2]

        try:
            response = self.heatmap_client(request)
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "[APFVisualizer] Heatmap service call failed: %s", str(exc))
            return False

        if not response.success or response.dim_x <= 0 or response.dim_y <= 0 or response.dim_z <= 0:
            rospy.logwarn_throttle(1.0, "[APFVisualizer] Heatmap response invalid: %s", response.message)
            return False

        self.heatmap_grid = {
            "resolution": self.heatmap_resolution,
            "min_bound": min_bound,
            "max_bound": [
                min_bound[0] + self.heatmap_resolution * (response.dim_x - 1),
                min_bound[1] + self.heatmap_resolution * (response.dim_y - 1),
                min_bound[2] + self.heatmap_resolution * (response.dim_z - 1),
            ],
            "dim_x": response.dim_x,
            "dim_y": response.dim_y,
            "dim_z": response.dim_z,
            "data": list(response.heatmap_data),
        }
        self.heatmap_dirty = False
        return True

    def sample_heatmap_value(self, point):
        grid = self.heatmap_grid
        if grid is None:
            return 0.0

        resolution = grid["resolution"]
        max_x_index = max(0, grid["dim_x"] - 1)
        max_y_index = max(0, grid["dim_y"] - 1)
        max_z_index = max(0, grid["dim_z"] - 1)

        fx = 0.0 if grid["dim_x"] <= 1 else clamp((point[0] - grid["min_bound"][0]) / resolution, 0.0, float(max_x_index))
        fy = 0.0 if grid["dim_y"] <= 1 else clamp((point[1] - grid["min_bound"][1]) / resolution, 0.0, float(max_y_index))
        fz = 0.0 if grid["dim_z"] <= 1 else clamp((point[2] - grid["min_bound"][2]) / resolution, 0.0, float(max_z_index))

        x0 = int(math.floor(fx))
        y0 = int(math.floor(fy))
        z0 = int(math.floor(fz))
        x1 = min(x0 + 1, max_x_index)
        y1 = min(y0 + 1, max_y_index)
        z1 = min(z0 + 1, max_z_index)

        tx = fx - float(x0)
        ty = fy - float(y0)
        tz = fz - float(z0)

        def sample(ix, iy, iz):
            index = iz * grid["dim_x"] * grid["dim_y"] + iy * grid["dim_x"] + ix
            if index < 0 or index >= len(grid["data"]):
                return 0.0
            return float(grid["data"][index])

        c000 = sample(x0, y0, z0)
        c100 = sample(x1, y0, z0)
        c010 = sample(x0, y1, z0)
        c110 = sample(x1, y1, z0)
        c001 = sample(x0, y0, z1)
        c101 = sample(x1, y0, z1)
        c011 = sample(x0, y1, z1)
        c111 = sample(x1, y1, z1)

        c00 = c000 * (1.0 - tx) + c100 * tx
        c10 = c010 * (1.0 - tx) + c110 * tx
        c01 = c001 * (1.0 - tx) + c101 * tx
        c11 = c011 * (1.0 - tx) + c111 * tx
        c0 = c00 * (1.0 - ty) + c10 * ty
        c1 = c01 * (1.0 - ty) + c11 * ty
        return c0 * (1.0 - tz) + c1 * tz

    def sample_heatmap_gradient(self, point):
        grid = self.heatmap_grid
        if grid is None:
            return [0.0, 0.0, 0.0]

        step = grid["resolution"]
        gx = (self.sample_heatmap_value([point[0] + step, point[1], point[2]]) -
              self.sample_heatmap_value([point[0] - step, point[1], point[2]])) / (2.0 * step)
        gy = (self.sample_heatmap_value([point[0], point[1] + step, point[2]]) -
              self.sample_heatmap_value([point[0], point[1] - step, point[2]])) / (2.0 * step)
        gz = (self.sample_heatmap_value([point[0], point[1], point[2] + step]) -
              self.sample_heatmap_value([point[0], point[1], point[2] - step])) / (2.0 * step)
        return [gx, gy, gz]

    def compute_repulsive_force(self, current_position):
        if self.heatmap_grid is None:
            return [0.0, 0.0, 0.0], 0.0

        clearance = self.sample_heatmap_value(current_position)
        gradient = self.sample_heatmap_gradient(current_position)
        gradient_direction, gradient_norm = vec_normalize(gradient)

        if gradient_norm <= 1e-9:
            return [0.0, 0.0, 0.0], clearance

        activation = clamp(
            (self.repulsive_clearance_threshold - clearance) / max(self.repulsive_clearance_threshold, 1e-6),
            0.0,
            1.0,
        )
        if self.debug_always_show_repulsive:
            activation = max(activation, self.debug_min_repulsive_activation)
        return vec_scale(gradient_direction, self.repulsive_gain * activation), clearance

    def clamp_force(self, force):
        unit_direction, magnitude = vec_normalize(force)
        if magnitude <= self.max_force_magnitude:
            return force
        return vec_scale(unit_direction, self.max_force_magnitude)

    def visual_force(self, force):
        unit_direction, magnitude = vec_normalize(force)
        if magnitude <= 1e-9:
            return force
        if magnitude >= self.min_visual_force_magnitude:
            return force
        return vec_scale(unit_direction, self.min_visual_force_magnitude)

    def publish_arrow(self, arm_name, marker_id, namespace_suffix, color_rgba, start_point, force_vector):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.base_frame
        marker.ns = "apf_{}_{}".format(arm_name, namespace_suffix)
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.arrow_shaft_diameter
        marker.scale.y = self.arrow_head_diameter
        marker.scale.z = self.arrow_head_length
        marker.color.r = color_rgba[0]
        marker.color.g = color_rgba[1]
        marker.color.b = color_rgba[2]
        marker.color.a = color_rgba[3]
        marker.lifetime = rospy.Duration(1.0)
        marker.points = [
            list_to_point(start_point),
            list_to_point(vec_add(start_point, vec_scale(self.visual_force(force_vector), self.force_scale))),
        ]
        self.marker_pub.publish(marker)

    def delete_arrow(self, arm_name, marker_id, namespace_suffix):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.base_frame
        marker.ns = "apf_{}_{}".format(arm_name, namespace_suffix)
        marker.id = marker_id
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    def timer_callback(self, _event):
        now = rospy.Time.now()
        current_positions = {
            "left": self.lookup_link_position(self.left_tip_link),
            "right": self.lookup_link_position(self.right_tip_link),
        }
        active_arms = ["left", "right"]
        if self.active_arm == "right":
            active_arms = ["right"]
        elif self.active_arm == "left":
            active_arms = ["left"]

        query_points = []
        for arm_name in active_arms:
            if current_positions[arm_name] is not None:
                query_points.append(current_positions[arm_name])
            if arm_name in self.goal_positions:
                query_points.append(self.goal_positions[arm_name])
        heatmap_ready = self.refresh_heatmap_if_needed(query_points)

        rospy.loginfo_throttle(
            1.0,
            "[APFVisualizer] status env=%s heatmap=%s marker_subs=%d left_tf=%s right_tf=%s left_goal=%s right_goal=%s",
            "yes" if self.environment_info is not None else "no",
            "yes" if heatmap_ready and self.heatmap_grid is not None else "no",
            self.marker_pub.get_num_connections(),
            "yes" if current_positions["left"] is not None else "no",
            "yes" if current_positions["right"] is not None else "no",
            "yes" if "left" in self.goal_positions else "no",
            "yes" if "right" in self.goal_positions else "no",
        )

        arm_specs = []
        if "left" in active_arms:
            arm_specs.append(("left", self.left_tip_link, 0))
        if "right" in active_arms:
            arm_specs.append(("right", self.right_tip_link, 10))

        inactive_arms = [arm for arm in ["left", "right"] if arm not in active_arms]
        for arm_name in inactive_arms:
            marker_id_offset = 0 if arm_name == "left" else 10
            self.delete_arrow(arm_name, marker_id_offset + 0, "attractive")
            self.delete_arrow(arm_name, marker_id_offset + 1, "repulsive")
            self.delete_arrow(arm_name, marker_id_offset + 2, "resultant")

        for arm_name, tip_link, marker_id_offset in arm_specs:
            current_position = current_positions.get(arm_name)
            if current_position is None:
                continue

            repulsive_force_raw, clearance = self.compute_repulsive_force(current_position)
            repulsive_force = self.clamp_force(repulsive_force_raw)
            self.publish_arrow(arm_name, marker_id_offset + 1, "repulsive",
                               (1.0, 0.15, 0.10, 0.95), current_position, repulsive_force)
            rospy.loginfo_throttle(
                1.0,
                "[APFVisualizer] %s arm clearance=%.3f repulsive_norm=%.4f goal=%s",
                arm_name,
                clearance,
                vec_norm(repulsive_force),
                "yes" if arm_name in self.goal_positions else "no",
            )

            goal_position = self.goal_positions.get(arm_name)
            goal_timestamp = self.goal_timestamps.get(arm_name)

            if goal_position is None or goal_timestamp is None or (now - goal_timestamp).to_sec() > self.goal_timeout:
                self.delete_arrow(arm_name, marker_id_offset + 0, "attractive")
                self.delete_arrow(arm_name, marker_id_offset + 2, "resultant")
                continue

            attractive_force = self.clamp_force(self.compute_attractive_force(current_position, goal_position))
            resultant_force = self.clamp_force(vec_add(attractive_force, repulsive_force))

            self.publish_arrow(arm_name, marker_id_offset + 0, "attractive",
                               (0.0, 0.35, 1.0, 0.95), current_position, attractive_force)
            self.publish_arrow(arm_name, marker_id_offset + 2, "resultant",
                               (0.10, 1.0, 0.20, 0.98), current_position, resultant_force)


if __name__ == "__main__":
    rospy.init_node("apf_force_visualizer")
    ApfForceVisualizer()
    rospy.loginfo("[APFVisualizer] Publishing APF arrows on %s", rospy.get_param("~marker_topic", "/dual_arm_trrt/apf_force_markers"))
    rospy.spin()
