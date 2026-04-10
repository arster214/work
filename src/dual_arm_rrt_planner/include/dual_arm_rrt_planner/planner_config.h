#pragma once

#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>

namespace dual_arm_rrt_planner
{
struct RobotConfig
{
  std::string planning_group;
  std::string left_arm_group;
  std::string right_arm_group;
  std::size_t configuration_dimension;
  std::vector<std::string> joint_order;
};

struct JointBoundConfig
{
  double lower;
  double upper;
  double max_velocity;
  bool has_acceleration_limits;
  double max_acceleration;
};

struct StateSpaceConfig
{
  std::string type;
  bool enforce_joint_bounds;
  std::map<std::string, JointBoundConfig> joint_bounds;
};

struct CollisionCheckingConfig
{
  bool check_self_collision;
  bool check_environment_collision;
  double state_validity_checking_resolution;
  double longest_valid_segment_fraction;
  bool allowed_start_state_in_collision;
  bool allowed_goal_state_in_collision;
};

struct TRRTConfig
{
  std::string planner_config_name;
  std::string planner_type;
  double range;
  double goal_bias;
  bool enable_connect;
  std::string sampling_strategy;
  int sobol_candidate_pool_size;
  bool sobol_sort_by_goal_distance;
  double temp_change_factor;
  double init_temperature;
  double frontier_threshold;
  double frontier_node_ratio;
  double cost_threshold;
  double min_parent_child_distance;
  double min_parent_child_distance_scale;
  int max_sampling_attempts_per_iteration;
};

struct SolveConfig
{
  double max_planning_time_seconds;
  int planning_attempts;
  double goal_tolerance;
  bool allow_approximate_solution;
  double max_approximate_goal_distance;
  bool simplify_solution;
  std::string simplification_strategy;
  int reduce_vertices_max_steps;
  int reduce_vertices_max_empty_steps;
  double reduce_vertices_range_ratio;
  int shortcut_max_steps;
  int shortcut_max_empty_steps;
  double shortcut_range_ratio;
  double shortcut_snap_to_vertex;
  bool smooth_bspline;
  int smooth_bspline_max_steps;
  double smooth_bspline_min_change;
  double preserve_tail_ratio;
  bool interpolate_solution;
  double interpolation_step;
};

struct CostObjectiveConfig
{
  std::string type;
  bool use_motion_cost_interpolation;
  double clearance_epsilon;
  double invalid_state_cost;
};

struct HeatmapConfig
{
  bool enabled;
  std::string environment_topic;
  std::string service_name;
  std::string left_query_link;
  std::string right_query_link;
  double resolution;
  double request_timeout_seconds;
  double workspace_padding;
  double safety_clearance_low;
  double safety_clearance_high;
  double objective_weight;
  double objective_clearance_threshold;
  double step_min_scale;
  double sampling_min_keep_probability;
};

struct ColorConfig
{
  double red;
  double green;
  double blue;
  double alpha;
};

struct TreeVisualizationConfig
{
  bool enabled;
  std::string marker_topic;
  bool marker_latch;
  std::string frame_id;
  std::string left_tip_link;
  std::string right_tip_link;
  bool publish_on_success;
  bool publish_on_failure;
  bool publish_vertices;
  int max_tree_vertices;
  int max_tree_edges;
  int max_solution_branch_points;
  int max_solution_path_points;
  double edge_scale;
  double vertex_scale;
  bool publish_solution_branch;
  double solution_branch_edge_scale;
  bool publish_solution_path;
  double solution_edge_scale;
  ColorConfig left_marker_rgba;
  ColorConfig right_marker_rgba;
  ColorConfig left_solution_branch_rgba;
  ColorConfig right_solution_branch_rgba;
  ColorConfig left_solution_rgba;
  ColorConfig right_solution_rgba;
};

struct PlannerConfig
{
  RobotConfig robot;
  StateSpaceConfig state_space;
  CollisionCheckingConfig collision_checking;
  TRRTConfig trrt;
  SolveConfig solve;
  CostObjectiveConfig cost_objective;
  HeatmapConfig heatmap;
  TreeVisualizationConfig visualization;
};

PlannerConfig loadPlannerConfig(const ros::NodeHandle& node_handle, const std::string& planner_config_name);
}  // namespace dual_arm_rrt_planner
