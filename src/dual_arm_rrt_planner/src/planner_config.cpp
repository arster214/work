#include <dual_arm_rrt_planner/planner_config.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <set>
#include <sstream>
#include <stdexcept>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace dual_arm_rrt_planner
{
namespace
{
std::string makeMissingRequiredParameterMessage(const ros::NodeHandle& node_handle, const std::string& parameter_name)
{
  return "Missing required parameter: " + node_handle.resolveName(parameter_name);
}

std::string trimString(const std::string& value)
{
  const auto first_non_space =
      std::find_if_not(value.begin(), value.end(), [](unsigned char character) { return std::isspace(character); });
  if (first_non_space == value.end())
  {
    return "";
  }

  const auto last_non_space =
      std::find_if_not(value.rbegin(), value.rend(), [](unsigned char character) { return std::isspace(character); })
          .base();

  return std::string(first_non_space, last_non_space);
}

std::string toLowerString(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
  return value;
}

std::string xmlRpcTypeToString(XmlRpc::XmlRpcValue::Type type)
{
  switch (type)
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return "boolean";
    case XmlRpc::XmlRpcValue::TypeInt:
      return "int";
    case XmlRpc::XmlRpcValue::TypeDouble:
      return "double";
    case XmlRpc::XmlRpcValue::TypeString:
      return "string";
    case XmlRpc::XmlRpcValue::TypeDateTime:
      return "datetime";
    case XmlRpc::XmlRpcValue::TypeBase64:
      return "base64";
    case XmlRpc::XmlRpcValue::TypeArray:
      return "array";
    case XmlRpc::XmlRpcValue::TypeStruct:
      return "struct";
    case XmlRpc::XmlRpcValue::TypeInvalid:
    default:
      return "invalid";
  }
}

XmlRpc::XmlRpcValue readRequiredXmlRpcParam(const ros::NodeHandle& node_handle, const std::string& parameter_name)
{
  XmlRpc::XmlRpcValue value;
  if (!node_handle.getParam(parameter_name, value))
  {
    throw std::runtime_error(makeMissingRequiredParameterMessage(node_handle, parameter_name));
  }
  return value;
}

std::runtime_error makeInvalidParameterTypeError(const std::string& resolved_parameter_name,
                                                 const std::string& expected_type,
                                                 XmlRpc::XmlRpcValue::Type actual_type)
{
  return std::runtime_error("Parameter " + resolved_parameter_name + " is present but cannot be parsed as " +
                            expected_type + ". Actual XmlRpc type: " + xmlRpcTypeToString(actual_type) + ".");
}

double parseDoubleString(const std::string& resolved_parameter_name, const std::string& raw_value)
{
  const std::string trimmed_value = trimString(raw_value);
  if (trimmed_value.empty())
  {
    throw std::runtime_error("Parameter " + resolved_parameter_name + " is present but cannot be parsed as a double.");
  }

  char* parse_end = nullptr;
  errno = 0;
  const double parsed_value = std::strtod(trimmed_value.c_str(), &parse_end);
  const bool has_unparsed_suffix = parse_end != (trimmed_value.c_str() + trimmed_value.size());

  if (has_unparsed_suffix || errno == ERANGE)
  {
    throw std::runtime_error("Parameter " + resolved_parameter_name + " is present but cannot be parsed as a double.");
  }

  return parsed_value;
}

int parseIntString(const std::string& resolved_parameter_name, const std::string& raw_value)
{
  const std::string trimmed_value = trimString(raw_value);
  if (trimmed_value.empty())
  {
    throw std::runtime_error("Parameter " + resolved_parameter_name + " is present but cannot be parsed as an int.");
  }

  char* parse_end = nullptr;
  errno = 0;
  const long parsed_value = std::strtol(trimmed_value.c_str(), &parse_end, 10);
  const bool has_unparsed_suffix = parse_end != (trimmed_value.c_str() + trimmed_value.size());

  if (has_unparsed_suffix || errno == ERANGE || parsed_value < INT_MIN || parsed_value > INT_MAX)
  {
    throw std::runtime_error("Parameter " + resolved_parameter_name + " is present but cannot be parsed as an int.");
  }

  return static_cast<int>(parsed_value);
}

bool parseBoolString(const std::string& resolved_parameter_name, const std::string& raw_value)
{
  const std::string normalized_value = toLowerString(trimString(raw_value));

  if (normalized_value == "true" || normalized_value == "1" || normalized_value == "yes" ||
      normalized_value == "on")
  {
    return true;
  }

  if (normalized_value == "false" || normalized_value == "0" || normalized_value == "no" ||
      normalized_value == "off")
  {
    return false;
  }

  throw std::runtime_error("Parameter " + resolved_parameter_name + " is present but cannot be parsed as a bool.");
}

template <typename T>
T readRequiredParam(const ros::NodeHandle& node_handle, const std::string& parameter_name)
{
  T value;
  if (!node_handle.getParam(parameter_name, value))
  {
    throw std::runtime_error(makeMissingRequiredParameterMessage(node_handle, parameter_name));
  }
  return value;
}

template <typename T>
T readOptionalParam(const ros::NodeHandle& node_handle, const std::string& parameter_name, const T& default_value)
{
  if (!node_handle.hasParam(parameter_name))
  {
    return default_value;
  }

  return readRequiredParam<T>(node_handle, parameter_name);
}

template <>
double readRequiredParam<double>(const ros::NodeHandle& node_handle, const std::string& parameter_name)
{
  const std::string resolved_parameter_name = node_handle.resolveName(parameter_name);
  XmlRpc::XmlRpcValue value = readRequiredXmlRpcParam(node_handle, parameter_name);

  switch (value.getType())
  {
    case XmlRpc::XmlRpcValue::TypeDouble:
      return static_cast<double>(value);
    case XmlRpc::XmlRpcValue::TypeInt:
      return static_cast<double>(static_cast<int>(value));
    case XmlRpc::XmlRpcValue::TypeString:
      return parseDoubleString(resolved_parameter_name, static_cast<std::string>(value));
    default:
      throw makeInvalidParameterTypeError(resolved_parameter_name, "a double", value.getType());
  }
}

template <>
int readRequiredParam<int>(const ros::NodeHandle& node_handle, const std::string& parameter_name)
{
  const std::string resolved_parameter_name = node_handle.resolveName(parameter_name);
  XmlRpc::XmlRpcValue value = readRequiredXmlRpcParam(node_handle, parameter_name);

  switch (value.getType())
  {
    case XmlRpc::XmlRpcValue::TypeInt:
      return static_cast<int>(value);
    case XmlRpc::XmlRpcValue::TypeDouble:
    {
      const double parsed_value = static_cast<double>(value);
      if (!std::isfinite(parsed_value) || std::floor(parsed_value) != parsed_value || parsed_value < INT_MIN ||
          parsed_value > INT_MAX)
      {
        throw std::runtime_error("Parameter " + resolved_parameter_name +
                                 " is present but cannot be parsed as an int.");
      }
      return static_cast<int>(parsed_value);
    }
    case XmlRpc::XmlRpcValue::TypeString:
      return parseIntString(resolved_parameter_name, static_cast<std::string>(value));
    default:
      throw makeInvalidParameterTypeError(resolved_parameter_name, "an int", value.getType());
  }
}

template <>
bool readRequiredParam<bool>(const ros::NodeHandle& node_handle, const std::string& parameter_name)
{
  const std::string resolved_parameter_name = node_handle.resolveName(parameter_name);
  XmlRpc::XmlRpcValue value = readRequiredXmlRpcParam(node_handle, parameter_name);

  switch (value.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return static_cast<bool>(value);
    case XmlRpc::XmlRpcValue::TypeInt:
    {
      const int parsed_value = static_cast<int>(value);
      if (parsed_value == 0)
      {
        return false;
      }
      if (parsed_value == 1)
      {
        return true;
      }
      throw std::runtime_error("Parameter " + resolved_parameter_name +
                               " is present but cannot be parsed as a bool.");
    }
    case XmlRpc::XmlRpcValue::TypeString:
      return parseBoolString(resolved_parameter_name, static_cast<std::string>(value));
    default:
      throw makeInvalidParameterTypeError(resolved_parameter_name, "a bool", value.getType());
  }
}

void validatePositive(const std::string& name, double value)
{
  if (value <= 0.0)
  {
    throw std::runtime_error(name + " must be greater than zero.");
  }
}

void validateNonNegative(const std::string& name, double value)
{
  if (value < 0.0)
  {
    throw std::runtime_error(name + " must be greater than or equal to zero.");
  }
}

void validateProbability(const std::string& name, double value)
{
  if (value < 0.0 || value > 1.0)
  {
    throw std::runtime_error(name + " must be within [0.0, 1.0].");
  }
}

void validateNonEmpty(const std::string& name, const std::string& value)
{
  if (trimString(value).empty())
  {
    throw std::runtime_error(name + " must not be empty.");
  }
}

ColorConfig readRequiredColorParam(const ros::NodeHandle& node_handle, const std::string& parameter_name)
{
  const std::vector<double> color_values = readRequiredParam<std::vector<double> >(node_handle, parameter_name);
  if (color_values.size() != 4U)
  {
    throw std::runtime_error("Parameter " + node_handle.resolveName(parameter_name) +
                             " must contain exactly four rgba values.");
  }

  ColorConfig color;
  color.red = color_values[0];
  color.green = color_values[1];
  color.blue = color_values[2];
  color.alpha = color_values[3];
  return color;
}

void validateColor(const std::string& name, const ColorConfig& color)
{
  validateProbability(name + "/red", color.red);
  validateProbability(name + "/green", color.green);
  validateProbability(name + "/blue", color.blue);
  validateProbability(name + "/alpha", color.alpha);
}
}  // namespace

PlannerConfig loadPlannerConfig(const ros::NodeHandle& node_handle, const std::string& planner_config_name)
{
  PlannerConfig config;

  config.robot.planning_group = readRequiredParam<std::string>(node_handle, "robot/planning_group");
  config.robot.left_arm_group = readRequiredParam<std::string>(node_handle, "robot/left_arm_group");
  config.robot.right_arm_group = readRequiredParam<std::string>(node_handle, "robot/right_arm_group");
  config.robot.joint_order = readRequiredParam<std::vector<std::string> >(node_handle, "robot/joint_order");
  config.robot.configuration_dimension =
      static_cast<std::size_t>(readRequiredParam<int>(node_handle, "robot/configuration_dimension"));

  config.state_space.type = readRequiredParam<std::string>(node_handle, "state_space/type");
  config.state_space.enforce_joint_bounds =
      readRequiredParam<bool>(node_handle, "state_space/enforce_joint_bounds");

  for (const std::string& joint_name : config.robot.joint_order)
  {
    JointBoundConfig joint_bound;
    joint_bound.lower =
        readRequiredParam<double>(node_handle, "state_space/joint_bounds/" + joint_name + "/lower");
    joint_bound.upper =
        readRequiredParam<double>(node_handle, "state_space/joint_bounds/" + joint_name + "/upper");
    joint_bound.max_velocity =
        readRequiredParam<double>(node_handle, "state_space/joint_bounds/" + joint_name + "/max_velocity");
    joint_bound.has_acceleration_limits = readRequiredParam<bool>(
        node_handle, "state_space/joint_bounds/" + joint_name + "/has_acceleration_limits");
    joint_bound.max_acceleration = readRequiredParam<double>(
        node_handle, "state_space/joint_bounds/" + joint_name + "/max_acceleration");
    config.state_space.joint_bounds[joint_name] = joint_bound;
  }

  config.collision_checking.check_self_collision =
      readRequiredParam<bool>(node_handle, "collision_checking/check_self_collision");
  config.collision_checking.check_environment_collision =
      readRequiredParam<bool>(node_handle, "collision_checking/check_environment_collision");
  config.collision_checking.state_validity_checking_resolution =
      readRequiredParam<double>(node_handle, "collision_checking/state_validity_checking_resolution");
  config.collision_checking.longest_valid_segment_fraction =
      readRequiredParam<double>(node_handle, "collision_checking/longest_valid_segment_fraction");
  config.collision_checking.allowed_start_state_in_collision =
      readRequiredParam<bool>(node_handle, "collision_checking/allowed_start_state_in_collision");
  config.collision_checking.allowed_goal_state_in_collision =
      readRequiredParam<bool>(node_handle, "collision_checking/allowed_goal_state_in_collision");

  config.solve.max_planning_time_seconds =
      readRequiredParam<double>(node_handle, "solve/max_planning_time_seconds");
  config.solve.planning_attempts = readRequiredParam<int>(node_handle, "solve/planning_attempts");
  config.solve.goal_tolerance = readRequiredParam<double>(node_handle, "solve/goal_tolerance");
  config.solve.allow_approximate_solution =
      readOptionalParam<bool>(node_handle, "solve/allow_approximate_solution", false);
  config.solve.max_approximate_goal_distance =
      readOptionalParam<double>(node_handle, "solve/max_approximate_goal_distance", config.solve.goal_tolerance);
  config.solve.simplify_solution = readRequiredParam<bool>(node_handle, "solve/simplify_solution");
  config.solve.simplification_strategy =
      readRequiredParam<std::string>(node_handle, "solve/simplification_strategy");
  config.solve.reduce_vertices_max_steps =
      readRequiredParam<int>(node_handle, "solve/reduce_vertices_max_steps");
  config.solve.reduce_vertices_max_empty_steps =
      readRequiredParam<int>(node_handle, "solve/reduce_vertices_max_empty_steps");
  config.solve.reduce_vertices_range_ratio =
      readRequiredParam<double>(node_handle, "solve/reduce_vertices_range_ratio");
  config.solve.shortcut_max_steps = readRequiredParam<int>(node_handle, "solve/shortcut_max_steps");
  config.solve.shortcut_max_empty_steps =
      readRequiredParam<int>(node_handle, "solve/shortcut_max_empty_steps");
  config.solve.shortcut_range_ratio =
      readRequiredParam<double>(node_handle, "solve/shortcut_range_ratio");
  config.solve.shortcut_snap_to_vertex =
      readRequiredParam<double>(node_handle, "solve/shortcut_snap_to_vertex");
  config.solve.smooth_bspline = readRequiredParam<bool>(node_handle, "solve/smooth_bspline");
  config.solve.smooth_bspline_max_steps =
      readRequiredParam<int>(node_handle, "solve/smooth_bspline_max_steps");
  config.solve.smooth_bspline_min_change =
      readRequiredParam<double>(node_handle, "solve/smooth_bspline_min_change");
  config.solve.preserve_tail_ratio = readOptionalParam<double>(node_handle, "solve/preserve_tail_ratio", 0.0);
  config.solve.interpolate_solution = readRequiredParam<bool>(node_handle, "solve/interpolate_solution");
  config.solve.interpolation_step = readRequiredParam<double>(node_handle, "solve/interpolation_step");

  config.cost_objective.type = readRequiredParam<std::string>(node_handle, "cost_objective/type");
  config.cost_objective.use_motion_cost_interpolation =
      readRequiredParam<bool>(node_handle, "cost_objective/use_motion_cost_interpolation");
  config.cost_objective.clearance_epsilon =
      readRequiredParam<double>(node_handle, "cost_objective/clearance_epsilon");
  config.cost_objective.invalid_state_cost =
      readRequiredParam<double>(node_handle, "cost_objective/invalid_state_cost");

  config.heatmap.enabled = readOptionalParam<bool>(node_handle, "heatmap/enabled", false);
  config.heatmap.environment_topic = readOptionalParam<std::string>(
      node_handle, "heatmap/environment_topic", "/map_process/environment_info");
  config.heatmap.service_name = readOptionalParam<std::string>(
      node_handle, "heatmap/service_name", "/clearance_heatmap_server/generate_clearance_heatmap");
  config.heatmap.left_query_link = readOptionalParam<std::string>(node_handle, "heatmap/left_query_link", "");
  config.heatmap.right_query_link = readOptionalParam<std::string>(node_handle, "heatmap/right_query_link", "");
  config.heatmap.resolution = readOptionalParam<double>(node_handle, "heatmap/resolution", 0.05);
  config.heatmap.request_timeout_seconds =
      readOptionalParam<double>(node_handle, "heatmap/request_timeout_seconds", 1.0);
  config.heatmap.workspace_padding = readOptionalParam<double>(node_handle, "heatmap/workspace_padding", 0.10);
  config.heatmap.safety_clearance_low =
      readOptionalParam<double>(node_handle, "heatmap/safety_clearance_low", 0.0);
  config.heatmap.safety_clearance_high =
      readOptionalParam<double>(node_handle, "heatmap/safety_clearance_high", 0.15);
  config.heatmap.objective_weight =
      readOptionalParam<double>(node_handle, "heatmap/objective_weight", 6.0);
  config.heatmap.objective_clearance_threshold =
      readOptionalParam<double>(node_handle, "heatmap/objective_clearance_threshold", 0.04);
  config.heatmap.step_min_scale = readOptionalParam<double>(node_handle, "heatmap/step_min_scale", 0.35);
  config.heatmap.sampling_min_keep_probability =
      readOptionalParam<double>(node_handle, "heatmap/sampling_min_keep_probability", 0.20);

  config.visualization.enabled = readRequiredParam<bool>(node_handle, "visualization/enabled");
  config.visualization.marker_topic = readRequiredParam<std::string>(node_handle, "visualization/marker_topic");
  config.visualization.marker_latch = readRequiredParam<bool>(node_handle, "visualization/marker_latch");
  config.visualization.frame_id = readRequiredParam<std::string>(node_handle, "visualization/frame_id");
  config.visualization.left_tip_link = readRequiredParam<std::string>(node_handle, "visualization/left_tip_link");
  config.visualization.right_tip_link =
      readRequiredParam<std::string>(node_handle, "visualization/right_tip_link");
  config.visualization.publish_on_success =
      readRequiredParam<bool>(node_handle, "visualization/publish_on_success");
  config.visualization.publish_on_failure =
      readRequiredParam<bool>(node_handle, "visualization/publish_on_failure");
  config.visualization.publish_vertices =
      readRequiredParam<bool>(node_handle, "visualization/publish_vertices");
  config.visualization.max_tree_vertices =
      readRequiredParam<int>(node_handle, "visualization/max_tree_vertices");
  config.visualization.max_tree_edges =
      readRequiredParam<int>(node_handle, "visualization/max_tree_edges");
  config.visualization.max_solution_branch_points =
      readRequiredParam<int>(node_handle, "visualization/max_solution_branch_points");
  config.visualization.max_solution_path_points =
      readRequiredParam<int>(node_handle, "visualization/max_solution_path_points");
  config.visualization.edge_scale = readRequiredParam<double>(node_handle, "visualization/edge_scale");
  config.visualization.vertex_scale = readRequiredParam<double>(node_handle, "visualization/vertex_scale");
  config.visualization.publish_solution_branch =
      readRequiredParam<bool>(node_handle, "visualization/publish_solution_branch");
  config.visualization.solution_branch_edge_scale =
      readRequiredParam<double>(node_handle, "visualization/solution_branch_edge_scale");
  config.visualization.publish_solution_path =
      readRequiredParam<bool>(node_handle, "visualization/publish_solution_path");
  config.visualization.solution_edge_scale =
      readRequiredParam<double>(node_handle, "visualization/solution_edge_scale");
  config.visualization.left_marker_rgba =
      readRequiredColorParam(node_handle, "visualization/left_marker_rgba");
  config.visualization.right_marker_rgba =
      readRequiredColorParam(node_handle, "visualization/right_marker_rgba");
  config.visualization.left_solution_branch_rgba =
      readRequiredColorParam(node_handle, "visualization/left_solution_branch_rgba");
  config.visualization.right_solution_branch_rgba =
      readRequiredColorParam(node_handle, "visualization/right_solution_branch_rgba");
  config.visualization.left_solution_rgba =
      readRequiredColorParam(node_handle, "visualization/left_solution_rgba");
  config.visualization.right_solution_rgba =
      readRequiredColorParam(node_handle, "visualization/right_solution_rgba");

  if (trimString(config.heatmap.left_query_link).empty())
  {
    config.heatmap.left_query_link = config.visualization.left_tip_link;
  }

  if (trimString(config.heatmap.right_query_link).empty())
  {
    config.heatmap.right_query_link = config.visualization.right_tip_link;
  }

  const std::string planner_namespace = "planner_configs/" + planner_config_name + "/";
  config.trrt.planner_config_name = planner_config_name;
  config.trrt.planner_type = readRequiredParam<std::string>(node_handle, planner_namespace + "type");
  config.trrt.range = readRequiredParam<double>(node_handle, planner_namespace + "range");
  config.trrt.goal_bias = readRequiredParam<double>(node_handle, planner_namespace + "goal_bias");
  config.trrt.enable_connect = readRequiredParam<bool>(node_handle, planner_namespace + "enable_connect");
  config.trrt.sampling_strategy =
      readRequiredParam<std::string>(node_handle, planner_namespace + "sampling_strategy");
  config.trrt.sobol_candidate_pool_size =
      readRequiredParam<int>(node_handle, planner_namespace + "sobol_candidate_pool_size");
  config.trrt.sobol_sort_by_goal_distance =
      readRequiredParam<bool>(node_handle, planner_namespace + "sobol_sort_by_goal_distance");
  config.trrt.temp_change_factor =
      readRequiredParam<double>(node_handle, planner_namespace + "temp_change_factor");
  config.trrt.init_temperature =
      readRequiredParam<double>(node_handle, planner_namespace + "init_temperature");
  config.trrt.frontier_threshold =
      readRequiredParam<double>(node_handle, planner_namespace + "frontier_threshold");
  config.trrt.frontier_node_ratio =
      readRequiredParam<double>(node_handle, planner_namespace + "frontier_node_ratio");
  config.trrt.cost_threshold =
      readRequiredParam<double>(node_handle, planner_namespace + "cost_threshold");
  config.trrt.min_parent_child_distance =
      readRequiredParam<double>(node_handle, planner_namespace + "min_parent_child_distance");
  config.trrt.min_parent_child_distance_scale =
      readRequiredParam<double>(node_handle, planner_namespace + "min_parent_child_distance_scale");
  config.trrt.max_sampling_attempts_per_iteration =
      readRequiredParam<int>(node_handle, planner_namespace + "max_sampling_attempts_per_iteration");

  if (config.robot.configuration_dimension != config.robot.joint_order.size())
  {
    std::ostringstream message_stream;
    message_stream << "robot/configuration_dimension is " << config.robot.configuration_dimension
                   << " but robot/joint_order contains " << config.robot.joint_order.size() << " joints.";
    throw std::runtime_error(message_stream.str());
  }

  if (config.robot.configuration_dimension == 0U)
  {
    throw std::runtime_error("robot/configuration_dimension must be greater than zero.");
  }

  std::set<std::string> unique_joint_names(config.robot.joint_order.begin(), config.robot.joint_order.end());
  if (unique_joint_names.size() != config.robot.joint_order.size())
  {
    throw std::runtime_error("robot/joint_order contains duplicate joint names.");
  }

  if (config.state_space.type != "real_vector")
  {
    throw std::runtime_error("Only state_space/type = real_vector is supported.");
  }

  if (!config.collision_checking.check_self_collision && !config.collision_checking.check_environment_collision)
  {
    throw std::runtime_error("At least one collision checking mode must be enabled.");
  }

  if (config.cost_objective.type != "mechanical_work" && config.cost_objective.type != "clearance")
  {
    throw std::runtime_error("Only cost_objective/type = mechanical_work or clearance is supported.");
  }

  if (config.trrt.planner_type != "geometric::TRRT" && config.trrt.planner_type != "ompl::geometric::TRRT" &&
      config.trrt.planner_type != "dual_arm_rrt_planner::LocalTRRT")
  {
    throw std::runtime_error("Only planner_configs/" + planner_config_name +
                             "/type = geometric::TRRT, ompl::geometric::TRRT, or "
                             "dual_arm_rrt_planner::LocalTRRT is supported.");
  }

  validatePositive("collision_checking/state_validity_checking_resolution",
                   config.collision_checking.state_validity_checking_resolution);
  validatePositive("collision_checking/longest_valid_segment_fraction",
                   config.collision_checking.longest_valid_segment_fraction);
  validateNonNegative("planner_configs/" + planner_config_name + "/range", config.trrt.range);
  validateProbability("planner_configs/" + planner_config_name + "/goal_bias", config.trrt.goal_bias);
  validatePositive("planner_configs/" + planner_config_name + "/init_temperature", config.trrt.init_temperature);
  validateNonNegative("planner_configs/" + planner_config_name + "/frontier_threshold",
                      config.trrt.frontier_threshold);
  validatePositive("planner_configs/" + planner_config_name + "/frontier_node_ratio",
                   config.trrt.frontier_node_ratio);
  validateNonNegative("planner_configs/" + planner_config_name + "/min_parent_child_distance",
                      config.trrt.min_parent_child_distance);
  validateNonNegative("planner_configs/" + planner_config_name + "/min_parent_child_distance_scale",
                      config.trrt.min_parent_child_distance_scale);
  validatePositive("solve/max_planning_time_seconds", config.solve.max_planning_time_seconds);
  validatePositive("solve/goal_tolerance", config.solve.goal_tolerance);
  validateNonNegative("solve/max_approximate_goal_distance", config.solve.max_approximate_goal_distance);
  validatePositive("solve/interpolation_step", config.solve.interpolation_step);
  validateProbability("solve/reduce_vertices_range_ratio", config.solve.reduce_vertices_range_ratio);
  validateProbability("solve/shortcut_range_ratio", config.solve.shortcut_range_ratio);
  validateProbability("solve/shortcut_snap_to_vertex", config.solve.shortcut_snap_to_vertex);
  validateProbability("solve/preserve_tail_ratio", config.solve.preserve_tail_ratio);
  validateNonNegative("solve/smooth_bspline_min_change", config.solve.smooth_bspline_min_change);
  validatePositive("cost_objective/clearance_epsilon", config.cost_objective.clearance_epsilon);
  validatePositive("cost_objective/invalid_state_cost", config.cost_objective.invalid_state_cost);
  validatePositive("heatmap/resolution", config.heatmap.resolution);
  validateNonNegative("heatmap/request_timeout_seconds", config.heatmap.request_timeout_seconds);
  validateNonNegative("heatmap/workspace_padding", config.heatmap.workspace_padding);
  validateNonNegative("heatmap/objective_weight", config.heatmap.objective_weight);
  validateNonNegative("heatmap/objective_clearance_threshold", config.heatmap.objective_clearance_threshold);
  validateProbability("heatmap/sampling_min_keep_probability", config.heatmap.sampling_min_keep_probability);
  validatePositive("visualization/edge_scale", config.visualization.edge_scale);
  validatePositive("visualization/vertex_scale", config.visualization.vertex_scale);
  validatePositive("visualization/solution_branch_edge_scale", config.visualization.solution_branch_edge_scale);
  validatePositive("visualization/solution_edge_scale", config.visualization.solution_edge_scale);
  validateColor("visualization/left_marker_rgba", config.visualization.left_marker_rgba);
  validateColor("visualization/right_marker_rgba", config.visualization.right_marker_rgba);
  validateColor("visualization/left_solution_branch_rgba", config.visualization.left_solution_branch_rgba);
  validateColor("visualization/right_solution_branch_rgba", config.visualization.right_solution_branch_rgba);
  validateColor("visualization/left_solution_rgba", config.visualization.left_solution_rgba);
  validateColor("visualization/right_solution_rgba", config.visualization.right_solution_rgba);

  if (config.visualization.max_tree_vertices < 0)
  {
    throw std::runtime_error("visualization/max_tree_vertices must be greater than or equal to zero.");
  }

  if (config.visualization.max_tree_edges < 0)
  {
    throw std::runtime_error("visualization/max_tree_edges must be greater than or equal to zero.");
  }

  if (config.visualization.max_solution_branch_points < 0)
  {
    throw std::runtime_error("visualization/max_solution_branch_points must be greater than or equal to zero.");
  }

  if (config.visualization.max_solution_path_points < 0)
  {
    throw std::runtime_error("visualization/max_solution_path_points must be greater than or equal to zero.");
  }

  config.trrt.sampling_strategy = toLowerString(trimString(config.trrt.sampling_strategy));
  if (config.trrt.sampling_strategy != "uniform" && config.trrt.sampling_strategy != "sobol")
  {
    throw std::runtime_error("planner_configs/" + planner_config_name +
                             "/sampling_strategy must be either uniform or sobol.");
  }

  if (config.trrt.max_sampling_attempts_per_iteration <= 0)
  {
    throw std::runtime_error("planner_configs/" + planner_config_name +
                             "/max_sampling_attempts_per_iteration must be greater than zero.");
  }

  if (config.trrt.sobol_candidate_pool_size <= 0)
  {
    throw std::runtime_error("planner_configs/" + planner_config_name +
                             "/sobol_candidate_pool_size must be greater than zero.");
  }

  if (config.solve.planning_attempts <= 0)
  {
    throw std::runtime_error("solve/planning_attempts must be greater than zero.");
  }

  if (config.solve.reduce_vertices_max_steps < 0)
  {
    throw std::runtime_error("solve/reduce_vertices_max_steps must be greater than or equal to zero.");
  }

  if (config.solve.reduce_vertices_max_empty_steps < 0)
  {
    throw std::runtime_error("solve/reduce_vertices_max_empty_steps must be greater than or equal to zero.");
  }

  if (config.solve.shortcut_max_steps < 0)
  {
    throw std::runtime_error("solve/shortcut_max_steps must be greater than or equal to zero.");
  }

  if (config.solve.shortcut_max_empty_steps < 0)
  {
    throw std::runtime_error("solve/shortcut_max_empty_steps must be greater than or equal to zero.");
  }

  if (config.solve.smooth_bspline_max_steps < 0)
  {
    throw std::runtime_error("solve/smooth_bspline_max_steps must be greater than or equal to zero.");
  }

  config.solve.simplification_strategy = toLowerString(trimString(config.solve.simplification_strategy));
  if (config.solve.simplification_strategy != "simplify_max" &&
      config.solve.simplification_strategy != "reduce_vertices" &&
      config.solve.simplification_strategy != "shortcut" &&
      config.solve.simplification_strategy != "reduce_shortcut")
  {
    throw std::runtime_error("solve/simplification_strategy must be one of simplify_max, reduce_vertices, "
                             "shortcut, or reduce_shortcut.");
  }

  if (config.visualization.enabled)
  {
    validateNonEmpty("visualization/marker_topic", config.visualization.marker_topic);
    validateNonEmpty("visualization/frame_id", config.visualization.frame_id);
    validateNonEmpty("visualization/left_tip_link", config.visualization.left_tip_link);
    validateNonEmpty("visualization/right_tip_link", config.visualization.right_tip_link);
  }

  if (config.heatmap.safety_clearance_high <= config.heatmap.safety_clearance_low)
  {
    throw std::runtime_error("heatmap/safety_clearance_high must be greater than heatmap/safety_clearance_low.");
  }

  if (config.heatmap.step_min_scale <= 0.0 || config.heatmap.step_min_scale > 1.0)
  {
    throw std::runtime_error("heatmap/step_min_scale must be within (0.0, 1.0].");
  }

  if (config.heatmap.enabled)
  {
    validateNonEmpty("heatmap/environment_topic", config.heatmap.environment_topic);
    validateNonEmpty("heatmap/service_name", config.heatmap.service_name);
    validateNonEmpty("heatmap/left_query_link", config.heatmap.left_query_link);
    validateNonEmpty("heatmap/right_query_link", config.heatmap.right_query_link);
  }

  for (const std::string& joint_name : config.robot.joint_order)
  {
    const JointBoundConfig& joint_bound = config.state_space.joint_bounds.at(joint_name);
    if (joint_bound.lower >= joint_bound.upper)
    {
      throw std::runtime_error("Joint bound lower limit must be smaller than upper limit for joint " + joint_name +
                               ".");
    }
    validatePositive("state_space/joint_bounds/" + joint_name + "/max_velocity", joint_bound.max_velocity);
    if (joint_bound.has_acceleration_limits)
    {
      validatePositive("state_space/joint_bounds/" + joint_name + "/max_acceleration",
                       joint_bound.max_acceleration);
    }
  }

  return config;
}
}  // namespace dual_arm_rrt_planner
