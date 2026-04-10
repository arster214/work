#include <dual_arm_rrt_planner/dual_arm_tree_visualizer.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace dual_arm_rrt_planner
{
namespace
{
bool isFinitePoint(const geometry_msgs::Point& point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

geometry_msgs::Point toGeometryPoint(const Eigen::Vector3d& vector)
{
  geometry_msgs::Point point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

std::size_t computeSamplingStride(std::size_t total_count, int configured_limit)
{
  if (configured_limit <= 0)
  {
    return 1U;
  }

  const std::size_t limit = static_cast<std::size_t>(configured_limit);
  if (total_count <= limit)
  {
    return 1U;
  }

  return (total_count + limit - 1U) / limit;
}

bool shouldKeepSample(std::size_t sample_index, std::size_t total_count, std::size_t stride)
{
  if (stride <= 1U)
  {
    return true;
  }

  return sample_index == 0U || sample_index + 1U == total_count || sample_index % stride == 0U;
}

void downsamplePointSequence(std::vector<geometry_msgs::Point>& points, int configured_limit, bool pairwise)
{
  if (configured_limit <= 0)
  {
    points.clear();
    return;
  }

  std::vector<geometry_msgs::Point> filtered_points;
  filtered_points.reserve(points.size());
  for (const geometry_msgs::Point& point : points)
  {
    if (isFinitePoint(point))
    {
      filtered_points.push_back(point);
    }
  }
  points.swap(filtered_points);

  if (pairwise && points.size() % 2U != 0U)
  {
    points.pop_back();
  }

  if (points.empty())
  {
    return;
  }

  if (!pairwise)
  {
    const std::size_t stride = computeSamplingStride(points.size(), configured_limit);
    if (stride <= 1U)
    {
      return;
    }

    std::vector<geometry_msgs::Point> downsampled_points;
    downsampled_points.reserve(static_cast<std::size_t>(configured_limit));
    for (std::size_t index = 0U; index < points.size(); ++index)
    {
      if (shouldKeepSample(index, points.size(), stride))
      {
        downsampled_points.push_back(points[index]);
      }
    }
    points.swap(downsampled_points);
    return;
  }

  const std::size_t pair_count = points.size() / 2U;
  const std::size_t stride = computeSamplingStride(pair_count, configured_limit);
  if (stride <= 1U)
  {
    return;
  }

  std::vector<geometry_msgs::Point> downsampled_points;
  downsampled_points.reserve(static_cast<std::size_t>(configured_limit) * 2U);
  for (std::size_t pair_index = 0U; pair_index < pair_count; ++pair_index)
  {
    if (shouldKeepSample(pair_index, pair_count, stride))
    {
      downsampled_points.push_back(points[pair_index * 2U]);
      downsampled_points.push_back(points[pair_index * 2U + 1U]);
    }
  }
  points.swap(downsampled_points);
}
}  // namespace

DualArmTreeVisualizer::DualArmTreeVisualizer(const PlannerConfig& config,
                                             const moveit::core::RobotModelConstPtr& robot_model,
                                             ros::NodeHandle node_handle)
  : config_(config), robot_model_(robot_model)
{
  if (!robot_model_)
  {
    throw std::runtime_error("DualArmTreeVisualizer received a null robot model.");
  }

  if (!config_.visualization.enabled)
  {
    return;
  }

  left_tip_link_model_ = robot_model_->getLinkModel(config_.visualization.left_tip_link);
  if (!left_tip_link_model_)
  {
    throw std::runtime_error("Tree visualization left_tip_link does not exist in the robot model: " +
                             config_.visualization.left_tip_link);
  }

  right_tip_link_model_ = robot_model_->getLinkModel(config_.visualization.right_tip_link);
  if (!right_tip_link_model_)
  {
    throw std::runtime_error("Tree visualization right_tip_link does not exist in the robot model: " +
                             config_.visualization.right_tip_link);
  }

  marker_publisher_ =
      node_handle.advertise<visualization_msgs::MarkerArray>(config_.visualization.marker_topic, 1,
                                                             config_.visualization.marker_latch);
}

void DualArmTreeVisualizer::clear()
{
  if (!config_.visualization.enabled || !marker_publisher_)
  {
    return;
  }

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(makeDeleteAllMarker(ros::Time::now()));
  marker_publisher_.publish(marker_array);
}

void DualArmTreeVisualizer::publishPlannerData(const ompl::base::PlannerData& planner_data,
                                               const std::string& planner_name, bool planning_succeeded,
                                               int attempt_index,
                                               const ompl::geometric::PathGeometric* solution_branch_path,
                                               const ompl::geometric::PathGeometric* solution_path,
                                               bool left_arm_active, bool right_arm_active)
{
  if (!config_.visualization.enabled || !marker_publisher_)
  {
    return;
  }

  if (planning_succeeded && !config_.visualization.publish_on_success)
  {
    return;
  }

  if (!planning_succeeded && !config_.visualization.publish_on_failure)
  {
    return;
  }

  const ros::Time stamp = ros::Time::now();
  const std::size_t vertex_stride =
      computeSamplingStride(static_cast<std::size_t>(planner_data.numVertices()), config_.visualization.max_tree_vertices);
  const std::size_t edge_stride =
      computeSamplingStride(static_cast<std::size_t>(planner_data.numEdges()), config_.visualization.max_tree_edges);
  std::vector<EndEffectorPointPair> vertex_point_pairs(planner_data.numVertices());
  std::vector<bool> has_valid_vertex_points(planner_data.numVertices(), false);
  std::vector<geometry_msgs::Point> left_edge_points;
  std::vector<geometry_msgs::Point> right_edge_points;
  std::vector<geometry_msgs::Point> left_vertex_points;
  std::vector<geometry_msgs::Point> right_vertex_points;
  std::vector<geometry_msgs::Point> left_static_anchor_points;
  std::vector<geometry_msgs::Point> right_static_anchor_points;

  left_edge_points.reserve(planner_data.numEdges() * 2U);
  right_edge_points.reserve(planner_data.numEdges() * 2U);
  left_vertex_points.reserve(planner_data.numVertices());
  right_vertex_points.reserve(planner_data.numVertices());

  for (unsigned int vertex_index = 0; vertex_index < planner_data.numVertices(); ++vertex_index)
  {
    const EndEffectorPointPair point_pair =
        computeEndEffectorPoints(planner_data.getVertex(vertex_index).getState());
    if (!point_pair.valid)
    {
      continue;
    }

    vertex_point_pairs[vertex_index] = point_pair;
    has_valid_vertex_points[vertex_index] = true;
    if (!left_arm_active && left_static_anchor_points.empty())
    {
      left_static_anchor_points.push_back(point_pair.left_point);
    }

    if (!right_arm_active && right_static_anchor_points.empty())
    {
      right_static_anchor_points.push_back(point_pair.right_point);
    }

    if (shouldKeepSample(static_cast<std::size_t>(vertex_index), static_cast<std::size_t>(planner_data.numVertices()),
                         vertex_stride))
    {
      if (left_arm_active)
      {
        left_vertex_points.push_back(point_pair.left_point);
      }

      if (right_arm_active)
      {
        right_vertex_points.push_back(point_pair.right_point);
      }
    }
  }

  std::size_t traversed_edge_count = 0U;
  for (unsigned int parent_index = 0; parent_index < planner_data.numVertices(); ++parent_index)
  {
    if (!has_valid_vertex_points[parent_index])
    {
      continue;
    }

    std::vector<unsigned int> child_indices;
    planner_data.getEdges(parent_index, child_indices);
    for (const unsigned int child_index : child_indices)
    {
      if (child_index >= has_valid_vertex_points.size() || !has_valid_vertex_points[child_index])
      {
        continue;
      }

      if (shouldKeepSample(traversed_edge_count, static_cast<std::size_t>(planner_data.numEdges()), edge_stride))
      {
        if (left_arm_active)
        {
          left_edge_points.push_back(vertex_point_pairs[parent_index].left_point);
          left_edge_points.push_back(vertex_point_pairs[child_index].left_point);
        }

        if (right_arm_active)
        {
          right_edge_points.push_back(vertex_point_pairs[parent_index].right_point);
          right_edge_points.push_back(vertex_point_pairs[child_index].right_point);
        }
      }
      ++traversed_edge_count;
    }
  }

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(makeDeleteAllMarker(stamp));
  downsamplePointSequence(left_edge_points, config_.visualization.max_tree_edges, true);
  downsamplePointSequence(right_edge_points, config_.visualization.max_tree_edges, true);
  downsamplePointSequence(left_vertex_points, config_.visualization.max_tree_vertices, false);
  downsamplePointSequence(right_vertex_points, config_.visualization.max_tree_vertices, false);
  downsamplePointSequence(left_static_anchor_points, 1, false);
  downsamplePointSequence(right_static_anchor_points, 1, false);
  if (left_arm_active)
  {
    if (!left_edge_points.empty())
    {
      marker_array.markers.push_back(
          makeLineListMarker(planner_name + "_left_tree_edges", 1, stamp, config_.visualization.left_marker_rgba,
                             left_edge_points));
    }
  }

  if (right_arm_active)
  {
    if (!right_edge_points.empty())
    {
      marker_array.markers.push_back(
          makeLineListMarker(planner_name + "_right_tree_edges", 2, stamp, config_.visualization.right_marker_rgba,
                             right_edge_points));
    }
  }

  if (config_.visualization.publish_vertices)
  {
    if (left_arm_active && !left_vertex_points.empty())
    {
      marker_array.markers.push_back(makeSphereListMarker(planner_name + "_left_tree_vertices", 3, stamp,
                                                          config_.visualization.left_marker_rgba, left_vertex_points));
    }
    else if (!left_static_anchor_points.empty())
    {
      marker_array.markers.push_back(
          makeSphereListMarker(planner_name + "_left_tree_static_anchor", 3, stamp,
                               config_.visualization.left_marker_rgba, left_static_anchor_points));
    }

    if (right_arm_active && !right_vertex_points.empty())
    {
      marker_array.markers.push_back(makeSphereListMarker(planner_name + "_right_tree_vertices", 4, stamp,
                                                          config_.visualization.right_marker_rgba, right_vertex_points));
    }
    else if (!right_static_anchor_points.empty())
    {
      marker_array.markers.push_back(
          makeSphereListMarker(planner_name + "_right_tree_static_anchor", 4, stamp,
                               config_.visualization.right_marker_rgba, right_static_anchor_points));
    }
  }

  if (planning_succeeded && config_.visualization.publish_solution_branch && solution_branch_path)
  {
    std::vector<geometry_msgs::Point> left_solution_branch_points;
    std::vector<geometry_msgs::Point> right_solution_branch_points;
    collectPathPoints(*solution_branch_path, left_solution_branch_points, right_solution_branch_points);
    downsamplePointSequence(left_solution_branch_points, config_.visualization.max_solution_branch_points, false);
    downsamplePointSequence(right_solution_branch_points, config_.visualization.max_solution_branch_points, false);

    if (left_arm_active && left_solution_branch_points.size() >= 2U)
    {
      marker_array.markers.push_back(makeLineStripMarker(
          planner_name + "_left_solution_branch", 5, stamp, config_.visualization.left_solution_branch_rgba,
          config_.visualization.solution_branch_edge_scale, left_solution_branch_points));
    }

    if (right_arm_active && right_solution_branch_points.size() >= 2U)
    {
      marker_array.markers.push_back(makeLineStripMarker(
          planner_name + "_right_solution_branch", 6, stamp, config_.visualization.right_solution_branch_rgba,
          config_.visualization.solution_branch_edge_scale, right_solution_branch_points));
    }
  }

  if (planning_succeeded && config_.visualization.publish_solution_path && solution_path)
  {
    std::vector<geometry_msgs::Point> left_solution_points;
    std::vector<geometry_msgs::Point> right_solution_points;
    collectPathPoints(*solution_path, left_solution_points, right_solution_points);
    downsamplePointSequence(left_solution_points, config_.visualization.max_solution_path_points, false);
    downsamplePointSequence(right_solution_points, config_.visualization.max_solution_path_points, false);

    if (left_arm_active && left_solution_points.size() >= 2U)
    {
      marker_array.markers.push_back(makeLineStripMarker(
          planner_name + "_left_solution_path", 7, stamp, config_.visualization.left_solution_rgba,
          config_.visualization.solution_edge_scale, left_solution_points));
    }

    if (right_arm_active && right_solution_points.size() >= 2U)
    {
      marker_array.markers.push_back(makeLineStripMarker(
          planner_name + "_right_solution_path", 8, stamp, config_.visualization.right_solution_rgba,
          config_.visualization.solution_edge_scale, right_solution_points));
    }
  }

  marker_publisher_.publish(marker_array);

  if (vertex_stride > 1U || edge_stride > 1U)
  {
    ROS_WARN_STREAM("[DualArmTRRT] Tree visualization was downsampled for RViz stability. Raw vertices="
                    << planner_data.numVertices() << ", raw edges=" << planner_data.numEdges()
                    << ", published vertex samples per arm=" << left_vertex_points.size()
                    << ", published edge samples per arm=" << (left_edge_points.size() / 2U)
                    << ", vertex stride=" << vertex_stride << ", edge stride=" << edge_stride << ".");
  }

  ROS_INFO_STREAM("[DualArmTRRT] Published tree markers for " << planner_name << " attempt " << attempt_index
                                                              << " with " << planner_data.numVertices()
                                                              << " vertices and " << planner_data.numEdges()
                                                              << " edges on topic "
                                                              << marker_publisher_.getTopic() << ".");
}

bool DualArmTreeVisualizer::publishSolutionPathOnly(const std::string& planner_name,
                                                    const ompl::geometric::PathGeometric& solution_path,
                                                    bool left_arm_active, bool right_arm_active)
{
  if (!config_.visualization.enabled || !marker_publisher_ || !config_.visualization.publish_solution_path)
  {
    return false;
  }

  std::vector<geometry_msgs::Point> left_solution_points;
  std::vector<geometry_msgs::Point> right_solution_points;
  collectPathPoints(solution_path, left_solution_points, right_solution_points);
  downsamplePointSequence(left_solution_points, config_.visualization.max_solution_path_points, false);
  downsamplePointSequence(right_solution_points, config_.visualization.max_solution_path_points, false);

  visualization_msgs::MarkerArray marker_array;
  const ros::Time stamp = ros::Time::now();
  bool published_any_marker = false;

  if (left_arm_active && left_solution_points.size() >= 2U)
  {
    marker_array.markers.push_back(makeLineStripMarker(
        planner_name + "_left_solution_path", 7, stamp, config_.visualization.left_solution_rgba,
        config_.visualization.solution_edge_scale, left_solution_points));
    published_any_marker = true;
  }

  if (right_arm_active && right_solution_points.size() >= 2U)
  {
    marker_array.markers.push_back(makeLineStripMarker(
        planner_name + "_right_solution_path", 8, stamp, config_.visualization.right_solution_rgba,
        config_.visualization.solution_edge_scale, right_solution_points));
    published_any_marker = true;
  }

  if (!published_any_marker)
  {
    ROS_WARN_STREAM("[DualArmTRRT] Unable to publish final solution highlight for " << planner_name
                    << " because fewer than two valid end-effector samples were available.");
    return false;
  }

  marker_publisher_.publish(marker_array);
  return true;
}

DualArmTreeVisualizer::EndEffectorPointPair DualArmTreeVisualizer::computeEndEffectorPoints(
    const ompl::base::State* state) const
{
  EndEffectorPointPair point_pair;
  if (!state || !left_tip_link_model_ || !right_tip_link_model_)
  {
    return point_pair;
  }

  moveit::core::RobotState robot_state(robot_model_);
  try
  {
    robot_state = toRobotState(state);
  }
  catch (const std::exception&)
  {
    return point_pair;
  }
  const Eigen::Vector3d left_translation = robot_state.getGlobalLinkTransform(left_tip_link_model_).translation();
  const Eigen::Vector3d right_translation = robot_state.getGlobalLinkTransform(right_tip_link_model_).translation();

  if (!std::isfinite(left_translation.x()) || !std::isfinite(left_translation.y()) ||
      !std::isfinite(left_translation.z()) || !std::isfinite(right_translation.x()) ||
      !std::isfinite(right_translation.y()) || !std::isfinite(right_translation.z()))
  {
    return point_pair;
  }

  point_pair.left_point = toGeometryPoint(left_translation);
  point_pair.right_point = toGeometryPoint(right_translation);
  point_pair.valid = true;
  return point_pair;
}

void DualArmTreeVisualizer::collectPathPoints(const ompl::geometric::PathGeometric& path,
                                              std::vector<geometry_msgs::Point>& left_points,
                                              std::vector<geometry_msgs::Point>& right_points) const
{
  left_points.clear();
  right_points.clear();
  left_points.reserve(path.getStateCount());
  right_points.reserve(path.getStateCount());

  for (std::size_t state_index = 0; state_index < path.getStateCount(); ++state_index)
  {
    const EndEffectorPointPair point_pair = computeEndEffectorPoints(path.getState(state_index));
    if (!point_pair.valid)
    {
      continue;
    }

    left_points.push_back(point_pair.left_point);
    right_points.push_back(point_pair.right_point);
  }
}

moveit::core::RobotState DualArmTreeVisualizer::toRobotState(const ompl::base::State* state) const
{
  const auto* joint_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
  if (!joint_state)
  {
    throw std::runtime_error("Tree visualization received a non-RealVector OMPL state.");
  }

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  for (std::size_t joint_index = 0; joint_index < config_.robot.joint_order.size(); ++joint_index)
  {
    robot_state.setVariablePosition(config_.robot.joint_order[joint_index], joint_state->values[joint_index]);
  }

  robot_state.update();
  return robot_state;
}

visualization_msgs::Marker DualArmTreeVisualizer::makeDeleteAllMarker(const ros::Time& stamp) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = config_.visualization.frame_id;
  marker.header.stamp = stamp;
  marker.action = visualization_msgs::Marker::DELETEALL;
  return marker;
}

visualization_msgs::Marker DualArmTreeVisualizer::makeLineListMarker(
    const std::string& marker_namespace, int marker_id, const ros::Time& stamp, const ColorConfig& color,
    const std::vector<geometry_msgs::Point>& points) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = config_.visualization.frame_id;
  marker.header.stamp = stamp;
  marker.ns = marker_namespace;
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = config_.visualization.edge_scale;
  marker.color.r = color.red;
  marker.color.g = color.green;
  marker.color.b = color.blue;
  marker.color.a = color.alpha;
  marker.points = points;
  if (marker.points.size() % 2U != 0U)
  {
    marker.points.pop_back();
  }
  return marker;
}

visualization_msgs::Marker DualArmTreeVisualizer::makeSphereListMarker(
    const std::string& marker_namespace, int marker_id, const ros::Time& stamp, const ColorConfig& color,
    const std::vector<geometry_msgs::Point>& points) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = config_.visualization.frame_id;
  marker.header.stamp = stamp;
  marker.ns = marker_namespace;
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = config_.visualization.vertex_scale;
  marker.scale.y = config_.visualization.vertex_scale;
  marker.scale.z = config_.visualization.vertex_scale;
  marker.color.r = color.red;
  marker.color.g = color.green;
  marker.color.b = color.blue;
  marker.color.a = color.alpha;
  marker.points = points;
  return marker;
}

visualization_msgs::Marker DualArmTreeVisualizer::makeLineStripMarker(
    const std::string& marker_namespace, int marker_id, const ros::Time& stamp, const ColorConfig& color,
    double scale, const std::vector<geometry_msgs::Point>& points) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = config_.visualization.frame_id;
  marker.header.stamp = stamp;
  marker.ns = marker_namespace;
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale;
  marker.color.r = color.red;
  marker.color.g = color.green;
  marker.color.b = color.blue;
  marker.color.a = color.alpha;
  marker.points = points;
  return marker;
}
}  // namespace dual_arm_rrt_planner
