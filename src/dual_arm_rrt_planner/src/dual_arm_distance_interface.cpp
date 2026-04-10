#include <dual_arm_rrt_planner/dual_arm_distance_interface.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace dual_arm_rrt_planner
{
namespace
{
DistancePairResult toDistancePairResult(const collision_detection::DistanceResultsData& data, DistanceDomain domain)
{
  DistancePairResult result;
  result.domain = domain;
  result.valid = std::isfinite(data.distance);
  result.collision = data.distance <= 0.0;
  result.distance = data.distance;
  result.first_name = data.link_names[0];
  result.second_name = data.link_names[1];
  result.first_body_type = data.body_types[0];
  result.second_body_type = data.body_types[1];
  result.first_nearest_point = data.nearest_points[0];
  result.second_nearest_point = data.nearest_points[1];
  result.normal = data.normal;
  return result;
}

void appendDistancePairs(const collision_detection::DistanceMap& distance_map, DistanceDomain domain,
                         std::vector<DistancePairResult>& pairs)
{
  for (const auto& entry : distance_map)
  {
    for (const collision_detection::DistanceResultsData& distance_data : entry.second)
    {
      pairs.push_back(toDistancePairResult(distance_data, domain));
    }
  }
}

collision_detection::DistanceRequest makeDistanceRequest(const PlannerConfig& config,
                                                         const moveit::core::RobotModelConstPtr& robot_model,
                                                         const collision_detection::AllowedCollisionMatrix& acm,
                                                         const DistanceQueryOptions& options)
{
  collision_detection::DistanceRequest request;
  request.enable_nearest_points = options.enable_nearest_points;
  request.enable_signed_distance = options.enable_signed_distance;
  request.compute_gradient = options.compute_gradient;
  request.verbose = options.verbose;
  request.distance_threshold = options.distance_threshold;
  request.max_contacts_per_body = options.max_contacts_per_body;
  request.group_name = config.robot.planning_group;
  request.acm = &acm;
  request.type =
      options.collect_all_pairs ? collision_detection::DistanceRequestTypes::ALL
                                : collision_detection::DistanceRequestTypes::SINGLE;
  request.enableGroup(robot_model);
  return request;
}
}  // namespace

DualArmDistanceInterface::DualArmDistanceInterface(const PlannerConfig& config,
                                                   const moveit::core::RobotModelConstPtr& robot_model,
                                                   const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                   const ompl::base::StateSpacePtr& state_space)
  : config_(config), robot_model_(robot_model), planning_scene_(planning_scene), state_space_(state_space)
{
  if (!robot_model_)
  {
    throw std::runtime_error("DualArmDistanceInterface received a null robot model.");
  }

  if (!planning_scene_)
  {
    throw std::runtime_error("DualArmDistanceInterface received a null planning scene.");
  }

  if (!state_space_)
  {
    throw std::runtime_error("DualArmDistanceInterface received a null OMPL state space.");
  }
}

DistanceQueryResult DualArmDistanceInterface::query(const ompl::base::State* state,
                                                    const DistanceQueryOptions& options) const
{
  DistanceQueryResult result;
  if (!state)
  {
    result.valid = false;
    result.in_bounds = false;
    result.minimum_distance = 0.0;
    return result;
  }

  if (!satisfiesConfiguredBounds(state))
  {
    result.valid = false;
    result.in_bounds = false;
    result.minimum_distance = 0.0;
    return result;
  }

  return query(toRobotState(state), options);
}

DistanceQueryResult DualArmDistanceInterface::query(const moveit::core::RobotState& robot_state,
                                                    const DistanceQueryOptions& options) const
{
  DistanceQueryResult result;
  result.valid = true;

  const collision_detection::AllowedCollisionMatrix& allowed_collision_matrix =
      planning_scene_->getAllowedCollisionMatrix();
  const auto distance_request = makeDistanceRequest(config_, robot_model_, allowed_collision_matrix, options);

  if (options.include_self_distance)
  {
    collision_detection::DistanceResult self_result;
    planning_scene_->getCollisionEnvUnpadded()->distanceSelf(distance_request, self_result, robot_state);

    result.has_self_distance = std::isfinite(self_result.minimum_distance.distance);
    result.self_distance = self_result.minimum_distance.distance;
    result.nearest_self_pair = toDistancePairResult(self_result.minimum_distance, DistanceDomain::SELF);
    result.collision = result.collision || self_result.collision || result.self_distance <= 0.0;

    if (options.collect_all_pairs)
    {
      appendDistancePairs(self_result.distances, DistanceDomain::SELF, result.self_pairs);
    }
  }

  if (options.include_environment_distance)
  {
    collision_detection::DistanceResult environment_result;
    planning_scene_->getCollisionEnvUnpadded()->distanceRobot(distance_request, environment_result, robot_state);

    result.has_environment_distance = std::isfinite(environment_result.minimum_distance.distance);
    result.environment_distance = environment_result.minimum_distance.distance;
    result.nearest_environment_pair =
        toDistancePairResult(environment_result.minimum_distance, DistanceDomain::ENVIRONMENT);
    result.collision = result.collision || environment_result.collision || result.environment_distance <= 0.0;

    if (options.collect_all_pairs)
    {
      appendDistancePairs(environment_result.distances, DistanceDomain::ENVIRONMENT, result.environment_pairs);
    }
  }

  if (result.has_self_distance && result.has_environment_distance)
  {
    if (result.self_distance <= result.environment_distance)
    {
      result.minimum_distance = result.self_distance;
      result.nearest_pair = result.nearest_self_pair;
      result.nearest_pair.domain = DistanceDomain::COMBINED;
    }
    else
    {
      result.minimum_distance = result.environment_distance;
      result.nearest_pair = result.nearest_environment_pair;
      result.nearest_pair.domain = DistanceDomain::COMBINED;
    }
  }
  else if (result.has_self_distance)
  {
    result.minimum_distance = result.self_distance;
    result.nearest_pair = result.nearest_self_pair;
    result.nearest_pair.domain = DistanceDomain::COMBINED;
  }
  else if (result.has_environment_distance)
  {
    result.minimum_distance = result.environment_distance;
    result.nearest_pair = result.nearest_environment_pair;
    result.nearest_pair.domain = DistanceDomain::COMBINED;
  }

  return result;
}

DistanceQueryResult DualArmDistanceInterface::query(const std::vector<double>& joint_values,
                                                    const DistanceQueryOptions& options) const
{
  return query(toRobotState(joint_values), options);
}

moveit::core::RobotState DualArmDistanceInterface::toRobotState(const ompl::base::State* state) const
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  assignStateToRobotState(state, robot_state);
  robot_state.update();
  return robot_state;
}

moveit::core::RobotState DualArmDistanceInterface::toRobotState(const std::vector<double>& joint_values) const
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  assignJointValuesToRobotState(joint_values, robot_state);
  robot_state.update();
  return robot_state;
}

void DualArmDistanceInterface::assignStateToRobotState(const ompl::base::State* state,
                                                       moveit::core::RobotState& robot_state) const
{
  const auto* joint_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
  if (!joint_state)
  {
    throw std::runtime_error("Received a non-RealVector OMPL state for the dual-arm distance interface.");
  }

  for (std::size_t index = 0; index < config_.robot.joint_order.size(); ++index)
  {
    robot_state.setVariablePosition(config_.robot.joint_order[index], joint_state->values[index]);
  }
}

void DualArmDistanceInterface::assignJointValuesToRobotState(const std::vector<double>& joint_values,
                                                             moveit::core::RobotState& robot_state) const
{
  if (joint_values.size() != config_.robot.joint_order.size())
  {
    throw std::runtime_error("Joint value vector dimension does not match the configured robot joint order.");
  }

  for (std::size_t index = 0; index < config_.robot.joint_order.size(); ++index)
  {
    robot_state.setVariablePosition(config_.robot.joint_order[index], joint_values[index]);
  }
}

bool DualArmDistanceInterface::satisfiesConfiguredBounds(const ompl::base::State* state) const
{
  if (!config_.state_space.enforce_joint_bounds)
  {
    return true;
  }

  return state_space_->satisfiesBounds(state);
}
}  // namespace dual_arm_rrt_planner
