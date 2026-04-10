#include <dual_arm_rrt_planner/dual_arm_collision_interface.h>

#include <stdexcept>

#include <moveit/collision_detection/collision_common.h>

namespace dual_arm_rrt_planner
{
DualArmCollisionInterface::DualArmCollisionInterface(const PlannerConfig& config,
                                                     const moveit::core::RobotModelConstPtr& robot_model,
                                                     const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                     const ompl::base::StateSpacePtr& state_space)
  : config_(config)
  , robot_model_(robot_model)
  , planning_scene_(planning_scene)
  , state_space_(state_space)
  , distance_interface_(std::make_shared<DualArmDistanceInterface>(config, robot_model, planning_scene, state_space))
{
  if (!robot_model_)
  {
    throw std::runtime_error("DualArmCollisionInterface received a null robot model.");
  }

  if (!planning_scene_)
  {
    throw std::runtime_error("DualArmCollisionInterface received a null planning scene.");
  }

  if (!state_space_)
  {
    throw std::runtime_error("DualArmCollisionInterface received a null OMPL state space.");
  }
}

bool DualArmCollisionInterface::isStateValid(const ompl::base::State* state) const
{
  if (!state || !satisfiesConfiguredBounds(state))
  {
    return false;
  }

  moveit::core::RobotState robot_state = toRobotState(state);

  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = config_.robot.planning_group;
  collision_detection::CollisionResult collision_result;
  const collision_detection::AllowedCollisionMatrix& allowed_collision_matrix =
      planning_scene_->getAllowedCollisionMatrix();

  if (config_.collision_checking.check_self_collision && config_.collision_checking.check_environment_collision)
  {
    planning_scene_->checkCollisionUnpadded(collision_request, collision_result, robot_state, allowed_collision_matrix);
  }
  else if (config_.collision_checking.check_self_collision)
  {
    planning_scene_->checkSelfCollision(collision_request, collision_result, robot_state, allowed_collision_matrix);
  }
  else
  {
    planning_scene_->getCollisionEnvUnpadded()->checkRobotCollision(collision_request, collision_result, robot_state,
                                                                    allowed_collision_matrix);
  }

  return !collision_result.collision;
}

double DualArmCollisionInterface::clearance(const ompl::base::State* state) const
{
  if (!state || !distance_interface_)
  {
    return 0.0;
  }

  DistanceQueryOptions options;
  options.include_self_distance = config_.collision_checking.check_self_collision;
  options.include_environment_distance = config_.collision_checking.check_environment_collision;
  options.enable_nearest_points = false;
  options.collect_all_pairs = false;
  options.enable_signed_distance = true;

  const DistanceQueryResult query_result = distance_interface_->query(state, options);
  if (!query_result.valid || !std::isfinite(query_result.minimum_distance))
  {
    return 0.0;
  }

  return std::max(0.0, query_result.minimum_distance);
}

moveit::core::RobotState DualArmCollisionInterface::toRobotState(const ompl::base::State* state) const
{
  if (!distance_interface_)
  {
    throw std::runtime_error("DualArmCollisionInterface distance interface is not available.");
  }

  return distance_interface_->toRobotState(state);
}

bool DualArmCollisionInterface::satisfiesConfiguredBounds(const ompl::base::State* state) const
{
  if (!config_.state_space.enforce_joint_bounds)
  {
    return true;
  }
  return state_space_->satisfiesBounds(state);
}
}  // namespace dual_arm_rrt_planner
