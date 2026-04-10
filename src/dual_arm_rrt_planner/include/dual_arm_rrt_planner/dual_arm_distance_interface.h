#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>

#include <dual_arm_rrt_planner/planner_config.h>

namespace dual_arm_rrt_planner
{
enum class DistanceDomain
{
  SELF,
  ENVIRONMENT,
  COMBINED
};

struct DistanceQueryOptions
{
  bool include_self_distance = true;
  bool include_environment_distance = true;
  bool enable_nearest_points = true;
  bool enable_signed_distance = false;
  bool compute_gradient = false;
  bool collect_all_pairs = false;
  bool verbose = false;
  double distance_threshold = std::numeric_limits<double>::max();
  std::size_t max_contacts_per_body = 1U;
};

struct DistancePairResult
{
  DistanceDomain domain = DistanceDomain::COMBINED;
  bool valid = false;
  bool collision = false;
  double distance = std::numeric_limits<double>::infinity();
  std::string first_name;
  std::string second_name;
  collision_detection::BodyType first_body_type = collision_detection::BodyType::WORLD_OBJECT;
  collision_detection::BodyType second_body_type = collision_detection::BodyType::WORLD_OBJECT;
  Eigen::Vector3d first_nearest_point = Eigen::Vector3d::Zero();
  Eigen::Vector3d second_nearest_point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
};

struct DistanceQueryResult
{
  bool valid = false;
  bool in_bounds = true;
  bool collision = false;
  bool has_self_distance = false;
  bool has_environment_distance = false;
  double minimum_distance = std::numeric_limits<double>::infinity();
  double self_distance = std::numeric_limits<double>::infinity();
  double environment_distance = std::numeric_limits<double>::infinity();
  DistancePairResult nearest_pair;
  DistancePairResult nearest_self_pair;
  DistancePairResult nearest_environment_pair;
  std::vector<DistancePairResult> self_pairs;
  std::vector<DistancePairResult> environment_pairs;
};

class DualArmDistanceInterface
{
public:
  DualArmDistanceInterface(const PlannerConfig& config, const moveit::core::RobotModelConstPtr& robot_model,
                           const planning_scene::PlanningSceneConstPtr& planning_scene,
                           const ompl::base::StateSpacePtr& state_space);

  DistanceQueryResult query(const ompl::base::State* state,
                            const DistanceQueryOptions& options = DistanceQueryOptions()) const;

  DistanceQueryResult query(const moveit::core::RobotState& robot_state,
                            const DistanceQueryOptions& options = DistanceQueryOptions()) const;

  DistanceQueryResult query(const std::vector<double>& joint_values,
                            const DistanceQueryOptions& options = DistanceQueryOptions()) const;

  moveit::core::RobotState toRobotState(const ompl::base::State* state) const;

  moveit::core::RobotState toRobotState(const std::vector<double>& joint_values) const;

private:
  void assignStateToRobotState(const ompl::base::State* state, moveit::core::RobotState& robot_state) const;

  void assignJointValuesToRobotState(const std::vector<double>& joint_values,
                                     moveit::core::RobotState& robot_state) const;

  bool satisfiesConfiguredBounds(const ompl::base::State* state) const;

  const PlannerConfig& config_;
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  ompl::base::StateSpacePtr state_space_;
};

using DualArmDistanceInterfacePtr = std::shared_ptr<DualArmDistanceInterface>;
}  // namespace dual_arm_rrt_planner
