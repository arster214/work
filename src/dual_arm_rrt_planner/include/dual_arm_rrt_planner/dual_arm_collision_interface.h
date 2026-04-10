#pragma once

#include <memory>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>

#include <dual_arm_rrt_planner/dual_arm_distance_interface.h>
#include <dual_arm_rrt_planner/planner_config.h>

namespace dual_arm_rrt_planner
{
class DualArmCollisionInterface
{
public:
  DualArmCollisionInterface(const PlannerConfig& config, const moveit::core::RobotModelConstPtr& robot_model,
                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const ompl::base::StateSpacePtr& state_space);

  bool isStateValid(const ompl::base::State* state) const;

  double clearance(const ompl::base::State* state) const;

  moveit::core::RobotState toRobotState(const ompl::base::State* state) const;

private:
  bool satisfiesConfiguredBounds(const ompl::base::State* state) const;

  const PlannerConfig& config_;
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  ompl::base::StateSpacePtr state_space_;
  DualArmDistanceInterfacePtr distance_interface_;
};

using DualArmCollisionInterfacePtr = std::shared_ptr<DualArmCollisionInterface>;
}  // namespace dual_arm_rrt_planner
